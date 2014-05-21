/*
 * dht22.c - support for the DHT22 Temperature and Humidity Sensor
 *
 * inspired by sht15.c driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * For further information, see the Documentation/hwmon/dht22 file.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mutex.h>
#include <linux/platform_data/dht22.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/time.h>

/* Aosong AM2303 (DHT22) counts */
#define DHT22_BITCOUNT			40	/* DATA = 8 bit integral RH + 8 bit decimal RH + 8 bit integral T + 8 bit decimal T + 8 bit checksum */

/* Aosong AM2303 (DHT22) sensor timings */
#define DHT22_START_LOW			18	/* (msecs) start signal low time */
#define DHT22_READING_TIMEOUT		10	/* (msecs) max time for reading data from sensor */
#define DHT22_READING_CYCLE		2000	/* (msecs) min time between 2 readings */

/* List of supported chips */
enum dht22_chips { dht22 };

/* Actions the driver may be doing */
enum dht22_state {
	DHT22_READING_NOTHING,
	DHT22_READING,
	DHT22_FAILURE,
};

/**
 * struct dht22_data - device instance specific data
 * @pdata:		platform data (gpio's etc).
 * @read_work:		bh of interrupt handler.
 * @wait_queue:		wait queue for getting values from device.
 * @val_temp:		last temperature value read from device.
 * @val_humid:		last humidity value read from device.
 * @checksum_ok:	last value read from the device passed CRC validation.
 * @checksumming:	flag used to enable the data validation with CRC.
 * @state:		state identifying the action the driver is doing.
 * @measurements_valid:	are the current stored measures valid (start condition).
 * @last_measurement:	time of last measure.
 * @read_lock:		mutex to ensure only one read in progress at a time.
 * @dev:		associate device structure.
 * @hwmon_dev:		device associated with hwmon subsystem.
 * @interrupt_handled:	flag used to indicate a handler has been scheduled.
 */
struct dht22_data {
	struct dht22_platform_data	*pdata;
	wait_queue_head_t		wait_queue;
	s16				val_temp;
	u16				val_humid;
	bool				checksum_ok;
	bool				checksumming;
	enum dht22_state		state;
	bool				measurements_valid;
	unsigned long			last_measurement;
	struct mutex			read_lock;
	struct device			*dev;
	struct device			*hwmon_dev;
	atomic_t			interrupt_handled;
	struct timespec			last_timestamp;
	int				last_signal;
	u64				bitstream_read;
	int				bitcount;
};

/**
 * dht22_transmission_start() - specific sequence for new transmission
 * @data:	device state data
 *
 */
static int dht22_transmission_start(struct dht22_data *data)
{
	int err;

	/* hold data signal low for DHT22_START_LOW ms */
	err = gpio_direction_output(data->pdata->gpio_data, 0);
	if (err)
		return err;
	mdelay(DHT22_START_LOW);

	return 0;
}

/**
 * dht22_measurement() - get a new value from device
 * @data:		device instance specific data
 */
static int dht22_measurement(struct dht22_data *data)
{
	int ret;
	int i;

	long delta_nsec = 0;
	u8 chksum = 0;
	data->val_temp 	= 0;
	data->val_humid = 0;
	data->bitstream_read = 0;
	data->bitcount = 0;

	/* send start signal */
	ret = dht22_transmission_start(data);
	if (ret)
		return ret;

	/* toggle to input */
	ret = gpio_direction_input(data->pdata->gpio_data);
	if (ret)
		return ret;

	/* save timestamp & signal value on measurement start */
	getrawmonotonic(&(data->last_timestamp));
	data->last_signal = gpio_get_value(data->pdata->gpio_data);

	/* enable IRQ */
	atomic_set(&data->interrupt_handled, 0);
	enable_irq(gpio_to_irq(data->pdata->gpio_data));

	ret = wait_event_timeout(data->wait_queue,
			 data->state != DHT22_READING,
			 msecs_to_jiffies(DHT22_READING_TIMEOUT));

	/* disable IRQ */
	disable_irq_nosync(gpio_to_irq(data->pdata->gpio_data));

	/* debug messages */
	dev_dbg(data->dev, "bitstream_read: 0x%016llx interrupts handled: %d\n", data->bitstream_read, atomic_read(&data->interrupt_handled));

	if (ret == 0) { /* timeout occurred */
		dev_err(data->dev, "read timeout\n");
		return -ETIME;
	}
	else if(data->state == DHT22_FAILURE) {
		dev_err(data->dev, "I/O error occured\n");
		return -EIO;
	}

	if(data->state == DHT22_READING_NOTHING /*atomic_read(&data->interrupt_handled) == 84*/) {
		if(data->checksumming) {
			chksum += ((data->bitstream_read & 0x000000FF00000000) >> 32);
			chksum += ((data->bitstream_read & 0x00000000FF000000) >> 24);
			chksum += ((data->bitstream_read & 0x0000000000FF0000) >> 16);
			chksum += ((data->bitstream_read & 0x000000000000FF00) >>  8);
			if(chksum != (data->bitstream_read & 0x00000000000000FF)) {
				dev_err(data->dev, "incorrect checksum\n");
				data->checksum_ok = false;
				return -EAGAIN;
			}
			data->checksum_ok = true;
		}
		data->val_humid = ((data->bitstream_read & 0x000000FFFF000000) >> 24);
		data->val_temp	= ((data->bitstream_read & 0x0000000000FFFF00) >>  8);
		dev_dbg(data->dev, "HUMIDITY: %d TEMPERATURE: %d\n", data->val_humid, data->val_temp);
	}
	
	return 0;
}

/**
 * dht22_update_measurements() - get updated measures from device if too old
 * @data:	device state
 */
static int dht22_update_measurements(struct dht22_data *data)
{
	int ret = 0;

	mutex_lock(&data->read_lock);
	if (time_after(jiffies, data->last_measurement + msecs_to_jiffies(DHT22_READING_CYCLE))
	    || !data->measurements_valid) {
		data->measurements_valid = false;
		data->state = DHT22_READING;
		ret = dht22_measurement(data);
		if (ret)
			goto unlock;
		data->measurements_valid = true;
		data->last_measurement = jiffies;
	}

unlock:
	mutex_unlock(&data->read_lock);
	return ret;
}

/**
 * dht22_show_temp() - show temperature measurement value in sysfs
 * @dev:	device.
 * @attr:	device attribute.
 * @buf:	sysfs buffer where measurement values are written to.
 *
 * Will be called on read access to temp1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t dht22_show_temp(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	int ret;
	struct dht22_data *data = dev_get_drvdata(dev);

	/* Technically no need to read humidity as well */
	ret = dht22_update_measurements(data);

	return ret ? ret : sprintf(buf, "%d\n", data->val_temp);
}

/**
 * dht22_show_humidity() - show humidity measurement value in sysfs
 * @dev:	device.
 * @attr:	device attribute.
 * @buf:	sysfs buffer where measurement values are written to.
 *
 * Will be called on read access to humidity1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t dht22_show_humidity(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int ret;
	struct dht22_data *data = dev_get_drvdata(dev);

	ret = dht22_update_measurements(data);

	return ret ? ret : sprintf(buf, "%d\n", data->val_humid);
}

static ssize_t show_name(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	return sprintf(buf, "%s\n", pdev->name);
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO,
			  dht22_show_temp, NULL, 0);
static SENSOR_DEVICE_ATTR(humidity1_input, S_IRUGO,
			  dht22_show_humidity, NULL, 0);
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);
static struct attribute *dht22_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_humidity1_input.dev_attr.attr,
	&dev_attr_name.attr,
	NULL,
};

static const struct attribute_group dht22_attr_group = {
	.attrs = dht22_attrs,
};

static irqreturn_t dht22_interrupt_fired(int irq, void *d)
{
	struct dht22_data *data = d;
	int signal;
	long delta_nsec;
	int edgeCount;
	int bitValue = -1;
	struct timespec current_timestamp;

	/* First disable the interrupt */
	disable_irq_nosync(irq);
	atomic_inc(&data->interrupt_handled);
	edgeCount = atomic_read(&data->interrupt_handled);

	/* save current timestamp */
	getrawmonotonic(&current_timestamp);

	/* save current signal value */
	signal = gpio_get_value(data->pdata->gpio_data);
	
	/* compare signal to previous signal: they must differ: 0 1 0 1 0 1 0 1 0 1 0 1 ... */
	if(signal == data->last_signal) {
		dev_err(data->dev, "edgeCount: %d FAILURE signal == last_signal\n", edgeCount);
		goto failure;
	}

	/* data transmitting starts after 3rd edge */
	if(edgeCount > 3) {

		delta_nsec = (current_timestamp.tv_sec - data->last_timestamp.tv_sec) * 1000000000;
		delta_nsec = current_timestamp.tv_nsec + delta_nsec - data->last_timestamp.tv_nsec;

		/* FALLING EDGE */
		if(signal == 0) {
			if(delta_nsec >= 16000 && delta_nsec <= 38000) /* 16 - 38 us ==> bit '0' */ 
				bitValue = 0;
			else if(delta_nsec >= 60000 && delta_nsec <= 80000) /* 60 - 80 us ==> bit '1' */ 
				bitValue = 1;
			else {
				dev_err(data->dev, "edgeCount: %d FAILURE wrong bit length\n", edgeCount);
				goto failure;
			}

			data->bitstream_read <<= 1;
			data->bitstream_read |= bitValue;
			++(data->bitcount);
		}

		/* RISING EDGE */
		else if(signal == 1) {
			//if(data->bitcount == 40) { /* final separator */
			//	if(!(delta_nsec >= 45000 && delta_nsec <= 49000)) { /* final separator not in range 45 - 49 us */
			//		dev_err(data->dev, "edgeCount: %d bitcount: %d FAILURE wrong last separator length\n", edgeCount, data->bitcount);
			//		goto failure;
			//	}
			//}
			//else if((data->bitcount & 0x7) == 0 && data->bitcount > 0) { /* byte separator */
			//	if(!(delta_nsec >= 62000 && delta_nsec <= 73000)) { /* byte separator not in range 62 - 73 us */
			//		dev_err(data->dev, "edgeCount: %d bitcount: %d FAILURE wrong byte separator length\n", edgeCount, data->bitcount);
			//		goto failure;
			//	}
			//}
			//else if(!(delta_nsec >= 48000 && delta_nsec <= 56000)) /* separator not in range 48 - 56 us */ {
			//	dev_err(data->dev, "edgeCount: %d bitcount: %d FAILURE wrong separator length\n", edgeCount, data->bitcount);
			//	goto failure;
			//}
			if(!(delta_nsec >= 40000 && delta_nsec <= 70000)) /* separator not in range 40 - 70 us */ {
				dev_err(data->dev, "edgeCount: %d bitcount: %d FAILURE wrong separator length\n", edgeCount, data->bitcount);
				goto failure;
			}
		}
	}
	
	// save last timestamp & signal value
	data->last_timestamp.tv_sec = current_timestamp.tv_sec;
	data->last_timestamp.tv_nsec = current_timestamp.tv_nsec;
	data->last_signal = signal;
	
	/* reading finished */
	if(edgeCount == 84) {
		data->state = DHT22_READING_NOTHING;
		goto wakeup;
	}
	else if(edgeCount > 84) {
		dev_err(data->dev, "edgeCount: %d FAILURE too many edges\n", edgeCount);
		goto failure;
	}
	
	/* state did not change */
	goto handled;

failure:
	/* reading failure*/
	data->state = DHT22_FAILURE;

wakeup:
	wake_up(&data->wait_queue);

handled:
	enable_irq(gpio_to_irq(data->pdata->gpio_data));
	return IRQ_HANDLED;
}

static int dht22_probe(struct platform_device *pdev)
{
	int ret;
	struct dht22_data *data;
	u8 status = 0;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	mutex_init(&data->read_lock);
	data->dev = &pdev->dev;
	init_waitqueue_head(&data->wait_queue);

	if (dev_get_platdata(&pdev->dev) == NULL) {
		dev_err(&pdev->dev, "no platform data supplied\n");
		return -EINVAL;
	}
	data->pdata = dev_get_platdata(&pdev->dev);
	if (data->pdata->checksum)
		data->checksumming = true;

	/* Try requesting the GPIOs */
	ret = devm_gpio_request(&pdev->dev, data->pdata->gpio_data,
				"DHT22 data");
	if (ret) {
		dev_err(&pdev->dev, "data line GPIO request failed\n");
		goto err_exit;
	}

	/* Try requesting IRQ */
	ret = devm_request_irq(&pdev->dev, gpio_to_irq(data->pdata->gpio_data),
			       dht22_interrupt_fired,
			       IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			       "dht22 data",
			       data);
	if (ret) {
		dev_err(&pdev->dev, "failed to get irq for data line\n");
		goto err_exit;
	}
	disable_irq_nosync(gpio_to_irq(data->pdata->gpio_data));

	/* create sysfs hooks */
	ret = sysfs_create_group(&pdev->dev.kobj, &dht22_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "sysfs create failed\n");
		goto err_exit;
	}

	/* register device */
	data->hwmon_dev = hwmon_device_register(data->dev);
	if (IS_ERR(data->hwmon_dev)) {
		ret = PTR_ERR(data->hwmon_dev);
		goto err_release_sysfs_group;
	}

	return 0;

err_release_sysfs_group:
	sysfs_remove_group(&pdev->dev.kobj, &dht22_attr_group);
err_exit:
	return ret;
}

static int dht22_remove(struct platform_device *pdev)
{
	struct dht22_data *data = platform_get_drvdata(pdev);

	/*
	 * Make sure any reads from the device are done and
	 * prevent new ones beginning
	 */
	mutex_lock(&data->read_lock);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&pdev->dev.kobj, &dht22_attr_group);

	mutex_unlock(&data->read_lock);

	return 0;
}

static struct platform_device_id dht22_device_ids[] = {
	{ "dht22", dht22 },
	{ }
};
MODULE_DEVICE_TABLE(platform, dht22_device_ids);

static struct platform_driver dht22_driver = {
	.driver = {
		.name = "dht22",
		.owner = THIS_MODULE,
	},
	.probe = dht22_probe,
	.remove = dht22_remove,
	.id_table = dht22_device_ids,
};
module_platform_driver(dht22_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Aosong(Guangzhou) Electronics DHT22 temperature and humidity sensor driver");
