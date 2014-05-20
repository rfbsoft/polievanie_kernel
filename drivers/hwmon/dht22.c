/*
 * dht22.c - support for the DHT22 Temperature and Humidity Sensor
 *
 * Portions Copyright (c) 2010-2012 Savoir-faire Linux Inc.
 *          Jerome Oufella <jerome.oufella@savoirfairelinux.com>
 *          Vivien Didelot <vivien.didelot@savoirfairelinux.com>
 *
 * Copyright (c) 2009 Jonathan Cameron
 *
 * Copyright (c) 2007 Wouter Horre
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

/* Commands */
#define DHT22_MEASURE_TEMP		0x03
#define DHT22_MEASURE_RH		0x05
#define DHT22_WRITE_STATUS		0x06
#define DHT22_READ_STATUS		0x07
#define DHT22_SOFT_RESET		0x1E

/* Min timings */
#define DHT22_TSCKL			100	/* (nsecs) clock low */
#define DHT22_TSCKH			100	/* (nsecs) clock high */
#define DHT22_TSU			150	/* (nsecs) data setup time */
#define DHT22_TSRST			11	/* (msecs) soft reset time */

/* Aosong AM2303 (DHT22) counts */
#define DHT22_BITCOUNT			40	/* DATA = 8 bit integral RH + 8 bit decimal RH + 8 bit integral T + 8 bit decimal T + 8 bit checksum */

/* Aosong AM2303 (DHT22) sensor timings */
//#define DHT22_START_LOW			500	/* (usecs) start signal low time */
#define DHT22_START_LOW			2	/* (msecs) start signal low time */
#define DHT22_START_WAIT_HIGH		40	/* (usecs) wait for DHT22 response (in high state) time */
#define DHT22_RESPONSE_LOW		80	/* (usecs) sensor send response low signal  time */
#define DHT22_RESPONSE_HIGH		80	/* (usecs) sensor send response high signal  time */
#define DHT22_BITSTART_LOW		50	/* (usecs) start transmit one bit time */
#define DHT22_BIT0_HIGH_MIN		26	/* (usecs) min time bit "0" */
#define DHT22_BIT0_HIGH_MAX		28	/* (usecs) max time bit "0" */
#define DHT22_BIT1_HIGH			70	/* (usecs)     time bit "1" */
#define DHT22_READING_TIMEOUT		6	/* (msecs) max time for reading data from sensor: (DHT22_BITSTART_LOW + DHT22_BIT1_HIGH) * DHT22_BITCOUNT + DHT22_RESPONSE_HIGH + DHT22_RESPONSE_LOW + ... */

/* Status Register Bits */
#define DHT22_STATUS_LOW_RESOLUTION	0x01
#define DHT22_STATUS_NO_OTP_RELOAD	0x02
#define DHT22_STATUS_HEATER		0x04
#define DHT22_STATUS_LOW_BATTERY	0x40

static struct timeval lasttv = {0, 0};
static unsigned int bitcount=0;
static unsigned int bytecount=0;
static unsigned int started=0;			//Indicate if we have started a read or not
static u64 dht22_value = 0;

#if 0
struct timespec {
	__kernel_time_t	tv_sec;			/* seconds */
	long		tv_nsec;		/* nanoseconds */
};
void getrawmonotonic(struct timespec *ts)
#endif
struct sensor_signal {
	int value;
	struct timespec timestamp;
};
static struct sensor_signal dht22_signal[100];

/* List of supported chips */
enum dht22_chips { dht22 };

/* Actions the driver may be doing */
enum dht22_state {
	DHT22_READING_NOTHING,
	DHT22_READING_TEMP,
	DHT22_READING_HUMID,
	/* AM2303 (DHT22) states */
	DHT22_READING,
	DHT22_FAILURE,
};

/**
 * struct dht22_temppair - elements of voltage dependent temp calc
 * @vdd:	supply voltage in microvolts
 * @d1:		see data sheet
 */
struct dht22_temppair {
	int vdd; /* microvolts */
	int d1;
};

/* Table 9 from datasheet - relates temperature calculation to supply voltage */
static const struct dht22_temppair temppoints[] = {
	{ 2500000, -39400 },
	{ 3000000, -39600 },
	{ 3500000, -39700 },
	{ 4000000, -39800 },
	{ 5000000, -40100 },
};

/* Table from CRC datasheet, section 2.4 */
static const u8 dht22_crc8_table[] = {
	0,	49,	98,	83,	196,	245,	166,	151,
	185,	136,	219,	234,	125,	76,	31,	46,
	67,	114,	33,	16,	135,	182,	229,	212,
	250,	203,	152,	169,	62,	15,	92,	109,
	134,	183,	228,	213,	66,	115,	32,	17,
	63,	14,	93,	108,	251,	202,	153,	168,
	197,	244,	167,	150,	1,	48,	99,	82,
	124,	77,	30,	47,	184,	137,	218,	235,
	61,	12,	95,	110,	249,	200,	155,	170,
	132,	181,	230,	215,	64,	113,	34,	19,
	126,	79,	28,	45,	186,	139,	216,	233,
	199,	246,	165,	148,	3,	50,	97,	80,
	187,	138,	217,	232,	127,	78,	29,	44,
	2,	51,	96,	81,	198,	247,	164,	149,
	248,	201,	154,	171,	60,	13,	94,	111,
	65,	112,	35,	18,	133,	180,	231,	214,
	122,	75,	24,	41,	190,	143,	220,	237,
	195,	242,	161,	144,	7,	54,	101,	84,
	57,	8,	91,	106,	253,	204,	159,	174,
	128,	177,	226,	211,	68,	117,	38,	23,
	252,	205,	158,	175,	56,	9,	90,	107,
	69,	116,	39,	22,	129,	176,	227,	210,
	191,	142,	221,	236,	123,	74,	25,	40,
	6,	55,	100,	85,	194,	243,	160,	145,
	71,	118,	37,	20,	131,	178,	225,	208,
	254,	207,	156,	173,	58,	11,	88,	105,
	4,	53,	102,	87,	192,	241,	162,	147,
	189,	140,	223,	238,	121,	72,	27,	42,
	193,	240,	163,	146,	5,	52,	103,	86,
	120,	73,	26,	43,	188,	141,	222,	239,
	130,	179,	224,	209,	70,	119,	36,	21,
	59,	10,	89,	104,	255,	206,	157,	172
};

/**
 * struct dht22_data - device instance specific data
 * @pdata:		platform data (gpio's etc).
 * @read_work:		bh of interrupt handler.
 * @wait_queue:		wait queue for getting values from device.
 * @val_temp:		last temperature value read from device.
 * @val_humid:		last humidity value read from device.
 * @val_status:		last status register value read from device.
 * @checksum_ok:	last value read from the device passed CRC validation.
 * @checksumming:	flag used to enable the data validation with CRC.
 * @state:		state identifying the action the driver is doing.
 * @measurements_valid:	are the current stored measures valid (start condition).
 * @status_valid:	is the current stored status valid (start condition).
 * @last_measurement:	time of last measure.
 * @last_status:	time of last status reading.
 * @read_lock:		mutex to ensure only one read in progress at a time.
 * @dev:		associate device structure.
 * @hwmon_dev:		device associated with hwmon subsystem.
 * @reg:		associated regulator (if specified).
 * @nb:			notifier block to handle notifications of voltage
 *                      changes.
 * @supply_uv:		local copy of supply voltage used to allow use of
 *                      regulator consumer if available.
 * @supply_uv_valid:	indicates that an updated value has not yet been
 *			obtained from the regulator and so any calculations
 *			based upon it will be invalid.
 * @update_supply_work:	work struct that is used to update the supply_uv.
 * @interrupt_handled:	flag used to indicate a handler has been scheduled.
 */
struct dht22_data {
	struct dht22_platform_data	*pdata;
	struct work_struct		read_work;
	wait_queue_head_t		wait_queue;
	uint16_t			val_temp;
	uint16_t			val_humid;
	u8				val_status;
	bool				checksum_ok;
	bool				checksumming;
	enum dht22_state		state;
	bool				measurements_valid;
	bool				status_valid;
	unsigned long			last_measurement;
	unsigned long			last_status;
	struct mutex			read_lock;
	struct device			*dev;
	struct device			*hwmon_dev;
	struct regulator		*reg;
	struct notifier_block		nb;
	int				supply_uv;
	bool				supply_uv_valid;
	struct work_struct		update_supply_work;
	atomic_t			interrupt_handled;
	struct timespec			last_timestamp;
	int				last_signal;
	u64				bitstream_read;
	int				bitcount;
};

/**
 * dht22_reverse() - reverse a byte
 * @byte:    byte to reverse.
 */
static u8 dht22_reverse(u8 byte)
{
	u8 i, c;

	for (c = 0, i = 0; i < 8; i++)
		c |= (!!(byte & (1 << i))) << (7 - i);
	return c;
}

/**
 * dht22_crc8() - compute crc8
 * @data:	dht22 specific data.
 * @value:	dht22 retrieved data.
 *
 * This implements section 2 of the CRC datasheet.
 */
static u8 dht22_crc8(struct dht22_data *data,
		const u8 *value,
		int len)
{
	u8 crc = dht22_reverse(data->val_status & 0x0F);

	while (len--) {
		crc = dht22_crc8_table[*value ^ crc];
		value++;
	}

	return crc;
}

/**
 * dht22_connection_reset() - reset the comms interface
 * @data:	dht22 specific data
 *
 * This implements section 3.4 of the data sheet
 */
static int dht22_connection_reset(struct dht22_data *data)
{
	int i, err;

	err = gpio_direction_output(data->pdata->gpio_data, 1);
	if (err)
		return err;
	ndelay(DHT22_TSCKL);
	gpio_set_value(data->pdata->gpio_sck, 0);
	ndelay(DHT22_TSCKL);
	for (i = 0; i < 9; ++i) {
		gpio_set_value(data->pdata->gpio_sck, 1);
		ndelay(DHT22_TSCKH);
		gpio_set_value(data->pdata->gpio_sck, 0);
		ndelay(DHT22_TSCKL);
	}
	return 0;
}

/**
 * dht22_send_bit() - send an individual bit to the device
 * @data:	device state data
 * @val:	value of bit to be sent
 */
static inline void dht22_send_bit(struct dht22_data *data, int val)
{
	gpio_set_value(data->pdata->gpio_data, val);
	ndelay(DHT22_TSU);
	gpio_set_value(data->pdata->gpio_sck, 1);
	ndelay(DHT22_TSCKH);
	gpio_set_value(data->pdata->gpio_sck, 0);
	ndelay(DHT22_TSCKL); /* clock low time */
}

/**
 * dht22_transmission_start() - specific sequence for new transmission
 * @data:	device state data
 *
 */
static int dht22_transmission_start(struct dht22_data *data)
{
	int err;

	/* hold data signal low for DHT22_START_LOW us */
	err = gpio_direction_output(data->pdata->gpio_data, 0);
	if (err)
		return err;
	//udelay(DHT22_START_LOW);
	mdelay(DHT22_START_LOW);

	getrawmonotonic(&(dht22_signal[0].timestamp));

	return 0;
}

/**
 * dht22_send_byte() - send a single byte to the device
 * @data:	device state
 * @byte:	value to be sent
 */
static void dht22_send_byte(struct dht22_data *data, u8 byte)
{
	int i;

	for (i = 0; i < 8; i++) {
		dht22_send_bit(data, !!(byte & 0x80));
		byte <<= 1;
	}
}

/**
 * dht22_wait_for_response() - checks for ack from device
 * @data:	device state
 */
static int dht22_wait_for_response(struct dht22_data *data)
{
	int err;

	err = gpio_direction_input(data->pdata->gpio_data);
	if (err)
		return err;
	gpio_set_value(data->pdata->gpio_sck, 1);
	ndelay(DHT22_TSCKH);
	if (gpio_get_value(data->pdata->gpio_data)) {
		gpio_set_value(data->pdata->gpio_sck, 0);
		dev_err(data->dev, "Command not acknowledged\n");
		err = dht22_connection_reset(data);
		if (err)
			return err;
		return -EIO;
	}
	gpio_set_value(data->pdata->gpio_sck, 0);
	ndelay(DHT22_TSCKL);
	return 0;
}

/**
 * dht22_send_cmd() - Sends a command to the device.
 * @data:	device state
 * @cmd:	command byte to be sent
 *
 * On entry, sck is output low, data is output pull high
 * and the interrupt disabled.
 */
static int dht22_send_cmd(struct dht22_data *data, u8 cmd)
{
	int err;

	err = dht22_transmission_start(data);
	if (err)
		return err;
	dht22_send_byte(data, cmd);
	return dht22_wait_for_response(data);
}

/**
 * dht22_soft_reset() - send a soft reset command
 * @data:	dht22 specific data.
 *
 * As described in section 3.2 of the datasheet.
 */
static int dht22_soft_reset(struct dht22_data *data)
{
	int ret;

	ret = dht22_send_cmd(data, DHT22_SOFT_RESET);
	if (ret)
		return ret;
	msleep(DHT22_TSRST);
	/* device resets default hardware status register value */
	data->val_status = 0;

	return ret;
}

/**
 * dht22_ack() - send a ack
 * @data:	dht22 specific data.
 *
 * Each byte of data is acknowledged by pulling the data line
 * low for one clock pulse.
 */
static int dht22_ack(struct dht22_data *data)
{
	int err;

	err = gpio_direction_output(data->pdata->gpio_data, 0);
	if (err)
		return err;
	ndelay(DHT22_TSU);
	gpio_set_value(data->pdata->gpio_sck, 1);
	ndelay(DHT22_TSU);
	gpio_set_value(data->pdata->gpio_sck, 0);
	ndelay(DHT22_TSU);
	gpio_set_value(data->pdata->gpio_data, 1);

	return gpio_direction_input(data->pdata->gpio_data);
}

/**
 * dht22_end_transmission() - notify device of end of transmission
 * @data:	device state.
 *
 * This is basically a NAK (single clock pulse, data high).
 */
static int dht22_end_transmission(struct dht22_data *data)
{
	int err;

	err = gpio_direction_output(data->pdata->gpio_data, 1);
	if (err)
		return err;
	ndelay(DHT22_TSU);
	gpio_set_value(data->pdata->gpio_sck, 1);
	ndelay(DHT22_TSCKH);
	gpio_set_value(data->pdata->gpio_sck, 0);
	ndelay(DHT22_TSCKL);
	return 0;
}

/**
 * dht22_read_byte() - Read a byte back from the device
 * @data:	device state.
 */
static u8 dht22_read_byte(struct dht22_data *data)
{
	int i;
	u8 byte = 0;

	for (i = 0; i < 8; ++i) {
		byte <<= 1;
		gpio_set_value(data->pdata->gpio_sck, 1);
		ndelay(DHT22_TSCKH);
		byte |= !!gpio_get_value(data->pdata->gpio_data);
		gpio_set_value(data->pdata->gpio_sck, 0);
		ndelay(DHT22_TSCKL);
	}
	return byte;
}

/**
 * dht22_send_status() - write the status register byte
 * @data:	dht22 specific data.
 * @status:	the byte to set the status register with.
 *
 * As described in figure 14 and table 5 of the datasheet.
 */
static int dht22_send_status(struct dht22_data *data, u8 status)
{
	int err;

	err = dht22_send_cmd(data, DHT22_WRITE_STATUS);
	if (err)
		return err;
	err = gpio_direction_output(data->pdata->gpio_data, 1);
	if (err)
		return err;
	ndelay(DHT22_TSU);
	dht22_send_byte(data, status);
	err = dht22_wait_for_response(data);
	if (err)
		return err;

	data->val_status = status;
	return 0;
}

/**
 * dht22_update_status() - get updated status register from device if too old
 * @data:	device instance specific data.
 *
 * As described in figure 15 and table 5 of the datasheet.
 */
static int dht22_update_status(struct dht22_data *data)
{
	int ret = 0;
	u8 status;
	u8 previous_config;
	u8 dev_checksum = 0;
	u8 checksum_vals[2];
	int timeout = HZ;

	mutex_lock(&data->read_lock);
	if (time_after(jiffies, data->last_status + timeout)
			|| !data->status_valid) {
		ret = dht22_send_cmd(data, DHT22_READ_STATUS);
		if (ret)
			goto unlock;
		status = dht22_read_byte(data);

		if (data->checksumming) {
			dht22_ack(data);
			dev_checksum = dht22_reverse(dht22_read_byte(data));
			checksum_vals[0] = DHT22_READ_STATUS;
			checksum_vals[1] = status;
			data->checksum_ok = (dht22_crc8(data, checksum_vals, 2)
					== dev_checksum);
		}

		ret = dht22_end_transmission(data);
		if (ret)
			goto unlock;

		/*
		 * Perform checksum validation on the received data.
		 * Specification mentions that in case a checksum verification
		 * fails, a soft reset command must be sent to the device.
		 */
		if (data->checksumming && !data->checksum_ok) {
			previous_config = data->val_status & 0x07;
			ret = dht22_soft_reset(data);
			if (ret)
				goto unlock;
			if (previous_config) {
				ret = dht22_send_status(data, previous_config);
				if (ret) {
					dev_err(data->dev,
						"CRC validation failed, unable "
						"to restore device settings\n");
					goto unlock;
				}
			}
			ret = -EAGAIN;
			goto unlock;
		}

		data->val_status = status;
		data->status_valid = true;
		data->last_status = jiffies;
	}

unlock:
	mutex_unlock(&data->read_lock);
	return ret;
}

/**
 * dht22_measurement() - get a new value from device
 * @data:		device instance specific data
 * @command:		command sent to request value
 * @timeout_msecs:	timeout after which comms are assumed
 *			to have failed are reset.
 */
static int dht22_measurement(struct dht22_data *data,
			     int command,
			     int timeout_msecs)
{
	int ret;
	u8 previous_config;

	ret = dht22_send_cmd(data, command);
	if (ret)
		return ret;

	ret = gpio_direction_input(data->pdata->gpio_data);
	if (ret)
		return ret;
	atomic_set(&data->interrupt_handled, 0);

	enable_irq(gpio_to_irq(data->pdata->gpio_data));
	if (gpio_get_value(data->pdata->gpio_data) == 0) {
		disable_irq_nosync(gpio_to_irq(data->pdata->gpio_data));
		/* Only relevant if the interrupt hasn't occurred. */
		if (!atomic_read(&data->interrupt_handled))
			schedule_work(&data->read_work);
	}
	ret = wait_event_timeout(data->wait_queue,
				 (data->state == DHT22_READING_NOTHING),
				 msecs_to_jiffies(timeout_msecs));
	if (data->state != DHT22_READING_NOTHING) { /* I/O error occurred */
		data->state = DHT22_READING_NOTHING;
		return -EIO;
	} else if (ret == 0) { /* timeout occurred */
		disable_irq_nosync(gpio_to_irq(data->pdata->gpio_data));
		ret = dht22_connection_reset(data);
		if (ret)
			return ret;
		return -ETIME;
	}

	/*
	 *  Perform checksum validation on the received data.
	 *  Specification mentions that in case a checksum verification fails,
	 *  a soft reset command must be sent to the device.
	 */
	if (data->checksumming && !data->checksum_ok) {
		previous_config = data->val_status & 0x07;
		ret = dht22_soft_reset(data);
		if (ret)
			return ret;
		if (previous_config) {
			ret = dht22_send_status(data, previous_config);
			if (ret) {
				dev_err(data->dev,
					"CRC validation failed, unable "
					"to restore device settings\n");
				return ret;
			}
		}
		return -EAGAIN;
	}

	return 0;
}

/**
 * am2303_measurement() - get a new value from device
 * @data:		device instance specific data
 */
static int am2303_measurement(struct dht22_data *data)
{
	int ret;
	int i;

	long delta_nsec = 0;
	u8 chksum = 0;
	u16 humid_bits = 0;
	s16 temp_bits = 0;
	data->val_temp 	= 0;
	data->val_humid = 0;
	data->bitstream_read = 0;
	data->bitcount = 0;

	/* TODO: the time interval between 2 measurements must be at least 1.7 sec !!! */

	for(i = 0; i < 100; ++i) {dht22_signal[i].timestamp.tv_sec = 0; dht22_signal[i].timestamp.tv_nsec = 0; dht22_signal[i].value = 0;}

	//do_gettimeofday(&lasttv);
	//bitcount = 0;
	//bytecount = 0;
	//started = 0;
	//dht22_value = 0;
	//mdelay(DHT22_READING_TIMEOUT);
	//disable_irq_nosync(gpio_to_irq(data->pdata->gpio_data));

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
			 msecs_to_jiffies(10/*DHT22_READING_TIMEOUT*/));

	/* disable IRQ */
	disable_irq_nosync(gpio_to_irq(data->pdata->gpio_data));

	/* debug messages */
	printk("--- dht22_value: 0x%016llx interrupts handled: %d\n", data->bitstream_read, atomic_read(&data->interrupt_handled));
	for(i = 1; i <= atomic_read(&data->interrupt_handled); ++i) {
		delta_nsec = (dht22_signal[i].timestamp.tv_sec - dht22_signal[i-1].timestamp.tv_sec) * 1000000000;
		delta_nsec = dht22_signal[i].timestamp.tv_nsec + delta_nsec - dht22_signal[i-1].timestamp.tv_nsec;
		printk("%2d %2d %d\n", i, delta_nsec / 1000, dht22_signal[i].value);
	}

	if (ret == 0) { /* timeout occurred */
		printk("--- dht22_value: read timeout\n");
		return -ETIME;
	}
	else if(data->state == DHT22_FAILURE) {
		printk("--- dht22_value: I/O error occured\n");
		return -EIO;
	}

	if(data->state == DHT22_READING_NOTHING /*atomic_read(&data->interrupt_handled) == 84*/) {
		chksum += ((data->bitstream_read & 0x000000FF00000000) >> 32);
		chksum += ((data->bitstream_read & 0x00000000FF000000) >> 24);
		chksum += ((data->bitstream_read & 0x0000000000FF0000) >> 16);
		chksum += ((data->bitstream_read & 0x000000000000FF00) >>  8);
		if(chksum == (data->bitstream_read & 0x00000000000000FF)) {
			humid_bits = ((data->bitstream_read & 0x000000FFFF000000) >> 24);
			temp_bits  = ((data->bitstream_read & 0x0000000000FFFF00) >>  8);
			printk("HUMIDITY: %d TEMPERATURE: %d\n", humid_bits, temp_bits);
		}
		else {
			printk("--- dht22_value: incorrect checksum\n");
			return -EAGAIN;
		}
	}
	
	return 0;
#if 0
	int ret;
	u8 previous_config;

	ret = dht22_send_cmd(data, command);
	if (ret)
		return ret;

	ret = gpio_direction_input(data->pdata->gpio_data);
	if (ret)
		return ret;
	atomic_set(&data->interrupt_handled, 0);

	enable_irq(gpio_to_irq(data->pdata->gpio_data));
	if (gpio_get_value(data->pdata->gpio_data) == 0) {
		disable_irq_nosync(gpio_to_irq(data->pdata->gpio_data));
		/* Only relevant if the interrupt hasn't occurred. */
		if (!atomic_read(&data->interrupt_handled))
			schedule_work(&data->read_work);
	}
	ret = wait_event_timeout(data->wait_queue,
				 (data->state == DHT22_READING_NOTHING),
				 msecs_to_jiffies(timeout_msecs));
	if (data->state != DHT22_READING_NOTHING) { /* I/O error occurred */
		data->state = DHT22_READING_NOTHING;
		return -EIO;
	} else if (ret == 0) { /* timeout occurred */
		disable_irq_nosync(gpio_to_irq(data->pdata->gpio_data));
		ret = dht22_connection_reset(data);
		if (ret)
			return ret;
		return -ETIME;
	}

	/*
	 *  Perform checksum validation on the received data.
	 *  Specification mentions that in case a checksum verification fails,
	 *  a soft reset command must be sent to the device.
	 */
	if (data->checksumming && !data->checksum_ok) {
		previous_config = data->val_status & 0x07;
		ret = dht22_soft_reset(data);
		if (ret)
			return ret;
		if (previous_config) {
			ret = dht22_send_status(data, previous_config);
			if (ret) {
				dev_err(data->dev,
					"CRC validation failed, unable "
					"to restore device settings\n");
				return ret;
			}
		}
		return -EAGAIN;
	}
#endif
}

/**
 * dht22_update_measurements() - get updated measures from device if too old
 * @data:	device state
 */
static int dht22_update_measurements(struct dht22_data *data)
{
	int ret = 0;
	int timeout = HZ;

	mutex_lock(&data->read_lock);
	if (time_after(jiffies, data->last_measurement + timeout)
	    || !data->measurements_valid) {

		data->state = DHT22_READING;
		ret = am2303_measurement(data);
		if (ret)
			goto unlock;

#if 0
		data->state = DHT22_READING_HUMID;
		ret = dht22_measurement(data, DHT22_MEASURE_RH, 160);
		if (ret)
			goto unlock;
		data->state = DHT22_READING_TEMP;
		ret = dht22_measurement(data, DHT22_MEASURE_TEMP, 400);
		if (ret)
			goto unlock;
		data->measurements_valid = true;
		data->last_measurement = jiffies;
#endif
	}

unlock:
	mutex_unlock(&data->read_lock);
	return ret;
}

/**
 * dht22_calc_temp() - convert the raw reading to a temperature
 * @data:	device state
 *
 * As per section 4.3 of the data sheet.
 */
static inline int dht22_calc_temp(struct dht22_data *data)
{
	int d1 = temppoints[0].d1;
	int d2 = (data->val_status & DHT22_STATUS_LOW_RESOLUTION) ? 40 : 10;
	int i;

	for (i = ARRAY_SIZE(temppoints) - 1; i > 0; i--)
		/* Find pointer to interpolate */
		if (data->supply_uv > temppoints[i - 1].vdd) {
			d1 = (data->supply_uv - temppoints[i - 1].vdd)
				* (temppoints[i].d1 - temppoints[i - 1].d1)
				/ (temppoints[i].vdd - temppoints[i - 1].vdd)
				+ temppoints[i - 1].d1;
			break;
		}

	return data->val_temp * d2 + d1;
}

/**
 * dht22_calc_humid() - using last temperature convert raw to humid
 * @data:	device state
 *
 * This is the temperature compensated version as per section 4.2 of
 * the data sheet.
 *
 * The sensor is assumed to be V3, which is compatible with V4.
 * Humidity conversion coefficients are shown in table 7 of the datasheet.
 */
static inline int dht22_calc_humid(struct dht22_data *data)
{
	int rh_linear; /* milli percent */
	int temp = dht22_calc_temp(data);
	int c2, c3;
	int t2;
	const int c1 = -4;

	if (data->val_status & DHT22_STATUS_LOW_RESOLUTION) {
		c2 = 648000; /* x 10 ^ -6 */
		c3 = -7200;  /* x 10 ^ -7 */
		t2 = 1280;
	} else {
		c2 = 40500;  /* x 10 ^ -6 */
		c3 = -28;    /* x 10 ^ -7 */
		t2 = 80;
	}

	rh_linear = c1 * 1000
		+ c2 * data->val_humid / 1000
		+ (data->val_humid * data->val_humid * c3) / 10000;
	return (temp - 25000) * (10000 + t2 * data->val_humid)
		/ 1000000 + rh_linear;
}

/**
 * dht22_show_status() - show status information in sysfs
 * @dev:	device.
 * @attr:	device attribute.
 * @buf:	sysfs buffer where information is written to.
 *
 * Will be called on read access to temp1_fault, humidity1_fault
 * and heater_enable sysfs attributes.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t dht22_show_status(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	int ret;
	struct dht22_data *data = dev_get_drvdata(dev);
	u8 bit = to_sensor_dev_attr(attr)->index;

	ret = dht22_update_status(data);

	return ret ? ret : sprintf(buf, "%d\n", !!(data->val_status & bit));
}

/**
 * dht22_store_heater() - change heater state via sysfs
 * @dev:	device.
 * @attr:	device attribute.
 * @buf:	sysfs buffer to read the new heater state from.
 * @count:	length of the data.
 *
 * Will be called on write access to heater_enable sysfs attribute.
 * Returns number of bytes actually decoded, negative errno on error.
 */
static ssize_t dht22_store_heater(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int ret;
	struct dht22_data *data = dev_get_drvdata(dev);
	long value;
	u8 status;

	if (kstrtol(buf, 10, &value))
		return -EINVAL;

	mutex_lock(&data->read_lock);
	status = data->val_status & 0x07;
	if (!!value)
		status |= DHT22_STATUS_HEATER;
	else
		status &= ~DHT22_STATUS_HEATER;

	ret = dht22_send_status(data, status);
	mutex_unlock(&data->read_lock);

	return ret ? ret : count;
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

	return ret ? ret : sprintf(buf, "%d\n",
				   dht22_calc_temp(data));
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

	return ret ? ret : sprintf(buf, "%d\n", dht22_calc_humid(data));
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
static SENSOR_DEVICE_ATTR(temp1_fault, S_IRUGO, dht22_show_status, NULL,
			  DHT22_STATUS_LOW_BATTERY);
static SENSOR_DEVICE_ATTR(humidity1_fault, S_IRUGO, dht22_show_status, NULL,
			  DHT22_STATUS_LOW_BATTERY);
static SENSOR_DEVICE_ATTR(heater_enable, S_IRUGO | S_IWUSR, dht22_show_status,
			  dht22_store_heater, DHT22_STATUS_HEATER);
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);
static struct attribute *dht22_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_humidity1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_fault.dev_attr.attr,
	&sensor_dev_attr_humidity1_fault.dev_attr.attr,
	&sensor_dev_attr_heater_enable.dev_attr.attr,
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
	getrawmonotonic(&(dht22_signal[edgeCount].timestamp));
	getrawmonotonic(&current_timestamp);

	/* save current signal value */
	signal = gpio_get_value(data->pdata->gpio_data);
	dht22_signal[edgeCount].value = signal;
	
	/* compare signal to previous signal: they must differ: 0 1 0 1 0 1 0 1 0 1 0 1 ... */
	if(signal == data->last_signal) {
		printk("edgeCount: %d FAILURE signal == last_signal\n", edgeCount);
		goto failure;
	}

	/* data transmitting starts after 3rd edge */
	if(edgeCount > 3) {
		//delta_nsec = (dht22_signal[edgeCount].timestamp.tv_sec - dht22_signal[edgeCount-1].timestamp.tv_sec) * 1000000000;
		//delta_nsec = dht22_signal[edgeCount].timestamp.tv_nsec + delta_nsec - dht22_signal[edgeCount-1].timestamp.tv_nsec;
		delta_nsec = (current_timestamp.tv_sec - data->last_timestamp.tv_sec) * 1000000000;
		delta_nsec = current_timestamp.tv_nsec + delta_nsec - data->last_timestamp.tv_nsec;

		/* FALLING EDGE */
		if(signal == 0) {
			if(delta_nsec >= 25000 && delta_nsec <= 28000) /* 25 - 28 us ==> bit '0' */ 
				bitValue = 0;
			else if(delta_nsec >= 67000 && delta_nsec <= 75000) /* 67 - 75 us ==> bit '1' */ 
				bitValue = 1;
			else {
				printk("edgeCount: %d FAILURE wrong bit length\n", edgeCount);
				goto failure;
			}

			data->bitstream_read <<= 1;
			data->bitstream_read |= bitValue;
			++(data->bitcount);
		}

		/* RISING EDGE */
		else if(signal == 1) {
			if(data->bitcount == 40) { /* final separator */
				if(!(delta_nsec >= 45000 && delta_nsec <= 49000)) { /* final separator not in range 45 - 49 us */
					printk("edgeCount: %d bitcount: %d FAILURE wrong last separator length\n", edgeCount, data->bitcount);
					goto failure;
				}
			}
			else if((data->bitcount & 0x7) == 0 && data->bitcount > 0) { /* byte separator */
				if(!(delta_nsec >= 62000 && delta_nsec <= 73000)) { /* byte separator not in range 62 - 73 us */
					printk("edgeCount: %d bitcount: %d FAILURE wrong byte separator length\n", edgeCount, data->bitcount);
					goto failure;
				}
			}
			else if(!(delta_nsec >= 48000 && delta_nsec <= 56000)) /* separator not in range 48 - 56 us */ {
				printk("edgeCount: %d bitcount: %d FAILURE wrong separator length\n", edgeCount, data->bitcount);
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
		printk("edgeCount: %d FAILURE too many edges\n", edgeCount);
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

static void am2303_bh_read_data(struct work_struct *work_s) {
	uint16_t val = 0;
	u8 dev_checksum = 0;
	u8 checksum_vals[3];
	
	struct timeval tv;
	long deltv;
	int delta_usecs = 0;
	int signal;

	struct dht22_data *data
		= container_of(work_s, struct dht22_data,
			       read_work);
	
	enable_irq(gpio_to_irq(data->pdata->gpio_data));
	return;

	//printk("--- dht22 gpio: %d\n", gpio_get_value(data->pdata->gpio_data));

	//udelay(5);
	//signal = gpio_get_value(data->pdata->gpio_data);
	signal = gpio_get_value(4);

	// get current time 
	do_gettimeofday(&tv);

	//getrawmonotonic(&(dht22_signal[atomic_read(&data->interrupt_handled)].timestamp));
	dht22_signal[atomic_read(&data->interrupt_handled)].value = signal;

	// get time since last interrupt in microseconds 
	deltv = tv.tv_sec-lasttv.tv_sec;
	delta_usecs = (int) (deltv*1000000 + (tv.tv_usec - lasttv.tv_usec));
	lasttv = tv;	//Save last interrupt time

	//if(delta_usecs > 67 && delta_usecs < 87)
	//if(atomic_read(&data->interrupt_handled) == 2)
	//	printk("---delta_usecs: %d\n", delta_usecs);

	if(started) {
		dht22_value <<= 1;
		if(delta_usecs > 90)
			dht22_value |= 1;
		// TODO
#if 0
			{
			if(data > 80)
				return IRQ_HANDLED;										//Start/spurious? signal
			if(data < 15)
				return IRQ_HANDLED;										//Spurious signal?
			if (data > 60)//55 
				dht[bytecount] = dht[bytecount] | (0x80 >> bitcount);	//Add a 1 to the data byte
			
			//Uncomment to log bits and durations - may affect performance and not be accurate!
			//printk("B:%d, d:%d, dt:%d\n", bytecount, bitcount, data);
			bitcount++;
			if(bitcount == 8)
				{
				bitcount = 0;
				bytecount++;
				}
			//if(bytecount == 5)
			//	printk(KERN_INFO DHT11_DRIVER_NAME "Result: %d, %d, %d, %d, %d\n", dht[0], dht[1], dht[2], dht[3], dht[4]);
			}
#endif
	}
	else {
		if(atomic_read(&data->interrupt_handled) == 2)
			started = 1;
	}	

	enable_irq(gpio_to_irq(data->pdata->gpio_data));
}

static void dht22_bh_read_data(struct work_struct *work_s)
{
	uint16_t val = 0;
	u8 dev_checksum = 0;
	u8 checksum_vals[3];
	struct dht22_data *data
		= container_of(work_s, struct dht22_data,
			       read_work);

	/* Firstly, verify the line is low */
	if (gpio_get_value(data->pdata->gpio_data)) {
		/*
		 * If not, then start the interrupt again - care here as could
		 * have gone low in meantime so verify it hasn't!
		 */
		atomic_set(&data->interrupt_handled, 0);
		enable_irq(gpio_to_irq(data->pdata->gpio_data));
		/* If still not occurred or another handler was scheduled */
		if (gpio_get_value(data->pdata->gpio_data)
		    || atomic_read(&data->interrupt_handled))
			return;
	}

	/* Read the data back from the device */
	val = dht22_read_byte(data);
	val <<= 8;
	if (dht22_ack(data))
		goto wakeup;
	val |= dht22_read_byte(data);

	if (data->checksumming) {
		/*
		 * Ask the device for a checksum and read it back.
		 * Note: the device sends the checksum byte reversed.
		 */
		if (dht22_ack(data))
			goto wakeup;
		dev_checksum = dht22_reverse(dht22_read_byte(data));
		checksum_vals[0] = (data->state == DHT22_READING_TEMP) ?
			DHT22_MEASURE_TEMP : DHT22_MEASURE_RH;
		checksum_vals[1] = (u8) (val >> 8);
		checksum_vals[2] = (u8) val;
		data->checksum_ok
			= (dht22_crc8(data, checksum_vals, 3) == dev_checksum);
	}

	/* Tell the device we are done */
	if (dht22_end_transmission(data))
		goto wakeup;

	switch (data->state) {
	case DHT22_READING_TEMP:
		data->val_temp = val;
		break;
	case DHT22_READING_HUMID:
		data->val_humid = val;
		break;
	default:
		break;
	}

	data->state = DHT22_READING_NOTHING;
wakeup:
	wake_up(&data->wait_queue);
}

static void dht22_update_voltage(struct work_struct *work_s)
{
	struct dht22_data *data
		= container_of(work_s, struct dht22_data,
			       update_supply_work);
	data->supply_uv = regulator_get_voltage(data->reg);
}

/**
 * dht22_invalidate_voltage() - mark supply voltage invalid when notified by reg
 * @nb:		associated notification structure
 * @event:	voltage regulator state change event code
 * @ignored:	function parameter - ignored here
 *
 * Note that as the notification code holds the regulator lock, we have
 * to schedule an update of the supply voltage rather than getting it directly.
 */
static int dht22_invalidate_voltage(struct notifier_block *nb,
				    unsigned long event,
				    void *ignored)
{
	struct dht22_data *data = container_of(nb, struct dht22_data, nb);

	if (event == REGULATOR_EVENT_VOLTAGE_CHANGE)
		data->supply_uv_valid = false;
	schedule_work(&data->update_supply_work);

	return NOTIFY_OK;
}

static int dht22_probe(struct platform_device *pdev)
{
	int ret;
	struct dht22_data *data;
	u8 status = 0;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	//INIT_WORK(&data->read_work, dht22_bh_read_data);
	INIT_WORK(&data->read_work, am2303_bh_read_data);
	INIT_WORK(&data->update_supply_work, dht22_update_voltage);
	platform_set_drvdata(pdev, data);
	mutex_init(&data->read_lock);
	data->dev = &pdev->dev;
	init_waitqueue_head(&data->wait_queue);

	if (dev_get_platdata(&pdev->dev) == NULL) {
		dev_err(&pdev->dev, "no platform data supplied\n");
		return -EINVAL;
	}
	data->pdata = dev_get_platdata(&pdev->dev);
	data->supply_uv = data->pdata->supply_mv * 1000;
	if (data->pdata->checksum)
		data->checksumming = true;
	if (data->pdata->no_otp_reload)
		status |= DHT22_STATUS_NO_OTP_RELOAD;
	if (data->pdata->low_resolution)
		status |= DHT22_STATUS_LOW_RESOLUTION;

	printk("--- %s platform data = gpio_data: %d checksum: %d\n", __func__, data->pdata->gpio_data, data->pdata->checksum);

	/*
	 * If a regulator is available,
	 * query what the supply voltage actually is!
	 */
#if 0
	data->reg = devm_regulator_get_optional(data->dev, "vcc");
	if (!IS_ERR(data->reg)) {
		int voltage;

		voltage = regulator_get_voltage(data->reg);
		if (voltage)
			data->supply_uv = voltage;

		ret = regulator_enable(data->reg);
		if (ret != 0) {
			dev_err(&pdev->dev,
				"failed to enable regulator: %d\n", ret);
			return ret;
		}

		/*
		 * Setup a notifier block to update this if another device
		 * causes the voltage to change
		 */
		data->nb.notifier_call = &dht22_invalidate_voltage;
		ret = regulator_register_notifier(data->reg, &data->nb);
		if (ret) {
			dev_err(&pdev->dev,
				"regulator notifier request failed\n");
			regulator_disable(data->reg);
			return ret;
		}
	}
#endif

	/* Try requesting the GPIOs */
#if 0
	ret = devm_gpio_request_one(&pdev->dev, data->pdata->gpio_sck,
			GPIOF_OUT_INIT_LOW, "DHT22 sck");
	if (ret) {
		dev_err(&pdev->dev, "clock line GPIO request failed\n");
		goto err_release_reg;
	}
#endif

	ret = devm_gpio_request(&pdev->dev, data->pdata->gpio_data,
				"DHT22 data");
	if (ret) {
		dev_err(&pdev->dev, "data line GPIO request failed\n");
		goto err_release_reg;
	}

	/* Try requesting IRQ */
	ret = devm_request_irq(&pdev->dev, gpio_to_irq(data->pdata->gpio_data),
			       dht22_interrupt_fired,
			       IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			       "dht22 data",
			       data);
	if (ret) {
		dev_err(&pdev->dev, "failed to get irq for data line\n");
		goto err_release_reg;
	}
	disable_irq_nosync(gpio_to_irq(data->pdata->gpio_data));

#if 0
	ret = dht22_connection_reset(data);
	if (ret)
		goto err_release_reg;

	ret = dht22_soft_reset(data);
	if (ret)
		goto err_release_reg;

	/* write status with platform data options */
	if (status) {
		ret = dht22_send_status(data, status);
		if (ret)
			goto err_release_reg;
	}
#endif

	ret = sysfs_create_group(&pdev->dev.kobj, &dht22_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "sysfs create failed\n");
		goto err_release_reg;
	}

	data->hwmon_dev = hwmon_device_register(data->dev);
	if (IS_ERR(data->hwmon_dev)) {
		ret = PTR_ERR(data->hwmon_dev);
		goto err_release_sysfs_group;
	}

	return 0;

err_release_sysfs_group:
	sysfs_remove_group(&pdev->dev.kobj, &dht22_attr_group);
err_release_reg:
	if (!IS_ERR(data->reg)) {
		regulator_unregister_notifier(data->reg, &data->nb);
		regulator_disable(data->reg);
	}
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
	if (dht22_soft_reset(data)) {
		mutex_unlock(&data->read_lock);
		return -EFAULT;
	}
	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&pdev->dev.kobj, &dht22_attr_group);
	if (!IS_ERR(data->reg)) {
		regulator_unregister_notifier(data->reg, &data->nb);
		regulator_disable(data->reg);
	}

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
