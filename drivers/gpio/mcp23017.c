/*
 *  mcp23017.c - 16 bit I/O ports
 *
 *  Derived from drivers/i2c/chips/pca9539.c
 *  Derived from drivers/gpio/gpio-pca953x.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/mcp23017.h>
#include <linux/slab.h>
#include <linux/hwmon-sysfs.h>
#include <linux/ctype.h>
#include <linux/kernel.h>

/* register addresses */
#define PCA953X_INPUT		0
#define PCA953X_OUTPUT		1
#define PCA953X_INVERT		2
#define PCA953X_DIRECTION	3

#define MCP_IODIR	0x00		/* init/reset:  all ones */
#define MCP_IPOL	0x01
#define MCP_GPINTEN	0x02
#define MCP_DEFVAL	0x03
#define MCP_INTCON	0x04
#define MCP_IOCON	0x05
#	define IOCON_MIRROR	(1 << 6)
#	define IOCON_SEQOP	(1 << 5)
#	define IOCON_HAEN	(1 << 3)
#	define IOCON_ODR	(1 << 2)
#	define IOCON_INTPOL	(1 << 1)
#define MCP_GPPU	0x06
#define MCP_INTF	0x07
#define MCP_INTCAP	0x08
#define MCP_GPIO	0x09
#define MCP_OLAT	0x0a

#define PCA_GPIO_MASK		0x00FF
#define PCA_INT				0x0100
#define PCA953X_TYPE		0x1000

static const struct i2c_device_id mcp23017_id[] = {
	{ "mcp23017", 16 | PCA953X_TYPE | PCA_INT, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcp23017_id);

struct mcp23017_chip {
	unsigned gpio_start;
	uint16_t reg_output;
	uint16_t reg_invert;
	uint16_t reg_direction;
	uint16_t reg_pullup;
	uint16_t reg_inten;
	uint16_t reg_intcap;
	uint16_t reg_intf;

	struct mutex i2c_lock;

	struct mutex irq_lock;
	uint16_t irq_mask;
	uint16_t irq_stat;
	uint16_t irq_trig_raise;
	uint16_t irq_trig_fall;
	int	 irq_base;

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	const char *const *names;
};

static int mcp23017_write_reg(struct mcp23017_chip *chip, int reg, uint16_t val)
{
	int ret = 0;

	ret = i2c_smbus_write_word_data(chip->client, reg << 1, val);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int mcp23017_read_reg(struct mcp23017_chip *chip, int reg, uint16_t *val)
{
	int ret;

    ret = i2c_smbus_read_word_data(chip->client, reg << 1);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	*val = (uint16_t)ret;
	return 0;
}

/* following are the sysfs callback functions */
static ssize_t mcp23017_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mcp23017_chip *chip = i2c_get_clientdata(client);

	struct sensor_device_attribute *	psa	= to_sensor_dev_attr(attr);
	uint16_t reg_val;
	int ret, reg_offset = 0;

	reg_offset = (psa->index) >> 1;

	/* reading allowed from all registers */
	if(reg_offset < PCA953X_INPUT || reg_offset > PCA953X_DIRECTION)
		return -EINVAL;

	mutex_lock(&chip->i2c_lock);
	ret = mcp23017_read_reg(chip, reg_offset, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return ret;
	}

	return sprintf(buf, "%d\n", (
			(psa->index & 0x1)
			? ((reg_val & 0xff00) >> 8)
			:  (reg_val & 0x00ff)
		)
	);
}

static ssize_t mcp23017_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct i2c_client * client = to_i2c_client(dev);
	struct mcp23017_chip * chip = i2c_get_clientdata(client);

	char * endp;
	unsigned long val = simple_strtoul(buf, &endp, 0);
	int rc;

	if ((*endp && !isspace(*endp)) || val > 0xff)
		return -EINVAL;

	struct sensor_device_attribute * psa = to_sensor_dev_attr(attr);
	uint16_t reg_val;
	int ret, reg_offset = 0;

	reg_offset = (psa->index) >> 1;
	
	/* writing allowed only to PCA953X_OUTPUT, PCA953X_INVERT and PCA953X_DIRECTION registers */
	if(reg_offset < PCA953X_OUTPUT || reg_offset > PCA953X_DIRECTION)
		return -EINVAL;

	mutex_lock(&chip->i2c_lock);

	/* prepare 16-bit value for register pair */
	switch(reg_offset) {
		case PCA953X_OUTPUT:
			reg_val = (
				(psa->index == 2)
				? ((chip->reg_output	& 0xff00) | val       )	/* output0 */
				: ((chip->reg_output	& 0x00ff) | (val << 8))	/* output1 */
			);
			break;
		case PCA953X_INVERT:
			reg_val = (
				(psa->index == 4)
				? ((chip->reg_invert	& 0xff00) | val       ) /* invert0 */
				: ((chip->reg_invert	& 0x00ff) | (val << 8)) /* invert1 */
			);
			break;
		case PCA953X_DIRECTION:
			reg_val = (
				(psa->index == 6)
				? ((chip->reg_direction	& 0xff00) | val       ) /* direction0 */
				: ((chip->reg_direction	& 0x00ff) | (val << 8)) /* direction1 */
			);
			break;
	}

	/* write 16-bit value into a register pair */
	ret = mcp23017_write_reg(chip, reg_offset, reg_val);
	if (ret) {
		rc = ret;
		goto exit;
	}

	/* save written value */
	switch(reg_offset) {
		case PCA953X_OUTPUT:	chip->reg_output	= reg_val; break;
		case PCA953X_INVERT:	chip->reg_invert	= reg_val; break;
		case PCA953X_DIRECTION:	chip->reg_direction	= reg_val; break;
	}

	rc = count;

exit:
	mutex_unlock(&chip->i2c_lock);
	return rc;
}

/* Define the device attributes */
#define PCA953X_ENTRY_RO(name, cmd_idx) \
	static SENSOR_DEVICE_ATTR(name, S_IRUGO, mcp23017_show, NULL, cmd_idx)

#define PCA953X_ENTRY_RW(name, cmd_idx) \
	static SENSOR_DEVICE_ATTR(name, S_IRUGO | S_IWUSR, mcp23017_show, \
				  mcp23017_store, cmd_idx)

PCA953X_ENTRY_RO(input0,	(PCA953X_INPUT << 1));
PCA953X_ENTRY_RO(input1,	((PCA953X_INPUT	<< 1) + 1));
PCA953X_ENTRY_RW(output0,	(PCA953X_OUTPUT << 1));
PCA953X_ENTRY_RW(output1,	((PCA953X_OUTPUT << 1) + 1));
PCA953X_ENTRY_RW(invert0,	(PCA953X_INVERT << 1));
PCA953X_ENTRY_RW(invert1,	((PCA953X_INVERT << 1) + 1));
PCA953X_ENTRY_RW(direction0,	(PCA953X_DIRECTION << 1));
PCA953X_ENTRY_RW(direction1,	((PCA953X_DIRECTION << 1) + 1));

static struct attribute *mcp23017_attributes[] = {
	&sensor_dev_attr_input0.dev_attr.attr,
	&sensor_dev_attr_input1.dev_attr.attr,
	&sensor_dev_attr_output0.dev_attr.attr,
	&sensor_dev_attr_output1.dev_attr.attr,
	&sensor_dev_attr_invert0.dev_attr.attr,
	&sensor_dev_attr_invert1.dev_attr.attr,
	&sensor_dev_attr_direction0.dev_attr.attr,
	&sensor_dev_attr_direction1.dev_attr.attr,
	NULL
};

static struct attribute_group mcp23017_defattr_group = {
	.attrs = mcp23017_attributes,
};

static int mcp23017_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct mcp23017_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct mcp23017_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	reg_val = chip->reg_direction | (1u << off);

	offset = MCP_IODIR;
	ret = mcp23017_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int mcp23017_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct mcp23017_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct mcp23017_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	/* set output level */
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	offset = MCP_GPIO;
	ret = mcp23017_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;

	/* then direction */
	reg_val = chip->reg_direction & ~(1u << off);
	offset = MCP_IODIR;
	ret = mcp23017_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_direction = reg_val;
	ret = 0;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int mcp23017_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct mcp23017_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct mcp23017_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	offset = MCP_GPIO;
	ret = mcp23017_read_reg(chip, offset, &reg_val);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << off)) ? 1 : 0;
}

static void mcp23017_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct mcp23017_chip *chip;
	uint16_t reg_val;
	int ret, offset = 0;

	chip = container_of(gc, struct mcp23017_chip, gpio_chip);

	mutex_lock(&chip->i2c_lock);
	if (val)
		reg_val = chip->reg_output | (1u << off);
	else
		reg_val = chip->reg_output & ~(1u << off);

	offset = MCP_OLAT;
	ret = mcp23017_write_reg(chip, offset, reg_val);
	if (ret)
		goto exit;

	chip->reg_output = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
}

static void mcp23017_setup_gpio(struct mcp23017_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input		= mcp23017_gpio_direction_input;
	gc->direction_output	= mcp23017_gpio_direction_output;
	gc->get					= mcp23017_gpio_get_value;
	gc->set					= mcp23017_gpio_set_value;
	gc->can_sleep			= 1;

	gc->base				= chip->gpio_start;
	gc->ngpio				= gpios;
	gc->label				= chip->client->name;
	gc->dev					= &chip->client->dev;
	gc->owner				= THIS_MODULE;
	gc->names				= chip->names;
}

static int mcp23017_gpio_to_irq(struct gpio_chip *gc, unsigned off)
{
	struct mcp23017_chip *chip;

	chip = container_of(gc, struct mcp23017_chip, gpio_chip);
	return chip->irq_base + off;
}

static void mcp23017_irq_mask(struct irq_data *d)
{
	struct mcp23017_chip *chip = irq_data_get_irq_chip_data(d);

	chip->irq_mask &= ~(1 << (d->irq - chip->irq_base));
}

static void mcp23017_irq_unmask(struct irq_data *d)
{
	struct mcp23017_chip *chip = irq_data_get_irq_chip_data(d);

	chip->irq_mask |= 1 << (d->irq - chip->irq_base);
}

static void mcp23017_irq_bus_lock(struct irq_data *d)
{
	struct mcp23017_chip *chip = irq_data_get_irq_chip_data(d);

	mutex_lock(&chip->irq_lock);
}

static void mcp23017_irq_bus_sync_unlock(struct irq_data *d)
{
	struct mcp23017_chip *chip = irq_data_get_irq_chip_data(d);
	uint16_t new_irqs;
	uint16_t level;

	/* Look for any newly setup interrupt */
	new_irqs = chip->irq_trig_fall | chip->irq_trig_raise;
	new_irqs &= ~chip->reg_direction;

	while (new_irqs) {
		level = __ffs(new_irqs);
		mcp23017_gpio_direction_input(&chip->gpio_chip, level);
		new_irqs &= ~(1 << level);
	}

	mutex_unlock(&chip->irq_lock);
}

static int mcp23017_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct mcp23017_chip *chip = irq_data_get_irq_chip_data(d);
	uint16_t level = d->irq - chip->irq_base;
	uint16_t mask = 1 << level;

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_FALLING)
		chip->irq_trig_fall |= mask;
	else
		chip->irq_trig_fall &= ~mask;

	if (type & IRQ_TYPE_EDGE_RISING)
		chip->irq_trig_raise |= mask;
	else
		chip->irq_trig_raise &= ~mask;

	return 0;
}

static struct irq_chip mcp23017_irq_chip = {
	.name					= "mcp23017",
	.irq_mask				= mcp23017_irq_mask,
	.irq_unmask				= mcp23017_irq_unmask,
	.irq_bus_lock			= mcp23017_irq_bus_lock,
	.irq_bus_sync_unlock	= mcp23017_irq_bus_sync_unlock,
	.irq_set_type			= mcp23017_irq_set_type,
};

static uint16_t mcp23017_irq_pending(struct mcp23017_chip *chip)
{
	uint16_t cur_stat;
	uint16_t old_stat;
	uint16_t pending;
	uint16_t trigger;
	int ret, offset = 0;

	offset = MCP_INTF;
	ret = mcp23017_read_reg(chip, offset, &(chip->reg_intf));
	if (ret)
		return 0;

	offset = MCP_INTCAP;
	ret = mcp23017_read_reg(chip, offset, &(chip->reg_intcap));
	if (ret)
		return 0;

	cur_stat =	chip->reg_intcap;
	/* Remove output pins from the equation */
	cur_stat &= chip->reg_direction;
	/* Remove pins not enabled for interrupt from the equation */
	cur_stat &= chip->reg_inten;

	old_stat = chip->irq_stat;
	trigger = (cur_stat ^ old_stat) & chip->irq_mask;

	if (!trigger)
		return 0;

	chip->irq_stat = cur_stat;

	pending = (old_stat & chip->irq_trig_fall) |
		  (cur_stat & chip->irq_trig_raise);
	pending &= trigger;

	return pending;
}

static irqreturn_t mcp23017_irq_handler(int irq, void *devid)
{
	struct mcp23017_chip *chip = devid;
	uint16_t pending;
	uint16_t level;

	pending = mcp23017_irq_pending(chip);

	if (!pending)
		return IRQ_HANDLED;

	do {
		level = __ffs(pending);
		handle_nested_irq(level + chip->irq_base);

		pending &= ~(1 << level);
	} while (pending);

	return IRQ_HANDLED;
}

static int mcp23017_irq_setup(struct mcp23017_chip *chip,
			     const struct i2c_device_id *id)
{
	struct i2c_client *client = chip->client;
	struct mcp23017_platform_data *pdata = client->dev.platform_data;
	int ret, offset = 0;

	if (pdata->irq_base != -1
			&& (id->driver_data & PCA_INT)) {
		int lvl;

		/* read GPIO port values */
		ret = mcp23017_read_reg(chip, MCP_GPIO, &chip->irq_stat);
		if (ret)
			goto out_failed;

		/*
		 * There is no way to know which GPIO line generated the
		 * interrupt.  We have to rely on the previous read for
		 * this purpose.
		 */
		/* filter out pins which are set for output (represented by 0-bits in reg_direction) */
		chip->irq_stat &= chip->reg_direction;
		/* filter out pins which are not enabled for interrupt */
		chip->irq_stat &= chip->reg_inten;
		chip->irq_base = pdata->irq_base;
		mutex_init(&chip->irq_lock);

		for (lvl = 0; lvl < chip->gpio_chip.ngpio; lvl++) {
			int irq = lvl + chip->irq_base;

			irq_clear_status_flags(irq, IRQ_NOREQUEST);

			irq_set_chip_data(irq, chip);
			irq_set_chip(irq, &mcp23017_irq_chip);
			irq_set_nested_thread(irq, true);
#ifdef CONFIG_ARM
			set_irq_flags(irq, IRQF_VALID);
#else
			irq_set_noprobe(irq);
#endif
		}

		ret = request_threaded_irq(client->irq,
					   NULL,
					   mcp23017_irq_handler,
					   IRQF_TRIGGER_RISING |
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   dev_name(&client->dev), chip);
		if (ret) {
			dev_err(&client->dev, "failed to request irq %d\n",
				client->irq);
			goto out_failed;
		}

		chip->gpio_chip.to_irq = mcp23017_gpio_to_irq;
	}

	return 0;

out_failed:
	chip->irq_base = -1;
	return ret;
}

static void mcp23017_irq_teardown(struct mcp23017_chip *chip)
{
	if (chip->irq_base != -1)
		free_irq(chip->client->irq, chip);
}

static int device_mcp23017_init(struct mcp23017_chip *chip, struct mcp23017_platform_data *pdata)
{
	int ret;

//	ret = mcp23017_read_reg(chip, PCA953X_OUTPUT, &chip->reg_output);
//	if (ret)
//		goto out;
//
//	ret = mcp23017_read_reg(chip, PCA953X_DIRECTION,
//			       &chip->reg_direction);
//	if (ret)
//		goto out;
//
//	/* set platform specific polarity inversion */
//	ret = mcp23017_write_reg(chip, PCA953X_INVERT, invert);
//	if (ret)
//		goto out;
//	chip->reg_invert = invert;


	/* read INTCAP reg */
	ret = mcp23017_read_reg(chip, MCP_INTCAP, &(chip->reg_intcap));
	if (ret)
		goto out;

	if(pdata) {

		/* set pullup register */
		ret = mcp23017_write_reg(chip, MCP_GPPU, pdata->pullup);
		if (ret)
			goto out;
		chip->reg_pullup = pdata->pullup;

		/* set interrupt enable register */
		ret = mcp23017_write_reg(chip, MCP_GPINTEN, pdata->inten);
		if (ret)
			goto out;
		chip->reg_inten = pdata->inten;

	}

	return 0;
out:
	return ret;
}

static int mcp23017_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	printk("--- %s\n", __func__);

	struct mcp23017_platform_data *pdata;
	struct mcp23017_chip *chip;
	int ret = 0;

	chip = kzalloc(sizeof(struct mcp23017_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	pdata = client->dev.platform_data;

	if (pdata == NULL) {
		dev_dbg(&client->dev, "no platform data\n");
		ret = -EINVAL;
		goto out_failed;
	}

	chip->client = client;

	chip->gpio_start = pdata->gpio_base;

	chip->names = pdata->names;

	mutex_init(&chip->i2c_lock);

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	//mcp23017_setup_gpio(chip, id->driver_data & PCA_GPIO_MASK);
	mcp23017_setup_gpio(chip, 16);

	/* specific settings: pullups, inversions, interrupt enable ... */
	device_mcp23017_init(chip, pdata);

	ret = mcp23017_irq_setup(chip, id);
	if (ret)
		goto out_failed;

	ret = gpiochip_add(&chip->gpio_chip);
	if (ret)
		goto out_failed_irq;

	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);

	/* Register sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj,
				 &mcp23017_defattr_group);
	if (ret)
		goto out_failed_sysfs;

	return 0;

out_failed_sysfs:
out_failed_irq:
	mcp23017_irq_teardown(chip);
out_failed:
	kfree(chip);
	return ret;
}

static int mcp23017_remove(struct i2c_client *client)
{
	struct mcp23017_platform_data *pdata = client->dev.platform_data;
	struct mcp23017_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

	sysfs_remove_group(&client->dev.kobj, &mcp23017_defattr_group);

	if (pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0) {
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
			return ret;
		}
	}

	ret = gpiochip_remove(&chip->gpio_chip);
	if (ret) {
		dev_err(&client->dev, "%s failed, %d\n",
				"gpiochip_remove()", ret);
		return ret;
	}

	mcp23017_irq_teardown(chip);
	kfree(chip);
	return 0;
}

static struct i2c_driver mcp23017_driver = {
	.driver = {
		.name	= "mcp23017",
	},
	.probe		= mcp23017_probe,
	.remove		= mcp23017_remove,
	.id_table	= mcp23017_id,
};

static int __init mcp23017_init(void)
{
	return i2c_add_driver(&mcp23017_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(mcp23017_init);

static void __exit mcp23017_exit(void)
{
	i2c_del_driver(&mcp23017_driver);
}
module_exit(mcp23017_exit);

MODULE_AUTHOR("eric miao <eric.miao@marvell.com>");
MODULE_DESCRIPTION("GPIO expander driver for MCP23017");
MODULE_LICENSE("GPL");
