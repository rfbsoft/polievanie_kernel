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

#define MCP_GPIO_MASK	0x00FF
#define MCP_IRQ		0x0100

static const struct i2c_device_id mcp23017_id[] = {
	{ "mcp23017", 16 | MCP_IRQ, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcp23017_id);

struct mcp23017_chip {
	unsigned gpio_start;
	uint16_t reg_iocon;
	uint16_t reg_output;
	uint16_t reg_direction;
	uint16_t reg_pullup;
	uint16_t reg_inten;
	uint16_t reg_intcap;
	uint16_t reg_intf;
	uint16_t reg_defval;
	uint16_t reg_intcon;

	struct mutex i2c_lock;

	struct mutex irq_lock;
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

	/* reading allowed from these registers */
	if(
		reg_offset != MCP_GPIO &&
		reg_offset != MCP_OLAT &&
		reg_offset != MCP_IODIR &&
		reg_offset != MCP_GPINTEN &&
		reg_offset != MCP_GPPU
	)
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

	return sprintf(buf, "0x%02x\n", (
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
	
	/* writing allowed to these registers */
	if(
		reg_offset != MCP_OLAT &&
		reg_offset != MCP_IODIR &&
		reg_offset != MCP_GPINTEN &&
		reg_offset != MCP_GPPU
	)
		return -EINVAL;

	mutex_lock(&chip->i2c_lock);

	/* prepare 16-bit value for register pair */
	switch(reg_offset) {
		case MCP_OLAT:
			reg_val = (
				(psa->index  & 0x1)
				? ((chip->reg_output	& 0x00ff) | (val << 8))	/* output1 */
				: ((chip->reg_output	& 0xff00) | val       )	/* output0 */
			);
			break;
		case MCP_IODIR:
			reg_val = (
				(psa->index & 0x1)
				? ((chip->reg_direction	& 0x00ff) | (val << 8)) /* direction1 */
				: ((chip->reg_direction	& 0xff00) | val       ) /* direction0 */
			);
			break;
		case MCP_GPINTEN:
			reg_val = (
				(psa->index & 0x1)
				? ((chip->reg_inten	& 0x00ff) | (val << 8)) /* inten1 */
				: ((chip->reg_inten	& 0xff00) | val       ) /* inten0 */
			);
			break;
		case MCP_GPPU:
			reg_val = (
				(psa->index & 0x1)
				? ((chip->reg_pullup	& 0x00ff) | (val << 8)) /* pullup1 */
				: ((chip->reg_pullup	& 0xff00) | val       ) /* pullup0 */
			);
			break;
	}

	/* write 16-bit value into a register pair */
	ret = mcp23017_write_reg(chip, reg_offset, reg_val);
	if (ret) {
		rc = -EIO;
		goto exit;
	}

	/* save written value */
	switch(reg_offset) {
		case MCP_OLAT:		chip->reg_output	= reg_val; break;
		case MCP_IODIR:		chip->reg_direction	= reg_val; break;
		case MCP_GPINTEN:	chip->reg_inten		= reg_val; break;
		case MCP_GPPU:		chip->reg_pullup	= reg_val; break;
	}

	rc = count;

exit:
	mutex_unlock(&chip->i2c_lock);
	return rc;
}

/* Define the device attributes */
#define MCP23017_ENTRY_RO(name, cmd_idx) \
	static SENSOR_DEVICE_ATTR(name, S_IRUGO, mcp23017_show, NULL, cmd_idx)

#define MCP23017_ENTRY_RW(name, cmd_idx) \
	static SENSOR_DEVICE_ATTR(name, S_IRUGO | S_IWUSR, mcp23017_show, \
				  mcp23017_store, cmd_idx)

MCP23017_ENTRY_RO(input0,	((MCP_GPIO << 1)       ));
MCP23017_ENTRY_RO(input1,	((MCP_GPIO << 1)    + 1));
MCP23017_ENTRY_RW(output0,	((MCP_OLAT << 1)       ));
MCP23017_ENTRY_RW(output1,	((MCP_OLAT << 1)    + 1));
MCP23017_ENTRY_RW(direction0,	((MCP_IODIR << 1)      ));
MCP23017_ENTRY_RW(direction1,	((MCP_IODIR << 1)   + 1));
MCP23017_ENTRY_RW(inten0,	((MCP_GPINTEN << 1)    ));
MCP23017_ENTRY_RW(inten1,	((MCP_GPINTEN << 1) + 1));
MCP23017_ENTRY_RW(pullup0,	((MCP_GPPU << 1)       ));
MCP23017_ENTRY_RW(pullup1,	((MCP_GPPU << 1)    + 1));

static struct attribute *mcp23017_attributes[] = {
	&sensor_dev_attr_input0.dev_attr.attr,
	&sensor_dev_attr_input1.dev_attr.attr,
	&sensor_dev_attr_output0.dev_attr.attr,
	&sensor_dev_attr_output1.dev_attr.attr,
	&sensor_dev_attr_direction0.dev_attr.attr,
	&sensor_dev_attr_direction1.dev_attr.attr,
	&sensor_dev_attr_inten0.dev_attr.attr,
	&sensor_dev_attr_inten1.dev_attr.attr,
	&sensor_dev_attr_pullup0.dev_attr.attr,
	&sensor_dev_attr_pullup1.dev_attr.attr,
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

uint8_t mcp23017_get_reg8(struct device *dev, unsigned reg_offset) {
	uint16_t reg_val = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct mcp23017_chip *chip = i2c_get_clientdata(client);

	if(reg_offset >= 0 && reg_offset <= 0x15) {
		mutex_lock(&chip->i2c_lock);
		mcp23017_read_reg(chip, reg_offset >> 1, &reg_val);
		mutex_unlock(&chip->i2c_lock);
	}
	return (reg_offset & 0x1) ? ((reg_val & 0xff00) >> 8) : (reg_val & 0x00ff);
}
EXPORT_SYMBOL_GPL(mcp23017_get_reg8);

void mcp23017_set_reg8(struct device *dev, unsigned reg_offset, uint8_t val) {
	int ret;
	uint16_t reg_val = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct mcp23017_chip *chip = i2c_get_clientdata(client);

	if((reg_offset >> 1) == MCP_OLAT) {
		mutex_lock(&chip->i2c_lock);

		/* prepare 16-bit value */
		switch(reg_offset >> 1) {
			case MCP_OLAT:
				reg_val = (
					(reg_offset  & 0x1)
					? ((chip->reg_output	& 0x00ff) | (val << 8))	/* OLATB */
					: ((chip->reg_output	& 0xff00) | val       )	/* OLATA */
				);
				break;
		}

		/* write 16-bit value into a register pair */
		if(reg_val != chip->reg_output) {
			ret = mcp23017_write_reg(chip, reg_offset >> 1, reg_val);
			if(!ret) {
				/* save written value */
				switch(reg_offset >> 1) {
					case MCP_OLAT:		chip->reg_output	= reg_val; break;
				}
			}
		}

		mutex_unlock(&chip->i2c_lock);
	}
}
EXPORT_SYMBOL_GPL(mcp23017_set_reg8);

static void mcp23017_setup_gpio(struct mcp23017_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input	= mcp23017_gpio_direction_input;
	gc->direction_output	= mcp23017_gpio_direction_output;
	gc->get			= mcp23017_gpio_get_value;
	gc->set			= mcp23017_gpio_set_value;
	gc->can_sleep		= 1;

	gc->base		= chip->gpio_start;
	gc->ngpio		= gpios;
	gc->label		= chip->client->name;
	gc->dev			= &chip->client->dev;
	gc->owner		= THIS_MODULE;
	gc->names		= chip->names;
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

	chip->reg_inten &= ~(1 << (d->irq - chip->irq_base));
}

static void mcp23017_irq_unmask(struct irq_data *d)
{
	struct mcp23017_chip *chip = irq_data_get_irq_chip_data(d);

	chip->reg_inten |= 1 << (d->irq - chip->irq_base);
}

static void mcp23017_irq_bus_lock(struct irq_data *d)
{
	struct mcp23017_chip *chip = irq_data_get_irq_chip_data(d);

	mutex_lock(&chip->irq_lock);
}

static void mcp23017_irq_bus_sync_unlock(struct irq_data *d)
{
	struct mcp23017_chip *chip = irq_data_get_irq_chip_data(d);

	mutex_lock(&chip->i2c_lock);
	mcp23017_write_reg(chip, MCP_GPINTEN,	chip->reg_inten);
	mcp23017_write_reg(chip, MCP_DEFVAL,	chip->reg_defval);
	mcp23017_write_reg(chip, MCP_INTCON,	chip->reg_intcon);
	mutex_unlock(&chip->i2c_lock);

	mutex_unlock(&chip->irq_lock);
}

static int mcp23017_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct mcp23017_chip *chip = irq_data_get_irq_chip_data(d);
	uint16_t level = d->irq - chip->irq_base;
	uint16_t mask = 1 << level;

	if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
		chip->reg_intcon &= ~mask;
		chip->irq_trig_raise |= mask;
		chip->irq_trig_fall  |= mask;
	}
	else if (type & IRQ_TYPE_EDGE_RISING) {
		chip->reg_intcon &= ~mask;
		chip->irq_trig_raise |= mask;
		chip->irq_trig_fall  &= ~mask;
	}
	else if (type & IRQ_TYPE_EDGE_FALLING) {
		chip->reg_intcon &= ~mask;
		chip->irq_trig_raise &= ~mask;
		chip->irq_trig_fall  |= mask;
	}
	else {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	return 0;
}

static struct irq_chip mcp23017_irq_chip = {
	.name			= "mcp23017",
	.irq_mask		= mcp23017_irq_mask,
	.irq_unmask		= mcp23017_irq_unmask,
	.irq_bus_lock		= mcp23017_irq_bus_lock,
	.irq_bus_sync_unlock	= mcp23017_irq_bus_sync_unlock,
	.irq_set_type		= mcp23017_irq_set_type,
};

static irqreturn_t mcp23017_irq_handler(int irq, void *devid)
{
	struct mcp23017_chip *chip = devid;
	int ret;
	int i;
	uint16_t intf;
	uint16_t intcap;

	mutex_lock(&chip->i2c_lock);

	/* read pins caused the interrupt */
	ret = mcp23017_read_reg(chip, MCP_INTF, &intf);
	if (ret) {
		mutex_unlock(&chip->i2c_lock);
		return IRQ_HANDLED;
	}
	chip->reg_intf = intf;

	/* read gpio ports catched on interrupt occurence */
	ret = mcp23017_read_reg(chip, MCP_INTCAP, &intcap);
	if (ret) {
		mutex_unlock(&chip->i2c_lock);
		return IRQ_HANDLED;
	}
	chip->reg_intcap = intcap;

	mutex_unlock(&chip->i2c_lock);

	for (i = 0; i < chip->gpio_chip.ngpio; i++) {
		if (
			(intf & BIT(i)) &&
			(
				(intcap  & BIT(i) & chip->irq_trig_raise) ||
				(~intcap & BIT(i) & chip->irq_trig_fall )
			)
		)
			handle_nested_irq(i + chip->irq_base);
	}

	return IRQ_HANDLED;
}

static int mcp23017_irq_setup(struct mcp23017_chip *chip,
			     const struct i2c_device_id *id)
{
	struct i2c_client *client = chip->client;
	struct mcp23017_platform_data *pdata = client->dev.platform_data;
	int ret, offset = 0;

	if (pdata->irq_base != -1 && (id->driver_data & MCP_IRQ)) {
		int lvl;

		/* read INTCAP register */
		ret = mcp23017_read_reg(chip, MCP_INTCAP, &chip->reg_intcap);
		if (ret)
			goto out_failed;

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
	uint16_t iocon;

	mutex_lock(&chip->i2c_lock);

	/* setup IOCON register */
	ret = mcp23017_read_reg(chip, MCP_IOCON, &chip->reg_iocon);
	if (ret)
		goto out;
	iocon = chip->reg_iocon;
	/* setup mirroring of INTA & INTB signals */
	if(pdata) {
		if(pdata->int_mirror)
			iocon |= IOCON_MIRROR | (IOCON_MIRROR << 8);
		else
			iocon &= ~(IOCON_MIRROR | (IOCON_MIRROR << 8));
	}
	/* setup polarity of the INT pin to active=low */
	iocon &= ~(IOCON_INTPOL | (IOCON_INTPOL << 8) | IOCON_ODR | (IOCON_ODR << 8));
	/* setup sequential operation to enabled (address pointer increments) */
	iocon &= ~(IOCON_SEQOP | (IOCON_SEQOP << 8));
	if(iocon != chip->reg_iocon) {
		ret = mcp23017_write_reg(chip, MCP_IOCON, iocon);
		if (ret)
			goto out;
		chip->reg_iocon = iocon;

	}

	/* read MCP_IODIR register */
	ret = mcp23017_read_reg(chip, MCP_IODIR, &chip->reg_direction);
	if (ret)
		goto out;

	/* read MCP_OLAT register */
	ret = mcp23017_read_reg(chip, MCP_OLAT, &chip->reg_output);
	if (ret)
		goto out;

	/* read MCP_GPINTEN register */
	ret = mcp23017_read_reg(chip, MCP_GPINTEN, &chip->reg_inten);
	if (ret)
		goto out;

	/* write MCP_DEFVAL register */
	ret = mcp23017_write_reg(chip, MCP_DEFVAL, 0);
	if (ret)
		goto out;
	chip->reg_defval = 0;

	/* all interrupts on change base on previous value and not default value */
	ret = mcp23017_write_reg(chip, MCP_INTCON, 0);
	if (ret)
		goto out;
	chip->reg_intcon = 0;

	/* read MCP_GPPU register */
	ret = mcp23017_read_reg(chip, MCP_GPPU, &chip->reg_pullup);
	if (ret)
		goto out;
	if(pdata && pdata->pullup != chip->reg_pullup) {
		/* set pullup register */
		ret = mcp23017_write_reg(chip, MCP_GPPU, pdata->pullup);
		if (ret)
			goto out;
		chip->reg_pullup = pdata->pullup;
	}

	ret = 0;

out:
	mutex_unlock(&chip->i2c_lock);
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
	mcp23017_setup_gpio(chip, id->driver_data & MCP_GPIO_MASK);

	/* specific settings: pullups, interrupt mirror ... */
	device_mcp23017_init(chip, pdata);

	/* ----- Interrupt setup ----- */
	/* disable irqs */
	if (chip->reg_inten != 0) {
		chip->reg_inten = 0;
		ret = mcp23017_write_reg(chip, MCP_GPINTEN, 0);
		if (ret)
			goto out_failed;
	}
	ret = mcp23017_irq_setup(chip, id);
	if (ret)
		goto out_failed;
	/* enable irqs */
	if(pdata && pdata->inten != chip->reg_inten) {
		/* set interrupt enable register */
		ret = mcp23017_write_reg(chip, MCP_GPINTEN, pdata->inten);
		if (ret)
			goto out_failed_irq;
		chip->reg_inten = pdata->inten;
	}


	ret = gpiochip_add(&chip->gpio_chip);
	if (ret)
		goto out_failed_irq;

	i2c_set_clientdata(client, chip);

	if (pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

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

MODULE_AUTHOR("Robert Fabian <rfabian2@gmail.com>");
MODULE_DESCRIPTION("GPIO expander driver for MCP23017");
MODULE_LICENSE("GPL");
