/*
 * Driver for the on-board character LCD found on some ARM reference boards
 * This is basically an Hitachi HD44780 LCD with a custom IP block to drive it
 * http://en.wikipedia.org/wiki/HD44780_Character_LCD
 * Currently it will just display the text "ARM Linux" and the linux version
 *
 * License terms: GNU General Public License (GPL) version 2
 * Author: Linus Walleij <triad@df.lth.se>
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/hwmon-sysfs.h>
#include <linux/ctype.h>
#include <linux/platform_data/hd44780.h>
#include <generated/utsrelease.h>

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

#define DRIVERNAME "hd44780"

/* --- Hitachi HD44780 display commands --- */
/* clear entire display */
#define HD_CMD_CLEAR			0x01U

/* return cursor home & returns display to unshifted position */
#define HD_CMD_HOME			0x02U

/* Cursor movement direction & */
/* display shift on/off on entering of a new character */
/*                                bitval1   / bitval0   */
/*   I/D (I)ncrement/(D)ecrement: increment / decrement */
/*   S   display (S)hift:         ON        / OFF       */
#define HD_CMD_ENTRYMODE		0x04U
#define HD_CMD_ENTRYMODE_INCREMENT	0x02U
#define HD_CMD_ENTRYMODE_DISPLAY_SHIFT	0x01U

/* Display & Cursor Control   bitval1 / bitval0 */
/*   D (D)isplay:             ON      / OFF     */
/*   C (C)ursor:              ON      / OFF     */
/*   B cursor (B)link:        ON      / OFF     */
#define HD_CMD_DISPCTRL			0x08U
#define HD_CMD_DISPCTRL_DISPLAY_ON	0x04U
#define HD_CMD_DISPCTRL_CURSOR_ON	0x02U
#define HD_CMD_DISPCTRL_CURSOR_BLINK	0x01U

/* Display shift or Cursor move         bitval1       / bitval0     */
/*   S/C display (S)hift/(C)ursor move: display shift / cursor move */
/*   R/L (R)ight/(L)eft:                to the right  / to the left */
#define HD_CMD_SHIFT			0x10U
#define HD_CMD_SHIFT_DISPLAY		0x08U
#define HD_CMD_SHIFT_RIGHT		0x04U

/* Function set                    bitval1 / bitval0               */
/*   DL (D)ata (L)ength:           8bit    / 4bit    communication */
/*   N  (N)umber of display lines: 2       / 1       lines         */
/*   F  (F)ont size:               5x10    / 5x8     pixels        */
#define HD_CMD_FUNCSET			0x20U
#define HD_CMD_FUNCSET_8BIT		0x10U
#define HD_CMD_FUNCSET_2LINES		0x08U
#define HD_CMD_FUNCSET_FONT5X10		0x04U

/* Set CGRAM address */
#define HD_CMD_SET_CGRAM		0x40U

/* Set DDRAM address */
#define HD_CMD_SET_DDRAM		0x80U

/* --- commands duration in useconds --- */
//#define HD_CMD_DURATION_CLEAR		1500
//#define HD_CMD_DURATION_HOME		1500
//#define HD_CMD_DURATION_WRITECHAR	45
//#define HD_CMD_DURATION			40
#define HD_CMD_DURATION_CLEAR		2000
#define HD_CMD_DURATION_HOME		2000
#define HD_CMD_DURATION_WRITECHAR	70
#define HD_CMD_DURATION				60

/* --- sysfs hook indexes --- */
#define LCD_CLEAR		 0
#define LCD_HOME		 1
#define LCD_ENTRYINCREMENT	 2
#define LCD_ENTRYSHIFT		 3
#define LCD_CTRLDISP		 4
#define LCD_CTRLCURS		 5
#define LCD_CTRLBLINK		 6
#define LCD_CURSSHIFTLEFT	 7
#define LCD_CURSSHIFTRIGHT	 8
#define LCD_DISPSHIFTLEFT	 9
#define LCD_DISPSHIFTRIGHT	10
#define LCD_PRINT			11
#define LCD_PRINTRAW		12
#define LCD_ROWS			13
#define LCD_COLS			14

/**
 * @dev: a pointer back to containing device
 */
struct hd44780 {
	struct device *dev;

	struct mutex status_lock;

	/*   I/D (I)ncrement/(D)ecrement: increment / decrement */
	/*   S   display (S)hift:         ON        / OFF       */
	uint8_t status_entrymode;

	/*   D (D)isplay:             ON      / OFF     */
	/*   C (C)ursor:              ON      / OFF     */
	/*   B cursor (B)link:        ON      / OFF     */
	uint8_t status_dispctrl;

	uint8_t gpio_port8_mask; /* ignore 0 bits */

};

static struct gpio hd44780_gpios[] = {
    {
        .flags  = GPIOF_OUT_INIT_LOW,
        .label  = "RS",
    },
    {
        .flags  = GPIOF_OUT_INIT_LOW,
        .label  = "R/W",
    },
    {
        .flags  = GPIOF_OUT_INIT_LOW,
        .label  = "E",
    },
    {
        .flags  = GPIOF_OUT_INIT_LOW,
        .label  = "DB4",
    },
    {
        .flags  = GPIOF_OUT_INIT_LOW,
        .label  = "DB5",
    },
    {
        .flags  = GPIOF_OUT_INIT_LOW,
        .label  = "DB6",
    },
    {
        .flags  = GPIOF_OUT_INIT_LOW,
        .label  = "DB7",
    },
};

static void hd44780_4bit_command(struct hd44780 *lcd, u8 cmd)
{
	struct hd44780_platform_data * pdata = lcd->dev->platform_data;
	uint8_t out8;

	if(pdata->set_output8) {
		out8 = 0x0;			// all bits low
		out8 |= pdata->bit_E; 		// E = high
		if(cmd & 0x10) out8 |= pdata->bit_DB4;	// DB4 ~ cmd4
		if(cmd & 0x20) out8 |= pdata->bit_DB5;	// DB5 ~ cmd5
		if(cmd & 0x40) out8 |= pdata->bit_DB6;	// DB6 ~ cmd6
		if(cmd & 0x80) out8 |= pdata->bit_DB7;	// DB7 ~ cmd7

		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);
		out8 &= ~(pdata->bit_E); // E = low
		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);

		out8 |= pdata->bit_E; 		// E = high
		out8 &= ~(pdata->bit_DB4 | pdata->bit_DB5 | pdata->bit_DB6 | pdata->bit_DB7); // clear DB4 to DB7
		if(cmd & 0x01) out8 |= pdata->bit_DB4;	// DB4 ~ cmd0
		if(cmd & 0x02) out8 |= pdata->bit_DB5;	// DB5 ~ cmd1
		if(cmd & 0x04) out8 |= pdata->bit_DB6;	// DB6 ~ cmd2
		if(cmd & 0x08) out8 |= pdata->bit_DB7;	// DB7 ~ cmd3

		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);
		out8 &= ~(pdata->bit_E); // E = low
		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);
	}
}

static void hd44780_4bit_char(struct hd44780 *lcd, u8 ch)
{
	struct hd44780_platform_data * pdata = lcd->dev->platform_data;
	uint8_t out8;

	if(pdata->set_output8) {
		out8 = 0x0;			// all bits low
		out8 |= pdata->bit_E; 		// E = high
		out8 |= pdata->bit_RS; 		// RS = high
		if(ch & 0x10) out8 |= pdata->bit_DB4;	// DB4 ~ ch4
		if(ch & 0x20) out8 |= pdata->bit_DB5;	// DB5 ~ ch5
		if(ch & 0x40) out8 |= pdata->bit_DB6;	// DB6 ~ ch6
		if(ch & 0x80) out8 |= pdata->bit_DB7;	// DB7 ~ ch7

		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);
		out8 &= ~(pdata->bit_E); // E = low
		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);

		out8 |= pdata->bit_E; 		// E = high
		out8 &= ~(pdata->bit_DB4 | pdata->bit_DB5 | pdata->bit_DB6 | pdata->bit_DB7); // clear DB4 to DB7
		if(ch & 0x01) out8 |= pdata->bit_DB4;	// DB4 ~ ch0
		if(ch & 0x02) out8 |= pdata->bit_DB5;	// DB5 ~ ch1
		if(ch & 0x04) out8 |= pdata->bit_DB6;	// DB6 ~ ch2
		if(ch & 0x08) out8 |= pdata->bit_DB7;	// DB7 ~ ch3

		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);
		out8 &= ~(pdata->bit_E); // E = low
		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);
	}
}

static void hd44780_4bit_print(struct hd44780 *lcd, int line, const char *str)
{
	struct hd44780_platform_data * pdata = lcd->dev->platform_data;
	u8 offset;
	int i;

	/* get line offset */
	if(line >= 0 && line < pdata->rows)
		offset = pdata->row_offset[line];
	else
		return;

	/* Set offset */
	hd44780_4bit_command(lcd, HD_CMD_SET_DDRAM | offset);
	udelay(HD_CMD_DURATION);

	/* Send string */
	for (i = 0; i < pdata->cols; ++i) {
		if(i < strlen(str))
			hd44780_4bit_char(lcd, str[i]);
		else
			hd44780_4bit_char(lcd, 0x20);
		udelay(HD_CMD_DURATION_WRITECHAR);
	}
}

static void hd44780_4bit_rawprint(struct hd44780 *lcd, const char *str)
{
	int i;

	/* Send string */
	for (i = 0; i < strlen(str); i++) {
		hd44780_4bit_char(lcd, str[i]);
		udelay(HD_CMD_DURATION_WRITECHAR);
	}
}

static void hd44780_8bit_command(struct hd44780 *lcd, u8 cmd)
{
	struct hd44780_platform_data * pdata = lcd->dev->platform_data;
	uint8_t out8;

	if(pdata->set_output8) {
		out8 = 0x0;			// all bits low
		out8 |= pdata->bit_E; 		// E = high
		if(cmd & 0x10) out8 |= pdata->bit_DB4;	// DB4
		if(cmd & 0x20) out8 |= pdata->bit_DB5;	// DB5
		if(cmd & 0x40) out8 |= pdata->bit_DB6;	// DB6
		if(cmd & 0x80) out8 |= pdata->bit_DB7;	// DB7

		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);
		out8 &= ~(pdata->bit_E); // E = low
		(*(pdata->set_output8))(lcd->dev->parent, out8, lcd->gpio_port8_mask);
	}
}

static void hd44780_4bit_init(struct hd44780 *lcd)
{
	hd44780_8bit_command(lcd, HD_CMD_FUNCSET | HD_CMD_FUNCSET_8BIT);
	msleep(5);
	hd44780_8bit_command(lcd, HD_CMD_FUNCSET | HD_CMD_FUNCSET_8BIT);
	udelay(100);
	hd44780_8bit_command(lcd, HD_CMD_FUNCSET | HD_CMD_FUNCSET_8BIT);
	udelay(100);
	/* Go to 4bit mode */
	hd44780_8bit_command(lcd, HD_CMD_FUNCSET);
	udelay(HD_CMD_DURATION);
	/*
	 * 4bit mode, 2 lines, 5x8 font, after this the number of lines
	 * and the font cannot be changed until the next initialization sequence
	 */
	hd44780_4bit_command(lcd, HD_CMD_FUNCSET | HD_CMD_FUNCSET_2LINES);
	udelay(HD_CMD_DURATION);

	mutex_lock(&lcd->status_lock);

	lcd->status_dispctrl = HD_CMD_DISPCTRL | HD_CMD_DISPCTRL_DISPLAY_ON;
	hd44780_4bit_command(lcd, lcd->status_dispctrl);
	udelay(HD_CMD_DURATION);

	lcd->status_entrymode = HD_CMD_ENTRYMODE | HD_CMD_ENTRYMODE_INCREMENT;
	hd44780_4bit_command(lcd, lcd->status_entrymode);
	udelay(HD_CMD_DURATION);

	mutex_unlock(&lcd->status_lock);

	hd44780_4bit_command(lcd, HD_CMD_CLEAR);
	udelay(HD_CMD_DURATION_CLEAR);
	hd44780_4bit_command(lcd, HD_CMD_HOME);
	udelay(HD_CMD_DURATION_HOME);
	/* Put something useful in the display */
	hd44780_4bit_print(lcd, 0, "ARM Linux");
	hd44780_4bit_print(lcd, 1, UTS_RELEASE);
}

/* ------------------ sysfs hooks BEGIN ------------------------ */
static ssize_t hd44780_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct platform_device * pdev = to_platform_device(dev);
	struct hd44780 *lcd = platform_get_drvdata(pdev);
	struct hd44780_platform_data * pdata = lcd->dev->platform_data;
	struct sensor_device_attribute * psa = to_sensor_dev_attr(attr);

	switch(psa->index) {
		case LCD_ROWS:
			return sprintf(buf, "%d\n", pdata->rows);
			break;
		case LCD_COLS:
			return sprintf(buf, "%d\n", pdata->cols);
			break;
		default:
			return -EINVAL;
	}
}

static ssize_t hd44780_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct platform_device * pdev = to_platform_device(dev);
	struct hd44780 *lcd = platform_get_drvdata(pdev);
	struct hd44780_platform_data * pdata = lcd->dev->platform_data;
	struct sensor_device_attribute * psa = to_sensor_dev_attr(attr);

	unsigned long val;
	char * endp;
	int i;
	int line;

	/* get numeric value */
	switch(psa->index) {
		case LCD_CLEAR:
		case LCD_HOME:
		case LCD_ENTRYINCREMENT:
		case LCD_ENTRYSHIFT:
		case LCD_CTRLDISP:
		case LCD_CTRLCURS:
		case LCD_CTRLBLINK:
		case LCD_CURSSHIFTLEFT:
		case LCD_CURSSHIFTRIGHT:
		case LCD_DISPSHIFTLEFT:
		case LCD_DISPSHIFTRIGHT:
			val = simple_strtoul(buf, &endp, 0);
			if ((*endp && !isspace(*endp)))
				return -EINVAL;
			break;
	}

	/* validate numeric value */
	switch(psa->index) {
		case LCD_CLEAR:
		case LCD_HOME:
			if(val != 1)
				return -EINVAL;
			break;
		case LCD_ENTRYINCREMENT:
		case LCD_ENTRYSHIFT:
		case LCD_CTRLDISP:
		case LCD_CTRLCURS:
		case LCD_CTRLBLINK:
			if(val != 0 && val != 1)
				return -EINVAL;
			break;
		case LCD_CURSSHIFTLEFT:
		case LCD_CURSSHIFTRIGHT:
		case LCD_DISPSHIFTLEFT:
		case LCD_DISPSHIFTRIGHT:
			if(val < 1 || val > 39)
				return -EINVAL;
			break;
	}

	/* formatted print settings */
	switch(psa->index) {
		case LCD_PRINT:
			mutex_lock(&lcd->status_lock);
			lcd->status_entrymode |= HD_CMD_ENTRYMODE_INCREMENT;
			hd44780_4bit_command(lcd, lcd->status_entrymode);
			udelay(HD_CMD_DURATION);
			lcd->status_entrymode &= ~HD_CMD_ENTRYMODE_DISPLAY_SHIFT;
			hd44780_4bit_command(lcd, lcd->status_entrymode);
			udelay(HD_CMD_DURATION);
			mutex_unlock(&lcd->status_lock);
			hd44780_4bit_command(lcd, HD_CMD_HOME);
			udelay(HD_CMD_DURATION_HOME);
			break;
	}

	switch(psa->index) {
		case LCD_CLEAR:
			hd44780_4bit_command(lcd, HD_CMD_CLEAR);
			udelay(HD_CMD_DURATION_CLEAR);
			break;
		case LCD_HOME:
			hd44780_4bit_command(lcd, HD_CMD_HOME);
			udelay(HD_CMD_DURATION_HOME);
			break;
		case LCD_ENTRYINCREMENT:
			mutex_lock(&lcd->status_lock);
			if(val)
				lcd->status_entrymode |= HD_CMD_ENTRYMODE_INCREMENT;
			else
				lcd->status_entrymode &= ~HD_CMD_ENTRYMODE_INCREMENT;
			hd44780_4bit_command(lcd, lcd->status_entrymode);
			udelay(HD_CMD_DURATION);
			mutex_unlock(&lcd->status_lock);
			break;
		case LCD_ENTRYSHIFT:
			mutex_lock(&lcd->status_lock);
			if(val)
				lcd->status_entrymode |= HD_CMD_ENTRYMODE_DISPLAY_SHIFT;
			else
				lcd->status_entrymode &= ~HD_CMD_ENTRYMODE_DISPLAY_SHIFT;
			hd44780_4bit_command(lcd, lcd->status_entrymode);
			udelay(HD_CMD_DURATION);
			mutex_unlock(&lcd->status_lock);
			break;
		case LCD_CTRLDISP:
			mutex_lock(&lcd->status_lock);
			if(val)
				lcd->status_dispctrl |= HD_CMD_DISPCTRL_DISPLAY_ON;
			else
				lcd->status_dispctrl &= ~HD_CMD_DISPCTRL_DISPLAY_ON;
			hd44780_4bit_command(lcd, lcd->status_dispctrl);
			udelay(HD_CMD_DURATION);
			mutex_unlock(&lcd->status_lock);
			break;
		case LCD_CTRLCURS:
			mutex_lock(&lcd->status_lock);
			if(val)
				lcd->status_dispctrl |= HD_CMD_DISPCTRL_CURSOR_ON;
			else
				lcd->status_dispctrl &= ~HD_CMD_DISPCTRL_CURSOR_ON;
			hd44780_4bit_command(lcd, lcd->status_dispctrl);
			udelay(HD_CMD_DURATION);
			mutex_unlock(&lcd->status_lock);
			break;
		case LCD_CTRLBLINK:
			mutex_lock(&lcd->status_lock);
			if(val)
				lcd->status_dispctrl |= HD_CMD_DISPCTRL_CURSOR_BLINK;
			else
				lcd->status_dispctrl &= ~HD_CMD_DISPCTRL_CURSOR_BLINK;
			hd44780_4bit_command(lcd, lcd->status_dispctrl);
			udelay(HD_CMD_DURATION);
			mutex_unlock(&lcd->status_lock);
			break;
		case LCD_CURSSHIFTLEFT:
			for(i = 0; i < val; ++i) {
				hd44780_4bit_command(lcd, HD_CMD_SHIFT);
				udelay(HD_CMD_DURATION);
			}
			break;
		case LCD_CURSSHIFTRIGHT:
			for(i = 0; i < val; ++i) {
				hd44780_4bit_command(lcd, HD_CMD_SHIFT | HD_CMD_SHIFT_RIGHT);
				udelay(HD_CMD_DURATION);
			}
			break;
		case LCD_DISPSHIFTLEFT:
			for(i = 0; i < val; ++i) {
				hd44780_4bit_command(lcd, HD_CMD_SHIFT | HD_CMD_SHIFT_DISPLAY);
				udelay(HD_CMD_DURATION);
			}
			break;
		case LCD_DISPSHIFTRIGHT:
			for(i = 0; i < val; ++i) {
				hd44780_4bit_command(lcd, HD_CMD_SHIFT | HD_CMD_SHIFT_DISPLAY | HD_CMD_SHIFT_RIGHT);
				udelay(HD_CMD_DURATION);
			}
			break;
		case LCD_PRINT:
			/* parse line number */
			line = buf[0] - '0';
			if(line < 0 || line >= pdata->rows)
				return -EINVAL;
			/* Put buffer in appropriate line of the display */
			hd44780_4bit_print(lcd, line, buf + 1);
			break;
		case LCD_PRINTRAW:
			hd44780_4bit_rawprint(lcd, buf);
			break;
	}

	return count;
}

/* write only attributes */
static SENSOR_DEVICE_ATTR(lcd_clear,          S_IWUSR, NULL, hd44780_store, LCD_CLEAR);
static SENSOR_DEVICE_ATTR(lcd_home,           S_IWUSR, NULL, hd44780_store, LCD_HOME);
static SENSOR_DEVICE_ATTR(lcd_increment,      S_IWUSR, NULL, hd44780_store, LCD_ENTRYINCREMENT);
static SENSOR_DEVICE_ATTR(lcd_shift,          S_IWUSR, NULL, hd44780_store, LCD_ENTRYSHIFT);
static SENSOR_DEVICE_ATTR(lcd_displayon,      S_IWUSR, NULL, hd44780_store, LCD_CTRLDISP);
static SENSOR_DEVICE_ATTR(lcd_cursoron,       S_IWUSR, NULL, hd44780_store, LCD_CTRLCURS);
static SENSOR_DEVICE_ATTR(lcd_cursorblink,    S_IWUSR, NULL, hd44780_store, LCD_CTRLBLINK);
static SENSOR_DEVICE_ATTR(lcd_cursshiftleft,  S_IWUSR, NULL, hd44780_store, LCD_CURSSHIFTLEFT);
static SENSOR_DEVICE_ATTR(lcd_cursshiftright, S_IWUSR, NULL, hd44780_store, LCD_CURSSHIFTRIGHT);
static SENSOR_DEVICE_ATTR(lcd_dispshiftleft,  S_IWUSR, NULL, hd44780_store, LCD_DISPSHIFTLEFT);
static SENSOR_DEVICE_ATTR(lcd_dispshiftright, S_IWUSR, NULL, hd44780_store, LCD_DISPSHIFTRIGHT);
static SENSOR_DEVICE_ATTR(lcd_print,          S_IWUSR, NULL, hd44780_store, LCD_PRINT);
static SENSOR_DEVICE_ATTR(lcd_printraw,       S_IWUSR, NULL, hd44780_store, LCD_PRINTRAW);

/* read only attributes */
static SENSOR_DEVICE_ATTR(lcd_rows,           S_IRUGO, hd44780_show, NULL, LCD_ROWS);
static SENSOR_DEVICE_ATTR(lcd_cols,           S_IRUGO, hd44780_show, NULL, LCD_COLS);

static struct attribute *hd44780_attributes[] = {
	&sensor_dev_attr_lcd_clear          .dev_attr.attr,
	&sensor_dev_attr_lcd_home           .dev_attr.attr,
	&sensor_dev_attr_lcd_increment      .dev_attr.attr,
	&sensor_dev_attr_lcd_shift          .dev_attr.attr,
	&sensor_dev_attr_lcd_displayon      .dev_attr.attr,
	&sensor_dev_attr_lcd_cursoron       .dev_attr.attr,
	&sensor_dev_attr_lcd_cursorblink    .dev_attr.attr,
	&sensor_dev_attr_lcd_cursshiftleft  .dev_attr.attr,
	&sensor_dev_attr_lcd_cursshiftright .dev_attr.attr,
	&sensor_dev_attr_lcd_dispshiftleft  .dev_attr.attr,
	&sensor_dev_attr_lcd_dispshiftright .dev_attr.attr,
	&sensor_dev_attr_lcd_print          .dev_attr.attr,
	&sensor_dev_attr_lcd_printraw       .dev_attr.attr,
	&sensor_dev_attr_lcd_rows           .dev_attr.attr,
	&sensor_dev_attr_lcd_cols           .dev_attr.attr,
	NULL,
};

static struct attribute_group hd44780_defattr_group = {
	.attrs = hd44780_attributes,
};
/* ------------------ sysfs hooks END -------------------------- */

static int /*__init*/ hd44780_probe(struct platform_device *pdev)
{
	int ret;
	struct hd44780 *lcd;
	struct resource *res;

	lcd = kzalloc(sizeof(struct hd44780), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	lcd->dev = &pdev->dev;
	struct hd44780_platform_data * pdata = lcd->dev->platform_data;

	/* check platform data */
	if(!pdata) {
		printk("--- %s platform data is not defined\n", __func__);
		return -EINVAL;
	}
	else if(pdata->rows < 1 || pdata->rows > HD44780_MAX_ROWS) {
		printk("--- %s platform data rows outside of interval <1,%d>\n", __func__, HD44780_MAX_ROWS);
		return -EINVAL;
	}
	else if(pdata->cols < 1 || pdata->cols > HD44780_MAX_COLS) {
		printk("--- %s platform data cols outside of interval <1,%d>\n", __func__, HD44780_MAX_COLS);
		return -EINVAL;
	}

	mutex_init(&lcd->status_lock);

	hd44780_gpios[0].gpio = pdata->gpio_RS;
	hd44780_gpios[1].gpio = pdata->gpio_RW;
	hd44780_gpios[2].gpio = pdata->gpio_E;
	hd44780_gpios[3].gpio = pdata->gpio_DB4;
	hd44780_gpios[4].gpio = pdata->gpio_DB5;
	hd44780_gpios[5].gpio = pdata->gpio_DB6;
	hd44780_gpios[6].gpio = pdata->gpio_DB7;

	/* create gpio port mask */
	if(pdata->set_output8) {
		lcd->gpio_port8_mask = 0x0;
		lcd->gpio_port8_mask |= pdata->bit_RS;
		lcd->gpio_port8_mask |= pdata->bit_RW;
		lcd->gpio_port8_mask |= pdata->bit_E;
		lcd->gpio_port8_mask |= pdata->bit_DB4;
		lcd->gpio_port8_mask |= pdata->bit_DB5;
		lcd->gpio_port8_mask |= pdata->bit_DB6;
		lcd->gpio_port8_mask |= pdata->bit_DB7;
	}

	/* request GPIO's */
	ret = gpio_request_array(ARRAY_AND_SIZE(hd44780_gpios));
	if (ret < 0) {
		ret = -EIO;
		goto out_no_gpio;
	}

	platform_set_drvdata(pdev, lcd);

	/* Initialize the display */
	hd44780_4bit_init(lcd);
	dev_info(&pdev->dev, "initialized HD44780 LCD\n");

	/* Register sysfs hooks */
	ret = sysfs_create_group(&pdev->dev.kobj,
				 &hd44780_defattr_group);
	if (ret)
		goto out_no_sysfs;

	return 0;

out_no_sysfs:
	/* free GPIO's */
	gpio_free_array(ARRAY_AND_SIZE(hd44780_gpios));
out_no_gpio:
	kfree(lcd);
	return ret;
}

static int /*__exit*/ hd44780_remove(struct platform_device *pdev)
{
	struct hd44780 *lcd = platform_get_drvdata(pdev);

	/* unregister sysfs hooks */
	sysfs_remove_group(&pdev->dev.kobj, &hd44780_defattr_group);

	/* free GPIO's */
	gpio_free_array(ARRAY_AND_SIZE(hd44780_gpios));

	if (lcd)
		kfree(lcd);

	return 0;
}

static int hd44780_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hd44780 *lcd = platform_get_drvdata(pdev);

	/* Power the display off */
	hd44780_4bit_command(lcd, HD_CMD_DISPCTRL);
	return 0;
}

static int hd44780_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct hd44780 *lcd = platform_get_drvdata(pdev);

	/* Turn the display back on */
	hd44780_4bit_command(lcd, HD_CMD_DISPCTRL | HD_CMD_DISPCTRL_DISPLAY_ON);
	return 0;
}

static const struct dev_pm_ops hd44780_pm_ops = {
	.suspend = hd44780_suspend,
	.resume = hd44780_resume,
};

static struct platform_driver hd44780_driver = {
	.probe  = hd44780_probe,
	.remove	= hd44780_remove,
	.driver = {
		.name = DRIVERNAME,
		.owner = THIS_MODULE,
		.pm = &hd44780_pm_ops,
	},
	//.remove = __exit_p(hd44780_remove),
};

//module_platform_driver_probe(hd44780_driver, hd44780_probe);
module_platform_driver(hd44780_driver);

MODULE_AUTHOR("Linus Walleij <triad@df.lth.se>");
MODULE_DESCRIPTION("HD44780 LCD Driver");
MODULE_LICENSE("GPL v2");
