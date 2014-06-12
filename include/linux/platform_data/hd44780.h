/*
 * HD44780 GPIO LCDdriver
 *
 * Licensed under the GPL-2.
 */

#ifndef __LINUX_PLATFORM_DATA_HD44780_H__
#define __LINUX_PLATFORM_DATA_HD44780_H__

/**
 * struct hd44780_platform_data - Platform data for the HD44780 LCD driver
 **/
struct hd44780_platform_data {
	int gpio_RS;
	int gpio_RW;
	int gpio_E;
	int gpio_DB7;
	int gpio_DB6;
	int gpio_DB5;
	int gpio_DB4;
};

#endif /* __LINUX_PLATFORM_DATA_HD44780_H__ */
