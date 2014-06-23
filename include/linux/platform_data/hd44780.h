/*
 * HD44780 GPIO LCDdriver
 *
 * Licensed under the GPL-2.
 */

#ifndef __LINUX_PLATFORM_DATA_HD44780_H__
#define __LINUX_PLATFORM_DATA_HD44780_H__

#define HD44780_MAX_ROWS	10
#define HD44780_MAX_COLS	80
/**
 * struct hd44780_platform_data - Platform data for the HD44780 LCD driver
 **/
struct hd44780_platform_data {
	/* display geometry */
	int rows;
	int cols;
	int row_offset[HD44780_MAX_ROWS];

	/* single GPIO's */
	int gpio_RS;
	int gpio_RW;
	int gpio_E;
	int gpio_DB7;
	int gpio_DB6;
	int gpio_DB5;
	int gpio_DB4;

	/* GPIO's in one 8-pin port of a GPIO expander */
	uint8_t bit_RS;
	uint8_t bit_RW;
	uint8_t bit_E;
	uint8_t bit_DB7;
	uint8_t bit_DB6;
	uint8_t bit_DB5;
	uint8_t bit_DB4;

	/* get & set functions of a 8-pin port of a GPIO expander */
	uint8_t (*get_input8)(struct device *dev);
	void (*set_output8)(struct device *dev, uint8_t val, uint8_t mask);
};

#endif /* __LINUX_PLATFORM_DATA_HD44780_H__ */
