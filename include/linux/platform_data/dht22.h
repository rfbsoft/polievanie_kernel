/*
 * dht22.h - support for the SHT15 Temperature and Humidity Sensor
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

/**
 * struct dht22_platform_data - dht22 connectivity info
 * @gpio_data:		no. of gpio to which bidirectional data line is
 *			connected.
 * @checksum:		flag to indicate the checksum should be validated.
 */
struct dht22_platform_data {
	int gpio_data;
	bool checksum;
};
