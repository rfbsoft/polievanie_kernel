#ifndef _LINUX_MCP23017_H
#define _LINUX_MCP23017_H

#include <linux/types.h>
#include <linux/i2c.h>

/* platform data for the MCP23017 16-bit I2C I/O expander driver */

struct mcp23017_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* initial polarity inversion setting */
	uint16_t	invert;

	/* initial pullup registers setting */
	uint16_t	pullup;

	/* initial interrupt enable registers setting */
	uint16_t	inten;

	/* interrupt base */
	int		irq_base;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	const char	*const *names;
};

#endif /* _LINUX_MCP23017_H */
