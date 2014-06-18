#ifndef _LINUX_MCP23017_H
#define _LINUX_MCP23017_H

#include <linux/types.h>
#include <linux/i2c.h>

uint8_t mcp23017_get_reg8(struct device *dev, unsigned reg_offset);
void mcp23017_set_reg8(struct device *dev, unsigned reg_offset, uint8_t val, uint8_t mask);

uint8_t mcp23017_get_input8A(struct device *dev);
uint8_t mcp23017_get_input8B(struct device *dev);
void mcp23017_set_output8A(struct device *dev, uint8_t val, uint8_t mask);
void mcp23017_set_output8B(struct device *dev, uint8_t val, uint8_t mask);

/* platform data for the MCP23017 16-bit I2C I/O expander driver */
struct mcp23017_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* initial pullup registers setting */
	uint16_t	pullup;

	/* initial interrupt enable registers setting */
	uint16_t	inten;

	/* INTA & INTB signals should be mirrored */
	bool		int_mirror;

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
