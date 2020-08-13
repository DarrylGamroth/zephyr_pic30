/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <drivers/pinmux.h>
#include <soc.h>

static int dm330030_pinmux_init(struct device *dev)
{
	ARG_UNUSED(dev);

#if 0
	struct device *p = device_get_binding(CONFIG_PINMUX_DSPIC33C_0_NAME);
#endif

#ifdef CONFIG_UART_DSPIC33C
#ifdef CONFIG_UART_DSPIC33C_PORT_0
	/* UART0 RX */
	/* UART0 TX */
#endif /* CONFIG_UART_DSPIC33C_PORT_0 */
#endif /* CONFIG_UART_DSPIC33C */

#ifdef CONFIG_SPI_DSPIC33C
	/* SPI1 */
#endif /* CONFIG_SPI_DSPIC33C */

#ifdef CONFIG_I2C_DSPIC33C
	/* I2C 0 */
#endif /* CONFIG_I2C_DSPIC33C */

	return 0;
}

SYS_INIT(dm330030_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
