/*
 * Copyright (c) 2020 Rubus Technologies Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Driver for Microchip PIC30 Interrupt Controller
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PIC30_INTC_
#define ZEPHYR_INCLUDE_DRIVERS_PIC30_INTC_

#include <zephyr/types.h>
#include <device.h>

/*
 * INTC Register Interface Base Addresses
 */

#define PIC30_INTC_BASE DT_INST_REG_ADDR(0)

/*
 * 0x000  Interrupt Status Flag Register Base
 */
#define PIC30_IFS_BASE      ((PIC30_INTC_BASE) + 0x00U)

/*
 * 0x020  Interrupt Enable Control Register Base
 */
#define PIC30_IEC_BASE      ((PIC30_INTC_BASE) + 0x20U)

/*
 * 0x040  Interrupt Priority Control Register Base
 */
#define PIC30_IPC_BASE      ((PIC30_INTC_BASE) + 0x40U)

/*
 * 0x0C0  Interrupt Control Regsiter 1
 */
#define PIC30_INTCON1       ((PIC30_INTC_BASE) + 0xC0U)

/*
 * 0x0C2  Interrupt Control Regsiter 1
 */
#define PIC30_INTCON2       ((PIC30_INTC_BASE) + 0xC2U)

/*
 * 0x0C4  Interrupt Control Regsiter 1
 */
#define PIC30_INTCON3       ((PIC30_INTC_BASE) + 0xC4U)

/*
 * 0x0C6  Interrupt Control Regsiter 1
 */
#define PIC30_INTCON4       ((PIC30_INTC_BASE) + 0xC6U)

/*
 * 0x0C6  Interrupt Control and Status Register
 */
#define PIC30_INTTREG       ((PIC30_INTC_BASE) + 0xC8U)

#ifndef _ASMLANGUAGE

/*
 * INTC Driver Interface Functions
 */

/**
 * @brief Initialise INTC driver
 *
 * @return 0 if successful
 */
int pic30_intc_init(void);

/**
 * @brief Enable interrupt
 *
 * @param irq interrupt ID
 */
void pic30_intc_irq_enable(unsigned int irq);

/**
 * @brief Disable interrupt
 *
 * @param irq interrupt ID
 */
void pic30_intc_irq_disable(unsigned int irq);

/**
 * @brief Check if an interrupt is enabled
 *
 * @param irq interrupt ID
 * @return Returns true if interrupt is enabled, false otherwise
 */
int pic30_intc_irq_is_enabled(unsigned int irq);

/**
 * @brief Set Interrupt Pending Flag
 *
 * @param irq interrupt ID
 */
void pic30_intc_irq_pending_set(unsigned int irq);

/**
 * @brief CLear Interrupt Pending Flag
 *
 * @param irq interrupt ID
 */
void pic30_intc_irq_pending_clear(unsigned int irq);

/**
 * @brief Check if an interrupt is pending
 *
 * @param irq interrupt ID
 * @return Returns true if interrupt is pending, false otherwise
 */
int pic30_intc_irq_is_pending(unsigned int irq);

/**
 * @brief Set interrupt priority
 *
 * @param irq interrupt ID
 * @param prio interrupt priority
 */
void pic30_intc_irq_set_priority(unsigned int irq, unsigned int prio);

/**
 * @brief Get active interrupt ID
 *
 * @return Returns the ID of an active interrupt
 */
uint8_t pic30_intc_get_active(void);

#endif /* !_ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_DRIVERS_PIC30_INTC_ */
