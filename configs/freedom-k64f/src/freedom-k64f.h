/************************************************************************************
 * configs/freedom-k64f/src/freedom-k64f.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIGS_FREEDOM_K64F_SRC_FREEDOM_K64F_H
#define __CONFIGS_FREEDOM_K64F_SRC_FREEDOM_K64F_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/kinetis/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* How many SPI modules does this chip support? The LM3S6918 supports 2 SPI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if KINETIS_NSPI < 1
#  undef CONFIG_KINETIS_SPI1
#  undef CONFIG_KINETIS_SPI2
#elif KINETIS_NSPI < 2
#  undef CONFIG_KINETIS_SPI2
#endif

/* FREEDOM-K64F GPIOs ****************************************************************/

#define GPIO_SD_CARDDETECT (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTE | PIN28)
#define GPIO_SD_WRPROTECT  (GPIO_PULLUP | PIN_PORTE | PIN27)

#define GPIO_SW1           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTA | PIN19)
#define GPIO_SW2           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTE | PIN26)

/* An RGB LED is connected through GPIO as shown below:
 *
 *   LED    K64
 *   ------ -------------------------------------------------------
 *   RED    PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT
 *   BLUE   PTB21/SPI2_SCK/FB_AD30/CMP1_OUT
 *   GREEN  PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB0_CLKIN
 */

#define GPIO_LED_R         (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE | PIN_PORTB | PIN22)
#define GPIO_LED_G         (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE | PIN_PORTB | PIN21)
#define GPIO_LED_B         (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE | PIN_PORTE | PIN26)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: k64_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the FREEDOM-K64F board.
 *
 ************************************************************************************/

void weak_function k64_spidev_initialize(void);

/************************************************************************************
 * Name: k64_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the FREEDOM-K64F board.
 *
 ************************************************************************************/

void weak_function k64_usbinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_FREEDOM_K64F_SRC_FREEDOM_K64F_H */
