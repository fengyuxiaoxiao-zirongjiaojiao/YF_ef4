/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * PX4YF-ef4 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>


#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

#undef TRACE_PINS

/* PX4IO connection configuration */

#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS4"
#define PX4IO_SERIAL_TX_GPIO           GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_USART6_RX
#define PX4IO_SERIAL_BASE              STM32_USART6_BASE
#define PX4IO_SERIAL_VECTOR            STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP         DMAMAP_USART6_TX
#define PX4IO_SERIAL_RX_DMAMAP         DMAMAP_USART6_RX
#define PX4IO_SERIAL_RCC_REG           STM32_RCC_APB2ENR
#define PX4IO_SERIAL_RCC_EN            RCC_APB2ENR_USART6EN
#define PX4IO_SERIAL_CLOCK             STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

/* PX4FMU GPIOs ***********************************************************************************/


/* LEDs are driven with push pull Anodes to 3.3V */

#define GPIO_nLED_RED        /* PG6 */  GPIO_RGB_RED
#define GPIO_nLED_BLUE       /* PG5 */  GPIO_RGB_BLUE
#define GPIO_nLED_GREEN      /* PG7 */  GPIO_RGB_GREEN

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)

/* N.B. there is no offset mapping needed for ADC3 because */
#define ADC3_CH(n)                  (n)

/* We are only use ADC3 for REV/VER. */

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */

#define PX4_ADC_GPIO  \
	/* PC0  */  GPIO_AD_RSSI,   \
	/* PC1  */  GPIO_AD_T1,   \
	/* PC2_C*/  GPIO_AD_VBN,   \
	/* PC3_C*/  GPIO_AD_VBP, \
	/* PF3  */  GPIO_AD_T3, \
	/* PF4  */  GPIO_AD_T2,  \
	/* PF5  */  GPIO_AD_IB,   \
	/* PF6  */  GPIO_AD_V24, \
	/* PF7  */  GPIO_AD_I24, \
	/* PF8  */  GPIO_AD_I7,   \
	/* PF9  */  GPIO_AD_V7P, \
	/* PF10 */  GPIO_AD_V7N \

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY1_CURRENT_CHANNEL            /* PF7 */  ADC3_CH(3)
#define ADC_BATTERY1_VOLTAGE_CHANNEL            /* PF6 */  ADC3_CH(8)
#define ADC_BATTERY2_CURRENT_CHANNEL            /* PF5 */  ADC3_CH(4)
#define ADC_BATTERY2_VOLTAGE_CHANNEL            /*PC3_C*/  ADC3_CH(1)


#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_CURRENT_CHANNEL) | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY1_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL))

#define HW_REV_VER_ADC_BASE STM32_ADC3_BASE

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* HEATER
 * PWM in future
 */
//TODO:没有设计心跳引脚
// #define GPIO_HEATER_OUTPUT   /* PB9  T17CH1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)
// #define HEATER_OUTPUT_EN(on_true)	       px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   8


/* Power supply control and monitoring GPIOs */

// #define GPIO_nPOWER_IN_A                /* PA15  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN15)
// #define GPIO_nPOWER_IN_B                /* PB12  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN12)
// #define GPIO_nPOWER_IN_C                /* PE15  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)

// #define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
// #define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
// #define BOARD_NUMBER_BRICKS             2
// #define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

// #define GPIO_VDD_5V_PERIPH_nEN          /* PE2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
// #define GPIO_VDD_5V_PERIPH_nOC          /* PE3  */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTE|GPIO_PIN3)
// #define GPIO_VDD_5V_HIPOWER_nEN         /* PC10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN10)
// #define GPIO_VDD_5V_HIPOWER_nOC         /* PC11 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTC|GPIO_PIN11)
// #define GPIO_VDD_3V3_SENSORS_EN         /* PB2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)

/* Define True logic Power Control in arch agnostic form */

// #define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_nEN, !(on_true))
// #define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_nEN, !(on_true))
// #define VDD_3V3_SENSORS_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS4_EN, (on_true))

/* Tone alarm output */
#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER
#define GPIO_TONE_ALARM         GPIO_BUZZER

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

// /* High-resolution timer */
// #define HRT_TIMER               8  /* use timer8 for the HRT */
// #define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

// /* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 3 */
// #define PWMIN_TIMER                       4
// #define PWMIN_TIMER_CHANNEL    /* T4C3 */ 3
// #define GPIO_PWM_IN            /* PD14 */ GPIO_TIM4_CH3IN_2

// #define SDIO_SLOTNO                    0  /* Only one slot */
// #define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
// #define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))

// /* FMUv6C never powers off the Servo rail */

// #define BOARD_ADC_SERVO_VALID     (1)

// #define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
// #define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))

// #define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_nOC))
// #define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_nOC))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_LED,		     	  \
		GPIO_BUZZER,		     	  \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_RGB_BLUE,	      		  \
		GPIO_RGB_GREEN,		     	  \
		GPIO_RGB_RED,		     	  \
		GPIO_NAVLIGHT_RIGHT,              \
		GPIO_NAVLIGHT_LEFT,               \
		GPIO_IMU_POWER,                   \
		GPIO_IMU_LED,                     \
		GPIO_IMU_HEATER,                  \
		GPIO_TONE_ALARM_IDLE,             \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_I2C_BUS_MTD      4,5

#define BOARD_OVERRIDE_I2C_DEVICE_EXTERNAL


#define BOARD_NUM_IO_TIMERS 5

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
