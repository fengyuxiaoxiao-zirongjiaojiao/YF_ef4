/************************************************************************************
 * boards/YF/ef4/nuttx-config/include/board.h
 *
 *   Copyright (C) 2016-2019 Gregory Nutt. All rights reserved.
 *   Authors: David Sidrane <david.sidrane@nscdg.com>
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
#ifndef __BOARDS_ARM_YF_EF4_INCLUDE_BOARD_H
#define __BOARDS_ARM_YF_EF4_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include "board_dma_map.h"

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32_sdmmc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* Clocking Configuration for EF4 Core Board */
/* The EF4 Core board provides:
 *   X1: 16 MHz crystal for HSE
 *   X2: 32.768 kHz crystal for LSE (RTC)
 */

#define STM32_BOARD_XTAL        16000000ul	/* 16 MHz crystal */
#define STM32_RTC_XTAL          32768    	/* 32.768 kHz crystal for RTC */

#define STM32_LSI_FREQUENCY     32000    	/* 32 kHz internal RC */
#define STM32_HSI_FREQUENCY     64000000ul 	/* 64 MHz internal RC */
#define STM32_CSI_FREQUENCY     4000000ul  	/* 4 MHz internal RC */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     STM32_RTC_XTAL

/* Main PLL Configuration */
#define STM32_BOARD_USEHSE	/* Use HSE as the PLL source */
#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* RTC Configuration */
#define STM32_RTCSEL           RCC_BDCR_RTCSEL_LSE  /* LSE as RTC source */
#define STM32_RTC_FREQUENCY    STM32_LSE_FREQUENCY  /* 32.768 kHz */

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (16,000,000 / 1) * 60 = 960 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 960 MHz / 2   = 480 MHz
 *   PLL1Q = PLL1_VCO/4  = 960 MHz / 4   = 240 MHz
 *   PLL1R = PLL1_VCO/8  = 960 MHz / 8   = 120 MHz
 */
#define STM32_PLLCFG_PLL1CFG    (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                                 RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
                                 RCC_PLLCFGR_DIVP1EN | \
                                 RCC_PLLCFGR_DIVQ1EN | \
                                 RCC_PLLCFGR_DIVR1EN)

#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(1)    /* HSE not divided */
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(60)       /* Multiply by 60 */
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)        /* SYSCLK = 480 MHz */
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(12)       /* 80 MHz for FDCAN */
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(2)        /* 480 MHz for TRACE */

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 1) * 60)  /* 960 MHz */
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)        /* 480 MHz */
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 12)       /* 80 MHz */
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)        /* 480 MHz */

/* PLL2 */

#define STM32_PLLCFG_PLL2CFG     (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
				  RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
				  RCC_PLLCFGR_DIVP2EN | \
				  RCC_PLLCFGR_DIVQ2EN | \
				  RCC_PLLCFGR_DIVR2EN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)    /* HSE divided by 2 */
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(32)       /* Multiply by 32 */
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(8)        /* 32 MHz for SPI/ADC */
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(8)        /* 32 MHz */
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(2)        /* 128 MHz for SDMMC */

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 32)  /* 256 MHz */
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 8)        /* 32 MHz */
#define STM32_PLL2Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 8)        /* 32 MHz */
#define STM32_PLL2R_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)        /* 128 MHz */

/* PLL3 Configuration - USB and Other Peripherals */
#define STM32_PLLCFG_PLL3CFG    (RCC_PLLCFGR_PLL3VCOSEL_WIDE | \
                                 RCC_PLLCFGR_PLL3RGE_4_8_MHZ | \
                                 RCC_PLLCFGR_DIVP3EN | \
                                 RCC_PLLCFGR_DIVQ3EN | \
                                 RCC_PLLCFGR_DIVR3EN)
#define STM32_PLLCFG_PLL3M      RCC_PLLCKSELR_DIVM3(4)    /* HSE divided by 4 */
#define STM32_PLLCFG_PLL3N      RCC_PLL3DIVR_N3(40)       /* Multiply by 40 */
#define STM32_PLLCFG_PLL3P      RCC_PLL3DIVR_P3(4)        /* 40 MHz */
#define STM32_PLLCFG_PLL3Q      RCC_PLL3DIVR_Q3(2)        /* 80 MHz */
#define STM32_PLLCFG_PLL3R      RCC_PLL3DIVR_R3(2)        /* 80 MHz */

#define STM32_VCO3_FREQUENCY    ((STM32_HSE_FREQUENCY / 4) * 40)  /* 160 MHz */
#define STM32_PLL3P_FREQUENCY   (STM32_VCO3_FREQUENCY / 4)       /* 40 MHz */
#define STM32_PLL3Q_FREQUENCY   (STM32_VCO3_FREQUENCY / 2)       /* 80 MHz */
#define STM32_PLL3R_FREQUENCY   (STM32_VCO3_FREQUENCY / 2)       /* 80 MHz */

/* System Clock Configuration */
#define STM32_SYSCLK_SOURCE     RCC_CFGR_SW_PLL1         /* PLL1 as system clock */
#define STM32_SYSCLK_FREQUENCY  STM32_PLL1P_FREQUENCY    /* 480 MHz */
#define STM32_CPUCLK_FREQUENCY  STM32_SYSCLK_FREQUENCY   /* 480 MHz (no divider) */

/* D1 domain Core prescaler - sets the CPU clock divider */
#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)

/* Configure Clock Assignments */

/* Bus Clock Dividers */
#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLK	   /* AHB = 480 MHz (no divider) */
#define STM32_HCLK_FREQUENCY    STM32_CPUCLK_FREQUENCY     /* 480 MHz */
#define STM32_ACLK_FREQUENCY    STM32_HCLK_FREQUENCY       /* 480 MHz */

/* APB1 clock (PCLK1) is HCLK/4 (120 MHz) */
#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd4  /* APB1 = 120 MHz */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB2 clock (PCLK2) is HCLK/2 (240 MHz) */
#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd2  /* APB2 = 240 MHz */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB3 clock (PCLK3) is HCLK/4 (120 MHz) */
#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd4   /* APB3 = 120 MHz */
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB4 clock (PCLK4) is HCLK/4 (120 MHz) */
#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd4   /* APB4 = 120 MHz */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* Timer clock frequencies */

/* Timers driven from APB1 will be twice PCLK1 */

/* Timer Clocks (APB timers run at 2x PCLK when APB divider > 1) */
#define STM32_APB1_TIM2_CLKIN    (2*STM32_PCLK1_FREQUENCY)   /* 240 MHz */
#define STM32_APB1_TIM3_CLKIN    (2*STM32_PCLK1_FREQUENCY)   /* 240 MHz */
#define STM32_APB1_TIM4_CLKIN    (2*STM32_PCLK1_FREQUENCY)   /* 240 MHz */
#define STM32_APB1_TIM5_CLKIN    (2*STM32_PCLK1_FREQUENCY)   /* 240 MHz */
#define STM32_APB1_TIM6_CLKIN    (2*STM32_PCLK1_FREQUENCY)   /* 240 MHz */
#define STM32_APB1_TIM7_CLKIN    (2*STM32_PCLK1_FREQUENCY)   /* 240 MHz */
#define STM32_APB1_TIM12_CLKIN   (2*STM32_PCLK1_FREQUENCY)   /* 240 MHz */
#define STM32_APB1_TIM13_CLKIN   (2*STM32_PCLK1_FREQUENCY)   /* 240 MHz */
#define STM32_APB1_TIM14_CLKIN   (2*STM32_PCLK1_FREQUENCY)   /* 240 MHz */

/* Timers driven from APB2 will be twice PCLK2 */
#define STM32_APB2_TIM1_CLKIN    (2*STM32_PCLK2_FREQUENCY)   /* 480 MHz */
#define STM32_APB2_TIM8_CLKIN    (2*STM32_PCLK2_FREQUENCY)   /* 480 MHz */
#define STM32_APB2_TIM15_CLKIN   (2*STM32_PCLK2_FREQUENCY)   /* 480 MHz */
#define STM32_APB2_TIM16_CLKIN   (2*STM32_PCLK2_FREQUENCY)   /* 480 MHz */
#define STM32_APB2_TIM17_CLKIN   (2*STM32_PCLK2_FREQUENCY)   /* 480 MHz */


/* Peripheral Clock Assignments */
/* USART */
#define STM32_USART1_CLKSRC     STM32_PCLK2_FREQUENCY  /* 240 MHz */
#define STM32_USART6_CLKSRC     STM32_PCLK2_FREQUENCY  /* 240 MHz */
#define STM32_USART2_CLKSRC     STM32_PCLK1_FREQUENCY  /* 120 MHz */
#define STM32_USART3_CLKSRC     STM32_PCLK1_FREQUENCY  /* 120 MHz */
#define STM32_UART4_CLKSRC      STM32_PCLK1_FREQUENCY  /* 120 MHz */
#define STM32_UART5_CLKSRC      STM32_PCLK1_FREQUENCY  /* 120 MHz */
#define STM32_USART7_CLKSRC     STM32_PCLK1_FREQUENCY  /* 120 MHz */
#define STM32_USART8_CLKSRC     STM32_PCLK1_FREQUENCY  /* 120 MHz */
#define STM32_LPUART1_CLKSRC    STM32_PCLK3_FREQUENCY  /* 120 MHz */

/* Kernel Clock Configuration
 *
 * Note: look at Table 54 in ST Manual
 */


/* I2C123 clock source */

#define STM32_RCC_D2CCIP2R_I2C123SRC STM32_PCLK1_FREQUENCY  /* 120 MHz */

/* I2C4 clock source */

#define STM32_RCC_D3CCIPR_I2C4SRC    STM32_PCLK4_FREQUENCY  /* 120 MHz */

/* SPI123 clock source */

#define STM32_RCC_D2CCIP1R_SPI123SRC STM32_PLL2P_FREQUENCY  /* 32 MHz */

/* SPI45 clock source */

#define STM32_RCC_D2CCIP1R_SPI45SRC  STM32_PCLK2_FREQUENCY  /* 240 MHz */

/* SPI6 clock source */

#define STM32_RCC_D3CCIPR_SPI6SRC    STM32_PLL2Q_FREQUENCY  /* 32 MHz */

/* USB 1 and 2 clock source */

#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_HSI48  /* 48 MHz HSI48 */
#define CONFIG_STM32H7_HSI48       48000000

/* ADC 1 2 3 clock source */
#define STM32_RCC_D3CCIPR_ADCSRC     STM32_PLL2P_FREQUENCY  /* 32 MHz */

/* FDCAN 1 2 clock source */
#define STM32_RCC_D2CCIP1R_FDCANSEL  STM32_PLL1Q_FREQUENCY 	/* 80MHz */
#define STM32_FDCANCLK               STM32_PLL1Q_FREQUENCY 	/* 80 MHz */

/* TRACE */
#define STM32_TRACE_CLKSRC      STM32_PLL1R_FREQUENCY  /* 480 MHz */

/* HRTIM */
#define STM32_HRTIM_CLKSRC      STM32_APB1_TIM2_CLKIN  /* 240 MHz */


/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *  1.05-1.15 V     55 MHz    0
 *  (VOS2 level)   110 MHz    1
 *                 165 MHz    2
 *                 220 MHz    3
 *  0.95-1.05 V     45 MHz    0
 *  (VOS3 level)    90 MHz    1
 *                 135 MHz    2
 *                 180 MHz    3
 *                 225 MHz    4
 *  ------------ ---------- -----------
 */

/* 配置FLASH等待状态数，当系统时钟为480MHz时需要设置为4 */
/* FLASH等待状态配置 */
#define STM32_VOS                  SCB_ACCR_VOS1  /* 设置为VOS1级别(1.15-1.26V) */
#define BOARD_FLASH_WAITSTATES     4              /* 480MHz ACLK在VOS1级别需要4等待状态 */

/* ADC通道定义 */
/* ADC Channels *************************************************************/

/* Single-ended ADC inputs */
#define GPIO_AD_RSSI     GPIO_ADC123_INP10       /* PC0  - ADC3_IN10 (RSSI信号强度) */
#define GPIO_AD_T1       GPIO_ADC123_INP11       /* PC1  - ADC3_IN11 (电源板温度采样) */
#define GPIO_AD_T3       GPIO_ADC3_INP5          /* PF3  - ADC3_IN5 (外置温度采样1) */
#define GPIO_AD_T2       GPIO_ADC3_INP9          /* PF4  - ADC3_IN9 (外置温度采样2) */
#define GPIO_AD_IB       GPIO_ADC3_INP4          /* PF5  - ADC3_IN4 (电池电流霍尔采样) */
#define GPIO_AD_V24      GPIO_ADC3_INP8          /* PF6  - ADC3_IN8 (系统电压采样) */
#define GPIO_AD_I24      GPIO_ADC3_INP3          /* PF7  - ADC3_IN3 (系统电流采样) */
#define GPIO_AD_I7       GPIO_ADC3_INP7          /* PF8  - ADC3_IN7 (舵机电流采样) */
#define GPIO_AD_V7P      GPIO_ADC3_INP2          /* PF9  - ADC3_IN2 (舵机电压正采样) */
#define GPIO_AD_V7N      GPIO_ADC3_INN2          /* PF10 - ADC3_IN2 (舵机电压负采样) */

/* Differential ADC inputs */
#define GPIO_AD_VBN      (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN2)          /* PC2_C - ADC3_IN1N (电池电压负采样) */
#define GPIO_AD_VBP      (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN3)         /* PC3_C - ADC3_IN1P (电池电压正采样) */

/* 电源板状态指示灯 */
#define GPIO_LED        (GPIO_OUTPUT | GPIO_PULLUP | GPIO_OUTPUT_SET | GPIO_SPEED_2MHz | \
                        GPIO_PORTC | GPIO_PIN13)  /* PC13 - 电源板指示灯 */

/* PWM输出通道 **************************************************************/

/* 旋翼电调驱动PWM (TIM3) */
#define GPIO_PWM1       GPIO_TIM3_CH2OUT_1       /* PA7  - TIM3_CH2 (GND与电池GND直通) */
#define GPIO_PWM2       GPIO_TIM3_CH3OUT_1       /* PB0  - TIM3_CH3 */
#define GPIO_PWM3       GPIO_TIM3_CH4OUT_1       /* PB1  - TIM3_CH4 */
#define GPIO_PWM4       GPIO_TIM3_CH1OUT_2       /* PB4  - TIM3_CH1 */

/* 舵机驱动PWM (TIM2) */
#define GPIO_PWM5       GPIO_TIM2_CH1OUT_3       /* PA5  - TIM2_CH1 (GND与舵机电压GND直通) */
#define GPIO_PWM6       GPIO_TIM2_CH2OUT_2       /* PB3  - TIM2_CH2 */
#define GPIO_PWM7       GPIO_TIM2_CH3OUT_2       /* PB10 - TIM2_CH3 */
#define GPIO_PWM8       GPIO_TIM2_CH4OUT_2       /* PB11 - TIM2_CH4 */

/* 舵机驱动PWM (TIM4) */
#define GPIO_PWM9       GPIO_TIM4_CH4OUT_2       /* PD15 - TIM4_CH4 (GND与舵机电压GND直通) */
#define GPIO_PWM10      GPIO_TIM4_CH3OUT_2       /* PD14 - TIM4_CH3 */
#define GPIO_PWM11      GPIO_TIM4_CH2OUT_2       /* PD13 - TIM4_CH2 */
#define GPIO_PWM12      GPIO_TIM4_CH1OUT_2       /* PD12 - TIM4_CH1 */

/* 5V外置驱动PWM (TIM1) - 用于非电调类设备 */
#define GPIO_PWM13      GPIO_TIM1_CH4OUT_2       /* PE14 - TIM1_CH4 (GND与系统GND直通) */
#define GPIO_PWM14      GPIO_TIM1_CH3OUT_2       /* PE13 - TIM1_CH3 */
#define GPIO_PWM15      GPIO_TIM1_CH2OUT_2       /* PE11 - TIM1_CH2 */
#define GPIO_PWM16      GPIO_TIM1_CH1OUT_2       /* PE9  - TIM1_CH1 */

/* 蜂鸣器PWM (TIM13) */
#define GPIO_BUZZER       GPIO_TIM13_CH1OUT_1    /* PA6 - TIM13_CH1 */

/* SD引脚配置 (原理图第6页) */
#define GPIO_SDIO_CK    GPIO_SDMMC1_CK     /* PC12 - SDIO_CK */
#define GPIO_SDIO_CMD   GPIO_SDMMC1_CMD    /* PD2 - SDIO_CMD */
#define GPIO_SDIO_D0    GPIO_SDMMC1_D0     /* PC8 - SDIO_D0 */
#define GPIO_SDIO_D1    GPIO_SDMMC1_D1     /* PC9 - SDIO_D1 */
#define GPIO_SDIO_D2    GPIO_SDMMC1_D2     /* PC10 - SDIO_D2 */
#define GPIO_SDIO_D3    GPIO_SDMMC1_D3     /* PC11 - SDIO_D3 */

/* USB引脚配置 */
#define GPIO_USB_DM     GPIO_OTGFS_DM    /* PA11 - USB_DM (Full-Speed) */
#define GPIO_USB_DP     GPIO_OTGFS_DP    /* PA12 - USB_DP (Full-Speed) */

/* CAN1 配置 */
#define GPIO_CAN1_RX    GPIO_CAN1_RX_3    /* PD0 - CAN1_RX */
#define GPIO_CAN1_TX    GPIO_CAN1_TX_3    /* PD1 - CAN1_TX */

/* CAN2 配置 */
#define GPIO_CAN2_RX    GPIO_CAN2_RX_1    /* PB12 - CAN2_RX */
#define GPIO_CAN2_TX    GPIO_CAN2_TX_1    /* PB13 - CAN2_TX */

/* USART1 引脚定义 (默认调试串口) */
#define GPIO_USART1_TX    GPIO_USART1_TX_2    /* PA9  - USART1发送引脚 */
#define GPIO_USART1_RX    GPIO_USART1_RX_2    /* PA10 - USART1接收引脚 */

/* USART2 引脚定义 */
#define GPIO_USART2_TX    GPIO_USART2_TX_2    /* PD5 - USART2发送引脚 */
#define GPIO_USART2_RX    GPIO_USART2_RX_2    /* PD6 - USART2接收引脚 */

/* UART5 引脚定义 */
// #define GPIO_UART5_TX     GPIO_UART5_TX_2     /* PB6 - UART5发送引脚 */
// #define GPIO_UART5_RX     GPIO_UART5_RX_2     /* PB5 - UART5接收引脚 */

/* UART7 引脚定义 (RTK串口) */
#define GPIO_UART7_RX     GPIO_UART7_RX_3     /* PE7 - UART7接收引脚 */
#define GPIO_UART7_TX     GPIO_UART7_TX_3     /* PE8 - UART7发送引脚 */

/* 422通信引脚定义 (基于USART3) */
#define GPIO_422_TX      GPIO_USART3_TX_3    /* PD8 - 422发送引脚 (USART3_TX) */
#define GPIO_422_RX      GPIO_USART3_RX_3    /* PD9 - 422接收引脚 (USART3_RX) */

/* 备用串口2(UART4 引脚定义) */
#define GPIO_UART4_TX     GPIO_UART4_TX_2     /* PA0 - UART4发送引脚 */
#define GPIO_UART4_RX     GPIO_UART4_RX_2     /* PA1 - UART4接收引脚 */

/* ADS UART 引脚定义 */
#define GPIO_ADS_RX      GPIO_UART5_RX_2     /* PB5 - ADS发送引脚 (UART5_RX) */
#define GPIO_ADS_TX      GPIO_UART5_TX_2     /* PB6 - ADS接收引脚 (UART5_TX) */

/* 备用串口1(USART6 引脚定义) */
#define GPIO_USART6_TX    GPIO_USART6_TX_1    /* PC6 - USART6发送引脚 */
#define GPIO_USART6_RX    GPIO_USART6_RX_1    /* PC7 - USART6接收引脚 */

/* RTK UART 引脚定义 (UART7) */
#define GPIO_RTK_RX      GPIO_UART7_RX_3     /* PE7 - RTK发送引脚 (UART7_RX) */
#define GPIO_RTK_TX      GPIO_UART7_TX_3     /* PE8 - RTK接收引脚 (UART7_TX) */

/* RTK状态引脚 */
#define GPIO_RTK_PPS        (GPIO_INPUT | GPIO_PULLUP | GPIO_EXTI | GPIO_PORTG | GPIO_PIN4) /* 秒脉冲输入 */
#define GPIO_RTK_PVT_STAT   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTG | GPIO_PIN3)  /* 位置有效性指示(高电平有效) */
#define GPIO_RTK_RTK_STA    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTG | GPIO_PIN2)  /* RTK定位状态指示(高电平有效) */
#define GPIO_RTK_ERR_STAT   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTD | GPIO_PIN11) /* 异常状态指示 */

// /* RTK中断配置 */
// #define GPIO_RTK_PPS_IRQ    (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN4)   /* 秒脉冲中断线 */
// #define GPIO_RTK_PVT_IRQ    (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN3)   /* 位置有效性中断线 */
// #define GPIO_RTK_RTK_IRQ    (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN2)   /* RTK状态中断线 */
// #define GPIO_RTK_ERR_IRQ    (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN11)  /* 错误状态中断线 */

// /* 秒脉冲(PPS)配置 */
// #define CONFIG_RTK_PPS_ENABLED      1       /* 启用秒脉冲支持 */
// #define CONFIG_RTK_PPS_GPIO         (GPIO_PORTG | GPIO_PIN4) /* PG4作为PPS输入 */
// #define CONFIG_RTK_PPS_IRQ          GPIO_RTK_PPS_IRQ  /* 使用EXTI4中断线 */
// #define CONFIG_RTK_PPS_PRIORITY     5       /* 中断优先级 */

// /* RTK状态LED配置(可选) */
// #define GPIO_RTK_LED_READY  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
//                             GPIO_OUTPUT_CLEAR | GPIO_PORTX | GPIO_PINX) /* 准备就绪LED */
// #define GPIO_RTK_LED_FIX    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
//                             GPIO_OUTPUT_CLEAR | GPIO_PORTX | GPIO_PINX) /* 定位锁定LED */

// /* RTK电源控制(可选) */
// #define GPIO_RTK_PWR_EN    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
//                            GPIO_OUTPUT_CLEAR | GPIO_PORTX | GPIO_PINX) /* 电源使能控制 */

// /* RTK复位控制(可选) */
// #define GPIO_RTK_RESET     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
//                            GPIO_OUTPUT_SET | GPIO_PORTX | GPIO_PINX) /* 复位控制引脚 */


/****************************************************************************
 * Sbus (串行总线) 配置
 ****************************************************************************/

/* Sbus UART引脚配置 */
#define GPIO_SBUS    	(GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN1)  /* PE1 - UART8 TX引脚 (实际配置为接收模式) */

/****************************************************************************
 * SPI1 Configuration - Updated Pinout
 ****************************************************************************/

/* SPI1引脚定义 */
#define GPIO_SPI1_SCK    (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_2MHz | GPIO_PORTG | GPIO_PIN11)
#define GPIO_SPI1_MISO   (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_2MHz | GPIO_PORTG | GPIO_PIN9)
#define GPIO_SPI1_MOSI   (GPIO_ALT | GPIO_AF5 | GPIO_SPEED_2MHz | GPIO_PORTD | GPIO_PIN7)

/* 设备片选引脚 */
#define GPIO_FRAM_CS     (GPIO_OUTPUT | GPIO_FLOAT | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN8)  /* FM25V20A片选 */
#define GPIO_IMU_CS      (GPIO_OUTPUT | GPIO_FLOAT | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_SET | GPIO_PORTG | GPIO_PIN8)  /* ICM-20602片选 */

/****************************************************************************
 * I2C1 Configuration
 ****************************************************************************/

/* I2C1引脚定义 */
#define GPIO_I2C1_SCL    (GPIO_ALT | GPIO_AF4 | GPIO_FLOAT | GPIO_SPEED_2MHz | GPIO_PORTB | GPIO_PIN8)
#define GPIO_I2C1_SDA    (GPIO_ALT | GPIO_AF4 | GPIO_FLOAT | GPIO_SPEED_2MHz | GPIO_PORTB | GPIO_PIN7)

/* 中断引脚 */
#define GPIO_I2C1_EXIT   (GPIO_INPUT | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN10)  /* 空速计中断引脚 */

/* RGB灯 */
#define GPIO_RGB_BLUE   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN5)
#define GPIO_RGB_RED    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)
#define GPIO_RGB_GREEN  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN7)

/* 航灯 */
#define GPIO_NAVLIGHT_RIGHT  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN10)
#define GPIO_NAVLIGHT_LEFT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN15)

/* 输入引脚配置宏 */
#define GPIO_INPUT_EXT1   GPIO_TIM5_CH3IN_1  // PA2/TIM5_CH3
#define GPIO_INPUT_EXT2   GPIO_TIM5_CH4IN_1  // PA3/TIM5_CH4

/* Debug引脚配置宏 */
// #define GPIO_SWDIO       GPIO_SWDIO        /* PA13 */
// #define GPIO_SWCLK       GPIO_SWCLK        /* PA14 */

/* IMU电源控制 */
#define GPIO_IMU_POWER   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN15)
/* IMU指示灯 */
#define GPIO_IMU_LED     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN13)
/* IMU加热控制 */
#define GPIO_IMU_HEATER  GPIO_TIM17_CH1OUT_1       /* PB9 */
/* ----- RM3100磁力计 ----- */
#define RM3100_CS_PIN       	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12) /* PE12 - RM3100片选引脚 */
#define RM3100_SPI2_SCK		GPIO_SPI2_SCK_5	/* PD3 - SPI2_SCK */
#define RM3100_SPI2_MISO        GPIO_SPI2_MISO_1 /* PB14 - SPI2_MISO */
#define RM3100_SPI2_MOSI        GPIO_SPI2_MOSI_1 /* PB15 - SPI2_MOSI */
#define RM3100_EXIT		(GPIO_INPUT|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN15) /* PF15 - RM3100中断引脚 */
/* ----- ICM-42688P-1陀螺仪1 ----- */
#define ICM42688_1_CS_PIN   	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN1) /* PG1 - ICM42688-1片选引脚 */
#define ICM42688_1_SPI4_SCK  	GPIO_SPI4_SCK_2 /* PE2 - SPI4_SCK */
#define ICM42688_1_SPI4_MISO 	GPIO_SPI4_MISO_2 /* PE5 - SPI4_MISO */
#define ICM42688_1_SPI4_MOSI 	GPIO_SPI4_MOSI_2 /* PE6 - SPI4_MOSI */
#define ICM42688_1_EXIT		(GPIO_INPUT|GPIO_PUSHPULL|GPIO_PORTF|GPIO_PIN14) /* PF14 - ICM42688-1中断引脚 */
/* ----- ICM-42688P-2陀螺仪2 ----- */
#define ICM42688_2_CS_PIN   	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN0) /* PG0 - ICM42688-2片选引脚 */
#define ICM42688_2_SPI6_SCK  	GPIO_SPI6_SCK_1 /* PG13 - SPI6_SCK */
#define ICM42688_2_SPI4_MISO 	GPIO_SPI6_MISO_1 /* PG12 - SPI6_MISO */
#define ICM42688_2_SPI4_MOSI 	GPIO_SPI6_MOSI_1 /* PG14 - SPI6_MOSI */
/* 没有引脚 - ICM42688-2中断引脚 */
/* ----- MS5611气压计(I2C) ----- */
#define MS5611_I2C2_SCL    	GPIO_I2C2_SCL_2  /* PF1 I2C2_SCL */
#define MS5611_I2C2_SDA    	GPIO_I2C2_SDA_2  /* PF0 I2C2_SDA */




/* Board provides GPIO or other Hardware for signaling to timing analyzer */

#if defined(CONFIG_BOARD_USE_PROBES)
# include "stm32_gpio.h"
# define PROBE_N(n) (1<<((n)-1))
# define PROBE_1    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)   /* PA8  AUX1 */
# define PROBE_2    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)  /* PE11 AUX2 */
# define PROBE_3    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)  /* PE13 AUX3 */
# define PROBE_4    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)  /* PE14 AUX4 */
# define PROBE_5    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)  /* PD14 AUX5 */
# define PROBE_6    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)  /* PD15 AUX6 */
# define PROBE_7    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)   /* PA0  AUX7 */
# define PROBE_8    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)   /* PA1  AUX8 */

# define PROBE_INIT(mask) \
	do { \
		if ((mask)& PROBE_N(1)) { stm32_configgpio(PROBE_1); } \
		if ((mask)& PROBE_N(2)) { stm32_configgpio(PROBE_2); } \
		if ((mask)& PROBE_N(3)) { stm32_configgpio(PROBE_3); } \
		if ((mask)& PROBE_N(4)) { stm32_configgpio(PROBE_4); } \
		if ((mask)& PROBE_N(5)) { stm32_configgpio(PROBE_5); } \
		if ((mask)& PROBE_N(6)) { stm32_configgpio(PROBE_6); } \
		if ((mask)& PROBE_N(7)) { stm32_configgpio(PROBE_7); } \
		if ((mask)& PROBE_N(8)) { stm32_configgpio(PROBE_8); } \
	} while(0)

# define PROBE(n,s)  do {stm32_gpiowrite(PROBE_##n,(s));}while(0)
# define PROBE_MARK(n) PROBE(n,false);PROBE(n,true)
#else
# define PROBE_INIT(mask)
# define PROBE(n,s)
# define PROBE_MARK(n)
#endif

#endif  /*__BOARDS_ARM_YF_EF4_INCLUDE_BOARD_H  */
