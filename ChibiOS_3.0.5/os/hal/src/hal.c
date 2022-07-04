/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal.c
 * @brief   HAL subsystem code.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   HAL initialization.
 * @details This function invokes the low level initialization code then
 *          initializes all the drivers enabled in the HAL. Finally the
 *          board-specific initialization is performed by invoking
 *          @p boardInit() (usually defined in @p board.c).
 *
 * @init
 */
void halInit(void) {

  /* Initializes the OS Abstraction Layer.*/
  // osalInit();

  /* Platform low level initializations.*/
  hal_lld_init();



#if (HAL_USE_PAL == TRUE) || defined(__DOXYGEN__)

  const PALConfig pal_default_config = {
  #if STM32_HAS_GPIOA
    {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR,
    VAL_GPIOA_ODR,   VAL_GPIOA_AFRL,   VAL_GPIOA_AFRH},
  #endif
  #if STM32_HAS_GPIOB
    {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR,
    VAL_GPIOB_ODR,   VAL_GPIOB_AFRL,   VAL_GPIOB_AFRH},
  #endif
  #if STM32_HAS_GPIOC
    {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR,
    VAL_GPIOC_ODR,   VAL_GPIOC_AFRL,   VAL_GPIOC_AFRH},
  #endif
  #if STM32_HAS_GPIOD
    {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR,
    VAL_GPIOD_ODR,   VAL_GPIOD_AFRL,   VAL_GPIOD_AFRH},
  #endif
  #if STM32_HAS_GPIOE
    {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR,
    VAL_GPIOE_ODR,   VAL_GPIOE_AFRL,   VAL_GPIOE_AFRH},
  #endif
  #if STM32_HAS_GPIOF
    {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR,
    VAL_GPIOF_ODR,   VAL_GPIOF_AFRL,   VAL_GPIOF_AFRH},
  #endif
  #if STM32_HAS_GPIOG
    {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR,
    VAL_GPIOG_ODR,   VAL_GPIOG_AFRL,   VAL_GPIOG_AFRH},
  #endif
  #if STM32_HAS_GPIOH
    {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR,
    VAL_GPIOH_ODR,   VAL_GPIOH_AFRL,   VAL_GPIOH_AFRH},
  #endif
  #if STM32_HAS_GPIOI
    {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR,
    VAL_GPIOI_ODR,   VAL_GPIOI_AFRL,   VAL_GPIOI_AFRH}
  #endif
  };

  palInit(&pal_default_config);
#endif
// #if (HAL_USE_ADC == TRUE) || defined(__DOXYGEN__)
//   adcInit();
// #endif
// #if (HAL_USE_CAN == TRUE) || defined(__DOXYGEN__)
//   canInit();
// #endif
// #if (HAL_USE_DAC == TRUE) || defined(__DOXYGEN__)
//   dacInit();
// #endif
// #if (HAL_USE_EXT == TRUE) || defined(__DOXYGEN__)
//   extInit();
// #endif
// #if (HAL_USE_GPT == TRUE) || defined(__DOXYGEN__)
//   gptInit();
// #endif
// #if (HAL_USE_I2C == TRUE) || defined(__DOXYGEN__)
//   i2cInit();
// #endif
// #if (HAL_USE_I2S == TRUE) || defined(__DOXYGEN__)
//   i2sInit();
// #endif
// #if (HAL_USE_ICU == TRUE) || defined(__DOXYGEN__)
//   icuInit();
// #endif
// #if (HAL_USE_MAC == TRUE) || defined(__DOXYGEN__)
//   macInit();
// #endif
// #if (HAL_USE_PWM == TRUE) || defined(__DOXYGEN__)
//   pwmInit();
// #endif
// #if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)
//   sdInit();
// #endif
// #if (HAL_USE_SDC == TRUE) || defined(__DOXYGEN__)
//   sdcInit();
// #endif
// #if (HAL_USE_SPI == TRUE) || defined(__DOXYGEN__)
//   // spiInit();
// #endif
// #if (HAL_USE_UART == TRUE) || defined(__DOXYGEN__)
//   uartInit();
// #endif
// #if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)
//   usbInit();
// #endif
// #if (HAL_USE_MMC_SPI == TRUE) || defined(__DOXYGEN__)
//   mmcInit();
// #endif
// #if (HAL_USE_SERIAL_USB == TRUE) || defined(__DOXYGEN__)
//   sduInit();
// #endif
// #if (HAL_USE_RTC == TRUE) || defined(__DOXYGEN__)
//   rtcInit();
// #endif

//   /* Community driver overlay initialization.*/
// #if defined(HAL_USE_COMMUNITY) || defined(__DOXYGEN__)
// #if (HAL_USE_COMMUNITY == TRUE) || defined(__DOXYGEN__)
//   halCommunityInit();
// #endif
// #endif

  /* Board specific initialization.*/
  // boardInit();

/*
 *  The ST driver is a special case, it is only initialized if the OSAL is
 *  configured to require it.
 */
// #if OSAL_ST_MODE != OSAL_ST_MODE_NONE
  stInit();
// #endif
}

/** @} */
