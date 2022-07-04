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
 * @file    STM32/st_lld.h
 * @brief   ST Driver subsystem low level driver header.
 * @details This header is designed to be include-able without having to
 *          include other files from the HAL.
 *
 * @addtogroup ST
 * @{
 */

#ifndef _ST_LLD_H_
#define _ST_LLD_H_

#include "mcuconf.h"
#include "stm32_registry.h"
#include "stm32_tim.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   SysTick timer IRQ priority.
 */
#if !defined(STM32_ST_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_ST_IRQ_PRIORITY               8
#endif

/**
 * @brief   TIMx unit (by number) to be used for free running operations.
 * @note    You must select a 32 bits timer if a 32 bits @p systick_t type
 *          is required.
 * @note    Timers 2, 3, 4 and 5 are supported.
 */
#if !defined(STM32_ST_USE_TIMER) || defined(__DOXYGEN__)
#define STM32_ST_USE_TIMER                  2
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if STM32_ST_USE_TIMER == 2
#if !STM32_HAS_TIM2
#error "TIM2 not present"
#endif
#define STM32_ST_TIM                              STM32_TIM2

#elif STM32_ST_USE_TIMER == 3
#if !STM32_HAS_TIM3
#error "TIM3 not present"
#endif
#define STM32_ST_TIM                              STM32_TIM3

#elif STM32_ST_USE_TIMER == 4
#if !STM32_HAS_TIM4
#error "TIM4 not present"
#endif
#define STM32_ST_TIM                              STM32_TIM4

#elif STM32_ST_USE_TIMER == 5
#if !STM32_HAS_TIM5
#error "TIM5 not present"
#endif
#define STM32_ST_TIM                              STM32_TIM5

#else
#error "STM32_ST_USE_TIMER specifies an unsupported timer"
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

void st_lld_init(void);



#endif /* _ST_LLD_H_ */

/** @} */
