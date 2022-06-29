/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The VESC firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include "conf_general.h"
#include "utils.h"
#include <math.h>
#include "hal.h"
// #include HW_SOURCE

// uint8_t hw_id_from_uuid(void) {
// 	uint8_t id = utils_crc32c(STM32_UUID_8, 12) & 0x7F;
// 	// CAN ID 10 and 11 are often used by DieBieMS / FlexiBMS
// 	uint8_t reserved[] = {10, 11};
// 	for (size_t i = 0; i < sizeof(reserved); ++i) {
// 		if (id == reserved[i]) {
// 			id = (id + 1) & 0x7F;
// 			i = 0;
// 		}
// 	}
// 	return id;
// }

// #if defined(HW_ID_PIN_GPIOS) && defined(HW_ID_PIN_PINS)
// uint8_t hw_id_from_pins(void) {
// 	stm32_gpio_t *hw_id_ports[]={HW_ID_PIN_GPIOS};
// 	const uint16_t hw_id_pins[] = {HW_ID_PIN_PINS};
// 	const uint16_t hw_id_pins_size = sizeof(hw_id_pins)/sizeof(uint16_t);

// 	const uint16_t DELAY_MS = 5;
// 	uint8_t trits[hw_id_pins_size];
// 	uint8_t id = 1u; //Start at 1
// 	for (uint8_t i=0; i<hw_id_pins_size; i++) {
// 		//Initialize pulldown
// 		palSetPadMode(hw_id_ports[i], hw_id_pins[i], PAL_MODE_INPUT_PULLDOWN);
		
// 		//Delay a little for the resistor to take affect
// 		chThdSleepMilliseconds(DELAY_MS);
// 		bool pin_set_pulldown = (palReadPad(hw_id_ports[i], hw_id_pins[i]));
// 		//Initialize pullup
// 		palSetPadMode(hw_id_ports[i], hw_id_pins[i], PAL_MODE_INPUT_PULLUP);
// 		//Delay a little for the resistor to take affect
// 		chThdSleepMilliseconds(DELAY_MS);
// 		bool pin_set_pullup = (palReadPad(hw_id_ports[i], hw_id_pins[i]));
// 		//Now determine the trit state
// 		if (!pin_set_pulldown && !pin_set_pullup) {
// 			//Tied to GND
// 			trits[i] = 1u;
// 		} else if (pin_set_pulldown && pin_set_pullup) {
// 			//Tied to VCC
// 			trits[i] = 2u;
// 		} else if (!pin_set_pulldown && pin_set_pullup) {
// 			//Floating
// 			trits[i] = 0u;
// 		} else {
// 			return hw_id_from_uuid();
// 			//To satisfy compiler warning
// 			trits[i] = 3u;
// 		}
// 		id += trits[i] * pow(3, i); 
// 		palSetPadMode(hw_id_ports[i], hw_id_pins[i], PAL_MODE_INPUT);
// 	}
// 	return id;
// }
// #endif //defined(HW_ID_PIN_GPIOS) && defined(HW_ID_PIN_PINS)

void hw_init_gpio(void) {
// #if defined(HW60_IS_MK3) || defined(HW60_IS_MK4) || defined(HW60_IS_MK5)
// 	chMtxObjectInit(&shutdown_mutex);
// #endif

	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(GPIOB, 0,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, 1,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// ENABLE_GATE
#ifdef HW60_VEDDER_FIRST_PCB
	palSetPadMode(GPIOB, 6,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
#else
	palSetPadMode(GPIOB, 5,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
#endif

	ENABLE_GATE();

	// Current filter
	palSetPadMode(GPIOD, 2,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	CURRENT_FILTER_OFF();

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// Phase filters
#ifdef PHASE_FILTER_GPIO
	palSetPadMode(PHASE_FILTER_GPIO, PHASE_FILTER_PIN,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	PHASE_FILTER_OFF();
#endif

	// Fault pin
	palSetPadMode(GPIOB, 7, PAL_MODE_INPUT_PULLUP);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
#if !defined(HW60_IS_MK3) && !defined(HW60_IS_MK4) && !defined(HW60_IS_MK5)
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);
#endif

	drv8301_init();

#if defined(HW60_IS_MK3) || defined(HW60_IS_MK4) || defined(HW60_IS_MK5)
	terminal_register_command_callback(
		"shutdown",
		"Shutdown VESC now.",
		0,
		terminal_shutdown_now);

	terminal_register_command_callback(
		"test_button",
		"Try sampling the shutdown button",
		0,
		terminal_button_test);
#endif
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_15Cycles);

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
}

#if HAL_USE_PAL || defined(__DOXYGEN__)
/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
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
#endif
