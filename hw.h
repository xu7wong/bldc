/*
	Copyright 2012 - 2020 Benjamin Vedder	benjamin@vedder.se

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

#ifndef HW_H_
#define HW_H_

#include "stm32f4xx_conf.h"
#include "datatypes.h"
#include "mcconf_default.h"
#include "appconf_default.h"

#define HW_NAME					"UBCO"

#ifndef SYSTEM_CORE_CLOCK
#define SYSTEM_CORE_CLOCK			168000000
#endif

#ifndef FOC_CONTROL_LOOP_FREQ_DIVIDER
#define FOC_CONTROL_LOOP_FREQ_DIVIDER	1
#endif

#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8

#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

#ifndef READ_HALL1_2
#define READ_HALL1_2()			READ_HALL1()
#endif
#ifndef READ_HALL2_2
#define READ_HALL2_2()			READ_HALL2()
#endif
#ifndef READ_HALL3_2
#define READ_HALL3_2()			READ_HALL3()
#endif

#define HW_ADC_CHANNELS			15
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5

#define HW_ADC_CHANNELS_EXTRA	0

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			3
#define ADC_IND_SENS3			6//2
#define ADC_IND_CURR1			1
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			7
// #define ADC_IND_EXT				6
// #define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS		13
#define ADC_IND_TEMP_MOTOR		10
#define ADC_IND_VIN_SENS		9
#define ADC_IND_VREFINT			12

#define ENABLE_GATE()			palSetPad(GPIOB, 5)
#define DISABLE_GATE()			palClearPad(GPIOB, 5)
// #endif
// #define DCCAL_ON()
// #define DCCAL_OFF()
#define IS_DRV_FAULT()			(!palReadPad(GPIOB, 7))

#define LED_GREEN_ON()			palSetPad(GPIOB, 0)
#define LED_GREEN_OFF()			palClearPad(GPIOB, 0)
#define LED_RED_ON()			palSetPad(GPIOB, 1)
#define LED_RED_OFF()			palClearPad(GPIOB, 1)

// #define HW_HAS_DRV8301
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS


#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					39000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.0005
#endif

#define FAC_CURRENT					((V_REG / 4095.0) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]

// #ifndef ADC_V_L4
// #define ADC_V_L4				ADC_V_L1
// #endif
// #ifndef ADC_V_L5
// #define ADC_V_L5				ADC_V_L2
// #endif
// #ifndef ADC_V_L6
// #define ADC_V_L6				ADC_V_L3
// #endif


#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

#ifndef GET_CURRENT1
#define GET_CURRENT1()		ADC_Value[ADC_IND_CURR1]
#endif

#ifndef GET_CURRENT2
#define GET_CURRENT2()		ADC_Value[ADC_IND_CURR2]
#endif

#ifndef GET_CURRENT3
#define GET_CURRENT3()		ADC_Value[ADC_IND_CURR3]
#endif

#ifndef ADC_VOLTS_PH_FACTOR
#define ADC_VOLTS_PH_FACTOR		1.0
#endif
#ifndef ADC_VOLTS_INPUT_FACTOR
#define ADC_VOLTS_INPUT_FACTOR	1.0
#endif

#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)
// #define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)


void hw_init_gpio(void);
void hw_setup_adc_channels(void);
void confgenerator_set_defaults_mcconf(mc_configuration *conf);
uint8_t conf_general_calculate_deadtime(float deadtime_ns, float core_clock_freq);

#endif /* HW_H_ */
