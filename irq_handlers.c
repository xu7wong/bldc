/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "isr_vector_table.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "hw.h"
// #include "encoder.h"

// CH_IRQ_HANDLER(ADC1_2_3_IRQHandler) {
// 	CH_IRQ_PROLOGUE();
// 	ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
// 	mc_interface_adc_inj_int_handler();
// 	CH_IRQ_EPILOGUE();
// }

// CH_IRQ_HANDLER(HW_ENC_EXTI_ISR_VEC) {
// 	if (EXTI_GetITStatus(HW_ENC_EXTI_LINE) != RESET) {
// 		// encoder_reset();

// 		// Clear the EXTI line pending bit
// 		EXTI_ClearITPendingBit(HW_ENC_EXTI_LINE);
// 	}
// }

// CH_IRQ_HANDLER(HW_ENC_TIM_ISR_VEC) {
// 	if (TIM_GetITStatus(HW_ENC_TIM, TIM_IT_Update) != RESET) {
// 		// encoder_tim_isr();

// 		// Clear the IT pending bit
// 		TIM_ClearITPendingBit(HW_ENC_TIM, TIM_IT_Update);
// 	}
// }

// CH_IRQ_HANDLER(TIM2_IRQHandler) {
// 	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {
// 		mcpwm_foc_tim_sample_int_handler();

// 		// Clear the IT pending bit
// 		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
// 	}
// 	TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
// }

// OSAL_IRQ_HANDLER(SysTick_Handler) {

//   OSAL_IRQ_PROLOGUE();

//   osalSysLockFromISR();
//   osalOsTimerHandlerI();
//   osalSysUnlockFromISR();

//   OSAL_IRQ_EPILOGUE();
// }
void SysTick_Handler(void){
	// chSysLockFromISR();
  	// chSysTimerHandlerI();
  	// chSysUnlockFromISR();
}

void TIM2_IRQHandler(void){
    if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {
		mcpwm_foc_tim_sample_int_handler();

		// Clear the IT pending bit
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
	}
}
// CH_IRQ_HANDLER(PVD_IRQHandler) {
// 	if (EXTI_GetITStatus(EXTI_Line16) != RESET) {
// 		// Log the fault. Supply voltage dropped below 2.9V,
// 		// could corrupt an ongoing flash programming
// 		mc_interface_fault_stop(FAULT_CODE_MCU_UNDER_VOLTAGE, false, true);

// 		// Clear the PVD pending bit
// 		EXTI_ClearITPendingBit(EXTI_Line16);
// 		EXTI_ClearFlag(EXTI_Line16);
// 	}
// }
void DMA2_Stream4_IRQHandler(void){
//	uint32_t flags;

//	//
//	//  	OSAL_IRQ_PROLOGUE();
//	//
//	flags = (DMA2->HISR >> 0) & STM32_DMA_ISR_MASK;
//	DMA2->HIFCR = flags << 0;
//	//  //if (dma_isr_redirX[12].dma_func)
//	//  //  dma_isr_redirX[12].dma_func(dma_isr_redirX[12].dma_param, flags);
//	mcpwm_foc_adc_int_handler();
//
	if(DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4))
	{

	/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);

//		if (debug_flag > 0) {
//			GPIOB->BSRR = GPIO_Pin_10;
//			debug_flag = 0;
//		} else {
//			GPIOB->BSRR = GPIO_Pin_10 << 16;
//			debug_flag = 1;
//		}

//		uint32_t flags;
//		flags = (DMA2->HISR >> 0) & STM32_DMA_ISR_MASK;
//		DMA2->HIFCR = flags << 0;

//		memcpy(ADC_debug, ADC_Value, 30);
		mcpwm_foc_adc_int_handler();
	}
	//if(DMA_GetITStatus(DMA2_Stream4, DMA_IT_TEIF4)){
	//	DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TEIF4);
	//}
}