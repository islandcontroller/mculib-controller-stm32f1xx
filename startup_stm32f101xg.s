.include "Core/startup_cm3.s"

.syntax unified
.cpu cortex-m3
.fpu softvfp
.thumb

/******************************************************************************
 * _vector_base
 * Interrupt Vector Table
 ******************************************************************************/
.section  .isr_vector, "a", %progbits
.balign 4
.globl  _vector_base
_vector_base:
  /* Main stack pointer                                                       */
  .word _estack
  /* Internal interrupts                                                      */
  .word _start
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  /* External Interrupts                                                      */
  .word WWDG_IRQHandler
  .word PVD_IRQHandler
  .word TAMPER_IRQHandler
  .word RTC_IRQHandler
  .word FLASH_IRQHandler
  .word RCC_IRQHandler
  .word EXTI0_IRQHandler
  .word EXTI1_IRQHandler
  .word EXTI2_IRQHandler
  .word EXTI3_IRQHandler
  .word EXTI4_IRQHandler
  .word DMA1_Channel1_IRQHandler
  .word DMA1_Channel2_IRQHandler
  .word DMA1_Channel3_IRQHandler
  .word DMA1_Channel4_IRQHandler
  .word DMA1_Channel5_IRQHandler
  .word DMA1_Channel6_IRQHandler
  .word DMA1_Channel7_IRQHandler
  .word ADC1_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word EXTI9_5_IRQHandler
  .word TIM9_IRQHandler
  .word TIM10_IRQHandler
  .word TIM11_IRQHandler
  .word 0
  .word TIM2_IRQHandler
  .word TIM3_IRQHandler
  .word TIM4_IRQHandler
  .word I2C1_EV_IRQHandler
  .word I2C1_ER_IRQHandler
  .word I2C2_EV_IRQHandler
  .word I2C2_ER_IRQHandler
  .word SPI1_IRQHandler
  .word SPI2_IRQHandler
  .word USART1_IRQHandler
  .word USART2_IRQHandler
  .word USART3_IRQHandler
  .word EXTI15_10_IRQHandler
  .word RTC_Alarm_IRQHandler
  .word 0
  .word TIM12_IRQHandler
  .word TIM13_IRQHandler
  .word TIM14_IRQHandler
  .word 0
  .word 0
  .word FSMC_IRQHandler
  .word 0
  .word TIM5_IRQHandler
  .word SPI3_IRQHandler
  .word UART4_IRQHandler
  .word UART5_IRQHandler
  .word TIM6_IRQHandler
  .word TIM7_IRQHandler  
  .word DMA2_Channel1_IRQHandler
  .word DMA2_Channel2_IRQHandler
  .word DMA2_Channel3_IRQHandler
  .word DMA2_Channel4_5_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  ldr.w pc,   [pc,    #-0x1e0]        /* RAM boot: jump to _start             */

/* Weak definitions of interrupt handlers in .text                            */
.section  .text.vector_handler, "ax", %progbits
.weak NMI_Handler
.weak HardFault_Handler
.weak MemManage_Handler
.weak BusFault_Handler
.weak UsageFault_Handler
.weak SVC_Handler
.weak DebugMon_Handler
.weak PendSV_Handler
.weak SysTick_Handler
.weak WWDG_IRQHandler
.weak PVD_IRQHandler
.weak TAMPER_IRQHandler
.weak RTC_IRQHandler
.weak FLASH_IRQHandler
.weak RCC_IRQHandler
.weak EXTI0_IRQHandler
.weak EXTI1_IRQHandler
.weak EXTI2_IRQHandler
.weak EXTI3_IRQHandler
.weak EXTI4_IRQHandler
.weak DMA1_Channel1_IRQHandler
.weak DMA1_Channel2_IRQHandler
.weak DMA1_Channel3_IRQHandler
.weak DMA1_Channel4_IRQHandler
.weak DMA1_Channel5_IRQHandler
.weak DMA1_Channel6_IRQHandler
.weak DMA1_Channel7_IRQHandler
.weak ADC1_IRQHandler
.weak EXTI9_5_IRQHandler
.weak TIM9_IRQHandler
.weak TIM10_IRQHandler
.weak TIM11_IRQHandler
.weak TIM2_IRQHandler
.weak TIM3_IRQHandler
.weak TIM4_IRQHandler
.weak I2C1_EV_IRQHandler
.weak I2C1_ER_IRQHandler
.weak I2C2_EV_IRQHandler
.weak I2C2_ER_IRQHandler
.weak SPI1_IRQHandler
.weak SPI2_IRQHandler
.weak USART1_IRQHandler
.weak USART2_IRQHandler
.weak USART3_IRQHandler
.weak EXTI15_10_IRQHandler
.weak RTC_Alarm_IRQHandler
.weak TIM12_IRQHandler
.weak TIM13_IRQHandler
.weak TIM14_IRQHandler
.weak FSMC_IRQHandler
.weak TIM5_IRQHandler
.weak SPI3_IRQHandler
.weak UART4_IRQHandler
.weak UART5_IRQHandler
.weak TIM6_IRQHandler
.weak TIM7_IRQHandler
.weak DMA2_Channel1_IRQHandler
.weak DMA2_Channel2_IRQHandler
.weak DMA2_Channel3_IRQHandler
.weak DMA2_Channel4_5_IRQHandler

/* Default to generic handler if not implemented                              */
.thumb_set NMI_Handler, Default_Handler
.thumb_set HardFault_Handler, Default_Handler
.thumb_set MemManage_Handler, Default_Handler
.thumb_set BusFault_Handler, Default_Handler
.thumb_set UsageFault_Handler, Default_Handler
.thumb_set SVC_Handler, Default_Handler
.thumb_set DebugMon_Handler, Default_Handler
.thumb_set PendSV_Handler, Default_Handler
.thumb_set SysTick_Handler, Default_Handler
.thumb_set WWDG_IRQHandler, Default_Handler
.thumb_set PVD_IRQHandler, Default_Handler
.thumb_set TAMPER_IRQHandler, Default_Handler
.thumb_set RTC_IRQHandler, Default_Handler
.thumb_set FLASH_IRQHandler, Default_Handler
.thumb_set RCC_IRQHandler, Default_Handler
.thumb_set EXTI0_IRQHandler, Default_Handler
.thumb_set EXTI1_IRQHandler, Default_Handler
.thumb_set EXTI2_IRQHandler, Default_Handler
.thumb_set EXTI3_IRQHandler, Default_Handler
.thumb_set EXTI4_IRQHandler, Default_Handler
.thumb_set DMA1_Channel1_IRQHandler, Default_Handler
.thumb_set DMA1_Channel2_IRQHandler, Default_Handler
.thumb_set DMA1_Channel3_IRQHandler, Default_Handler
.thumb_set DMA1_Channel4_IRQHandler, Default_Handler
.thumb_set DMA1_Channel5_IRQHandler, Default_Handler
.thumb_set DMA1_Channel6_IRQHandler, Default_Handler
.thumb_set DMA1_Channel7_IRQHandler, Default_Handler
.thumb_set ADC1_IRQHandler, Default_Handler
.thumb_set EXTI9_5_IRQHandler, Default_Handler
.thumb_Set TIM9_IRQHandler, Default_Handler
.thumb_Set TIM10_IRQHandler, Default_Handler
.thumb_Set TIM11_IRQHandler, Default_Handler
.thumb_set TIM2_IRQHandler, Default_Handler
.thumb_set TIM3_IRQHandler, Default_Handler
.thumb_set TIM4_IRQHandler, Default_Handler
.thumb_set I2C1_EV_IRQHandler, Default_Handler
.thumb_set I2C1_ER_IRQHandler, Default_Handler
.thumb_set I2C2_EV_IRQHandler, Default_Handler
.thumb_set I2C2_ER_IRQHandler, Default_Handler
.thumb_set SPI1_IRQHandler, Default_Handler
.thumb_set SPI2_IRQHandler, Default_Handler
.thumb_set USART1_IRQHandler, Default_Handler
.thumb_set USART2_IRQHandler, Default_Handler
.thumb_set USART3_IRQHandler, Default_Handler
.thumb_set EXTI15_10_IRQHandler, Default_Handler
.thumb_set RTC_Alarm_IRQHandler, Default_Handler
.thumb_set TIM12_IRQHandler, Default_Handler
.thumb_set TIM13_IRQHandler, Default_Handler
.thumb_set TIM14_IRQHandler, Default_Handler
.thumb_set FSMC_IRQHandler, Default_Handler
.thumb_set TIM5_IRQHandler, Default_Handler
.thumb_set SPI3_IRQHandler, Default_Handler
.thumb_set UART4_IRQHandler, Default_Handler
.thumb_set UART5_IRQHandler, Default_Handler
.thumb_set TIM6_IRQHandler, Default_Handler
.thumb_set TIM7_IRQHandler, Default_Handler
.thumb_set DMA2_Channel1_IRQHandler, Default_Handler
.thumb_set DMA2_Channel2_IRQHandler, Default_Handler
.thumb_set DMA2_Channel3_IRQHandler, Default_Handler
.thumb_set DMA2_Channel4_5_IRQHandler, Default_Handler


/******************************************************************************
 * Default_Handler
 * Default interrupt handler for unimplemented interrupts
 ******************************************************************************/
.section  .text.Default_Handler
.balign 4
.globl  Default_Handler
.type   Default_Handler, %function
Default_Handler:
  b     Default_Handler
