/*****************************************************************************/
/**
*  @file      STARTUP.C
*  @brief     <b> STARTUP C File </b>
*  @details   File functionality description:
*  This file provides Startup interrupt table and some important interrupts.
*  @author    Axford
*  @version   v0.1
*  @date      2016.01.28
*  @bug       see Release Note
*  @par History:          
*   v0.1: Axford, 2016.01.28, initial version.
*  @warning   usage policy:
* Copyright (c) 2016  Desay SV Automotive Co., Ltd - All Rights Reserved
* Reproduction and Communication of this document is strictly prohibited
* unless specifically authorized in writing by DesaySV.
*/
/*****************************************************************************/
#include "../../CONFIG/INC/CONFIG.H"
#include "../../CONFIG/INC/TYPES.H"
#include "../INC/MCU.H"
#include "../INC/SAMV70.H"
#include "core_cm7.h"
#include "../../POWER/INC/POWER_MANA_IF.H"
typedef  void( * const ExecFuncPtr )( void )  __irq ;
extern unsigned int Image$$ARM_LIB_STACK$$ZI$$Limit;
extern int __main( void );
extern void SysTick_Handler( void );
extern void SVC_Handler( void )	;
extern void PendSV_Handler( void );

extern void CanIsr_0(void);
extern void CanIsr_1(void);
extern void GPIOA_Handler(void);
extern void GPIOB_Handler(void);
extern void GPIOC_Handler(void);
extern void GPIOD_Handler(void);
extern void GPIOE_Handler(void);

extern void Uart0_Handler(void);
extern void Uart1_Handler(void);
extern void Uart2_Handler(void);
extern void Uart3_Handler(void);
extern void Uart4_Handler(void);

extern void Spi0_Handler(void);
extern void Spi1_Handler(void);

extern void I2C0_Handler(void);
extern void I2C1_Handler(void);
extern void I2C2_Handler(void);
extern void ADC0_Handler(void);
extern void ADC1_Handler(void);
extern void TC0_Handler(void);
extern void USART0_Handler(void);
extern void USART1_Handler(void);
extern void USART2_Handler(void);
extern void XDMA_Handler(void);
extern void RTT_Handler(void);
static void Reset_Handler(void);
static __irq void NMI_Handler( void );
static __irq void HardFault_Handler( void );
static __irq void MemManage_Handler( void );
static __irq void BusFault_Handler( void );
static __irq void  UsageFault_Handler( void );
static __irq void  DebugMon_Handler( void );
static __irq void DummyHandler( void );

/**
 * @brief  exceptons_table
 *  
 *  
 */
#pragma arm section rodata="exceptions_area"
ExecFuncPtr exceptons_table[] =
{ 
    ( ExecFuncPtr )&Image$$ARM_LIB_STACK$$ZI$$Limit,/* Top of Stack */
    ( ExecFuncPtr )Reset_Handler,/* Reset Handler */
    NMI_Handler,/* NMI Handler */
    HardFault_Handler,/* Hard Fault Handler */
    MemManage_Handler,/* MPU Fault Handler */
    BusFault_Handler,/* Bus Fault Handler */
    UsageFault_Handler,/* Usage Fault Handler */
    0,/* Reserved */
    0,/* Reserved */
    0,/* Reserved */
    0,/* Reserved */
    ( ExecFuncPtr )SVC_Handler,/* SVCall Handler */
    DebugMon_Handler,/* Debug Monitor Handler */
    0,/* Reserved */
    ( ExecFuncPtr )PendSV_Handler,/* PendSV Handler */
    ( ExecFuncPtr )SysTick_Handler,/* SysTick Handler */
    
   /* External Interrupts*/
    DummyHandler,/* 0 Supply Controller */
    DummyHandler,/* 1 Reset Controller */
    DummyHandler,/* 2 Real Time Clock */
    ( ExecFuncPtr )RTT_Handler,/* 3 Real Time Timer */
    DummyHandler,/* 4 Watchdog Timer */
    DummyHandler,/* 5 Power Management Controller */
    DummyHandler,/* 6 Enhanced Embedded Flash Controller */
    ( ExecFuncPtr )Uart0_Handler,/* 7 UART 0 */
    ( ExecFuncPtr )Uart1_Handler,/* 8 UART 1 */
    0,/* 9 Reserved */
    ( ExecFuncPtr )GPIOA_Handler,/* 10 Parallel I/O Controller A */
    ( ExecFuncPtr )GPIOB_Handler,/* 11 Parallel I/O Controller B */
    ( ExecFuncPtr )GPIOC_Handler,/* 12 Parallel I/O Controller C */
    ( ExecFuncPtr )USART0_Handler,/* 13 USART 0 */
    ( ExecFuncPtr )USART1_Handler,/* 14 USART 1 */
    ( ExecFuncPtr )USART2_Handler,/* 15 USART 2 */
    ( ExecFuncPtr )GPIOD_Handler,/* 16 Parallel I/O Controller D */
    ( ExecFuncPtr )GPIOE_Handler,/* 17 Parallel I/O Controller E */
    DummyHandler,/* 18 Multimedia Card Interface */
    ( ExecFuncPtr )I2C0_Handler,/* 19 Two Wire Interface 0 HS */
    ( ExecFuncPtr )I2C1_Handler,/* 20 Two Wire Interface 1 HS */
    ( ExecFuncPtr )Spi0_Handler,/* 21 Serial Peripheral Interface 0 */
    DummyHandler,/* 22 Synchronous Serial Controller */
    ( ExecFuncPtr )TC0_Handler,/* 23 Timer/Counter 0 */
    DummyHandler,/* 24 Timer/Counter 1 */
    DummyHandler,/* 25 Timer/Counter 2 */
    DummyHandler,/* 26 Timer/Counter 3 */
    DummyHandler,/* 27 Timer/Counter 4 */
    DummyHandler,/* 28 Timer/Counter 5 */
    ( ExecFuncPtr )ADC0_Handler,/* 29 Analog Front End 0 */
    DummyHandler,/* 30 Digital To Analog Converter */
    DummyHandler,/* 31 Pulse Width Modulation 0 */
    DummyHandler,/* 32 Integrity Check Monitor */
    DummyHandler,/* 33 Analog Comparator */
    DummyHandler,/* 34 USB Host / Device Controller */
    ( ExecFuncPtr )CanIsr_0,/* 35 MCAN Controller 0 */
    0,/* 36 Reserved */
    DummyHandler,/* 37 MCAN Controller 1 */
    0,/* 38 Reserved */
    DummyHandler,/* 39 Ethernet MAC */
    ( ExecFuncPtr )ADC1_Handler,/* 40 Analog Front End 1 */
    ( ExecFuncPtr )I2C2_Handler,/* 41 Two Wire Interface 2 HS */
    ( ExecFuncPtr )Spi1_Handler,/* 42 Serial Peripheral Interface 1 */
    DummyHandler,/* 43 Quad I/O Serial Peripheral Interface */
    ( ExecFuncPtr )Uart2_Handler,/* 44 UART 2 */
    ( ExecFuncPtr )Uart3_Handler,/* 45 UART 3 */
    ( ExecFuncPtr )Uart4_Handler,/* 46 UART 4 */
    DummyHandler,/* 47 Timer/Counter 6 */
    DummyHandler,/* 48 Timer/Counter 7 */
    DummyHandler,/* 49 Timer/Counter 8 */
    DummyHandler,/* 50 Timer/Counter 9 */
    DummyHandler,/* 51 Timer/Counter 10 */
    DummyHandler,/* 52 Timer/Counter 11 */
    DummyHandler,/* 53 MediaLB */
    0,/* 54 Reserved */
    0,/* 55 Reserved */
    DummyHandler,/* 56 AES */
    DummyHandler,/* 57 True Random Generator */
    ( ExecFuncPtr )XDMA_Handler,/* 58 DMA */
    DummyHandler,/* 59 Camera Interface */
    DummyHandler,/* 60 Pulse Width Modulation 1 */
    0,/* 61 Reserved */
    DummyHandler,/* 62 SDRAM Controller */
    DummyHandler,/* 63 Reinforced Secure Watchdog Timer */
};
#pragma arm section

/**
 * @fn void Reset_Handler(void)
 * @brief  Reset interrupt routine
 *  
 * @return none
 *  
 */
void Reset_Handler(void)
{
	SCB->VTOR = (int)exceptons_table;
	mcu_init();
	__main();
}    

/**
 * @fn void NMI_Handler( void )
 * @brief  NMI interrupt routine
 *  
 * @return none
 *  
 */
__irq void NMI_Handler( void )
{
    pw_hard_fault_handle();
}

/**
 * @fn void HardFault_Handler( void )
 * @brief  HardFault interrupt routine
 *  
 * @return none
 *  
 */
__irq void HardFault_Handler( void )
{
    pw_hard_fault_handle();
}

/**
 * @fn void MemManage_Handler( void )
 * @brief  Memory access error interrupt routine
 *  
 * @return none
 *  
 */
__irq void MemManage_Handler( void )
{
    pw_hard_fault_handle();
}


/**
 * @fn void BusFault_Handler( void )
 * @brief  BusFault interrupt routine
 *  
 * @return none
 *  
 */
__irq void BusFault_Handler( void )
{
    pw_hard_fault_handle();
}


/**
 * @fn void UsageFault_Handler( void )
 * @brief  UsageFault interrupt routine
 *  
 * @return none
 *  
 */
__irq void  UsageFault_Handler( void )
{
    pw_hard_fault_handle();
}



/**
 * @fn void DebugMon_Handler( void )
 * @brief  DebugMon interrupt routine
 *  
 * @return none
 *  
 */
__irq void  DebugMon_Handler( void )
{
    pw_hard_fault_handle();
}


/**
 * @fn void DummyHandler( void )
 * @brief  default interrupt routine
 *  
 * @return none
 *  
 */
__irq void DummyHandler( void )
{
    pw_hard_fault_handle();
}
