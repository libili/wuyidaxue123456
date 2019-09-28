/*****************************************************************************/
/**
*  @file      SAMV70.H
*  @brief     <b> SAMV70 H File </b>
*  @details   File functionality description:
*  This file provides IC interrupt number definiton and ARm feature definition.
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
#ifndef SV_SAMV70_H
#define SV_SAMV70_H
/**< Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M7 Processor Exceptions Numbers ******************************/
  NonMaskableInt_IRQn   = -14, /**<  2 Non Maskable Interrupt                */
  HardFault_IRQn        = -13, /**<  3 HardFault Interrupt                   */
  MemoryManagement_IRQn = -12, /**<  4 Cortex-M7 Memory Management Interrupt */
  BusFault_IRQn         = -11, /**<  5 Cortex-M7 Bus Fault Interrupt         */
  UsageFault_IRQn       = -10, /**<  6 Cortex-M7 Usage Fault Interrupt       */
  SVCall_IRQn           = -5,  /**< 11 Cortex-M7 SV Call Interrupt           */
  DebugMonitor_IRQn     = -4,  /**< 12 Cortex-M7 Debug Monitor Interrupt     */
  PendSV_IRQn           = -2,  /**< 14 Cortex-M7 Pend SV Interrupt           */
  SysTick_IRQn          = -1,  /**< 15 Cortex-M7 System Tick Interrupt       */
/******  SAMV71Q20 specific Interrupt Numbers *********************************/
  
  SUPC_IRQn            =  0, /**<  0 SAMV71Q20 Supply Controller (SUPC) */
  RSTC_IRQn            =  1, /**<  1 SAMV71Q20 Reset Controller (RSTC) */
  RTC_IRQn             =  2, /**<  2 SAMV71Q20 Real Time Clock (RTC) */
  RTT_IRQn             =  3, /**<  3 SAMV71Q20 Real Time Timer (RTT) */
  WDT_IRQn             =  4, /**<  4 SAMV71Q20 Watchdog Timer (WDT) */
  PMC_IRQn             =  5, /**<  5 SAMV71Q20 Power Management Controller (PMC) */
  EFC_IRQn             =  6, /**<  6 SAMV71Q20 Enhanced Embedded Flash Controller (EFC) */
  UART0_IRQn           =  7, /**<  7 SAMV71Q20 UART 0 (UART0) */
  UART1_IRQn           =  8, /**<  8 SAMV71Q20 UART 1 (UART1) */
  PIOA_IRQn            = 10, /**< 10 SAMV71Q20 Parallel I/O Controller A (PIOA) */
  PIOB_IRQn            = 11, /**< 11 SAMV71Q20 Parallel I/O Controller B (PIOB) */
  PIOC_IRQn            = 12, /**< 12 SAMV71Q20 Parallel I/O Controller C (PIOC) */
  USART0_IRQn          = 13, /**< 13 SAMV71Q20 USART 0 (USART0) */
  USART1_IRQn          = 14, /**< 14 SAMV71Q20 USART 1 (USART1) */
  USART2_IRQn          = 15, /**< 15 SAMV71Q20 USART 2 (USART2) */
  PIOD_IRQn            = 16, /**< 16 SAMV71Q20 Parallel I/O Controller D (PIOD) */
  PIOE_IRQn            = 17, /**< 17 SAMV71Q20 Parallel I/O Controller E (PIOE) */
  HSMCI_IRQn           = 18, /**< 18 SAMV71Q20 Multimedia Card Interface (HSMCI) */
  TWIHS0_IRQn          = 19, /**< 19 SAMV71Q20 Two Wire Interface 0 HS (TWIHS0) */
  TWIHS1_IRQn          = 20, /**< 20 SAMV71Q20 Two Wire Interface 1 HS (TWIHS1) */
  SPI0_IRQn            = 21, /**< 21 SAMV71Q20 Serial Peripheral Interface 0 (SPI0) */
  SSC_IRQn             = 22, /**< 22 SAMV71Q20 Synchronous Serial Controller (SSC) */
  TC0_IRQn             = 23, /**< 23 SAMV71Q20 Timer/Counter 0 (TC0) */
  TC1_IRQn             = 24, /**< 24 SAMV71Q20 Timer/Counter 1 (TC1) */
  TC2_IRQn             = 25, /**< 25 SAMV71Q20 Timer/Counter 2 (TC2) */
  TC3_IRQn             = 26, /**< 26 SAMV71Q20 Timer/Counter 3 (TC3) */
  TC4_IRQn             = 27, /**< 27 SAMV71Q20 Timer/Counter 4 (TC4) */
  TC5_IRQn             = 28, /**< 28 SAMV71Q20 Timer/Counter 5 (TC5) */
  AFEC0_IRQn           = 29, /**< 29 SAMV71Q20 Analog Front End 0 (AFEC0) */
  DACC_IRQn            = 30, /**< 30 SAMV71Q20 Digital To Analog Converter (DACC) */
  PWM0_IRQn            = 31, /**< 31 SAMV71Q20 Pulse Width Modulation 0 (PWM0) */
  ICM_IRQn             = 32, /**< 32 SAMV71Q20 Integrity Check Monitor (ICM) */
  ACC_IRQn             = 33, /**< 33 SAMV71Q20 Analog Comparator (ACC) */
  USBHS_IRQn           = 34, /**< 34 SAMV71Q20 USB Host / Device Controller (USBHS) */
  MCAN0_IRQn           = 35, /**< 35 SAMV71Q20 MCAN Controller 0 (MCAN0) */
  MCAN1_IRQn           = 37, /**< 37 SAMV71Q20 MCAN Controller 1 (MCAN1) */
  GMAC_IRQn            = 39, /**< 39 SAMV71Q20 Ethernet MAC (GMAC) */
  AFEC1_IRQn           = 40, /**< 40 SAMV71Q20 Analog Front End 1 (AFEC1) */
  TWIHS2_IRQn          = 41, /**< 41 SAMV71Q20 Two Wire Interface 2 HS (TWIHS2) */
  SPI1_IRQn            = 42, /**< 42 SAMV71Q20 Serial Peripheral Interface 1 (SPI1) */
  QSPI_IRQn            = 43, /**< 43 SAMV71Q20 Quad I/O Serial Peripheral Interface (QSPI) */
  UART2_IRQn           = 44, /**< 44 SAMV71Q20 UART 2 (UART2) */
  UART3_IRQn           = 45, /**< 45 SAMV71Q20 UART 3 (UART3) */
  UART4_IRQn           = 46, /**< 46 SAMV71Q20 UART 4 (UART4) */
  TC6_IRQn             = 47, /**< 47 SAMV71Q20 Timer/Counter 6 (TC6) */
  TC7_IRQn             = 48, /**< 48 SAMV71Q20 Timer/Counter 7 (TC7) */
  TC8_IRQn             = 49, /**< 49 SAMV71Q20 Timer/Counter 8 (TC8) */
  TC9_IRQn             = 50, /**< 50 SAMV71Q20 Timer/Counter 9 (TC9) */
  TC10_IRQn            = 51, /**< 51 SAMV71Q20 Timer/Counter 10 (TC10) */
  TC11_IRQn            = 52, /**< 52 SAMV71Q20 Timer/Counter 11 (TC11) */
  MLB_IRQn             = 53, /**< 53 SAMV71Q20 MediaLB (MLB) */
  AES_IRQn             = 56, /**< 56 SAMV71Q20 AES (AES) */
  TRNG_IRQn            = 57, /**< 57 SAMV71Q20 True Random Generator (TRNG) */
  XDMAC_IRQn           = 58, /**< 58 SAMV71Q20 DMA (XDMAC) */
  ISI_IRQn             = 59, /**< 59 SAMV71Q20 Camera Interface (ISI) */
  PWM1_IRQn            = 60, /**< 60 SAMV71Q20 Pulse Width Modulation 1 (PWM1) */
  SDRAMC_IRQn          = 62, /**< 62 SAMV71Q20 SDRAM Controller (SDRAMC) */
  RSWDT_IRQn           = 63, /**< 63 SAMV71Q20 Reinforced Secure Watchdog Timer (RSWDT) */
  PERIPH_COUNT_IRQn    = 64  /**< Number of peripheral IDs */
} IRQn_Type;


/**
 * \brief Configuration of the Cortex-M7 Processor and Core Peripherals 
 */

#define __CM7_REV              0x0000 /**< SAMV71Q20 core revision number ([15:8] revision number, [7:0] patch number) */
#define __MPU_PRESENT          1      /**< SAMV71Q20 does provide a MPU */
#define __NVIC_PRIO_BITS       3      /**< SAMV71Q20 uses 3 Bits for the Priority Levels */
#define __FPU_PRESENT          1      /**< SAMV71Q20 does provide a FPU                */
#define __FPU_DP               1      /**< SAMV71Q20 Double precision FPU              */
#define __ICACHE_PRESENT       1      /**< SAMV71Q20 does provide an Instruction Cache */
#define __DCACHE_PRESENT       1      /**< SAMV71Q20 does provide a Data Cache         */
#define __DTCM_PRESENT         1      /**< SAMV71Q20 does provide a Data TCM           */
#define __ITCM_PRESENT         1      /**< SAMV71Q20 does provide an Instruction TCM   */
#define __Vendor_SysTickConfig 0      /**< Set to 1 if different SysTick Config is used */

#endif /*SV_SAMV70_H */
