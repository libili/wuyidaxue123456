/*****************************************************************************/
/**
*  @file      WDG.C
*  @brief     <b> WDG C File </b>
*  @details   File functionality description:
*  This file is to implement the Watchdog driver.
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
#include "../../CONFIG/INC/DRV_CFG.H"
#include "../INC/WDG.H"

/*register description */
typedef struct
{
	volatile  int CR;
	volatile int MR;
	volatile int SR;
}WDT_Type;
#define WDT_BASE  0x400E1850
#define WDG   ((WDT_Type*)WDT_BASE)
/*Bits*/
#define KEY			((unsigned int)0xA5<<24)
#define WDRSTT 		    0x01

#define WDIDLEHLT		(1<<29)   /*   stops when the system is in idle state*/
#define WDDBGHLT     	(1<<28)  /* Watchdog Debug Halt */
#define WDD(x) 			(x<<16)  /*  Watchdog Delta Value */
#define WDDIS           (1<<15)  /*  Watchdog Disable*/
#define WDRSTEN         (1<<13)  /* Watchdog Reset Enable */
#define WDFIEN			(1<<12)  /*  Watchdog Fault Interrupt Enable*/
#define WDV(x)				(x<<0)   /*  Watchdog Counter Value*/

#define WDERR			(1<<1)
#define WDUNF			(1<<0)
/*end register description*/
/**
 * @fn void WATCHDOG_Enable( void )
 * @brief config the watchdog hardware and enable it
 * @param none
 */
void WATCHDOG_Enable( void )
{
	unsigned short count = 0xFFF;
    extern const McuDrvConfigsTy   mcu_drv_configs;
	const WatchDogConfigTy* pconf= mcu_drv_configs.watchdog;
	if(pconf)
	{
		count = pconf->period/4;
	}	
	WDG->MR = WDIDLEHLT | WDDBGHLT |WDRSTEN |/* WDFIEN */WDD(count) | WDV(count);
	WDG->CR = KEY|WDRSTT;
}


/**
 * @fn void WATCHDOG_Disable( void )
 * @brief disable watchdog 
 * @param none
 */
void WATCHDOG_Disable( void )
{
	WDG->MR  = WDDIS;
}


/**
 * @fn void WATCHDOG_Clear( void )
 * @brief refresh watchdog 
 * @param none
 */
void WATCHDOG_Clear( void )
{
	WDG->CR = KEY | WDRSTT;
}

