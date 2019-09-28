/*****************************************************************************/
/**
*  @file      WDG.H
*  @brief     <b> WDG H File </b>
*  @details   File functionality description:
*  This file provides the Watchdog driver interface.
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
#ifndef _SV_WDG
#define _SV_WDG

/**
 * @fn void WATCHDOG_Enable( void )
 * @brief config the watchdog hardware and enable it
 * @param none
 */
void WATCHDOG_Enable( void );


/**
 * @fn void WATCHDOG_Disable( void )
 * @brief disable watchdog 
 * @param none
 */
void WATCHDOG_Disable( void );


/**
 * @fn void WATCHDOG_Clear( void )
 * @brief refresh watchdog 
 * @param none
 */
void WATCHDOG_Clear( void );

#endif /* _SV_WDG */
