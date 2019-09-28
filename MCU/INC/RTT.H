/*****************************************************************************/
/**
*  @file      TC.H
*  @brief     <b> TC H File </b>
*  @details   File functionality description:
*  This file provides the RTT driver interface.
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
#ifndef __SV_RTT_H
#define __SV_RTT_H

typedef struct{
    int interval;/* =n*ms */
}RTT_ConfigTy;


/**
 * @fn void rtt_init(RTT_ConfigTy* config)
 * @brief  rtt hardware init with the config
 *  
 * @return none
 *  
 */
void rtt_init(RTT_ConfigTy* config);

/**
 * @fn unsigned int rtt_getValue(void)
 * @brief  getting the rtt value
 *  
 * @return unsigned int
 *  
 */
unsigned int rtt_getValue(void);

/**
 * @fn rtt_setAlarm(const unsigned int alarm,void(*call_back)(void))
 * @brief  setting Alarm with or without callback function
 *  
 * @return none
 *  
 */
void rtt_setAlarm(const unsigned int alarm,void(*call_back)(void));
#endif/* __SV_RTT_H  */


