/*****************************************************************************/
/**
*  @file      TC.H
*  @brief     <b> TC H File </b>
*  @details   File functionality description:
*  This file provides the TC driver interface.
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
#ifndef __SV_TC_H
#define __SV_TC_H

typedef void (*Tc_callback)(void);


typedef struct{
    int peroid;/* =n*us */
    Tc_callback  cb;
}TC_ConfigTy;



/**
 * @fn void tc_init(int channel,TC_ConfigTy* config)
 * @brief init TC hardware with the config
 *  
 * @return none
 *  
 */
void tc_init(int channel,TC_ConfigTy* config);

/**
 * @fn unsigned int tc_getRate(int channel)
 * @brief getting the TC counter rate 0.1% unit
 *  
 * @return unsigned int
 *  
 */
unsigned int tc_getRate(int channel);


/**
 * @fn tc_deinit(int channel)
 * @brief deinit TC hardware 
 *  
 * @return none
 *  
 */
void tc_deinit(int channel);
#endif/* __SV_TC_H  */


