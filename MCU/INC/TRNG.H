/*****************************************************************************/
/**
*  @file      TRNG.H
*  @brief     <b> TRNG H File </b>
*  @details   File functionality description:
*  This file provides the true random number driver interface.
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
#ifndef __SV_TRNG_H
#define __SV_TRNG_H

/**
 * @fn void trng_init(void)
 * @brief true random hardware init
 *  
 * @return none
 *  
 */
void trng_init(void);


/**
 * @fn unsigned int trng_getValue(void)
 * @brief getting true random 32bits value
 *  
 * @return unsigned int
 *  
 */
unsigned int trng_getValue(void);
/**
 * @fn void trng_deint(void)
 * @brief deint the true random hardware
 *  
 * @return none
 *  
 */
void trng_deint(void);


#endif/* __SV_TRNG_H  */


