/*****************************************************************************/
/**
*  @file      PWM.H
*  @brief     <b> PWM H File </b>
*  @details   File functionality description:
*  This file provides the PWM driver interface.
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
#ifndef __SV_PWM_H
#define __SV_PWM_H

typedef struct
{
	unsigned int period;
    unsigned int duty;
}PWM_ConfigTy;



/**
 * @fn void pwm_init(int channel,PWM_ConfigTy* config)unsigned int trng_getValue(void)
 * @brief initial the pwm hardware with the config
 * @param [in] config 
 * @return none
 *  
 */
void pwm_init(int channel,PWM_ConfigTy* config);

/**
 * @fn void pwm_setDuty(int channel,int duty)
 * @brief setting the pwm duty 
 * @param [in] channel
 * @param [in] duty

 * @return none
 *  
 */
void pwm_setDuty(int channel,int duty);

/**
 * @fn void pwm_setPeriod(int channel,int period)
 * @brief setting the pwm period 
 * @param [in] channel
 * @param [in] period

 * @return none
 *  
 */
void pwm_setPeriod(int channel,int period);

/**
 * @fn void pwm_enable(int channel)
 * @brief enable the pwm channel to output
 * @param [in] channel

 * @return none
 *  
 */
void pwm_enable(int channel);

/**
 * @fn void pwm_disable(int channel)
 * @brief disable the pwm channel to output
 * @param [in] channel

 * @return none
 *  
 */
void pwm_disable(int channel);

/**
 * @fn void pwm_deint(int channel)
 * @brief deinit the pwm hardware
 * @param [in] channel

 * @return none
 *  
 */
void pwm_deinit(int channel);
#endif/* __SV_PWM_H  */
