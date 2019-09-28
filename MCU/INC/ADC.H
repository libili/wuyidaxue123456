/*****************************************************************************/
/**
*  @file      ADC.H
*  @brief     <b> ADC H File </b>
*  @details   File functionality description:
*  This file provides the adc driver interface.
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
#ifndef __SV_ADC_H
#define __SV_ADC_H

#define AFE0_AD(x)    1<<x
#define AFE1_AD(x)    1<<(12+x)
/** 
 * @brief define adc channel
 */
typedef enum{
    AFE0_AD0,
    AFE0_AD1,
    AFE0_AD2,
    AFE0_AD3,
    AFE0_AD4,
    AFE0_AD5,
    AFE0_AD6,
    AFE0_AD7,
    AFE0_AD8,
    AFE0_AD9,
    AFE0_AD10,
    AFE0_AD11,
    AFE1_AD0,
    AFE1_AD1,
    AFE1_AD2,
    AFE1_AD3,
    AFE1_AD4,
    AFE1_AD5,
    AFE1_AD6,
    AFE1_AD7,
    AFE1_AD8,
    AFE1_AD9,
    AFE1_AD10,
    AFE1_AD11,
}ADC_CH_TYPE;

/** 
 * @fn void adc_init(void)
 * @brief init adc hardware
 * @param[in] none 
 * @return none
 */
void adc_init(void);

/** 
 * @fn void adc_start_convert(void)
 * @brief trigger to start convert adc
 * @param[in] none 
 * @return none
 */

void adc_start_convert(void);


/** 
 * @fn int adc_getValue(const int channel)
 * @brief get the channel adc result value
 * @param[in] channel 
 * @return int
 */
int adc_getValue(const int channel);



/** 
 * @fn int adc_getValue(const int channel)
 *  get the channel adc result voltage
 * @param[in] channel 
 * @return int  mV
 */
int adc_getVoltage(const int channel);


/** 
 * @fn void adc_deinit(void)
 * @brief  deinit adc hardware
 * @param[in] channel 
 * @return int
 */
void adc_deinit(void);

#endif/* __SV_ADC_H  */
