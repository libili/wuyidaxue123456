/*****************************************************************************/
/**
*  * @file      ADC.C
*  * @brief     <b> ADC C File </b>
*  * @details   File functionality description:
*  This file is to implemet adc driver.
*  * @author    Axford
*  * @version   v0.1
*  * @date      2016.01.28
*  * @bug       see Release Note
*  * @par History:          
*   v0.1: Axford, 2016.01.28, initial version.
*  * @warning   usage policy:
* Copyright (c) 2016  Desay SV Automotive Co., Ltd - All Rights Reserved
* Reproduction and Communication of this document is strictly prohibited
* unless specifically authorized in writing by DesaySV.
*/
/*****************************************************************************/
#include "../../CONFIG/INC/DRV_CFG.H"
#include "../INC/MCU.H"
#include "../INC/SAMV70.H"
#include "../INC/ADC.H"
#include <string.h>
/** 
 * @brief define adc controller registers
 */
typedef struct {
  volatile 			int AFEC_CR;       /**< \brief (Afec Offset: 0x00) AFEC Control Register */
  volatile 			int AFEC_MR;       /**< \brief (Afec Offset: 0x04) AFEC Mode Register */
  volatile 			int AFEC_EMR;      /**< \brief (Afec Offset: 0x08) AFEC Extended Mode Register */
  volatile 			int AFEC_SEQ1R;    /**< \brief (Afec Offset: 0x0C) AFEC Channel Sequence 1 Register */
  volatile 			int AFEC_SEQ2R;    /**< \brief (Afec Offset: 0x10) AFEC Channel Sequence 2 Register */
  volatile 			int AFEC_CHER;     /**< \brief (Afec Offset: 0x14) AFEC Channel Enable Register */
  volatile 			int AFEC_CHDR;     /**< \brief (Afec Offset: 0x18) AFEC Channel Disable Register */
  volatile const  	int AFEC_CHSR;     /**< \brief (Afec Offset: 0x1C) AFEC Channel Status Register */
  volatile const  	int AFEC_LCDR;     /**< \brief (Afec Offset: 0x20) AFEC Last Converted Data Register */
  volatile 			int AFEC_IER;      /**< \brief (Afec Offset: 0x24) AFEC Interrupt Enable Register */
  volatile 			int AFEC_IDR;      /**< \brief (Afec Offset: 0x28) AFEC Interrupt Disable Register */
  volatile const  	int AFEC_IMR;      /**< \brief (Afec Offset: 0x2C) AFEC Interrupt Mask Register */
  volatile const  	int AFEC_ISR;      /**< \brief (Afec Offset: 0x30) AFEC Interrupt Status Register */
  volatile const  	int Reserved1[6];
  volatile const  	int AFEC_OVER;     /**< \brief (Afec Offset: 0x4C) AFEC Overrun Status Register */
  volatile 			int AFEC_CWR;      /**< \brief (Afec Offset: 0x50) AFEC Compare Window Register */
  volatile 			int AFEC_CGR;      /**< \brief (Afec Offset: 0x54) AFEC Channel Gain Register */
  volatile const  	int Reserved2[2];
  volatile 			int AFEC_DIFFR;    /**< \brief (Afec Offset: 0x60) AFEC Channel Differential Register */
  volatile 			int AFEC_CSELR;    /**< \brief (Afec Offset: 0x64) AFEC Channel Selection Register */
  volatile const  	int AFEC_CDR;      /**< \brief (Afec Offset: 0x68) AFEC Channel Data Register */
  volatile 			int AFEC_COCR;     /**< \brief (Afec Offset: 0x6C) AFEC Channel Offset Compensation Register */
  volatile 			int AFEC_TEMPMR;   /**< \brief (Afec Offset: 0x70) AFEC Temperature Sensor Mode Register */
  volatile 			int AFEC_TEMPCWR;  /**< \brief (Afec Offset: 0x74) AFEC Temperature Compare Window Register */
  volatile const  	int Reserved3[7];
  volatile 			int AFEC_ACR;      /**< \brief (Afec Offset: 0x94) AFEC Analog Control Register */
  volatile const  	int Reserved4[2];
  volatile 			int AFEC_SHMR;     /**< \brief (Afec Offset: 0xA0) AFEC Sample & Hold Mode Register */
  volatile const  	int Reserved5[11];
  volatile 			int AFEC_COSR;     /**< \brief (Afec Offset: 0xD0) AFEC Correction Select Register */
  volatile 			int AFEC_CVR;      /**< \brief (Afec Offset: 0xD4) AFEC Correction Values Register */
  volatile 			int AFEC_CECR;     /**< \brief (Afec Offset: 0xD8) AFEC Channel Error Correction Register */
  volatile const  	int Reserved6[2];
  volatile 			int AFEC_WPMR;     /**< \brief (Afec Offset: 0xE4) AFEC Write Protection Mode Register */
  volatile const  	int AFEC_WPSR;     /**< \brief (Afec Offset: 0xE8) AFEC Write Protection Status Register */
}AdcTypedef;

#define AFEC_CR_SWRST (0x1u << 0) /**< \brief (AFEC_CR) Software Reset */
#define AFEC_CR_START (0x1u << 1) /**< \brief (AFEC_CR) Start Conversion */

typedef struct{
	unsigned int base;
	unsigned int fclk;
	unsigned int irq;
	unsigned int afe_clock;
	unsigned int startup;
	unsigned int tracktim;
	unsigned int transfer;
}AdcDescTypedef;
#define ADC_CTRL_NUM   2

const AdcDescTypedef adcDesc[ADC_CTRL_NUM]={
	{	.base = 0x4003C000,
		.fclk= 100000000,
		.irq=29,
		.afe_clock = 10000000,
		.startup = 5,/*80*100 = 8000ns*/
		.tracktim = 19 ,/*20*10 = 2us*/
		.transfer = 2,/* (2+6)*100 = 800ns*/
	},
    {	.base = 0x40064000,
		.fclk= 100000000,
		.irq=40,
		.afe_clock = 10000000,
		.startup = 5,/*80*100 = 8us*/
		.tracktim = 19 ,/*20*100 = 2us*/
		.transfer = 2,/* (2+6)*100 = 800ns*/
	}
};

#define INVALID_VOLTAGE  0xFFFF
#define ADC_DEF_BYTE    0xFF
#define ADC_CH_MAX        24
#define ADC_CH_MASK      0xFFF
/* static */ unsigned short adc_value_array[5][ADC_CH_MAX];
static unsigned index;
static int adc_channel_is_ready(const int channel);


/** 
 * @fn void adc_init(void)
 * @brief init adc hardware
 * @param[in] none 
 * @return none
 */
void adc_init(void)
{
    memset(adc_value_array,ADC_DEF_BYTE,sizeof(unsigned short)*5*ADC_CH_MAX);
    for(int ctrl = 0; ctrl <ADC_CTRL_NUM; ctrl++)
    {
        AdcTypedef* adc = (AdcTypedef*)adcDesc[ctrl].base;
        mcu_peri_clk_enable(adcDesc[ctrl].irq);
        /*  Reset the controller */
        adc->AFEC_CR = AFEC_CR_SWRST;
        /* Reset Mode Register */
        adc->AFEC_MR = 0;
        adc->AFEC_OVER;     
        //set mode Register
        adc->AFEC_MR = (0x1u << 23)|((adcDesc[ctrl].fclk/adcDesc[ctrl].afe_clock -1)<<8)|\
					(adcDesc[ctrl].transfer<<28)|(adcDesc[ctrl].tracktim <<24)|(adcDesc[ctrl].startup<<16);
        adc->AFEC_EMR = (1<<24)|(1<<25);
        adc->AFEC_ACR = (1<<8)|(1<<3)|(1<<2);
        adc->AFEC_CHER = 0;
        adc->AFEC_IER = 0;
        adc->AFEC_DIFFR = 0;//no diff;
        for(int i = 0; i < 12; i++)
        {
            adc->AFEC_CSELR = i;
            adc->AFEC_CGR = 0;//no gain
            adc->AFEC_COCR = 0x200;//no offset	
        }
        adc->AFEC_SHMR = 0;
        irq_enable(adcDesc[ctrl].irq);
    }
	 
}

/** 
 * @fn int getHighestEnableChannel(int channels)
 * @brief  get the HighestEnableChannel
 * @param[in] channels 
 * @return int
 */
static inline int getHighestEnableChannel(int channels)
{
    int i = 0,highest = 0;
    for(; i <12; i++)
    {
        if(channels<(1<<i))
        {
            break;
        }
        else
        {
            highest = i;
        }        
    }
    return highest;    
}

/** 
 * @fn void adc_start_convert(void)
 * @brief trigger to start convert adc
 * @param[in] none 
 * @return none
 */
void adc_start_convert(void)
{ 
    extern const McuDrvConfigsTy   mcu_drv_configs;
    const   AdcConfigTy* adc_config = mcu_drv_configs.adc;
    for(int ctrl = 0; ctrl <ADC_CTRL_NUM; ctrl++)
    {
        AdcTypedef* adc = (AdcTypedef*)adcDesc[ctrl].base;
        if(adc->AFEC_IMR&ADC_CH_MASK)
        {
            continue;
        }
        unsigned short en_channel;
        en_channel = (adc_config->en_channels&(0xFFFu<<ctrl*12))>>(ctrl*12);
        if(en_channel)
        {
            adc->AFEC_CHER = en_channel&ADC_CH_MASK;
            adc->AFEC_IER = 1<<getHighestEnableChannel(en_channel&ADC_CH_MASK);
            adc->AFEC_CR = AFEC_CR_START;
        }    
        
    }  
}

/** 
 * @fn int adc_getValue(const int channel)
 * @brief get the channel adc result value
 * @param[in] channel 
 * @return int
 */
int adc_getValue(const int channel)
{
	if((channel >= ADC_CH_MAX)||(!adc_channel_is_ready(channel)) )
	{
		return INVALID_VOLTAGE;
	}	
	//get the max first
	int max = adc_value_array[0][channel];
	int min = adc_value_array[0][channel];
	int val = adc_value_array[0][channel];
	for(int i = 1;i <5; i++)
	{
		if(max < adc_value_array[i][channel])
		{
			max = adc_value_array[i][channel];
		}
		if(min > adc_value_array[i][channel])
		{
			min = adc_value_array[i][channel];
		}
		val += adc_value_array[i][channel];
	}
	val = (val - max -min)/(5-2);	
	return val;
}


/** 
 * @fn int adc_getValue(const int channel)
 * @brief get the channel adc result voltage
 * @param[in] channel 
 * @return int  mV
 */
int adc_getVoltage(const int channel)
{
    int vol = adc_getValue(channel);
    return (vol*3300/4096);	
}


/** 
 * @fn void adc_deinit(void)
 * @brief deinit adc hardware
 * @param[in] channel 
 * @return int
 */
void adc_deinit(void)
{
    for(int ctrl = 0; ctrl <2; ctrl++)
    {
        AdcTypedef* adc = (AdcTypedef*)adcDesc[ctrl].base;
        irq_disable(adcDesc[ctrl].irq); 
        /*  Reset the controller */
        adc->AFEC_CR = AFEC_CR_SWRST;
        /* power off*/
        mcu_peri_clk_disable(adcDesc[ctrl].irq);
    }  
}


/** 
 * @fn int adc_channel_is_ready(const int channel)
 * @brief check the adc channel is ready or not
 * @param[in] channel 
 * @return int
 */
int adc_channel_is_ready(const int channel)
{
	if(channel  < ADC_CH_MAX)
	{
		for(int i = 0; i < 5; i++)
		{
			if(adc_value_array[i][channel] == INVALID_VOLTAGE)
			{
				return 0;
			}
		}
		return 1;	
	}
	return 0;	
}

/** 
 * @fn void ADC_Handler(unsigned char ctrl)
 * @brief adc hardware interrupt routine 
 * @param[in] ctrl 
 * @return none
 */
void ADC_Handler(unsigned char ctrl)
{
    if(ctrl >= ADC_CTRL_NUM)
        return;
	AdcTypedef* adc = (AdcTypedef*)adcDesc[ctrl].base;
	int isr = adc->AFEC_ISR;
	for(int i = 0; i <12; i++)
	{
		if((isr&(1<<i))==(1<<i))
		{
			adc->AFEC_CSELR = i;
			adc_value_array[index][i+ctrl*12] = adc->AFEC_CDR;
            //disable this channel
            adc->AFEC_IDR = 1<<i;     
		}	
	}
    if(++index >= 5)
	{
		index = 0;
	}      
}

/** 
 * @fn void ADC0_Handler(void)
 * @brief adc0 hardware interrupt routine 
 * @param[in] none 
 * @return none
*/
void ADC0_Handler(void)
{
   ADC_Handler(0); 
}


/** 
 * @fn void ADC1_Handler(void)
 * @brief adc1 hardware interrupt routine 
 * @param[in] none 
 * @return none
*/
void ADC1_Handler(void)
{
    ADC_Handler(1);
} 

