/*****************************************************************************/
/**
*  @file      PWM.C
*  @brief     <b> PWM C File </b>
*  @details   File functionality description:
*  This file is to implement PWM driver.
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
#include "../INC/MCU.H"
#include "../INC/PWM.H"
#include "../INC/SAMV70.H"


typedef struct {
  volatile 	int PWM_CMR;     /**< \brief (PwmCh_num Offset: 0x0) PWM Channel Mode Register */
  volatile 	int PWM_CDTY;    /**< \brief (PwmCh_num Offset: 0x4) PWM Channel Duty Cycle Register */
  volatile  int PWM_CDTYUPD; /**< \brief (PwmCh_num Offset: 0x8) PWM Channel Duty Cycle Update Register */
  volatile 	int PWM_CPRD;    /**< \brief (PwmCh_num Offset: 0xC) PWM Channel Period Register */
  volatile  int PWM_CPRDUPD; /**< \brief (PwmCh_num Offset: 0x10) PWM Channel Period Update Register */
  volatile const  int PWM_CCNT;    /**< \brief (PwmCh_num Offset: 0x14) PWM Channel Counter Register */
  volatile 	int PWM_DT;      /**< \brief (PwmCh_num Offset: 0x18) PWM Channel Dead Time Register */
  volatile  int PWM_DTUPD;   /**< \brief (PwmCh_num Offset: 0x1C) PWM Channel Dead Time Update Register */
} PwmCh_num;
/** \brief PwmCmp hardware registers */
typedef struct {
  volatile 	int PWM_CMPV;    /**< \brief (PwmCmp Offset: 0x0) PWM Comparison 0 Value Register */
  volatile  int PWM_CMPVUPD; /**< \brief (PwmCmp Offset: 0x4) PWM Comparison 0 Value Update Register */
  volatile 	int PWM_CMPM;    /**< \brief (PwmCmp Offset: 0x8) PWM Comparison 0 Mode Register */
  volatile  int PWM_CMPMUPD; /**< \brief (PwmCmp Offset: 0xC) PWM Comparison 0 Mode Update Register */
}PwmCmp;

#define PWMCMP_NUMBER 8
#define PWMCH_NUM_NUMBER 4

typedef struct
{
    volatile 	int  PWM_CLK;                      /**< \brief (Pwm Offset: 0x00) PWM Clock Register */
    volatile  	int  PWM_ENA;                      /**< \brief (Pwm Offset: 0x04) PWM Enable Register */
    volatile  	int  PWM_DIS;                      /**< \brief (Pwm Offset: 0x08) PWM Disable Register */
    volatile const  int  PWM_SR;                       /**< \brief (Pwm Offset: 0x0C) PWM Status Register */
    volatile  	int  PWM_IER1;                     /**< \brief (Pwm Offset: 0x10) PWM Interrupt Enable Register 1 */
    volatile  	int  PWM_IDR1;                     /**< \brief (Pwm Offset: 0x14) PWM Interrupt Disable Register 1 */
    volatile const  int  PWM_IMR1;                     /**< \brief (Pwm Offset: 0x18) PWM Interrupt Mask Register 1 */
    volatile const  int  PWM_ISR1;                     /**< \brief (Pwm Offset: 0x1C) PWM Interrupt Status Register 1 */
    volatile 	int  PWM_SCM;                      /**< \brief (Pwm Offset: 0x20) PWM Sync Channels Mode Register */
    volatile  	int  PWM_DMAR;                     /**< \brief (Pwm Offset: 0x24) PWM DMA Register */
    volatile 	int  PWM_SCUC;                     /**< \brief (Pwm Offset: 0x28) PWM Sync Channels Update Control Register */
    volatile 	int  PWM_SCUP;                     /**< \brief (Pwm Offset: 0x2C) PWM Sync Channels Update Period Register */
    volatile  	int  PWM_SCUPUPD;                  /**< \brief (Pwm Offset: 0x30) PWM Sync Channels Update Period Update Register */
    volatile  	int  PWM_IER2;                     /**< \brief (Pwm Offset: 0x34) PWM Interrupt Enable Register 2 */
    volatile  	int  PWM_IDR2;                     /**< \brief (Pwm Offset: 0x38) PWM Interrupt Disable Register 2 */
    volatile const  int  PWM_IMR2;                     /**< \brief (Pwm Offset: 0x3C) PWM Interrupt Mask Register 2 */
    volatile const  int  PWM_ISR2;                     /**< \brief (Pwm Offset: 0x40) PWM Interrupt Status Register 2 */
    volatile 	int  PWM_OOV;                      /**< \brief (Pwm Offset: 0x44) PWM Output Override Value Register */
    volatile 	int  PWM_OS;                       /**< \brief (Pwm Offset: 0x48) PWM Output Selection Register */
    volatile  	int  PWM_OSS;                      /**< \brief (Pwm Offset: 0x4C) PWM Output Selection Set Register */
    volatile  	int  PWM_OSC;                      /**< \brief (Pwm Offset: 0x50) PWM Output Selection Clear Register */
    volatile  	int  PWM_OSSUPD;                   /**< \brief (Pwm Offset: 0x54) PWM Output Selection Set Update Register */
    volatile  	int  PWM_OSCUPD;                   /**< \brief (Pwm Offset: 0x58) PWM Output Selection Clear Update Register */
    volatile 	int  PWM_FMR;                      /**< \brief (Pwm Offset: 0x5C) PWM Fault Mode Register */
    volatile const  int  PWM_FSR;                      /**< \brief (Pwm Offset: 0x60) PWM Fault Status Register */
    volatile  	int  PWM_FCR;                      /**< \brief (Pwm Offset: 0x64) PWM Fault Clear Register */
    volatile 	int  PWM_FPV1;                     /**< \brief (Pwm Offset: 0x68) PWM Fault Protection Value Register 1 */
    volatile 	int  PWM_FPE;                      /**< \brief (Pwm Offset: 0x6C) PWM Fault Protection Enable Register */
    volatile const  int  Reserved1[3];
    volatile 	int  PWM_ELMR[8];                  /**< \brief (Pwm Offset: 0x7C) PWM Event Line 0 Mode Register */
    volatile const  int  Reserved2[1];
    volatile 	int  PWM_SSPR;                     /**< \brief (Pwm Offset: 0xA0) PWM Spread Spectrum Register */
    volatile  	int  PWM_SSPUP;                    /**< \brief (Pwm Offset: 0xA4) PWM Spread Spectrum Update Register */
    volatile const  int  Reserved3[2];
    volatile 	int  PWM_SMMR;                     /**< \brief (Pwm Offset: 0xB0) PWM Stepper Motor Mode Register */
    volatile const  int  Reserved4[3];
    volatile 	int  PWM_FPV2;                     /**< \brief (Pwm Offset: 0xC0) PWM Fault Protection Value 2 Register */
    volatile const  int  Reserved5[8];
    volatile  	int  PWM_WPCR;                     /**< \brief (Pwm Offset: 0xE4) PWM Write Protection Control Register */
    volatile const  int  PWM_WPSR;                     /**< \brief (Pwm Offset: 0xE8) PWM Write Protection Status Register */
    volatile const  int  Reserved6[17];
    PwmCmp    PWM_CMP[PWMCMP_NUMBER];       /**< \brief (Pwm Offset: 0x130) 0 .. 7 */
    volatile const  int  Reserved7[20];
    PwmCh_num PWM_CH_NUM[PWMCH_NUM_NUMBER]; /**< \brief (Pwm Offset: 0x200) ch_num = 0 .. 3 */
    volatile const  int  Reserved8[96];
    volatile  	int  PWM_CMUPD0;                   /**< \brief (Pwm Offset: 0x400) PWM Channel Mode Update Register (ch_num = 0) */
    volatile const  int  Reserved9[7];
    volatile  	int  PWM_CMUPD1;                   /**< \brief (Pwm Offset: 0x420) PWM Channel Mode Update Register (ch_num = 1) */
    volatile const  int  Reserved10[2];
    volatile 	int  PWM_ETRG1;                    /**< \brief (Pwm Offset: 0x42C) PWM External Trigger Register (trg_num = 1) */
    volatile 	int  PWM_LEBR1;                    /**< \brief (Pwm Offset: 0x430) PWM Leading-Edge Blanking Register (trg_num = 1) */
    volatile const  int  Reserved11[3];
    volatile  	int  PWM_CMUPD2;                   /**< \brief (Pwm Offset: 0x440) PWM Channel Mode Update Register (ch_num = 2) */
    volatile const  int  Reserved12[2];
    volatile 	int  PWM_ETRG2;                    /**< \brief (Pwm Offset: 0x44C) PWM External Trigger Register (trg_num = 2) */
    volatile 	int  PWM_LEBR2;                    /**< \brief (Pwm Offset: 0x450) PWM Leading-Edge Blanking Register (trg_num = 2) */
    volatile const  int  Reserved13[3];
    volatile  	int  PWM_CMUPD3;                   /**< \brief (Pwm Offset: 0x460) PWM Channel Mode Update Register (ch_num = 3) */
    volatile const  int  Reserved14[2];
    volatile 	int  PWM_ETRG3;                    /**< \brief (Pwm Offset: 0x46C) PWM External Trigger Register (trg_num = 3) */
    volatile 	int  PWM_LEBR3;                    /**< \brief (Pwm Offset: 0x470) PWM Leading-Edge Blanking Register (trg_num = 3) */
    volatile const  int  Reserved15[6];
    volatile 	int  PWM_ETRG4;                    /**< \brief (Pwm Offset: 0x48C) PWM External Trigger Register (trg_num = 4) */
    volatile 	int  PWM_LEBR4;                    /**< \brief (Pwm Offset: 0x490) PWM Leading-Edge Blanking Register (trg_num = 4) */
}PwmTypedef;

#define PWM_HIGH_FREQ    2000
typedef struct{
	unsigned int base;
	unsigned int fclk;
	unsigned int irq;
	unsigned int pwm_hclk;/*this clock for the freq large 	than 50k*/
	unsigned int pwm_lclk;/*this clock for the freq little 	than 50k*/
}PwmDescTypedef;

static const PwmDescTypedef  pwmDesc[2]={
	{0x40020000,100000000,31,100000000,500000},
	{0x4005C000,100000000,60,100000000,500000},
};

typedef struct
{
	int hw_init:1;
	int ch_init[PWMCH_NUM_NUMBER];
	int period[PWMCH_NUM_NUMBER];
	int duty[PWMCH_NUM_NUMBER];
}PwmHalStatusDef;

PwmHalStatusDef pwmHalStatus[2];
#define PWM_CHANNEL_MAX   8


/**
 * @fn void pwm_init(int channel,PWM_ConfigTy* config)unsigned int trng_getValue(void)
 * @brief initial the pwm hardware with the config
 * @param [in] config 
 * @return none
 *  
 */
void pwm_init(int channel,PWM_ConfigTy* config)
{
	if(channel < PWM_CHANNEL_MAX )
	{
		int hal_ch = channel/PWMCH_NUM_NUMBER;
		PwmTypedef* pwm = (PwmTypedef*)pwmDesc[hal_ch].base;
		if(!pwmHalStatus[hal_ch].hw_init)
		{
			pwmHalStatus[hal_ch].hw_init = 1;
			/*first to enable the pwm hardware */
			mcu_peri_clk_enable(pwmDesc[hal_ch].irq);
			/*disable all interrupt*/
			pwm->PWM_IDR1 = 0xF000F;
			pwm->PWM_IDR2 = 0xFFFF09;
			
			/*clock A for the high frequnce 
			  clock B for the low  frequnce 	
			*/
			pwm->PWM_CLK = pwmDesc[hal_ch].fclk/pwmDesc[hal_ch].pwm_hclk|\
						   ((pwmDesc[hal_ch].fclk/pwmDesc[hal_ch].pwm_lclk)<<16);
			pwm->PWM_SCM = 0;				
		}
		//first diable the channel
		pwm->PWM_DIS = 1 << channel%PWMCH_NUM_NUMBER;
		if(config->period > PWM_HIGH_FREQ)
		{
			pwm->PWM_CH_NUM[channel%PWMCH_NUM_NUMBER].PWM_CMR = 11;/*select ClockA*/
		}
		else
		{
			pwm->PWM_CH_NUM[channel%PWMCH_NUM_NUMBER].PWM_CMR = 12;/*select ClockB*/
		}		
		pwm_setPeriod(channel,config->period);
		pwm_setDuty(channel,config->duty);
		pwmHalStatus[hal_ch].ch_init[channel%PWMCH_NUM_NUMBER] = 1;	
	}	
} 

/**
 * @fn void pwm_setDuty(int channel,int duty)
 * @brief setting the pwm duty 
 * @param [in] channel
 * @param [in] duty

 * @return none
 *  
 */
void pwm_setDuty(int channel,int duty)
{
	if(channel < PWM_CHANNEL_MAX )
	{
		int hal_ch = channel/PWMCH_NUM_NUMBER;
		PwmTypedef* pwm = (PwmTypedef*)pwmDesc[hal_ch].base;
		unsigned short targetVal = pwm->PWM_CH_NUM[channel%PWMCH_NUM_NUMBER].PWM_CPRD *duty/1000;
        pwmHalStatus[hal_ch].duty[channel%PWMCH_NUM_NUMBER] = duty;
        if(targetVal < 1)
        {
            targetVal = 1;
        }   
		if((pwm->PWM_SR&(1<< channel%PWMCH_NUM_NUMBER)) == 0)
		{ 
			pwm->PWM_CH_NUM[channel%PWMCH_NUM_NUMBER].PWM_CDTY = targetVal;
		}	
		else
		{
			pwm->PWM_CH_NUM[channel%PWMCH_NUM_NUMBER].PWM_CDTYUPD = targetVal;
		}	
	}
}

/**
 * @fn void pwm_setPeriod(int channel,int period)
 * @brief setting the pwm period 
 * @param [in] channel
 * @param [in] period

 * @return none
 *  
 */
void pwm_setPeriod(int channel,int period)
{
	if(channel < PWM_CHANNEL_MAX )
	{
		int hal_ch = channel/PWMCH_NUM_NUMBER;
		PwmTypedef* pwm = (PwmTypedef*)pwmDesc[hal_ch].base;
		pwm->PWM_CH_NUM[channel%PWMCH_NUM_NUMBER].PWM_CMR = (period > PWM_HIGH_FREQ?11:12);	
        pwmHalStatus[hal_ch].period[channel%PWMCH_NUM_NUMBER] = period;
		if((pwm->PWM_SR&(1<< channel%PWMCH_NUM_NUMBER)) == 0)
		{

			pwm->PWM_CH_NUM[channel%PWMCH_NUM_NUMBER].PWM_CPRD = \
				(period > PWM_HIGH_FREQ?pwmDesc[hal_ch].pwm_hclk/period:pwmDesc[hal_ch].pwm_lclk/period);
				
		}	
		else
		{
			pwm->PWM_CH_NUM[channel%PWMCH_NUM_NUMBER].PWM_CPRDUPD = \
				(period > PWM_HIGH_FREQ?pwmDesc[hal_ch].pwm_hclk/period:pwmDesc[hal_ch].pwm_lclk/period);	
		}	
	}	
}

/**
 * @fn void pwm_enable(int channel)
 * @brief enable the pwm channel to output
 * @param [in] channel

 * @return none
 *  
 */
void pwm_enable(int channel)
{
	if(channel < PWM_CHANNEL_MAX )
	{
		PwmTypedef* pwm = (PwmTypedef*)pwmDesc[channel/PWMCH_NUM_NUMBER].base;
		pwm->PWM_ENA = 1 << channel%PWMCH_NUM_NUMBER;
	}	
}

/**
 * @fn void pwm_disable(int channel)
 * @brief disable the pwm channel to output
 * @param [in] channel

 * @return none
 *  
 */
void pwm_disable(int channel)
{
	if(channel < PWM_CHANNEL_MAX )
	{
		PwmTypedef* pwm = (PwmTypedef*)pwmDesc[channel/PWMCH_NUM_NUMBER].base;
		pwm->PWM_DIS = 1 << channel%PWMCH_NUM_NUMBER;
	}
}


/**
 * @fn void pwm_deint(int channel)
 * @brief deinit the pwm hardware
 * @param [in] channel

 * @return none
 *  
 */
void pwm_deinit(int channel)
{
	if(channel >= PWM_CHANNEL_MAX )
		return;
	int hal_ch = channel/PWMCH_NUM_NUMBER;
	pwmHalStatus[hal_ch].ch_init[channel%PWMCH_NUM_NUMBER] = 0;	
	int i;
	for(i = 0; i < PWMCH_NUM_NUMBER; i++)
	{
		if(pwmHalStatus[hal_ch].ch_init[i])
		{
			break;
		}	
	}
	if(i == PWMCH_NUM_NUMBER)
	{
		pwmHalStatus[hal_ch].hw_init = 0;
			/*first to enable the pwm hardware */
		mcu_peri_clk_disable(pwmDesc[hal_ch].irq);
	}	
}



