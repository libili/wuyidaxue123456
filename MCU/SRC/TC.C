/*****************************************************************************/
/**
*  @file      TC.C
*  @brief     <b> TC C File </b>
*  @details   File functionality description:
*  This file is to implement TC driver.
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
#include "../INC/TC.H"
#include "../INC/SAMV70.H"

typedef struct {
  volatile  		int TC_CCR;       /**< \brief (TcChannel Offset: 0x0) Channel Control Register */
  volatile 			int TC_CMR;       /**< \brief (TcChannel Offset: 0x4) Channel Mode Register */
  volatile 			int TC_SMMR;      /**< \brief (TcChannel Offset: 0x8) Stepper Motor Mode Register */
  volatile 	const  	int TC_RAB;       /**< \brief (TcChannel Offset: 0xC) Register AB */
  volatile 	const  	int TC_CV;        /**< \brief (TcChannel Offset: 0x10) Counter Value */
  volatile 			int TC_RA;        /**< \brief (TcChannel Offset: 0x14) Register A */
  volatile 			int TC_RB;        /**< \brief (TcChannel Offset: 0x18) Register B */
  volatile 			int TC_RC;        /**< \brief (TcChannel Offset: 0x1C) Register C */
  volatile const  	int TC_SR;        /**< \brief (TcChannel Offset: 0x20) Status Register */
  volatile  		int TC_IER;       /**< \brief (TcChannel Offset: 0x24) Interrupt Enable Register */
  volatile  		int TC_IDR;       /**< \brief (TcChannel Offset: 0x28) Interrupt Disable Register */
  volatile 	const  	int TC_IMR;       /**< \brief (TcChannel Offset: 0x2C) Interrupt Mask Register */
  volatile 			int TC_EMR;       /**< \brief (TcChannel Offset: 0x30) Extended Mode Register */
  volatile 	const  	int Reserved1[3];
} TcChannel;
/** \brief Tc hardware registers */
#define TCCHANNEL_NUMBER 3
typedef struct {
    TcChannel TC_CHANNEL[TCCHANNEL_NUMBER]; /**< \brief (Tc Offset: 0x0) channel = 0 .. 2 */
	volatile  		int  TC_BCR;                       /**< \brief (Tc Offset: 0xC0) Block Control Register */
	volatile 		int  TC_BMR;                       /**< \brief (Tc Offset: 0xC4) Block Mode Register */
	volatile  		int  TC_QIER;                      /**< \brief (Tc Offset: 0xC8) QDEC Interrupt Enable Register */
	volatile  		int  TC_QIDR;                      /**< \brief (Tc Offset: 0xCC) QDEC Interrupt Disable Register */
	volatile const  int  TC_QIMR;                      /**< \brief (Tc Offset: 0xD0) QDEC Interrupt Mask Register */
	volatile const  int  TC_QISR;                      /**< \brief (Tc Offset: 0xD4) QDEC Interrupt Status Register */
	volatile 		int  TC_FMR;                       /**< \brief (Tc Offset: 0xD8) Fault Mode Register */
	volatile const  int  Reserved1[2];
	volatile 		int  TC_WPMR;                      /**< \brief (Tc Offset: 0xE4) Write Protection Mode Register */
}TcTypedef;
#define TC_MODULE_NUM   4

#define SWTRG   (1u<<2)
#define CLKDIS  (1u<<1)
#define CLKEN   (1u<<0)

#define WAVE    (1u<<15)
#define TIMER_MCK8 (0x1u)
#define CLKSTA   (1u<<16)
#define CPCS     (1u<<4)  

typedef struct
{
    unsigned int base;
    unsigned int irq;
    unsigned int fclk;
}TcDescTypedef;


const static TcDescTypedef tcDesc[TC_MODULE_NUM]={
    {0x4000C000,23,100000000},
    {0x40010000,24,100000000},
    {0x40014000,25,100000000},
    {0x40054000,26,100000000}
};



typedef struct{
   int ch_init;
   Tc_callback cbs[TCCHANNEL_NUMBER];
}TcStatusDef;


TcStatusDef tcStatus[TC_MODULE_NUM];

/**
 * @fn void tc_init(int channel,TC_ConfigTy* config)
 * @brief init TC hardware with the config
 *  
 * @return none
 *  
 */
void tc_init(int channel,TC_ConfigTy* config)
{
    if(channel <TC_MODULE_NUM*TCCHANNEL_NUMBER)
    {
        int m_channel = channel%TCCHANNEL_NUMBER;
        int m_module = channel/TCCHANNEL_NUMBER;
        TcTypedef* tc = (TcTypedef*)tcDesc[m_module].base;
        if(tcStatus[m_module].ch_init==0)
        {
            mcu_peri_clk_enable(tcDesc[m_module].irq);
            irq_enable(tcDesc[m_module].irq);
        }
        tcStatus[m_module].ch_init |= 1<<m_channel;
        tc->TC_CHANNEL[m_channel].TC_CCR = CLKDIS; 
        tc->TC_CHANNEL[m_channel].TC_SR;
        int tarFreq = 1000000/config->peroid;
        int tarDiv = tcDesc[m_module].fclk/8/tarFreq;
        tc->TC_CHANNEL[m_channel].TC_RC = tarDiv;
        tc->TC_CHANNEL[m_channel].TC_CMR = TIMER_MCK8|WAVE|(2<<13);
        tcStatus[m_module].cbs[m_channel] = config->cb;
        tc->TC_CHANNEL[m_channel].TC_IER = CPCS;
        tc->TC_CHANNEL[m_channel].TC_CCR = CLKEN |SWTRG ;
    }    
}

/**
 * @fn unsigned int tc_getRate(int channel)
 * @brief getting the TC counter rate 0.1% unit
 *  
 * @return unsigned int
 *  
 */
unsigned int tc_getRate(int channel)
{
    if(channel <TC_MODULE_NUM*TCCHANNEL_NUMBER)
    {
        int m_channel = channel%TCCHANNEL_NUMBER;
        int m_module = channel/TCCHANNEL_NUMBER;
        TcTypedef* tc = (TcTypedef*)tcDesc[m_module].base;
       return tc->TC_CHANNEL[m_channel].TC_CV*1000/tc->TC_CHANNEL[m_channel].TC_RC;       
    }    
    return 0;
}


/**
 * @fn tc_deinit(int channel)
 * @brief deinit TC hardware 
 *  
 * @return none
 *  
 */
void tc_deinit(int channel)
{
   if(channel <TC_MODULE_NUM*TCCHANNEL_NUMBER)
   {
        int m_channel = channel%TCCHANNEL_NUMBER;
        int m_module = channel/TCCHANNEL_NUMBER;
        TcTypedef* tc = (TcTypedef*)tcDesc[m_module].base;
        tc->TC_CHANNEL[m_channel].TC_CCR = CLKDIS;
        tcStatus[m_module].ch_init &= ~(1<< m_channel);
        if(tcStatus[m_module].ch_init==0)
        {
            mcu_peri_clk_disable(tcDesc[m_module].irq);
            irq_disable(tcDesc[m_module].irq);
        }      
   }     
}

/**
 * @fn void TC_Handler(int module)
 * @brief TC intterupt routine
 *  
 * @return none
 *  
 */
void TC_Handler(int module)
{
    if(module <TC_MODULE_NUM)
    {
        TcTypedef* tc = (TcTypedef*)tcDesc[module].base;
        for(int i = 0; i <TCCHANNEL_NUMBER;i++ )
        {
            int sr = tc->TC_CHANNEL[i].TC_SR;
            if((sr&CLKSTA)!= CLKSTA)
                continue;
            if((sr&CPCS)&&(tc->TC_CHANNEL[i].TC_IMR&CPCS))
            {
                //callback
                if(tcStatus[module].cbs[i] != 0)
                {
                    tcStatus[module].cbs[i]();
                }    
            }        
        }
    }    
}

/**
 * @fn void TC0_Handler(void)
 * @brief TC0 intterupt routine
 *  
 * @return none
 *  
 */
void TC0_Handler(void)
{
    TC_Handler(0);
}



/**
 * @fn void TC1_Handler(void)
 * @brief TC1 intterupt routine
 *  
 * @return none
 *  
 */
void TC1_Handler(void)
{
    TC_Handler(1);
}


/**
 * @fn void TC2_Handler(void)
 * @brief TC2 intterupt routine
 *  
 * @return none
 *  
 */
void TC2_Handler(void)
{
    TC_Handler(2);
}


/**
 * @fn void TC3_Handler(void)
 * @brief TC3 intterupt routine
 *  
 * @return none
 *  
 */
void TC3_Handler(void)
{
    TC_Handler(3);
}

