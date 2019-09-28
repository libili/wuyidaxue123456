/*****************************************************************************/
/**
*  @file      TC.C
*  @brief     <b> TC C File </b>
*  @details   File functionality description:
*  This file is to implement RTT driver.
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
#include "../INC/SAMV70.H"
#include "../INC/RTT.H"
typedef struct {
  volatile unsigned int RTT_MR; /**< \brief (Rtt Offset: 0x00) Mode Register */
  volatile unsigned int RTT_AR; /**< \brief (Rtt Offset: 0x04) Alarm Register */
  volatile const unsigned int RTT_VR; /**< \brief (Rtt Offset: 0x08) Value Register */
  volatile const unsigned int RTT_SR; /**< \brief (Rtt Offset: 0x0C) Status Register */
}RttTypedef;
#define RTT_READ_LIMIT  10
#define ALMIEN        (1u<<16)
#define ALMS          (1u)
#define RTTRST        (1u<<18)
typedef struct
{
    unsigned int base;
    unsigned int irq;
    unsigned int sclk;
}RttDescTypedef;


const static RttDescTypedef rttDesc[1]={
    {0x400E1830,3,32000},
};

typedef void (*RTT_Callback_Type)(void);
static RTT_Callback_Type rtt_call_back[1];

/**
 * @fn void rtt_init(RTT_ConfigTy* config)
 * @brief  rtt hardware init with the config
 *  
 * @return none
 *  
 */
void rtt_init(RTT_ConfigTy* config)
{
    RttTypedef* rtt = (RttTypedef*)rttDesc[0].base;
    rtt->RTT_SR;
    rtt->RTT_AR = 0xFFFFFFFF;
    rtt->RTT_MR = (rttDesc[0].sclk*config->interval/1000)|RTTRST;
}

/**
 * @fn unsigned int rtt_getValue(void)
 * @brief  getting the rtt value
 *  
 * @return unsigned int
 *  
 */
unsigned int rtt_getValue(void)
{
    RttTypedef* rtt = (RttTypedef*)rttDesc[0].base;
    int counter = 0,value1,value2;
    do{
        value1 = rtt->RTT_VR;
        value2 = rtt->RTT_VR;
        counter++;
    }while((value1!=value2) && (counter > RTT_READ_LIMIT));
    return value1;
}

/**
 * @fn rtt_setAlarm(const unsigned int alarm,void(*call_back)(void))
 * @brief  setting Alarm with or without callback function
 *  
 * @return none
 *  
 */
void rtt_setAlarm(const unsigned int alarm,void(*call_back)(void))
{
    RttTypedef* rtt = (RttTypedef*)rttDesc[0].base;
    rtt->RTT_MR &= ~ALMIEN;
    rtt->RTT_AR = alarm;
    if(0 != call_back)
    {
       rtt_call_back[0] = call_back;
       irq_enable(rttDesc[0].irq);
       rtt->RTT_MR |= ALMIEN;
    }    
}


/**
 * @fn void RTT_Handler(void)
 * @brief  RTT hardware interrupt routine
 *  
 * @return none
 *  
 */
void RTT_Handler(void)
{
   RttTypedef* rtt = (RttTypedef*)rttDesc[0].base;
   int sr = rtt->RTT_SR;
   if((sr&ALMS)&&(0 != rtt_call_back[0]))
   {
       rtt_call_back[0]();
   }    
       
}



