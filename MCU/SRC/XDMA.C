/**
*  @file      XDMA.C
*  @brief     <b> XDMA C File </b>
*  @details   File functionality description:
*  This file  is to implement the XDMA driver.
*  @author    Axford
*  @version   v0.1
*  @date      2016.03.15
*  @bug       see Release Note
*  @par History:          
*   v0.1: Axford, 2016.03.15, initial version.
*  @warning   usage policy:
* Copyright (c) 2016  Desay SV Automotive Co., Ltd - All Rights Reserved
* Reproduction and Communication of this document is strictly prohibited
* unless specifically authorized in writing by DesaySV.
* FOR MORE INFORMATION PLEASE READ CAREFULLY THE LICENSE AGREEMENT LOCATED
* IN THE ROOT DIRECTORY OF THIS SOFTWARE PACKAGE.
*/
#include "../INC/XDMA.H"
#include "../INC/MCU.H"
typedef struct {
  volatile          unsigned int XDMAC_CIE;     /**< \brief (XdmacChid Offset: 0x0) Channel Interrupt Enable Register */
  volatile          unsigned int XDMAC_CID;     /**< \brief (XdmacChid Offset: 0x4) Channel Interrupt Disable Register */
  volatile          unsigned int XDMAC_CIM;     /**< \brief (XdmacChid Offset: 0x8) Channel Interrupt Mask Register */
  volatile const    unsigned int XDMAC_CIS;     /**< \brief (XdmacChid Offset: 0xC) Channel Interrupt Status Register */
  volatile          unsigned int XDMAC_CSA;     /**< \brief (XdmacChid Offset: 0x10) Channel Source Address Register */
  volatile          unsigned int XDMAC_CDA;     /**< \brief (XdmacChid Offset: 0x14) Channel Destination Address Register */
  volatile          unsigned int XDMAC_CNDA;    /**< \brief (XdmacChid Offset: 0x18) Channel Next Descriptor Address Register */
  volatile          unsigned int XDMAC_CNDC;    /**< \brief (XdmacChid Offset: 0x1C) Channel Next Descriptor Control Register */
  volatile          unsigned int XDMAC_CUBC;    /**< \brief (XdmacChid Offset: 0x20) Channel Microblock Control Register */
  volatile          unsigned int XDMAC_CBC;     /**< \brief (XdmacChid Offset: 0x24) Channel Block Control Register */
  volatile          unsigned int XDMAC_CC;      /**< \brief (XdmacChid Offset: 0x28) Channel Configuration Register */
  volatile          unsigned int XDMAC_CDS_MSP; /**< \brief (XdmacChid Offset: 0x2C) Channel Data Stride Memory Set Pattern */
  volatile          unsigned int XDMAC_CSUS;    /**< \brief (XdmacChid Offset: 0x30) Channel Source Microblock Stride */
  volatile          unsigned int XDMAC_CDUS;    /**< \brief (XdmacChid Offset: 0x34) Channel Destination Microblock Stride */
  volatile const    unsigned int Reserved1[2];
} XdmacChid;
/** \brief Xdmac hardware registers */
#define XDMACCHID_NUMBER 24
typedef struct {
  volatile          unsigned int  XDMAC_GTYPE;                  /**< \brief (Xdmac Offset: 0x00) Global Type Register */
  volatile const    unsigned int  XDMAC_GCFG;                   /**< \brief (Xdmac Offset: 0x04) Global Configuration Register */
  volatile          unsigned int  XDMAC_GWAC;                   /**< \brief (Xdmac Offset: 0x08) Global Weighted Arbiter Configuration Register */
  volatile          unsigned int  XDMAC_GIE;                    /**< \brief (Xdmac Offset: 0x0C) Global Interrupt Enable Register */
  volatile          unsigned int  XDMAC_GID;                    /**< \brief (Xdmac Offset: 0x10) Global Interrupt Disable Register */
  volatile const    unsigned int  XDMAC_GIM;                    /**< \brief (Xdmac Offset: 0x14) Global Interrupt Mask Register */
  volatile const    unsigned int  XDMAC_GIS;                    /**< \brief (Xdmac Offset: 0x18) Global Interrupt Status Register */
  volatile          unsigned int  XDMAC_GE;                     /**< \brief (Xdmac Offset: 0x1C) Global Channel Enable Register */
  volatile          unsigned int  XDMAC_GD;                     /**< \brief (Xdmac Offset: 0x20) Global Channel Disable Register */
  volatile const    unsigned int  XDMAC_GS;                     /**< \brief (Xdmac Offset: 0x24) Global Channel Status Register */
  volatile          unsigned int  XDMAC_GRS;                    /**< \brief (Xdmac Offset: 0x28) Global Channel Read Suspend Register */
  volatile          unsigned int  XDMAC_GWS;                    /**< \brief (Xdmac Offset: 0x2C) Global Channel Write Suspend Register */
  volatile          unsigned int  XDMAC_GRWS;                   /**< \brief (Xdmac Offset: 0x30) Global Channel Read Write Suspend Register */
  volatile          unsigned int  XDMAC_GRWR;                   /**< \brief (Xdmac Offset: 0x34) Global Channel Read Write Resume Register */
  volatile          unsigned int  XDMAC_GSWR;                   /**< \brief (Xdmac Offset: 0x38) Global Channel Software Request Register */
  volatile const    unsigned int  XDMAC_GSWS;                   /**< \brief (Xdmac Offset: 0x3C) Global Channel Software Request Status Register */
  volatile          unsigned int  XDMAC_GSWF;                   /**< \brief (Xdmac Offset: 0x40) Global Channel Software Flush Request Register */
  volatile const    unsigned int  Reserved1[3];
  XdmacChid         XDMAC_CHID[XDMACCHID_NUMBER]; /**< \brief (Xdmac Offset: 0x50) chid = 0 .. 23 */
} Xdmac;

typedef struct{
	unsigned int base;
	unsigned int fclk;
	unsigned int irq;
}XdmaDescTypedef;

#define CC_TYPE(x)  (((x != MEM_TRAN))<<0)
#define CC_DSYNC(x) (((x)&0x01)<<4)
#define CC_SAM(x)   (((x)&0x01)<<16)
#define CC_DAM(x)   (((x)&0x01)<<18)
#define CC_SAM_MASK  (0x03U<<16)
#define CC_DAM_MASK  (0x03U<<18)
#define CC_PERID(x)   ((x&0x7F)<<24)
#define CC_DEF_IF       (0x03U<<13)
#define DISABLE_MASK    0x7F
typedef void (*XdmaCallBackType)(const void *);
XdmaCallBackType dma_call_backs[XDMACCHID_NUMBER];
void* dma_channel_params[XDMACCHID_NUMBER];
int dma_ccs[XDMACCHID_NUMBER];
const XdmaDescTypedef XdmaDesc[1]={
    {0x40078000,100000000,58}
};


 /* @brief define some dedicated channel here*/
static const char dedicated_channels[]={
    XDMA_MEM,
    XDMA_SPI0_TX,
    XDMA_SPI0_RX,
    XDMA_SPI1_TX,
    XDMA_SPI1_RX,
    XDMA_USART0_TX,
    XDMA_USART0_RX,
    XDMA_USART1_TX,
    XDMA_USART1_RX,
    XDMA_USART2_TX,
    XDMA_USART2_RX,
    XDMA_UART0_TX,
    XDMA_UART0_RX,
    XDMA_UART1_TX,
    XDMA_UART1_RX,
    XDMA_UART2_TX,
    XDMA_UART2_RX,
    XDMA_UART3_TX,
    XDMA_UART3_RX,
    XDMA_UART4_TX,
    XDMA_UART4_RX,
    
};

/** 
 * @fn void xdma_reset_channel(const unsigned char channel)
 * @brief   reset the channel 
 * @param[in] channel 
 * @return none
 */
void xdma_reset_channel(const unsigned char channel)
{
    Xdmac* xdma = (Xdmac*)(XdmaDesc[0].base);
    if(channel < XDMACCHID_NUMBER)
    {
        //disable the channel first
        xdma->XDMAC_GD = 1<< channel;
        /*Reading XDMAC_CIS to Clear the pending Interrupt Status bit*/
        xdma->XDMAC_CHID[channel].XDMAC_CIS;
        xdma->XDMAC_CHID[channel].XDMAC_CC = dma_ccs[channel];
        xdma->XDMAC_CHID[channel].XDMAC_CNDC = 0;
        xdma->XDMAC_CHID[channel].XDMAC_CBC = 0;
        xdma->XDMAC_CHID[channel].XDMAC_CDS_MSP = 0;
        xdma->XDMAC_CHID[channel].XDMAC_CSUS = 0;
        xdma->XDMAC_CHID[channel].XDMAC_CDUS = 0;
    }    
       
}


/** 
* @fn char xdma_allocate_channel(XdmaChannelDescType* channelDesc)
* @brief   get dedicated channel 
* @param[in] perip 
* @return char target channel
*/
char xdma_allocate_channel(XdmaChannelDescType* channelDesc)
{
   signed char tarChannel = -1;
    Xdmac* xdma = (Xdmac*)(XdmaDesc[0].base);
    /*check if it's a dedicated channel*/
    for(int i = 0; i< sizeof(dedicated_channels);i++)
    {
        if(channelDesc->perip == dedicated_channels[i])
        {
            tarChannel = i;
            break; 
        }    
    }
    if(tarChannel== -1)
    {
        /*else get a free channel*/
        for(int i = sizeof(dedicated_channels); i < XDMACCHID_NUMBER; i++ )
        {
            if((xdma->XDMAC_GS&(1 << i))== 0)
            {
                tarChannel = i;
                break;
            }    
        }  
    }    
    if(tarChannel != -1)
    {
        
        dma_ccs[tarChannel] = CC_TYPE(channelDesc->type)|CC_DSYNC(channelDesc->type == MEM_PER_TRAN)|CC_PERID(channelDesc->perip)|CC_DEF_IF;
        
    }    
    return tarChannel;
}


/** 
* @fn void xdma_free_channel(const unsigned char channel)
* @brief     free the channel 
* @param[in] channel 
* @return none
*/
void xdma_free_channel(const unsigned char channel)
{
    if(channel < XDMACCHID_NUMBER)
    {
        Xdmac* xdma = (Xdmac*)(XdmaDesc[0].base);
        //disable the channel
        xdma->XDMAC_GD = 1<< channel;
        
        /*unregister DMA call_back function*/
        dma_call_backs[channel] = 0;
        
        //disable channel interrupt 
        xdma->XDMAC_CHID[channel].XDMAC_CID = DISABLE_MASK;
        
        //clear interrupt flag
        xdma->XDMAC_CHID[channel].XDMAC_CIS;
    }    
 
}



/** 
 * @fn void xdma_free_channel(const unsigned char channel)
 * @brief    free the channel 
 * @param[in] channel 
 * @return none
 */
void xdma_transfer(const unsigned channel,XDMA_MsgTy* msg)
{
    if(channel < XDMACCHID_NUMBER)
    {
        Xdmac* xdma = (Xdmac*)(XdmaDesc[0].base);
        xdma->XDMAC_CHID[channel].XDMAC_CSA = msg->srcAddr;
        xdma->XDMAC_CHID[channel].XDMAC_CDA = msg->destAddr;
        xdma->XDMAC_CHID[channel].XDMAC_CUBC = msg->length;
        xdma->XDMAC_CHID[channel].XDMAC_CC = dma_ccs[channel]|CC_SAM(msg->isSrcAddrInc)|CC_DAM(msg->isDestAddrInc);
        /*Register dma callback function */
        dma_call_backs[channel] = msg->call_back;
        dma_channel_params[channel] =msg->param;
        /*Enable the Microblock interrupt */
        xdma->XDMAC_CHID[channel].XDMAC_CIE = 1<<0;
        
        /*enable the Channel x Interrupt Enable bit */
        xdma->XDMAC_GIE = 1<<channel;
        
        /*Enable IRQ*/
        irq_enable(XdmaDesc[0].irq);
        /* Enable channel */
        xdma->XDMAC_GE = 1<<channel;
    }    
}


/** 
 * @fn void XDMA_Handler(void)
 * @brief    Handle the XDMA interrupt
 * @param[in] none 
 * @return none
 */
void XDMA_Handler(void)
{
    Xdmac* xdma = (Xdmac*)(XdmaDesc[0].base);
    int gis = xdma->XDMAC_GIS;
    for(int i = 0; i < XDMACCHID_NUMBER; i ++)
    {
        if(gis&(1<<i))
        {
            /*Reading CIS to clear interrupt flag*/
            xdma->XDMAC_CHID[i].XDMAC_CIS;
            if(dma_call_backs[i]!= 0)
            {
                dma_call_backs[i](dma_channel_params[i]);
            }    
        }    

    }
}


