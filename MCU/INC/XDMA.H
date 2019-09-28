/*****************************************************************************/
/**
*  @file      XDMA.C
*  @brief     <b> XDMA C File </b>
*  @details   File functionality description:
*  This file provides the xdma driver interface.
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
#ifndef SV_XDMA_H
#define SV_XDMA_H

typedef enum{
    XDMA_HSMCI,
    XDMA_SPI0_TX,
    XDMA_SPI0_RX,
    XDMA_SPI1_TX,
    XDMA_SPI1_RX,
    XDMA_QSPI_TX,
    XDMA_QSPI_RX,
    XDMA_USART0_TX,
    XDMA_USART0_RX,
    XDMA_USART1_TX,
    XDMA_USART1_RX,
    XDMA_USART2_TX,
    XDMA_USART2_RX,
    XDMA_PWM0,
    XDMA_TWIHS0_TX,
    XDMA_TWIHS0_RX,
    XDMA_TWIHS1_TX,
    XDMA_TWIHS1_RX,
    XDMA_TWIHS2_TX,
    XDMA_TWIHS2_RX,
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
    XDMA_DACC,
    XDMA_SSC_TX,
    XDMA_SSC_RX,
    XDMA_PIOA,
    XDMA_AFEC0,
    XDMA_AFEC1,
    XDMA_AES_TX,
    XDMA_AES_RX,
    XDMA_PWM1,
    XDMA_TC0,
    XDMA_TC1,
    XDMA_TC2,
    XDMA_TC3,
    XDMA_MEM = 0xFF,
}XDMA_Peripheral_TypeDef;


typedef struct{
    unsigned int srcAddr;
    unsigned int destAddr;
    unsigned int length;
    unsigned int isSrcAddrInc:1;
    unsigned int isDestAddrInc:1;
    void(*call_back)(const void*);
    void * param;
}XDMA_MsgTy;


typedef struct
{
#define MEM_TRAN 0    
#define PER_MEM_TRAN 1
#define MEM_PER_TRAN 2
    unsigned int type:2;
    XDMA_Peripheral_TypeDef perip;
}XdmaChannelDescType;

/** 
 * @fn void xdma_reset_channel(const unsigned char channel)
 * @brief   reset the channel 
 * @param[in] channel 
 * @return none
 */
void xdma_reset_channel(const unsigned char channel);


/** 
* @fn char xdma_allocate_channel(XdmaChannelDescType* channelDesc)
* @brief   get dedicated channel 
* @param[in] perip 
* @return char target channel
*/
char xdma_allocate_channel(XdmaChannelDescType* channelDesc);

/** 
* @fn void xdma_free_channel(const unsigned char channel)
* @brief     free the channel 
* @param[in] channel 
* @return none
*/
void xdma_free_channel(const unsigned char channel);



/** 
 * @fn void xdma_free_channel(const unsigned char channel)
 * @brief    free the channel 
 * @param[in] channel 
 * @return none
 */
void xdma_transfer(const unsigned channel,XDMA_MsgTy* msg);

#endif
