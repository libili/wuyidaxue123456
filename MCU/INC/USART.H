/*****************************************************************************/
/**
*  @file      USART.H
*  @brief     <b> USART H File </b>
*  @details   File functionality description:
*  This file provides the USART driver interface.
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
#ifndef __SV_USART
#define __SV_USART
#include "../INC/UART.H"
#include "../INC/SPI.H"

typedef enum{
    USART_NORMAL,
    USART_FLOW = 0x02,
    USART_LIN_MASTER= 0x0A,
    USART_LIN_SLAVE,
    USART_SPI_MASTER=0x0E,
    USART_SPI_SLAVE
}USART_MODE_TYPE;

typedef struct{
    USART_MODE_TYPE usart_mode;
    void* pconfig;
}USART_ConfigTy;

typedef struct  {
#define USART_WE	0
#define USART_RD    1
    unsigned char flags;    //
    unsigned char* buf;
    unsigned int  len;      //transfer length
    void (*complete_handler)(const void*);
}USART_MsgTy;

/**
 * @fn void usart_init(const int channel,const USART_ConfigTy* config)
 * @brief initial the usart hardware with the config
 * @param [in] ch 
 * @param [in] config 
 * @return none
 *  
 */
void usart_init(const int channel,const USART_ConfigTy* config);

/**
 * @fn void usart_transfer(const int channel,const USART_MsgTy* msg)
 * @brief start to transfer a usart message
 * @param [in] ch 
 * @param [in] msg 
 * @return none
 *  
 */
void usart_transfer(const int channel,const USART_MsgTy* msg);

/** 
 * @fn  void usart_deinit(const int channel)
 * @brief to deinit usart driver
 * @param[in] ch the usart channel
 * @return none 
*/
void usart_deinit(const int channel);


#endif/* __SV_USART  */
