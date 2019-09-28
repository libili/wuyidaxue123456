/*****************************************************************************/
/**
*  @file      UART.H
*  @brief     <b> UART H File </b>
*  @details   File functionality description:
*  This file provides the uart driver interface.
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
#ifndef __SV_UART
#define __SV_UART

typedef enum
{
	UART_CH_0,
	UART_CH_1,
	UART_CH_2,
	UART_CH_3,
	UART_CH_4,
	UART_CH_MAX,
}UART_CH_TY;
typedef struct
{
	unsigned int baudrate:24;
	unsigned int parity:2;/*None=0,Even =1 or Odd = 2, */
	unsigned int filter:1;/*Enable=1 Disable=0*/
    void* param; //for callback identification
    char    dma_tx;         /* is Transmit DMA Enable ? */
    char    dma_rx;         /* is Receive DMA Enable ? */
}UART_ConfigTy;

typedef struct  {
#define UART_WE	0
#define UART_RD 1
    unsigned char flags;    //
    unsigned char* buf;
    unsigned int  len;      //transfer length
    void (*complete_handler)(const void*);
}UART_MsgTy;

/**
 * @fn void uart_init(const UART_CH_TY ch,const UART_ConfigTy* config)
 * @brief initial the uart hardware with the config
 * @param [in] ch 
 * @param [in] config 
 * @return none
 *  
 */
void uart_init(const UART_CH_TY ch,const UART_ConfigTy* config);


/**
 * @fn void uart_transfer(const UART_CH_TY ch,UART_MsgTy* msg)
 * @brief start to transfer a uart message
 * @param [in] ch 
 * @param [in] msg 
 * @return none
 *  
 */
void uart_transfer(const UART_CH_TY ch,UART_MsgTy* msg);

/**
 * @fn void uart_deinit(const UART_CH_TY ch)
 * @brief deinit the uart hardware
 * @param [in] ch 
 * @return none
 *  
 */
void uart_deinit(const UART_CH_TY ch);




































#endif/* __SV_UART  */
