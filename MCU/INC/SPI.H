/*****************************************************************************/
/**
*  @file      SPI.H
*  @brief     <b> SPI H File </b>
*  @details   File functionality description:
*  This file provides spi driver interface.
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
#ifndef SV_SPI_H
#define SV_SPI_H

typedef enum
{
	SPI_SLAVE_MODE,
	SPI_MASTER_MODE,
}SPI_MODE_TY;


typedef enum
{
	SPI_CH_0,
	SPI_CH_1,
    SPI_CH_MAX,
}SPI_CH_TY;


typedef struct
{
    unsigned int   spi_mode:2;
    /*master=1 or slave = 0*/
	unsigned int  op_mode:1;
    /*slave mode:only 0 can select,master mode:0-3 */
    unsigned int chip_sel:2;
    unsigned int  speed_maxHZ;
    void* param;
    char    dma_tx;         /* is Transmit DMA Enable ? */
    char    dma_rx;         /* is Receive DMA Enable ? */
}SPI_ConfigTy;

typedef struct  {
#define SPI_M_WE	0
#define SPI_M_RD 	1
#define SPI_M_WE2	2   
#define SPI_M_RD2	3
    unsigned char flags;
    unsigned char* buf;
    unsigned int  len;
    void (*complete_handler)(const void*);
}SPI_MsgTy;

void spi_init(SPI_CH_TY ch,SPI_ConfigTy* config);
void spi_transfer(const SPI_CH_TY ch,SPI_MsgTy* msg);
void spi_deinit(SPI_CH_TY ch);
#endif /*SV_SPI_H */

