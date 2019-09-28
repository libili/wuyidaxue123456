/*****************************************************************************/
/**
*  @file      UART.C
*  @brief     <b> UART C File </b>
*  @details   File functionality description:
*  This file to implement the UART driver.
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
#include "../INC/UART.H"
#include "../INC/MCU.H"
#include "../INC/XDMA.H"
#include <stdio.h>

typedef struct {
  volatile   		unsigned int UART_CR;       /**< \brief (Uart Offset: 0x0000) Control Register */
  volatile  		unsigned int UART_MR;       /**< \brief (Uart Offset: 0x0004) Mode Register */
  volatile   		unsigned int UART_IER;      /**< \brief (Uart Offset: 0x0008) Interrupt Enable Register */
  volatile   		unsigned int UART_IDR;      /**< \brief (Uart Offset: 0x000C) Interrupt Disable Register */
  volatile const	unsigned int UART_IMR;      /**< \brief (Uart Offset: 0x0010) Interrupt Mask Register */
  volatile const  	unsigned int UART_SR;       /**< \brief (Uart Offset: 0x0014) Status Register */
  volatile const  	unsigned int UART_RHR;      /**< \brief (Uart Offset: 0x0018) Receive Holding Register */
  volatile   		unsigned int UART_THR;      /**< \brief (Uart Offset: 0x001C) Transmit Holding Register */
  volatile  		unsigned int UART_BRGR;     /**< \brief (Uart Offset: 0x0020) Baud Rate Generator Register */
  volatile  		unsigned int UART_CMPR;     /**< \brief (Uart Offset: 0x0024) Comparison Register */
  volatile const  	unsigned int Reserved1[47];
  volatile  		unsigned int UART_WPMR;     /**< \brief (Uart Offset: 0x00E4) Write Protection Mode Register */
  volatile const  	unsigned int Reserved2[5];
  volatile const  	unsigned int UART_VERSION;  /**< \brief (Uart Offset: 0x00FC) Version Register */
} UartTypeDef;

#define RSTRX	(1<<2)  
#define RSTTX	(1<<3)
#define RXEN	(1<<4)
#define RXDIS	(1<<5)
#define TXEN	(1<<6)
#define TXDIS	(1<<7)
#define RSTSTA	(1<<8)


#define RXRDY   (1<<0)  /* Receiver Ready */
#define TXRDY   (1<<1)  /* Transmitter Ready */
#define OVRE    (1<<5)  /* Overrun Error */
#define FRAME   (1<<6)  /* Framing Error */
#define PARE    (1<<7)  /*  Parity Error */
#define TXEMPTY (1<<9) /*  Transmitter Empty */
#define CMP     (1<<15) /*  Comparison Match */

typedef struct{
	unsigned int base;
	unsigned int fclk;
	unsigned int irq;
    XDMA_Peripheral_TypeDef tx_dma_perip;
    XDMA_Peripheral_TypeDef rx_dma_perip;
}UartDescTypedef;



static const UartDescTypedef  uartDesc[UART_CH_MAX]={
	{0x400E0800,100000000,7,XDMA_UART0_TX,XDMA_UART0_RX},
	{0x400E0A00,100000000,8,XDMA_UART1_TX,XDMA_UART1_RX},
	{0x400E1A00,100000000,44,XDMA_UART2_TX,XDMA_UART2_RX},
	{0x400E1C00,100000000,45,XDMA_UART3_TX,XDMA_UART3_RX},
	{0x400E1E00,100000000,46,XDMA_UART4_TX,XDMA_UART4_RX},
};

typedef struct
{
    void (*rx_complete_handler)(const void*);
    void (*tx_complete_handler)(const void*);
    unsigned char* 	rx_buf;
    unsigned int  	rx_length;
    unsigned int 	rx_counter;
    unsigned char*  tx_buf;
    unsigned int  	tx_length;
    unsigned int	tx_counter;
             char   tx_dma_ch;
             char   rx_dma_ch;
    void* param;// for callback identification
}UartHalComTypeDef; 

UartHalComTypeDef uartHalCom[UART_CH_MAX];
static void uart_isr_rx_transfer(const UART_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*));
static void uart_isr_tx_transfer(const UART_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*));
static void uart_dma_tx_transfer(const UART_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*));
static void uart_dma_rx_transfer(const UART_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*));


/**
 * @fn void uart_init(const UART_CH_TY ch,const UART_ConfigTy* config)
 * @brief initial the uart hardware with the config
 * @param [in] ch 
 * @param [in] config 
 * @return none
 *  
 */
void uart_init(const UART_CH_TY ch,const UART_ConfigTy* config)
{
	UartTypeDef* uartx;
	if(ch < UART_CH_MAX)
	{
		uartx = (UartTypeDef* )uartDesc[ch].base;
		mcu_peri_clk_enable(uartDesc[ch].irq);
        
        /*if use DMA for TX need to allocate DMA channel and reset it*/
		if(config->dma_tx)
        {
            XdmaChannelDescType ch_cfg={
            .type = MEM_PER_TRAN,
            .perip = uartDesc[ch].tx_dma_perip    
            };
            uartHalCom[ch].tx_dma_ch = xdma_allocate_channel(&ch_cfg);
            xdma_reset_channel(uartHalCom[ch].tx_dma_ch);
        }
        /*if use DMA for RX need to allocate DMA channel and reset it*/
        if(config->dma_rx)
        {
            XdmaChannelDescType ch_cfg={
            .type = PER_MEM_TRAN,
            .perip = uartDesc[ch].rx_dma_perip    
            };
            uartHalCom[ch].rx_dma_ch = xdma_allocate_channel(&ch_cfg);
            xdma_reset_channel(uartHalCom[ch].rx_dma_ch);
        } 
        /*reset TX/RX/Status Register*/
        uartx->UART_CR = RSTRX|RSTTX|RSTSTA;
        
        //disable all interrupt as default state
        uartx->UART_IDR = RXRDY|TXRDY|OVRE|FRAME|PARE|TXEMPTY|CMP;
		if(!config->parity)
		{
			uartx->UART_MR = (config->filter<<4)|(4<<9);
		}
		else
		{
			uartx->UART_MR = (config->filter<<4)|((config->parity-1)<<9);
		}	
		uartx->UART_BRGR = uartDesc[ch].fclk/16/config->baudrate;
        /*enable TX/RX now*/
		uartx->UART_CR = RXEN|TXEN;
        uartHalCom[ch].param = config->param;
		irq_enable(uartDesc[ch].irq);    
	}	
}


/**
 * @fn void uart_transfer(const UART_CH_TY ch,UART_MsgTy* msg)
 * @brief start to transfer a uart message
 * @param [in] ch 
 * @param [in] msg 
 * @return none
 *  
 */
void uart_transfer(const UART_CH_TY ch,UART_MsgTy* msg)
{
    if(msg->flags ==UART_RD )
	{
        if(uartHalCom[ch].rx_dma_ch)
        {
            uart_dma_rx_transfer(ch,msg->buf,msg->len,msg->complete_handler);
            return;
        }
		uart_isr_rx_transfer(ch,msg->buf,msg->len,msg->complete_handler);	
	}
	else
	{
            
        if(uartHalCom[ch].tx_dma_ch)
        {
            uart_dma_tx_transfer(ch,msg->buf,msg->len,msg->complete_handler); 
            return;
        }
        uart_isr_tx_transfer(ch,msg->buf,msg->len,msg->complete_handler);    	
	}
}


/**
 * @fn void uart_isr_rx_transfer(const UART_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief start to transfer a uart message in interrupt way
 * @param [in] ch 
 * @param [in] buf 
 * @param [in] length 
 * @param [in] callback 
 * @return none
 *  
 */
void uart_isr_rx_transfer(const UART_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(ch < UART_CH_MAX)
    {
        UartTypeDef* uartx = (UartTypeDef* )uartDesc[ch].base;
        uartHalCom[ch].rx_length = length;
		uartHalCom[ch].rx_complete_handler = callback;
        uartHalCom[ch].rx_counter = 0;
        uartHalCom[ch].rx_buf = buf;
        uartx->UART_IER = RXRDY;
    }    
}

/**
 * @fn void uart_isr_tx_transfer(const UART_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief start to transfer a uart message in interrupt way
 * @param [in] ch 
 * @param [in] buf 
 * @param [in] length 
 * @param [in] callback 
 * @return none
 *  
 */
void uart_isr_tx_transfer(const UART_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(ch < UART_CH_MAX)
	{
        UartTypeDef* uartx = (UartTypeDef* )uartDesc[ch].base;
        uartHalCom[ch].tx_counter = 0;
        uartHalCom[ch].tx_buf = (unsigned char*)buf;
        uartHalCom[ch].tx_length = length;
        uartHalCom[ch].tx_complete_handler = callback;
        uartx->UART_IER = TXRDY;
    }    
}


/**
 * @fn void uart_dma_tx_transfer(const UART_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief start to transfer a uart message in dma way
 * @param [in] ch 
 * @param [in] buf 
 * @param [in] length 
 * @param [in] callback 
 * @return none
 *  
 */
void uart_dma_tx_transfer(const UART_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*))
{
	if(ch < UART_CH_MAX)
	{
        UartTypeDef* uartx = (UartTypeDef* )uartDesc[ch].base;
        XDMA_MsgTy dmsg={.srcAddr = (unsigned int)buf,
                         .destAddr =(unsigned int)&uartx->UART_THR,
                         .length = length,
                         .isSrcAddrInc = 1,
                         .isDestAddrInc = 0,
                         .call_back = callback,
                         .param = uartHalCom[ch].param
                        };
        xdma_transfer(uartHalCom[ch].tx_dma_ch,&dmsg); 
    }    
}

/**
 * @fn void uart_dma_rx_transfer(const UART_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief start to transfer a uart message in dma way
 * @param [in] ch 
 * @param [in] buf 
 * @param [in] length 
 * @param [in] callback 
 * @return none
 *  
 */
void uart_dma_rx_transfer(const UART_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(ch < UART_CH_MAX)
	{
        UartTypeDef* uartx = (UartTypeDef* )uartDesc[ch].base;
        XDMA_MsgTy dmsg={.srcAddr =(unsigned int)&uartx->UART_RHR,
                         .destAddr =(unsigned int)(buf),
                         .length = length,
                         .isSrcAddrInc = 0,
                         .isDestAddrInc = 1,
                         .call_back = callback,
                         .param = uartHalCom[ch].param
                        };
        xdma_transfer(uartHalCom[ch].tx_dma_ch,&dmsg); 
    }    
}

/**
 * @fn void uart_deinit(const UART_CH_TY ch)
 * @brief deinit the uart hardware
 * @param [in] ch 
 * @return none
 *  
 */
void uart_deinit(const UART_CH_TY ch)
{
    UartTypeDef* uartx;
	if(ch < UART_CH_MAX)
	{
		//reset UART
		uartx = (UartTypeDef* )uartDesc[ch].base;
		uartx->UART_CR = RSTRX|RSTTX|RSTSTA;
		//disable irq
		irq_disable(uartDesc[ch].irq);
		//close PMC clock
		mcu_peri_clk_disable(uartDesc[ch].irq);
	}	
}


/**
 * @fn void Uart_Handler(const UART_CH_TY ch)
 * @brief uart interrupt routine
 * @param [in] ch 
 * @return none
 *  
 */
void Uart_Handler(const UART_CH_TY ch)
{
    UartTypeDef* uartx;
	if(ch < UART_CH_MAX)
	{
        uartx = (UartTypeDef* )uartDesc[ch].base;
        //handle the over run/frame/parity error
        if(uartx->UART_SR&(OVRE|FRAME|PARE))
        {
            uartx->UART_CR |= RSTSTA;
            return;
        }
        else if((uartx->UART_SR&RXRDY)&&(uartx->UART_IMR&RXRDY))
        {
            //receive byte handle
            unsigned char rev_byte = uartx->UART_RHR;
            if(uartHalCom[ch].rx_buf !=0)
            {
                uartHalCom[ch].rx_buf[uartHalCom[ch].rx_counter++] = rev_byte;
                if(uartHalCom[ch].rx_counter >= uartHalCom[ch].rx_length)
                {
                    uartHalCom[ch].rx_buf = 0;
                    if(uartHalCom[ch].rx_complete_handler!=0)
                    {
                        uartHalCom[ch].rx_complete_handler(uartHalCom[ch].param);
                    }
                }
            }    
        }        
        if((uartx->UART_SR&TXRDY)&&(uartx->UART_IMR&TXRDY))
        { 
            if(uartHalCom[ch].tx_buf != 0)
            {
        	  uartx->UART_THR = uartHalCom[ch].tx_buf[uartHalCom[ch].tx_counter++];
        	  if(uartHalCom[ch].tx_counter >= uartHalCom[ch].tx_length)
        	  {
        		  uartHalCom[ch].tx_buf = 0;
        		  //disable interrupt
                   uartx->UART_IDR = TXRDY; 
        		  if(uartHalCom[ch].tx_complete_handler !=0)
        		  {
        			  uartHalCom[ch].tx_complete_handler(uartHalCom[ch].param);
        		  }
        	  }
          }      
        }    
    }   
}


/**
 * @fn void Uart0_Handler(void)
 * @brief uart0 interrupt routine
 * @param none   
 * @return none
 *  
 */
void Uart0_Handler(void)
{
    Uart_Handler(UART_CH_0);
}

/**
 * @fn void Uart1_Handler(void)
 * @brief uart1 interrupt routine
 * @param none   
 * @return none
 *  
 */
void Uart1_Handler(void)
{
    Uart_Handler(UART_CH_1);
}

/**
 * @fn void Uart2_Handler(void)
 * @brief uart2 interrupt routine
 * @param none   
 * @return none
 *  
 */
void Uart2_Handler(void)
{
    Uart_Handler(UART_CH_2);
}

/**
 * @fn void Uart3_Handler(void)
 * @brief uart3 interrupt routine
 * @param none   
 * @return none
 *  
 */
void Uart3_Handler(void)
{
    Uart_Handler(UART_CH_3);
}

/**
 * @fn void Uart4_Handler(void)
 * @brief uart4 interrupt routine
 * @param none   
 * @return none
 *  
 */
void Uart4_Handler(void)
{
    Uart_Handler(UART_CH_4);
}


