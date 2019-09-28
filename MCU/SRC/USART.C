/*****************************************************************************/
/**
*  @file      USART.C
*  @brief     <b> USART C File </b>
*  @details   File functionality description:
*  This file to implement the USART driver.
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
#include "../INC/MCU.H"
#include "../INC/XDMA.H"
#include "../INC/USART.H"
#include <string.h>

typedef struct{
  volatile          unsigned int US_CR;         /**< \brief (Usart Offset: 0x0000) Control Register */
  volatile          unsigned int US_MR;         /**< \brief (Usart Offset: 0x0004) Mode Register */
  volatile          unsigned int US_IER;        /**< \brief (Usart Offset: 0x0008) Interrupt Enable Register */
  volatile          unsigned int US_IDR;        /**< \brief (Usart Offset: 0x000C) Interrupt Disable Register */
  volatile const    unsigned int US_IMR;        /**< \brief (Usart Offset: 0x0010) Interrupt Mask Register */
  volatile const    unsigned int US_CSR;        /**< \brief (Usart Offset: 0x0014) Channel Status Register */
  volatile const    unsigned int US_RHR;        /**< \brief (Usart Offset: 0x0018) Receive Holding Register */
  volatile          unsigned int US_THR;        /**< \brief (Usart Offset: 0x001C) Transmit Holding Register */
  volatile          unsigned int US_BRGR;       /**< \brief (Usart Offset: 0x0020) Baud Rate Generator Register */
  volatile          unsigned int US_RTOR;       /**< \brief (Usart Offset: 0x0024) Receiver Time-out Register */
  volatile          unsigned int US_TTGR;       /**< \brief (Usart Offset: 0x0028) Transmitter Timeguard Register */
  volatile const    unsigned int Reserved1[5];
  volatile          unsigned int US_FIDI;       /**< \brief (Usart Offset: 0x0040) FI DI Ratio Register */
  volatile const    unsigned int US_NER;        /**< \brief (Usart Offset: 0x0044) Number of Errors Register */
  volatile const    unsigned int Reserved2[2];
  volatile          unsigned int US_MAN;        /**< \brief (Usart Offset: 0x0050) Manchester Configuration Register */
  volatile          unsigned int US_LINMR;      /**< \brief (Usart Offset: 0x0054) LIN Mode Register */
  volatile          unsigned int US_LINIR;      /**< \brief (Usart Offset: 0x0058) LIN Identifier Register */
  volatile const    unsigned int US_LINBRR;     /**< \brief (Usart Offset: 0x005C) LIN Baud Rate Register */
  volatile          unsigned int US_LONMR;      /**< \brief (Usart Offset: 0x0060) LON Mode Register */
  volatile          unsigned int US_LONPR;      /**< \brief (Usart Offset: 0x0064) LON Preamble Register */
  volatile          unsigned int US_LONDL;      /**< \brief (Usart Offset: 0x0068) LON Data Length Register */
  volatile          unsigned int US_LONL2HDR;   /**< \brief (Usart Offset: 0x006C) LON L2HDR Register */
  volatile const    unsigned int US_LONBL;      /**< \brief (Usart Offset: 0x0070) LON Backlog Register */
  volatile          unsigned int US_LONB1TX;    /**< \brief (Usart Offset: 0x0074) LON Beta1 Tx Register */
  volatile          unsigned int US_LONB1RX;    /**< \brief (Usart Offset: 0x0078) LON Beta1 Rx Register */
  volatile          unsigned int US_LONPRIO;    /**< \brief (Usart Offset: 0x007C) LON Priority Register */
  volatile          unsigned int US_IDTTX;      /**< \brief (Usart Offset: 0x0080) LON IDT Tx Register */
  volatile          unsigned int US_IDTRX;      /**< \brief (Usart Offset: 0x0084) LON IDT Rx Register */
  volatile          unsigned int US_ICDIFF;     /**< \brief (Usart Offset: 0x0088) IC DIFF Register */
  volatile const    unsigned int Reserved3[22];
  volatile          unsigned int US_WPMR;       /**< \brief (Usart Offset: 0x00E4) Write Protection Mode Register */
  volatile const    unsigned int US_WPSR;       /**< \brief (Usart Offset: 0x00E8) Write Protection Status Register */
}UsartTypeDef;

/* -------- US_CR : (USART Offset: 0x0000) Control Register -------- */
#define US_CR_RSTRX (0x1u << 2) /**< \brief (US_CR) Reset Receiver */
#define US_CR_RSTTX (0x1u << 3) /**< \brief (US_CR) Reset Transmitter */
#define US_CR_RXEN  (0x1u << 4) /**< \brief (US_CR) Receiver Enable */
#define US_CR_RXDIS (0x1u << 5) /**< \brief (US_CR) Receiver Disable */
#define US_CR_TXEN  (0x1u << 6) /**< \brief (US_CR) Transmitter Enable */
#define US_CR_TXDIS (0x1u << 7) /**< \brief (US_CR) Transmitter Disable */
#define US_CR_RSTSTA (0x1u << 8) /**< \brief (US_CR) Reset Status Bits */
#define US_CR_STTBRK (0x1u << 9) /**< \brief (US_CR) Start Break */
#define US_CR_STPBRK (0x1u << 10) /**< \brief (US_CR) Stop Break */
#define US_CR_STTTO (0x1u << 11) /**< \brief (US_CR) Clear TIMEOUT Flag and Start Time-out After Next Character Received */
#define US_CR_SENDA (0x1u << 12) /**< \brief (US_CR) Send Address */
#define US_CR_RSTIT (0x1u << 13) /**< \brief (US_CR) Reset Iterations */
#define US_CR_RSTNACK (0x1u << 14) /**< \brief (US_CR) Reset Non Acknowledge */
#define US_CR_RETTO (0x1u << 15) /**< \brief (US_CR) Start Time-out Immediately */
#define US_CR_RTSEN (0x1u << 18) /**< \brief (US_CR) Request to Send Pin Control */
#define US_CR_RTSDIS (0x1u << 19) /**< \brief (US_CR) Request to Send Pin Control */
#define US_CR_LINABT (0x1u << 20) /**< \brief (US_CR) Abort LIN Transmission */
#define US_CR_LINWKUP (0x1u << 21) /**< \brief (US_CR) Send LIN Wakeup Signal */
#define US_CR_FCS   (0x1u << 18) /**< \brief (US_CR) Force SPI Chip Select */
#define US_CR_RCS   (0x1u << 19) /**< \brief (US_CR) Release SPI Chip Select */

#define US_IER_MANE         (0x1u << 24)
#define US_IER_CTSIC        (0x1u << 19)
#define US_IER_UNRE         (0x1u << 10)
#define US_IER_TXEMPTY      (0x1u << 9)
#define US_IER_TIMEOUT      (0x1u << 8)
#define US_IER_PARE         (0x1u << 7)
#define US_IER_FRAME        (0x1u << 6)
#define US_IER_OVRE         (0x1u << 5)
#define US_IER_RXBRK        (0x1u << 2)
#define US_IER_TXRDY        (0x1u << 1)
#define US_IER_RXRDY        (0x1u << 0)
#define CHRL_8BIT           (0x03<<6)
#define OVER                (0x1u << 19)
typedef struct{
	unsigned int base;
	unsigned int fclk;
	unsigned int irq;
    XDMA_Peripheral_TypeDef tx_dma_perip;
    XDMA_Peripheral_TypeDef rx_dma_perip;
}UsartDescTypedef;
#define USART_CH_NUM    3

static const UsartDescTypedef usartDesc[USART_CH_NUM]={
    {0x40024000,100000000,13,XDMA_USART0_TX,XDMA_USART0_RX},
    {0x40028000,100000000,14,XDMA_USART1_TX,XDMA_USART1_RX},
    {0x4002C000,100000000,15,XDMA_USART2_TX,XDMA_USART2_RX}
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
}UsartHalComTypeDef;
const static int spi_mode_map[4]={0x001,0x000,0x101,0x100};
UsartHalComTypeDef usartHalCom[USART_CH_NUM];
static void usart_isr_rx_transfer(const int channel,unsigned char* buf,const int length,void(*callback)(const void*));
static void usart_isr_tx_transfer(const int channel,const unsigned char* buf,const int length,void(*callback)(const void*));
static void usart_dma_tx_transfer(const int channel,const unsigned char* buf,const int length,void(*callback)(const void*));
static void usart_dma_rx_transfer(const int channel,unsigned char* buf,const int length,void(*callback)(const void*));
/**
 * @fn void usart_init(const int channel,const USART_ConfigTy* config)
 * @brief initial the usart hardware with the config
 * @param [in] ch 
 * @param [in] config 
 * @return none
 *  
 */
void usart_init(const int channel,const USART_ConfigTy* config)
{
    if(channel < USART_CH_NUM)
    {
        UsartTypeDef* usart = (UsartTypeDef*)usartDesc[channel].base;
        irq_disable(usartDesc[channel].irq);
        memset(&usartHalCom[channel],0,sizeof(UsartHalComTypeDef));
        mcu_peri_clk_enable(usartDesc[channel].irq);
        char dma_tx = 0;
        char dma_rx = 0;
                /*if use DMA for TX need to allocate DMA channel and reset it*/ 
        /* Reset and disable receiver & transmitter*/
        usart->US_CR =  US_CR_RSTRX | US_CR_RSTTX |\
                        US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
        usart->US_IDR = 0xFFFFFFFF;
        usart->US_CSR;
        usart->US_MR =  (config->usart_mode&0x0F);
        if((config->usart_mode == USART_NORMAL)||
            (config->usart_mode == USART_FLOW))
        {
            UART_ConfigTy* uconfig = (UART_ConfigTy*)(config->pconfig);
            usart->US_BRGR = usartDesc[channel].fclk/8/uconfig->baudrate;
            int parity_type = (uconfig->parity==0)?4:(uconfig->parity-1);
            usart->US_MR = CHRL_8BIT|OVER|(config->usart_mode&0x0F)|(parity_type<<9)|(uconfig->filter<<28);
            usartHalCom[channel].param = uconfig->param;
            dma_tx = uconfig->dma_tx;
            dma_rx = uconfig->dma_rx;
        }
        else if((config->usart_mode == USART_SPI_MASTER)||
                (config->usart_mode == USART_SPI_SLAVE ))
        {
            SPI_ConfigTy* sconfig = (SPI_ConfigTy*)(config->pconfig); 
            usart->US_BRGR = (usartDesc[channel].fclk+sconfig->speed_maxHZ/2)/sconfig->speed_maxHZ;
            usart->US_MR = CHRL_8BIT|(spi_mode_map[sconfig->spi_mode]<<8)|(config->usart_mode&0x0F); 
            usartHalCom[channel].param = sconfig->param;
            dma_tx = sconfig->dma_tx;
            dma_rx = sconfig->dma_rx;
        }
        else if((config->usart_mode == USART_LIN_MASTER)||
                (config->usart_mode == USART_LIN_SLAVE ))
        {

        }
        if(dma_tx)
        {
            XdmaChannelDescType ch_cfg={
            .type = MEM_PER_TRAN,
            .perip = usartDesc[channel].tx_dma_perip   
            };
            usartHalCom[channel].tx_dma_ch = xdma_allocate_channel(&ch_cfg);
            xdma_reset_channel(usartHalCom[channel].tx_dma_ch);
        }
        /*if use DMA for RX need to allocate DMA channel and reset it*/
        if(dma_rx)
        {
            XdmaChannelDescType ch_cfg={
            .type = PER_MEM_TRAN,
            .perip = usartDesc[channel].rx_dma_perip    
            };
            usartHalCom[channel].rx_dma_ch = xdma_allocate_channel(&ch_cfg);
            xdma_reset_channel(usartHalCom[channel].rx_dma_ch);
        }
        usart->US_CR = US_CR_RXEN|US_CR_TXEN;
        irq_enable(usartDesc[channel].irq);    
    }    
}

/**
 * @fn void usart_transfer(const int channel,const USART_MsgTy* msg)
 * @brief start to transfer a usart message
 * @param [in] ch 
 * @param [in] msg 
 * @return none
 *  
 */
void usart_transfer(const int channel,const USART_MsgTy* msg)
{
    if(channel < USART_CH_NUM)
	{
		if(msg->flags ==SPI_M_RD )
		{
            if(usartHalCom[channel].rx_dma_ch)
            {
                usart_dma_rx_transfer(channel,msg->buf,msg->len,msg->complete_handler); 
                return;
            }
			usart_isr_rx_transfer(channel,msg->buf,msg->len,msg->complete_handler);
		}
		else
		{
            if(usartHalCom[channel].tx_dma_ch)
            {
                usart_dma_tx_transfer(channel,msg->buf,msg->len,msg->complete_handler); 
                return;
            }
			usart_isr_tx_transfer(channel,msg->buf,msg->len,msg->complete_handler);
		}

	}
}


/** 
 * @fn  void usart_isr_rx_transfer(const int channel,unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief to read usart datas with the interrupt way
 * @param[in] ch the usart channel
 * @param[in] buf the point to read datas
 * @param[in] length the size to read
 * @param[in] callback the function to run after read finish
 * @return none 
*/
void usart_isr_rx_transfer(const int channel,unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(channel < USART_CH_NUM)
    {
        UsartTypeDef* usart = (UsartTypeDef*)usartDesc[channel].base;
        usartHalCom[channel].rx_length = length;
		usartHalCom[channel].rx_complete_handler = callback;
		usartHalCom[channel].rx_counter = 0;
        usartHalCom[channel].rx_buf = buf;
        usart->US_IER = US_IER_RXRDY|US_IER_UNRE; 
    }    
}

/** 
 * @fn  void usart_isr_tx_transfer(const int channel,const unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief to write usart datas with the interrupt way
 * @param[in] ch the usart channel
 * @param[in] buf the point to write datas
 * @param[in] length the size to write
 * @param[in] callback the function to run after write finish
 * @return none 
*/
void usart_isr_tx_transfer(const int channel,const unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(channel < USART_CH_NUM)
    {
        UsartTypeDef* usart = (UsartTypeDef*)usartDesc[channel].base;
        usartHalCom[channel].tx_length = length;
        usartHalCom[channel].tx_complete_handler = callback;
		usartHalCom[channel].tx_counter = 0;
		usartHalCom[channel].tx_buf = (unsigned char*)buf;
        /*enable TX ready interrup*/ 
        usart->US_IER = US_IER_TXRDY; 
    }    
}


/** 
 * @fn  void usart_dma_tx_transfer(const int channel,const unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief to read usart datas with the dma way
 * @param[in] ch the usart channel
 * @param[in] buf the point to read datas
 * @param[in] length the size to read
 * @param[in] callback the function to run after read finish
 * @return none 
*/
void usart_dma_tx_transfer(const int channel,const unsigned char* buf,const int length,void(*callback)(const void*))
{
	if(channel < USART_CH_NUM)
    {
        UsartTypeDef* usart = (UsartTypeDef*)usartDesc[channel].base;
        XDMA_MsgTy dmsg={.srcAddr = (unsigned int)buf,
                         .destAddr =(unsigned int)&usart->US_THR,
                         .length = length,
                         .isSrcAddrInc = 1,
                         .isDestAddrInc = 0,
                         .call_back = callback,
                         .param = usartHalCom[channel].param
                        };
        xdma_transfer(usartHalCom[channel].tx_dma_ch,&dmsg);
    }    
}

/** 
 * @fn  void usart_dma_rx_transfer(const int channel,unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief to write usart datas with the dma way
 * @param[in] ch the usart channel
 * @param[in] buf the point to write datas
 * @param[in] length the size to write
 * @param[in] callback the function to run after write finish
 * @return none 
*/
void usart_dma_rx_transfer(const int channel,unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(channel < USART_CH_NUM)
    {
        UsartTypeDef* usart = (UsartTypeDef*)usartDesc[channel].base;
        XDMA_MsgTy dmsg={.srcAddr =(unsigned int)&usart->US_RHR,
                         .destAddr =(unsigned int)buf,
                         .length = length,
                         .isSrcAddrInc = 0,
                         .isDestAddrInc = 1,
                         .call_back = callback,
                         .param = usartHalCom[channel].param
                        };
        xdma_transfer(usartHalCom[channel].rx_dma_ch,&dmsg);
    } 
}

/** 
 * @fn  void usart_deinit(const int channel)
 * @brief to deinit usart driver
 * @param[in] ch the usart channel
 * @return none 
*/
void usart_deinit(const int channel)
{
    if(channel < USART_CH_NUM)
    {
        UsartTypeDef* usart = (UsartTypeDef*)usartDesc[channel].base;
        usart->US_IDR = 0xFFFFFFFF;
        usart->US_CSR;
        irq_disable(usartDesc[channel].irq);
        usart->US_CR =  US_CR_RSTRX | US_CR_RSTTX |\
                        US_CR_RXDIS | US_CR_TXDIS | US_CR_RSTSTA;
        mcu_peri_clk_disable(usartDesc[channel].irq);    
    }    
}

/** 
 * @fn  void USART_Handler(const int channel)
 * @brief usart driver interrupt rountine
 * @param[in] ch the usart channel
 * @return none 
*/
void USART_Handler(const int channel)
{
    if(channel < USART_CH_NUM)
    {
        UsartTypeDef* usart = (UsartTypeDef*)usartDesc[channel].base;
        int csr = usart->US_CSR;
        if(csr&(US_IER_UNRE|US_IER_OVRE))
        {
            //error handling here
            usart->US_CR = US_CR_RSTSTA;//
        }
        else if(csr&US_IER_RXRDY)
        {
            unsigned char rx_byte = usart->US_RHR;
            if(usartHalCom[channel].rx_buf !=0)
            {
                usartHalCom[channel].rx_buf[usartHalCom[channel].rx_counter++] = rx_byte;
                if(usartHalCom[channel].rx_counter >= usartHalCom[channel].rx_length)
                {
                    usartHalCom[channel].rx_buf = 0;
                    if(usartHalCom[channel].rx_complete_handler!=0)
                    {
                        usartHalCom[channel].rx_complete_handler(usartHalCom[channel].param);
                    }
                }        
            }    
        }
        else if(csr&(US_IER_TXRDY|US_IER_TXEMPTY))
        {
            if(usartHalCom[channel].tx_buf != 0)
            {
        	  usart->US_THR = usartHalCom[channel].tx_buf[usartHalCom[channel].tx_counter++];
        	  if(usartHalCom[channel].tx_counter >= usartHalCom[channel].tx_length)
        	  {
        		  usartHalCom[channel].tx_buf = 0;
        		  //disable interrupt
        		  usart->US_IDR = US_IER_TXRDY;
        		  if(usartHalCom[channel].tx_complete_handler !=0)
        		  {
        			  usartHalCom[channel].tx_complete_handler(usartHalCom[channel].param);
        		  }
        	  }
          }    
        }        
    }    
}

/** 
 * @fn  void USART0_Handler(void)
 * @brief usart0 driver interrupt rountine
 * @return none 
*/
void USART0_Handler(void)
{
    USART_Handler(0);
}

/** 
 * @fn  void USART1_Handler(void)
 * @brief usart1 driver interrupt rountine
 * @return none 
*/
void USART1_Handler(void)
{
    USART_Handler(1);
}


/** 
 * @fn  void USART2_Handler(void)
 * @brief usart2 driver interrupt rountine
 * @return none 
*/
void USART2_Handler(void)
{
    USART_Handler(2);
}


