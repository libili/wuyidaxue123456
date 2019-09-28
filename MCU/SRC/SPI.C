/*****************************************************************************/
/**
*  @file      SPI.C
*  @brief     <b> SPI C File </b>
*  @details   File functionality description:
*  This file  is to implement the SPI driver
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
#include "../INC/XDMA.H"
#include "../INC/SPI.H"


typedef struct {
  volatile  		int SPI_CR;        /**< \brief (Spi Offset: 0x00) Control Register */
  volatile  		int SPI_MR;        /**< \brief (Spi Offset: 0x04) Mode Register */
  volatile const  	int SPI_RDR;       /**< \brief (Spi Offset: 0x08) Receive Data Register */
  volatile   		int SPI_TDR;       /**< \brief (Spi Offset: 0x0C) Transmit Data Register */
  volatile const  	int SPI_SR;        /**< \brief (Spi Offset: 0x10) Status Register */
  volatile   		int SPI_IER;       /**< \brief (Spi Offset: 0x14) Interrupt Enable Register */
  volatile   		int SPI_IDR;       /**< \brief (Spi Offset: 0x18) Interrupt Disable Register */
  volatile const  	int SPI_IMR;       /**< \brief (Spi Offset: 0x1C) Interrupt Mask Register */
  volatile const  	int Reserved1[4];
  volatile  		int SPI_CSR[4];    /**< \brief (Spi Offset: 0x30) Chip Select Register */
  volatile const  	int Reserved2[41];
  volatile  		int SPI_WPMR;      /**< \brief (Spi Offset: 0xE4) Write Protection Mode Register */
  volatile const  	int SPI_WPSR;      /**< \brief (Spi Offset: 0xE8) Write Protection Status Register */
  volatile const  	int Reserved3[4];
  volatile const  	int SPI_VERSION;   /**< \brief (Spi Offset: 0xFC) Version Register */
}SpiTypedef;

#define CR_SPIEN (0x1u << 0) /**< \brief (SPI_CR) SPI Enable */
#define CR_SPIDIS (0x1u << 1) /**< \brief (SPI_CR) SPI Disable */
#define CR_SWRST (0x1u << 7) /**< \brief (SPI_CR) SPI Software Reset */
#define CR_LASTXFER (0x1u << 24) /**< \brief (SPI_CR) Last Transfer */


#define UNDES   (0x01u<<10)
#define TXEMPTY (0x01u<<9)
#define NSSR    (0x01u<<8)
#define OVRES    (0x01<<3)
#define MODF     (0x01<<2)
#define TDRE     (0x01<<1)
#define RDRF     (0x01<<0)
typedef struct{
	unsigned int base;
	unsigned int fclk;
	unsigned int irq;
    unsigned char dlybs_ns;
    unsigned char dlybct_ns;/*delay between bytes*/
    XDMA_Peripheral_TypeDef tx_dma_perip;
    XDMA_Peripheral_TypeDef rx_dma_perip;
}SpiDescTypedef;

static const SpiDescTypedef  spiDesc[2]={
	{0x40008000,100000000,21,100,0,XDMA_SPI0_TX,XDMA_SPI0_RX},
	{0x40058000,100000000,42,100,0,XDMA_SPI1_TX,XDMA_SPI1_RX},
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
             char	tx_for_master_rx;
	unsigned int    dummy_tx_counter;
             char   tx_dma_ch;
             char   rx_dma_ch;
    void* param;
}SpiHalComTypeDef; 

static SpiHalComTypeDef spiHalCom[2];
static const unsigned char spi_tx_dummy_data = 0xFF;
static       unsigned char spi_rx_dummy_data;
const static unsigned char spi_mode_map[4]={2,0,3,1};
const static unsigned char spi_npcs_map[4]={0,1,3,7};
static void spi_dma_rx_transfer(const SPI_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*));
static void spi_dma_tx_transfer(const SPI_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*));
static void spi_isr_rx_transfer(const SPI_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*));
static void spi_isr_tx_transfer(const SPI_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*));
/** 
 * @fn  void spi_init(SPI_CH_TY ch,SPI_ConfigTy* config)
 * @brief to init spi driver
 * @param[in] ch the spi channel
 * @param[in] config the spi configuration parameter
 * @return int 
*/
void spi_init(SPI_CH_TY ch,SPI_ConfigTy* config)
{
	if(ch < SPI_CH_MAX)
    {
       SpiTypedef* spix = (SpiTypedef*)spiDesc[ch].base; 
       mcu_peri_clk_enable(spiDesc[ch].irq);
       spix->SPI_CR = CR_SPIDIS;
       spix->SPI_CR = CR_SWRST;
       spix->SPI_CR = CR_SWRST; 
       /*disable all the interrupt as default*/ 
       spix->SPI_IDR = UNDES|TXEMPTY|NSSR|OVRES|MODF|TDRE|RDRF;
       spix->SPI_MR = ((int)spi_npcs_map[config->chip_sel]<<16)|config->op_mode|(1<<4);
       int scbr = (spiDesc[ch].fclk+config->speed_maxHZ/2)/config->speed_maxHZ;
       int dlybs = spiDesc[ch].dlybs_ns*1000000000/spiDesc[ch].fclk;
       spix->SPI_CSR[config->chip_sel] =  (scbr<<8)|spi_mode_map[config->spi_mode]|(dlybs<<16);
        /*if use DMA for TX need to allocate DMA channel and reset it*/
		if(config->dma_tx)
        {
            XdmaChannelDescType ch_cfg={
            .type = MEM_PER_TRAN,
            .perip = spiDesc[ch].tx_dma_perip    
            };
            spiHalCom[ch].tx_dma_ch = xdma_allocate_channel(&ch_cfg);
            xdma_reset_channel(spiHalCom[ch].tx_dma_ch);
        }
        /*if use DMA for RX need to allocate DMA channel and reset it*/
        if(config->dma_rx)
        {
            XdmaChannelDescType ch_cfg={
            .type = PER_MEM_TRAN,
            .perip = spiDesc[ch].rx_dma_perip    
            };
            spiHalCom[ch].rx_dma_ch = xdma_allocate_channel(&ch_cfg);
            xdma_reset_channel(spiHalCom[ch].rx_dma_ch);
        } 
       spix->SPI_SR;
       irq_enable(spiDesc[ch].irq); 
       spix->SPI_CR = CR_SPIEN;
       spix->SPI_CR = CR_SPIEN;
       spiHalCom[ch].param = config->param;     
    }    
}

/** 
 * @fn  void spi_deinit(SPI_CH_TY ch)
 * @brief to deinit spi driver
 * @param[in] ch the spi channel
 * @return int 
*/
void spi_deinit(SPI_CH_TY ch)
{
    if(ch < SPI_CH_MAX)
    {
       SpiTypedef* spix = (SpiTypedef*)spiDesc[ch].base; 
       mcu_peri_clk_enable(spiDesc[ch].irq);
       spix->SPI_CR = CR_SPIDIS;
       spix->SPI_CR = CR_SWRST;
    }    
}    


/** 
 * @fn  void spi_transfer(const SPI_CH_TY ch,SPI_MsgTy* msg)
 * @brief to transfer spi messages
 * @param[in] ch the spi channel
 * @param[in] msg the spi messages
 * @return none 
*/
void spi_transfer(const SPI_CH_TY ch,SPI_MsgTy* msg)
{
	if(ch < SPI_CH_MAX)
	{
        SpiTypedef* spix = (SpiTypedef*)spiDesc[ch].base;
		spiHalCom[ch].tx_for_master_rx = 0;
		if((msg->flags ==SPI_M_RD)||( msg->flags ==SPI_M_RD2))
		{
            if(spiHalCom[ch].rx_dma_ch)
            {
                spi_dma_rx_transfer(ch,msg->buf,msg->len,msg->complete_handler); 
                if(msg->flags ==SPI_M_RD2)
                {
                   spi_dma_tx_transfer(ch,&spi_tx_dummy_data,msg->len,0);  
                }    
                return;
            }
			spi_isr_rx_transfer(ch,msg->buf,msg->len,msg->complete_handler); 
			if( msg->flags ==SPI_M_RD2)
			{
				spiHalCom[ch].dummy_tx_counter = 0;
                spiHalCom[ch].tx_for_master_rx = 1;
				//trigger tx ready interrupt to send data
            	spix->SPI_IER = TDRE;
			}
		}
		else
		{
            if(spiHalCom[ch].tx_dma_ch)
            {
                if(msg->flags == SPI_M_WE2)
                {
                    spi_dma_rx_transfer(ch,&spi_rx_dummy_data,msg->len,0); 
                }    
                spi_dma_tx_transfer(ch,msg->buf,msg->len,msg->complete_handler);            
                return;
            }
			spi_isr_tx_transfer(ch,msg->buf,msg->len,msg->complete_handler);
		}
	}
}


/** 
 * @fn  void spi_isr_rx_transfer(const SPI_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*))void spi_transfer(const SPI_CH_TY ch,SPI_MsgTy* msg)
 * @brief to read spi datas with the interrupt way
 * @param[in] ch the spi channel
 * @param[in] buf the point to read datas
 * @param[in] length the size to read
 * @param[in] callback the function to run after read finish
 * @return none 
*/
void spi_isr_rx_transfer(const SPI_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(ch < SPI_CH_MAX)
    {
        SpiTypedef* spix = (SpiTypedef*)spiDesc[ch].base;
        spiHalCom[ch].rx_length = length;
		spiHalCom[ch].rx_complete_handler = callback;
		spiHalCom[ch].rx_counter = 0;
        spiHalCom[ch].rx_buf = buf;
        //enable rx interrupt     
        spix->SPI_IER = RDRF;
    }    
}

/** 
 * @fn  void spi_isr_tx_transfer(const SPI_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief to write spi datas with the interrupt way
 * @param[in] ch the spi channel
 * @param[in] buf the point to write datas
 * @param[in] length the size to write
 * @param[in] callback the function to run after write finish
 * @return none 
*/
void spi_isr_tx_transfer(const SPI_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(ch < SPI_CH_MAX)
    {
        SpiTypedef* spix = (SpiTypedef*)spiDesc[ch].base;
        spiHalCom[ch].tx_length = length;
		spiHalCom[ch].tx_complete_handler = callback;
		spiHalCom[ch].tx_counter = 0;
        spiHalCom[ch].tx_buf = (unsigned char*)buf;
        //trigger tx ready interrupt to send data
        spix->SPI_IER = TDRE; 
    }
}


/** 
 * @fn  void spi_dma_rx_transfer(const SPI_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief to read spi datas with the dma way
 * @param[in] ch the spi channel
 * @param[in] buf the point to read datas
 * @param[in] length the size to read
 * @param[in] callback the function to run after read finish
 * @return none 
*/
void spi_dma_rx_transfer(const SPI_CH_TY ch,unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(ch < SPI_CH_MAX)
    {
        SpiTypedef* spix = (SpiTypedef*)spiDesc[ch].base;
        XDMA_MsgTy dmsg={.srcAddr =(unsigned int)&spix->SPI_RDR,
                         .destAddr =(unsigned int)buf,
                         .length = length,
                         .isSrcAddrInc = 0,
                         .isDestAddrInc = 1,
                         .call_back = callback,
                         .param = spiHalCom[ch].param
                        };
        if(buf == &spi_rx_dummy_data)
        {
            dmsg.isDestAddrInc = 0;
        }    
        xdma_transfer(spiHalCom[ch].rx_dma_ch,&dmsg);
    }                            
}

/** 
 * @fn  void spi_dma_tx_transfer(const SPI_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*))
 * @brief to write spi datas with the dma way
 * @param[in] ch the spi channel
 * @param[in] buf the point to write datas
 * @param[in] length the size to write
 * @param[in] callback the function to run after write finish
 * @return none 
*/
void spi_dma_tx_transfer(const SPI_CH_TY ch,const unsigned char* buf,const int length,void(*callback)(const void*))
{
    if(ch < SPI_CH_MAX)
    {
        SpiTypedef* spix = (SpiTypedef*)spiDesc[ch].base;
        XDMA_MsgTy dmsg={.srcAddr = (unsigned int)(buf),
                         .destAddr =(unsigned int)&spix->SPI_TDR,
                         .length = length,
                         .isSrcAddrInc = 1,
                         .isDestAddrInc = 0,
                         .call_back = callback,
                         .param = spiHalCom[ch].param
                        };
        if(buf==&spi_tx_dummy_data)
        {
            dmsg.isSrcAddrInc = 0;
        }    
        xdma_transfer(spiHalCom[ch].tx_dma_ch,&dmsg);
    }    
    
}

    
/** 
 * @fn  void Spi_Handler(const SPI_CH_TY ch)
 * @brief the spi intterupt rountine
 * @param[in] ch the spi channel
 * @return none 
*/
void Spi_Handler(const SPI_CH_TY ch)
{
    if(ch < SPI_CH_MAX)
    {
       SpiTypedef* spix = (SpiTypedef*)spiDesc[ch].base;
        int sr = spix->SPI_SR;
        if(sr&RDRF)
       {
          unsigned char rx_byte = spix->SPI_RDR;
          if(spiHalCom[ch].rx_buf != 0)
          {
        	  spiHalCom[ch].rx_buf[spiHalCom[ch].rx_counter++] = rx_byte;
             if(spiHalCom[ch].rx_counter >= spiHalCom[ch].rx_length)
             {
            	 spiHalCom[ch].rx_buf = 0;
                 //disable interrupt
        		  spix->SPI_IDR = RDRF;
            	 if(spiHalCom[ch].rx_complete_handler!=0)
            	 {
            		 spiHalCom[ch].rx_complete_handler(spiHalCom[ch].param);
            	 }
             }
          }    
           
       }
       if(sr&(TDRE|TXEMPTY))
       {
            if(spiHalCom[ch].tx_buf != 0)
            {
              spix->SPI_TDR = spiHalCom[ch].tx_buf[spiHalCom[ch].tx_counter++];			
        	  if(spiHalCom[ch].tx_counter >= spiHalCom[ch].tx_length)
        	  {
        		  spiHalCom[ch].tx_buf = 0;
        		  //disable interrupt
        		  spix->SPI_IDR = TDRE;
        		  if(spiHalCom[ch].tx_complete_handler !=0)
        		  {
        			  spiHalCom[ch].tx_complete_handler(spiHalCom[ch].param);
        		  }
        	  }
            }
            else if(spiHalCom[ch].tx_for_master_rx)
            {
                spix->SPI_TDR = 0xFF;
                if(++spiHalCom[ch].dummy_tx_counter >= spiHalCom[ch].rx_length)
                {
                    //disable interrupt
                    spix->SPI_IDR = TDRE;
                    spiHalCom[ch].tx_for_master_rx = 0;
                }
            }	
       }     
    }       
}    



/** 
 * @fn  void Spi0_Handler(void)
 * @brief the spi0 intterupt rountine
 * @return none 
*/
void Spi0_Handler(void)
{
    Spi_Handler(SPI_CH_0);
}

/** 
 * @fn  void Spi1_Handler(void)
 * @brief the spi1 intterupt rountine
 * @return none 
*/
void Spi1_Handler(void)
{
    Spi_Handler(SPI_CH_1);
}

