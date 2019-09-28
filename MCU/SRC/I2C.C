/*****************************************************************************/
/**
*  @file      I2C.C
*  @brief     <b> I2C C File </b>
*  @details   File functionality description:
*  This file  is to implement the i2c driver
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
#include "../../CONFIG/INC/TYPES.H"
#include "../INC/MCU.H"
#include "../INC/SAMV70.H"
#include "../INC/I2C.H"
#include <cmsis_os.h>
#include "../../OS/THREAD.H"
#include "../../SYS/INC/TIMER_IF.H"
typedef struct {
  volatile  unsigned int TWIHS_CR;      /**< \brief (Twihs Offset: 0x00) Control Register */
  volatile  unsigned int TWIHS_MMR;     /**< \brief (Twihs Offset: 0x04) Master Mode Register */
  volatile  unsigned int TWIHS_SMR;     /**< \brief (Twihs Offset: 0x08) Slave Mode Register */
  volatile  unsigned int TWIHS_IADR;    /**< \brief (Twihs Offset: 0x0C) Internal Address Register */
  volatile  unsigned int TWIHS_CWGR;    /**< \brief (Twihs Offset: 0x10) Clock Waveform Generator Register */
  volatile const unsigned int Reserved1[3];
  volatile const unsigned int TWIHS_SR;      /**< \brief (Twihs Offset: 0x20) Status Register */
  volatile  unsigned int TWIHS_IER;     /**< \brief (Twihs Offset: 0x24) Interrupt Enable Register */
  volatile  unsigned int TWIHS_IDR;     /**< \brief (Twihs Offset: 0x28) Interrupt Disable Register */
  volatile const unsigned int TWIHS_IMR;     /**< \brief (Twihs Offset: 0x2C) Interrupt Mask Register */
  volatile const unsigned int TWIHS_RHR;     /**< \brief (Twihs Offset: 0x30) Receive Holding Register */
  volatile  unsigned int TWIHS_THR;     /**< \brief (Twihs Offset: 0x34) Transmit Holding Register */
  volatile  unsigned int TWIHS_SMBTR;   /**< \brief (Twihs Offset: 0x38) SMBus Timing Register */
  volatile const unsigned int Reserved2[2];
  volatile  unsigned int TWIHS_FILTR;   /**< \brief (Twihs Offset: 0x44) Filter Register */
  volatile const unsigned int Reserved3[1];
  volatile  unsigned int TWIHS_SWMR;    /**< \brief (Twihs Offset: 0x4C) SleepWalking Matching Register */
  volatile const unsigned int Reserved4[37];
  volatile  unsigned int TWIHS_WPMR;    /**< \brief (Twihs Offset: 0xE4) Write Protection Mode Register */
  volatile const unsigned int TWIHS_WPSR;    /**< \brief (Twihs Offset: 0xE8) Write Protection Status Register */
}I2cTypedef;


#define TWIHS_CR_START (0x1u << 0) /**< \brief (TWIHS_CR) Send a START Condition */
#define TWIHS_CR_STOP (0x1u << 1) /**< \brief (TWIHS_CR) Send a STOP Condition */
#define TWIHS_CR_MSEN (0x1u << 2) /**< \brief (TWIHS_CR) TWIHS Master Mode Enabled */
#define TWIHS_CR_MSDIS (0x1u << 3) /**< \brief (TWIHS_CR) TWIHS Master Mode Disabled */
#define TWIHS_CR_SVEN (0x1u << 4) /**< \brief (TWIHS_CR) TWIHS Slave Mode Enabled */
#define TWIHS_CR_SVDIS (0x1u << 5) /**< \brief (TWIHS_CR) TWIHS Slave Mode Disabled */
#define TWIHS_CR_QUICK (0x1u << 6) /**< \brief (TWIHS_CR) SMBus Quick Command */
#define TWIHS_CR_SWRST (0x1u << 7) /**< \brief (TWIHS_CR) Software Reset */
#define TWIHS_CR_HSEN (0x1u << 8) /**< \brief (TWIHS_CR) TWIHS High-Speed Mode Enabled */
#define TWIHS_CR_HSDIS (0x1u << 9) /**< \brief (TWIHS_CR) TWIHS High-Speed Mode Disabled */
#define TWIHS_CR_SMBEN (0x1u << 10) /**< \brief (TWIHS_CR) SMBus Mode Enabled */
#define TWIHS_CR_SMBDIS (0x1u << 11) /**< \brief (TWIHS_CR) SMBus Mode Disabled */
#define TWIHS_CR_PECEN (0x1u << 12) /**< \brief (TWIHS_CR) Packet Error Checking Enable */
#define TWIHS_CR_PECDIS (0x1u << 13) /**< \brief (TWIHS_CR) Packet Error Checking Disable */
#define TWIHS_CR_PECRQ (0x1u << 14) /**< \brief (TWIHS_CR) PEC Request */
#define TWIHS_CR_CLEAR (0x1u << 15) /**< \brief (TWIHS_CR) Bus CLEAR Command */


#define TWIHS_SR_TXCOMP (0x1u << 0) /**< \brief (TWIHS_SR) Transmission Completed (cleared by writing TWIHS_THR) */
#define TWIHS_SR_RXRDY (0x1u << 1) /**< \brief (TWIHS_SR) Receive Holding Register Ready (cleared by reading TWIHS_RHR) */
#define TWIHS_SR_TXRDY (0x1u << 2) /**< \brief (TWIHS_SR) Transmit Holding Register Ready (cleared by writing TWIHS_THR) */
#define TWIHS_SR_SVREAD (0x1u << 3) /**< \brief (TWIHS_SR) Slave Read */
#define TWIHS_SR_SVACC (0x1u << 4) /**< \brief (TWIHS_SR) Slave Access */
#define TWIHS_SR_GACC (0x1u << 5) /**< \brief (TWIHS_SR) General Call Access (cleared on read) */
#define TWIHS_SR_OVRE (0x1u << 6) /**< \brief (TWIHS_SR) Overrun Error (cleared on read) */
#define TWIHS_SR_UNRE (0x1u << 7) /**< \brief (TWIHS_SR) Underrun Error (cleared on read) */
#define TWIHS_SR_NACK (0x1u << 8) /**< \brief (TWIHS_SR) Not Acknowledged (cleared on read) */
#define TWIHS_SR_ARBLST (0x1u << 9) /**< \brief (TWIHS_SR) Arbitration Lost (cleared on read) */
#define TWIHS_SR_SCLWS (0x1u << 10) /**< \brief (TWIHS_SR) Clock Wait State */
#define TWIHS_SR_EOSACC (0x1u << 11) /**< \brief (TWIHS_SR) End Of Slave Access (cleared on read) */
#define TWIHS_SR_MCACK (0x1u << 16) /**< \brief (TWIHS_SR) Master Code Acknowledge (cleared on read) */
#define TWIHS_SR_TOUT (0x1u << 18) /**< \brief (TWIHS_SR) Timeout Error (cleared on read) */
#define TWIHS_SR_PECERR (0x1u << 19) /**< \brief (TWIHS_SR) PEC Error (cleared on read) */
#define TWIHS_SR_SMBDAM (0x1u << 20) /**< \brief (TWIHS_SR) SMBus Default Address Match (cleared on read) */
#define TWIHS_SR_SMBHHM (0x1u << 21) /**< \brief (TWIHS_SR) SMBus Host Header Address Match (cleared on read) */
#define TWIHS_SR_SCL (0x1u << 24) /**< \brief (TWIHS_SR) SCL line value */
#define TWIHS_SR_SDA (0x1u << 25) /**< \brief (TWIHS_SR) SDA line value */



typedef struct
{
	unsigned int base;
	unsigned int fclk;
	unsigned int irq;
}I2cDescTypedef;    


static const I2cDescTypedef  i2cDesc[3]={
	{0x40018000,100000000,19},
	{0x4001C000,100000000,20},
    {0x40060000,100000000,41},
};

osMutexDef(i2c0_mut);
osMutexDef(i2c1_mut);
osMutexDef(i2c2_mut);
osSemaphoreDef(i2c0_sem);
osSemaphoreDef(i2c1_sem);
osSemaphoreDef(i2c2_sem);
typedef struct
{
    int tx_length;
    int tx_counter;
    unsigned char* tx_buf;
    int rx_length;
    int rx_counter;
    int err_flag;
    unsigned char* rx_buf; 
    osSemaphoreId sem_id;
    const osSemaphoreDef_t *semDef;    
    osMutexId mutex_id;
	const osMutexDef_t * mutexDef;
}I2cHalComTypeDef; 
volatile I2cHalComTypeDef i2cHalCom[I2C_CH_MAX] ={
    {.mutexDef=osMutex(i2c0_mut),.semDef=osSemaphore(i2c0_sem)},
    {.mutexDef=osMutex(i2c1_mut),.semDef=osSemaphore(i2c1_sem)},
    {.mutexDef=osMutex(i2c2_mut),.semDef=osSemaphore(i2c2_sem)},
};
typedef struct
{
    unsigned char addr_size:2;
    unsigned char addr[3];
}I2cMemTypeDef;
static int i2c_read_message(I2C_CH_TY ch,const unsigned char slave,I2cMemTypeDef* mem_type,unsigned char* r_buf,int length);
static int i2c_write_message(I2C_CH_TY ch,const unsigned char slave,I2cMemTypeDef* mem_type,const unsigned char* w_buf,int length);
static int i2c_clear_devices(I2C_CH_TY ch);
/** 
 * @fn  void i2c_init(I2C_CH_TY ch,I2C_ConfigTy* config)
 * @brief init and configurate i2c driver
 * @param[in] ch the i2c channel
 * @param[in] config the i2c channel config parameter
 * @return none
*/
void i2c_init(I2C_CH_TY ch,I2C_ConfigTy* config)
{
    unsigned int dwCkDiv = 0 ;
	unsigned int dwClDiv ;
	unsigned int dwOk = 0 ;
	if(ch < I2C_CH_MAX)
    {
       I2cTypedef* i2c = (I2cTypedef*)i2cDesc[ch].base;
       if(i2cHalCom[ch].mutex_id == 0)
            i2cHalCom[ch].mutex_id = osMutexCreate(i2cHalCom[ch].mutexDef);     
       osMutexWait(i2cHalCom[ch].mutex_id, osWaitForever); 
/*        extern const McuDrvConfigsTy   mcu_drv_configs;
        const   I2CConfigTy* i2c_config = mcu_drv_configs.i2c;
        unsigned scl = i2c_config->i2cs[ch].scl;
        unsigned sda = i2c_config->i2cs[ch].sda;
        //first umMap pin
        i2c_config->i2cs[ch].pinUnMapFunc(); 
        gpio_direction_input(sda);
        if(!gpio_get_value(sda))
        {
            while(1);
        }   
        i2c_config->i2cs[ch].pinMapFunc(); */
       mcu_peri_clk_enable(i2cDesc[ch].irq);
       i2c->TWIHS_RHR;
       	/* TWI Slave Mode Disabled, TWI Master Mode Disabled. */
        i2c->TWIHS_CR = TWIHS_CR_SVDIS ;
        i2c->TWIHS_CR = TWIHS_CR_MSDIS ;
        i2c->TWIHS_CR = TWIHS_CR_MSEN ;
        /*setting the clock*/
        /* Configure clock */
        while ( !dwOk ) {
		dwClDiv = ((i2cDesc[ch].fclk / (2 * config->speed_maxHZ)) - 4) / (1<<dwCkDiv) ;
		if ( dwClDiv <= 255 ) {
			dwOk = 1 ;
		} else {
			dwCkDiv++ ;
		}
	}
        i2c->TWIHS_CWGR = 0 ;
	    i2c->TWIHS_CWGR = (dwCkDiv << 16) | (dwClDiv << 8) | dwClDiv ;
        /* Set master mode */
        
        irq_enable(i2cDesc[ch].irq);
        
        osMutexRelease(i2cHalCom[ch].mutex_id);
    }   
}

/** 
 * @fn  int i2c_transfer(I2C_CH_TY ch, I2C_MsgTy *msgs,int num)
 * @brief to transfer i2c messages
 * @param[in] ch the i2c channel
 * @param[in] megs the i2c messages
 * @param[in] num the number of messages
 * @return int 
*/
int i2c_transfer(I2C_CH_TY ch, I2C_MsgTy *msgs,int num)
{
    int result = 1;
    if(num > 2)
	{
        return result;
	}
    //get lock
	osMutexWait(i2cHalCom[ch].mutex_id, osWaitForever);
    if(i2c_clear_devices(ch) != 0)
        return -1;
	else if((num==2)&&(msgs[0].flags==I2C_M_WE)&&(msgs[1].flags==I2C_M_RD))
	{
		

        I2cMemTypeDef mem={0};
        mem.addr_size = msgs[0].len;
        for(int i = 0; i<mem.addr_size; i++)
        {
             mem.addr[i] = msgs[0].buf[i];
        }
		result = i2c_read_message(ch,msgs[0].addr,&mem,msgs[1].buf,msgs[1].len);
   		
	}
    else if((num==2)&&(msgs[0].flags==I2C_M_WE)&&(msgs[1].flags==I2C_M_WE))
    {
        I2cMemTypeDef mem={0};
        mem.addr_size = msgs[0].len;
        for(int i = 0; i<mem.addr_size; i++)
        {
             mem.addr[i] = msgs[0].buf[i];
        }
        result = i2c_write_message(ch,msgs[0].addr,&mem,msgs[1].buf,msgs[1].len);
    }    
	else if((num==1)&&(msgs[0].flags==I2C_M_WE))
	{
        result = i2c_write_message(ch,msgs[0].addr,0,msgs[0].buf,msgs[0].len);
	}
	else if((num==1)&&(msgs[0].flags==I2C_M_RD))
	{
        result = i2c_read_message(ch,msgs[0].addr,0,msgs[0].buf,msgs[0].len);   		
	}
    //release lock
    osMutexRelease(i2cHalCom[ch].mutex_id); 
	return result;
}



/** 
 * @fn  int i2c_write_message(I2C_CH_TY ch,const unsigned char slave,I2cMemTypeDef* mem_type,const unsigned char* w_buf,int length)
 * @brief to write one i2c message
 * @param[in] ch the i2c channel
 * @param[in] slave the i2c slave device address
 * @param[in] mem_type 
 * @param[in] w_buf datas point to write
 * @param[in] length datas size to write
 * @return int
*/
int i2c_write_message(I2C_CH_TY ch,const unsigned char slave,I2cMemTypeDef* mem_type,const unsigned char* w_buf,int length)
{
    int ret = 0;
    if(ch < I2C_CH_MAX)
    {
       I2cTypedef* i2c = (I2cTypedef*)i2cDesc[ch].base; 
       /* TWI Slave Mode Disabled, TWI Master Mode Disabled. */
        i2c->TWIHS_CR = TWIHS_CR_SVDIS;
        i2c->TWIHS_CR = TWIHS_CR_MSEN;
        unsigned char m_addr_size  = 0;   
        if(mem_type)
        {
            m_addr_size = mem_type->addr_size;
        }  
        i2c->TWIHS_MMR = (slave<<16)|(int)(m_addr_size<<8);
        if(m_addr_size)
        {
            int iadr = 0;
            for(int i = 0; i <m_addr_size;i++ )
            {
                iadr <<=8;
                iadr |= mem_type->addr[i]; 
            }
            i2c->TWIHS_IADR = iadr;
        }
        i2cHalCom[ch].tx_buf = (unsigned char*)w_buf;
        i2cHalCom[ch].tx_counter = 1;
        i2cHalCom[ch].tx_length = length;
        i2c->TWIHS_THR = w_buf[0];
        i2cHalCom[ch].sem_id = osSemaphoreCreate (i2cHalCom[ch].semDef,0);
        i2cHalCom[ch].err_flag = 0;
        //single byte to write
        i2c->TWIHS_IDR = 0xFFFF;   
        if(i2cHalCom[ch].tx_length == 1)
        {
            i2c->TWIHS_CR = TWIHS_CR_STOP;
            i2c->TWIHS_IER = TWIHS_SR_TXCOMP;
        }
         
        i2c->TWIHS_IER = TWIHS_SR_TXRDY|TWIHS_SR_NACK|TWIHS_SR_ARBLST;
        int sem = osSemaphoreWait (i2cHalCom[ch].sem_id, 500);
        osSemaphoreDelete(i2cHalCom[ch].sem_id);
        if((i2cHalCom[ch].err_flag!=0)||(sem == 0))
            ret = 1;    //bool
    }
    return ret;    
}    


/** 
 * @fn  int i2c_read_message(I2C_CH_TY ch,const unsigned char slave,I2cMemTypeDef* mem_type,unsigned char* r_buf,int length)
 * @brief to read one message
 * @param[in] ch the i2c channel
 * @param[in] slave the i2c slave device address
 * @param[in] mem_type 
 * @param[in] r_buf datas point to read
 * @param[in] length datas size to read
 * @return int
*/
int i2c_read_message(I2C_CH_TY ch,const unsigned char slave,I2cMemTypeDef* mem_type,unsigned char* r_buf,int length)
{
    int ret = 0;
    if(ch < I2C_CH_MAX)
    {
       I2cTypedef* i2c = (I2cTypedef*)i2cDesc[ch].base; 
       /* TWI Slave Mode Disabled, TWI Master Mode Disabled. */
        i2c->TWIHS_CR = TWIHS_CR_SVDIS;
        i2c->TWIHS_CR = TWIHS_CR_MSEN;
        unsigned char m_addr_size  = 0;   
        if(mem_type)
        {
            m_addr_size = mem_type->addr_size;
        } 
        i2c->TWIHS_MMR = (slave<<16)|(int)(m_addr_size<<8)|(1<<12);
        if(m_addr_size)
        {
            int iadr = 0;
            for(int i = 0; i <m_addr_size;i++ )
            {
                iadr <<=8;
                iadr |= mem_type->addr[i]; 
            }
            i2c->TWIHS_IADR = iadr;
        }
        i2cHalCom[ch].rx_buf = (unsigned char*)r_buf;
        i2cHalCom[ch].rx_counter = 0;
        i2cHalCom[ch].rx_length = length;
        i2cHalCom[ch].sem_id = osSemaphoreCreate (i2cHalCom[ch].semDef,0);
        i2cHalCom[ch].err_flag = 0;
        if(i2cHalCom[ch].rx_length == 1)
        {
            i2c->TWIHS_CR = TWIHS_CR_START|TWIHS_CR_STOP;
        }
        else
        {
            i2c->TWIHS_CR = TWIHS_CR_START;
        }
        i2c->TWIHS_IER = TWIHS_SR_RXRDY|TWIHS_SR_NACK|TWIHS_SR_ARBLST;  
        int sem = osSemaphoreWait (i2cHalCom[ch].sem_id, 500);
        osSemaphoreDelete(i2cHalCom[ch].sem_id); 
        if((i2cHalCom[ch].err_flag!=0)||(sem == 0))
            ret = 1;    //bool
    }
    return ret;    
}

/** 
 * @fn  void I2C_Handler(I2C_CH_TY ch)
 * @brief the i2c interrupt rountine 
 * @param[in] ch the i2c channel
 * @return none
*/
void I2C_Handler(I2C_CH_TY ch)
{
    if(ch < I2C_CH_MAX)
    {
       I2cTypedef* i2c = (I2cTypedef*)i2cDesc[ch].base; 
       int status = i2c->TWIHS_SR;
       if(status&(TWIHS_SR_NACK|TWIHS_SR_ARBLST))
       {
            i2c->TWIHS_IDR = 0xFFFF;
           i2cHalCom[ch].err_flag = 0x01;//no ack or lost arblst
           osSemaphoreRelease(i2cHalCom[ch].sem_id);
           
       }    
       if(status&TWIHS_SR_RXRDY)
       {
           i2cHalCom[ch].rx_buf[i2cHalCom[ch].rx_counter]= i2c->TWIHS_RHR;
           i2cHalCom[ch].rx_counter++;
           if(i2cHalCom[ch].rx_counter == i2cHalCom[ch].rx_length - 1 )
           {
                i2c->TWIHS_CR |= TWIHS_CR_STOP;
           }
           else if(i2cHalCom[ch].rx_counter >= i2cHalCom[ch].rx_length) 
           {
               i2c->TWIHS_IDR = TWIHS_SR_RXRDY; 
               i2c->TWIHS_IER = TWIHS_SR_TXCOMP;
           }    
       }     
       else if((i2c->TWIHS_IMR&TWIHS_SR_TXRDY)&&(status&TWIHS_SR_TXRDY))
       {
            if(i2cHalCom[ch].tx_counter < i2cHalCom[ch].tx_length)
            {
                i2c->TWIHS_THR = i2cHalCom[ch].tx_buf[i2cHalCom[ch].tx_counter];
                i2cHalCom[ch].tx_counter++;
            }
            else
            {
                i2c->TWIHS_IDR = TWIHS_SR_TXRDY;
                i2c->TWIHS_IER = TWIHS_SR_TXCOMP;
                if(i2cHalCom[ch].tx_length!= 1)
                {
                    i2c->TWIHS_CR |= TWIHS_CR_STOP;
                }      
            }        
       }    
        if((i2c->TWIHS_IMR&TWIHS_SR_TXCOMP)&&(status&TWIHS_SR_TXCOMP))
       {
            i2c->TWIHS_IDR = TWIHS_SR_TXCOMP;
            osSemaphoreRelease(i2cHalCom[ch].sem_id);
           //inform  
       }    
    }    
}

/** 
 * @fn  void I2C0_Handler(void)
 * @brief the i2c0 interrupt rountine 
 * @return none
*/
void I2C0_Handler(void)
{
    I2C_Handler(I2C_CH_0);
}

/** 
 * @fn  void I2C1_Handler(void)
 * @brief the i2c1 interrupt rountine 
 * @return none
*/
void I2C1_Handler(void)
{
    I2C_Handler(I2C_CH_1);
}    


/** 
 * @fn  void I2C1_Handler(void)
 * @brief the i2c1 interrupt rountine 
 * @return none
*/
void I2C2_Handler(void)
{
    I2C_Handler(I2C_CH_2);
} 

/**
@fn i2c_io_start0
@brief to send a start signal
@return none
*/ 
static void i2c_io_start0(unsigned scl,unsigned sda)   
{
    gpio_direction_output(scl,GPIO_DATA_HIGH);
    gpio_direction_output(sda,GPIO_DATA_HIGH);
    tm_delay_us(5);
    gpio_direction_output(sda,GPIO_DATA_LOW);
    tm_delay_us(5);
    gpio_direction_output(scl,GPIO_DATA_LOW);
    tm_delay_us(5);    // tLOW, Min:4.7us
    return;
}


/**
@fn i2c_io_stop
@brief to send a stop signal
@return none
*/ 
static void i2c_io_stop(unsigned scl,unsigned sda)   
{
    gpio_direction_output(sda,GPIO_DATA_LOW);
    tm_delay_us(5);            // tSU, Min:4.7us
    gpio_direction_output(scl,GPIO_DATA_HIGH);
    tm_delay_us(5);            // tSU, Min:4.7us
    gpio_direction_output(sda,GPIO_DATA_HIGH);
    return;
}

/** 
 * @fn  int i2c_clear_devices(I2C_CH_TY ch)
* @brief to clear i2c devices
* @param[in] ch the i2c channel
* @return int 
*/
int i2c_clear_devices(I2C_CH_TY ch)
{
    int result = 0;
    if(ch < I2C_CH_MAX)
    {
        extern const McuDrvConfigsTy   mcu_drv_configs;
        const   I2CConfigTy* i2c_config = mcu_drv_configs.i2c;
        unsigned scl = i2c_config->i2cs[ch].scl;
        unsigned sda = i2c_config->i2cs[ch].sda;
        //first umMap pin
        i2c_config->i2cs[ch].pinUnMapFunc();
        //get the sda pin status
        gpio_direction_output(scl,GPIO_DATA_HIGH);    //
        gpio_direction_input(sda);
        int counter = 0;
        while((GPIO_DATA_LOW == gpio_get_value(sda)) && (counter < 5))
        {
            counter++;
            for(int i = 0; i <9; i++)
            {    
                i2c_io_start0(scl,sda);
            }
            i2c_io_stop(scl,sda);  
        }
        
        
        if(counter >= 5)
        {
            result =  -1;
        }    
        else 
        {
            result = 0;
        }    
         //Map pin again  
        i2c_config->i2cs[ch].pinMapFunc();
    }    
    return result;
}


