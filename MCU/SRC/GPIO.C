/*****************************************************************************/
/**
*  @file      GPIO.C
*  @brief     <b> GPIO C File </b>
*  @details   File functionality description:
*  This file is to implement GPIO driver.
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
#include "../INC/GPIO.H"
#include "../INC/MCU.H"
#include <string.h>
typedef struct {
  volatile          unsigned int PIO_PER;        /**< \brief (Pio Offset: 0x0000) PIO Enable Register */
  volatile          unsigned int PIO_PDR;        /**< \brief (Pio Offset: 0x0004) PIO Disable Register */
  volatile 	const   unsigned int PIO_PSR;        /**< \brief (Pio Offset: 0x0008) PIO Status Register */
  volatile 	const   unsigned int Reserved1[1];
  volatile          unsigned int PIO_OER;        /**< \brief (Pio Offset: 0x0010) Output Enable Register */
  volatile          unsigned int PIO_ODR;        /**< \brief (Pio Offset: 0x0014) Output Disable Register */
  volatile 	const   unsigned int PIO_OSR;        /**< \brief (Pio Offset: 0x0018) Output Status Register */
  volatile 	const   unsigned int Reserved2[1];
  volatile          unsigned int PIO_IFER;       /**< \brief (Pio Offset: 0x0020) Glitch Input Filter Enable Register */
  volatile          unsigned int PIO_IFDR;       /**< \brief (Pio Offset: 0x0024) Glitch Input Filter Disable Register */
  volatile 	const   unsigned int PIO_IFSR;       /**< \brief (Pio Offset: 0x0028) Glitch Input Filter Status Register */
  volatile 	const   unsigned int Reserved3[1];
  volatile          unsigned int PIO_SODR;       /**< \brief (Pio Offset: 0x0030) Set Output Data Register */
  volatile          unsigned int PIO_CODR;       /**< \brief (Pio Offset: 0x0034) Clear Output Data Register */
  volatile          unsigned int PIO_ODSR;       /**< \brief (Pio Offset: 0x0038) Output Data Status Register */
  volatile 	const   unsigned int PIO_PDSR;       /**< \brief (Pio Offset: 0x003C) Pin Data Status Register */
  volatile          unsigned int PIO_IER;        /**< \brief (Pio Offset: 0x0040) Interrupt Enable Register */
  volatile          unsigned int PIO_IDR;        /**< \brief (Pio Offset: 0x0044) Interrupt Disable Register */
  volatile 	const   unsigned int PIO_IMR;        /**< \brief (Pio Offset: 0x0048) Interrupt Mask Register */
  volatile 	const   unsigned int PIO_ISR;        /**< \brief (Pio Offset: 0x004C) Interrupt Status Register */
  volatile          unsigned int PIO_MDER;       /**< \brief (Pio Offset: 0x0050) Multi-driver Enable Register */
  volatile          unsigned int PIO_MDDR;       /**< \brief (Pio Offset: 0x0054) Multi-driver Disable Register */
  volatile 	const   unsigned int PIO_MDSR;       /**< \brief (Pio Offset: 0x0058) Multi-driver Status Register */
  volatile 	const   unsigned int Reserved4[1];
  volatile          unsigned int PIO_PUDR;       /**< \brief (Pio Offset: 0x0060) Pull-up Disable Register */
  volatile          unsigned int PIO_PUER;       /**< \brief (Pio Offset: 0x0064) Pull-up Enable Register */
  volatile 	const   unsigned int PIO_PUSR;       /**< \brief (Pio Offset: 0x0068) Pad Pull-up Status Register */
  volatile 	const   unsigned int Reserved5[1];
  volatile          unsigned int PIO_ABCDSR[2];  /**< \brief (Pio Offset: 0x0070) Peripheral Select Register */
  volatile 	const   unsigned int Reserved6[2];
  volatile          unsigned int PIO_IFSCDR;     /**< \brief (Pio Offset: 0x0080) Input Filter Slow Clock Disable Register */
  volatile          unsigned int PIO_IFSCER;     /**< \brief (Pio Offset: 0x0084) Input Filter Slow Clock Enable Register */
  volatile 	const   unsigned int PIO_IFSCSR;     /**< \brief (Pio Offset: 0x0088) Input Filter Slow Clock Status Register */
  volatile          unsigned int PIO_SCDR;       /**< \brief (Pio Offset: 0x008C) Slow Clock Divider Debouncing Register */
  volatile          unsigned int PIO_PPDDR;      /**< \brief (Pio Offset: 0x0090) Pad Pull-down Disable Register */
  volatile          unsigned int PIO_PPDER;      /**< \brief (Pio Offset: 0x0094) Pad Pull-down Enable Register */
  volatile 	const   unsigned int PIO_PPDSR;      /**< \brief (Pio Offset: 0x0098) Pad Pull-down Status Register */
  volatile 	const   unsigned int Reserved7[1];
  volatile          unsigned int PIO_OWER;       /**< \brief (Pio Offset: 0x00A0) Output Write Enable */
  volatile          unsigned int PIO_OWDR;       /**< \brief (Pio Offset: 0x00A4) Output Write Disable */
  volatile 	const   unsigned int PIO_OWSR;       /**< \brief (Pio Offset: 0x00A8) Output Write Status Register */
  volatile 	const   unsigned int Reserved8[1];
  volatile          unsigned int PIO_AIMER;      /**< \brief (Pio Offset: 0x00B0) Additional Interrupt Modes Enable Register */
  volatile          unsigned int PIO_AIMDR;      /**< \brief (Pio Offset: 0x00B4) Additional Interrupt Modes Disable Register */
  volatile 	const   unsigned int PIO_AIMMR;      /**< \brief (Pio Offset: 0x00B8) Additional Interrupt Modes Mask Register */
  volatile 	const   unsigned int Reserved9[1];
  volatile          unsigned int PIO_ESR;        /**< \brief (Pio Offset: 0x00C0) Edge Select Register */
  volatile          unsigned int PIO_LSR;        /**< \brief (Pio Offset: 0x00C4) Level Select Register */
  volatile 	const   unsigned int PIO_ELSR;       /**< \brief (Pio Offset: 0x00C8) Edge/Level Status Register */
  volatile 	const   unsigned int Reserved10[1];
  volatile          unsigned int PIO_FELLSR;     /**< \brief (Pio Offset: 0x00D0) Falling Edge/Low-Level Select Register */
  volatile          unsigned int PIO_REHLSR;     /**< \brief (Pio Offset: 0x00D4) Rising Edge/High-Level Select Register */
  volatile 	const   unsigned int PIO_FRLHSR;     /**< \brief (Pio Offset: 0x00D8) Fall/Rise - Low/High Status Register */
  volatile 	const   unsigned int Reserved11[1];
  volatile 	const   unsigned int PIO_LOCKSR;     /**< \brief (Pio Offset: 0x00E0) Lock Status */
  volatile          unsigned int PIO_WPMR;       /**< \brief (Pio Offset: 0x00E4) Write Protection Mode Register */
  volatile 	const   unsigned int PIO_WPSR;       /**< \brief (Pio Offset: 0x00E8) Write Protection Status Register */
  volatile 	const   unsigned int Reserved12[4];
  volatile 	const   unsigned int PIO_VERSION;    /**< \brief (Pio Offset: 0x00FC) Version Register */
  volatile          unsigned int PIO_SCHMITT;    /**< \brief (Pio Offset: 0x0100) Schmitt Trigger Register */
  volatile 	const   unsigned int Reserved13[5];
  volatile          unsigned int PIO_DRIVER;     /**< \brief (Pio Offset: 0x0118) I/O Drive Register */
  volatile 	const   unsigned int Reserved14[13];
  volatile          unsigned int PIO_PCMR;       /**< \brief (Pio Offset: 0x0150) Parallel Capture Mode Register */
  volatile          unsigned int PIO_PCIER;      /**< \brief (Pio Offset: 0x0154) Parallel Capture Interrupt Enable Register */
  volatile          unsigned int PIO_PCIDR;      /**< \brief (Pio Offset: 0x0158) Parallel Capture Interrupt Disable Register */
  volatile 	const   unsigned int PIO_PCIMR;      /**< \brief (Pio Offset: 0x015C) Parallel Capture Interrupt Mask Register */
  volatile 	const   unsigned int PIO_PCISR;      /**< \brief (Pio Offset: 0x0160) Parallel Capture Interrupt Status Register */
  volatile 	const   unsigned int PIO_PCRHR;      /**< \brief (Pio Offset: 0x0164) Parallel Capture Reception Holding Register */
}GPIO_Typedef;

typedef struct{
	unsigned int base;
	unsigned int irq;
}GpioDescTypedef;
const static GpioDescTypedef gpioDesc[5]={
	{	0x400E0E00,10 	},
	{	0x400E1000,11	},
	{	0x400E1200,12	},
	{	0x400E1400,16	},
	{	0x400E1600,17	},
};
typedef void (*GPIO_IRQ_CB)(void);
static GPIO_IRQ_CB  irq_cbs[GPIO_P_MAX];

/***********************************************************************
*  Name        : gpio_set_all
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void gpio_set_all(const GpiosConfDef* gpio)
{
    for(int i = 0; i <5; i++)
    {
        GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[i].base;
        //input default 
        gpiox->PIO_PDR = gpio->port[i].per;
        gpiox->PIO_PER = ~(gpio->port[i].per);
        gpiox->PIO_ABCDSR[0] = gpio->port[i].sel_a;
        gpiox->PIO_ABCDSR[1] = gpio->port[i].sel_b;
        gpiox->PIO_ODR = 0xFFFFFFFF;
        gpiox->PIO_OER = gpio->port[i].dir;
        gpiox->PIO_CODR = 0xFFFFFFFF;
        gpiox->PIO_SODR = gpio->port[i].data;
    }
}    
/***********************************************************************
*  Name        : gpio_set_cfg
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void gpio_set_cfg(unsigned gpio,GPIO_ModeTy mode)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[(gpio/32)].base;
	int rel_no = gpio%32;
    if(mode==GPIO_MODE_IO || mode == GPIO_MODE_AF_OD)
    {
        gpiox->PIO_PER = 1<<rel_no;
        if(mode == GPIO_MODE_AF_OD)
        {
            gpiox->PIO_MDER = 1<<rel_no;
            return;
        }
        gpiox->PIO_MDDR = 1<<rel_no;    
    }
    else
    {
        gpiox->PIO_ABCDSR[0] &= ~(1<<rel_no);
        gpiox->PIO_ABCDSR[1] &= ~(1<<rel_no);
        if(mode==GPIO_MODE_ALT_FUNCTION_B)
        {
            gpiox->PIO_ABCDSR[0] |= 1<<rel_no;
            
        }
        else if(mode==GPIO_MODE_ALT_FUNCTION_C)
        {
            gpiox->PIO_ABCDSR[1] |= 1<<rel_no;
        }    
        else if(mode==GPIO_MODE_ALT_FUNCTION_D)
        {
            gpiox->PIO_ABCDSR[0] |= 1<<rel_no;
            gpiox->PIO_ABCDSR[1] |= 1<<rel_no;
        }    
        gpiox->PIO_PDR = 1<<rel_no;
    }    
    
}
/***********************************************************************
*  Name        : void gpio_set_filter(unsigned gpio, int enable)
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
 void gpio_set_filter(unsigned gpio, int enable)
 {
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[(gpio/32)].base;
	int rel_no = gpio%32;
    if(enable)
    {
        gpiox->PIO_IFSCER = 1<< rel_no;
        gpiox->PIO_IFER   = 1<< rel_no;
    }
    else
    {
        gpiox->PIO_IFDR   = 1<< rel_no;
        gpiox->PIO_IFSCDR = 1<< rel_no;
    }    
 }    


/***********************************************************************
*  Name        : gpio_set_value
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void gpio_set_value(unsigned gpio, int value)
{
	GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[(gpio/32)].base;
	int rel_no = gpio%32;
	if(value)
	{
		gpiox->PIO_SODR = 1<< rel_no;
	}
	else
	{
		gpiox->PIO_CODR = 1<< rel_no;
	}
}


/***********************************************************************
*  Name        : gpio_get_value
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
int gpio_get_value(unsigned gpio)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[(gpio/32)].base;
	int rel_no = gpio%32;
    return (gpiox->PIO_PDSR&(1<<rel_no)?GPIO_DATA_HIGH:GPIO_DATA_LOW);
}

/***********************************************************************
*  Name        : gpio_direction_output
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void gpio_direction_output(unsigned gpio, int value)
{
	GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[(gpio/32)].base;
	int rel_no = gpio%32;
	if(value)
	{
		gpiox->PIO_SODR = 1<< rel_no;
	}
	else
	{
		gpiox->PIO_CODR = 1<< rel_no;
	}
	gpiox->PIO_OER = 1<< rel_no;
	gpiox->PIO_PER = 1<< rel_no;
}


/***********************************************************************
*  Name        : gpio_direction_input
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void gpio_direction_input(unsigned gpio)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[(gpio/32)].base;
	int rel_no = gpio%32;
    if(gpio < GPIO_P_MAX)
    {
        gpiox->PIO_ODR = 1<< rel_no;
        gpiox->PIO_PER = 1<< rel_no;
    }     
}


/***********************************************************************
*  Name        : gpio_setpull
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void gpio_setpull(unsigned gpio,GPIO_PullTy pull)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[(gpio/32)].base;
	int rel_no = gpio%32;
    if(gpio < GPIO_P_MAX)
    {
        if(pull== GPIO_PULL_DOWN)
        {
            gpiox->PIO_PUDR = 1<< rel_no;
            gpiox->PIO_PPDER = 1<< rel_no;
        }
        else if(pull== GPIO_PULL_UP)
        {
            gpiox->PIO_PPDDR = 1<< rel_no;
            gpiox->PIO_PUER = 1<< rel_no;
        }
        else
        {
            gpiox->PIO_PUDR = 1<< rel_no;
            gpiox->PIO_PPDDR = 1<< rel_no;
        }        
    }    
}    
/***********************************************************************
*  Name        : gpio_request_irq
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void gpio_request_irq(unsigned gpio,GPIO_TrigTy type,void(*gpio_irq_cb)(void))
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[(gpio/32)].base;
	int rel_no = gpio%32;
    if(gpio <GPIO_P_MAX)
    {
       irq_cbs[gpio] =  gpio_irq_cb;
       if(type!= GPIO_TRIG_BOTH_EDGES)
       {
           if((type==GPIO_TRIG_RISING_EDGE)||(type==GPIO_TRIG_FALLING_EDGE))
           {
              gpiox->PIO_ESR = 1<< rel_no;
           }
           else
           {
               gpiox->PIO_LSR = 1<< rel_no;  
           }
           if((type==GPIO_TRIG_LOW_LEVEL)||(type==GPIO_TRIG_FALLING_EDGE))
           { 
                gpiox->PIO_FELLSR = 1<< rel_no; 
           }
           else
           {
                gpiox->PIO_REHLSR = 1<< rel_no;
           }
           gpiox->PIO_AIMER = 1<< rel_no;
       }
       else
       {
           gpiox->PIO_AIMDR = 1<< rel_no;
          
       }
       gpiox->PIO_ISR;
       gpiox->PIO_IER = 1<< rel_no;   
       irq_enable(gpioDesc[(gpio/32)].irq); 
    }    
}

/***********************************************************************
*  Name        : gpio_free_irq
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void gpio_free_irq(unsigned gpio)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[(gpio/32)].base;
	int rel_no = gpio%32;
    if(gpio < GPIO_P_MAX )
    {
       irq_cbs[gpio] = 0; 
       gpiox->PIO_IDR = 1<< rel_no; 
       if(gpiox->PIO_IMR==0xFFFFFFFF)
       {
           irq_disable(gpioDesc[(gpio/32)].irq);  
       }     
    }    
}

/***********************************************************************
*  Name        : gpio_free_all_irqs
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void gpio_free_all_irqs(void)
{
    for(int i = 0; i <5; i++)
    {
        GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[i].base;
        gpiox->PIO_IDR = 0xFFFFFFFF;
        irq_disable(gpioDesc[i].irq); 
    }
    memset(irq_cbs,0,sizeof(irq_cbs));
    
}
/***********************************************************************
*  Name        : GPIOA_Handler
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void GPIOA_Handler(void)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[0].base;
    int isr = gpiox->PIO_ISR;
    for(int i= 0; i <32; i++)
    {
        if((isr&0x01)&&(irq_cbs[i]!= 0))
        {
            irq_cbs[i]();
        }
        isr >>= 1;    
    }
}


/***********************************************************************
*  Name        : GPIOB_Handler
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void GPIOB_Handler(void)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[1].base;
    int isr = gpiox->PIO_ISR;
    for(int i= 0; i <32; i++)
    {
        if((isr&0x01)&&(irq_cbs[32+i]!= 0))
        {
            irq_cbs[32+i]();
        }
        isr >>= 1;     
    }
}


/***********************************************************************
*  Name        : GPIOC_Handler
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void GPIOC_Handler(void)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[2].base;
    int isr = gpiox->PIO_ISR;
    for(int i= 0; i <32; i++)
    {
        if((isr&0x01)&&(irq_cbs[64+i]!= 0))
        {
            irq_cbs[64+i]();
        }
        isr >>= 1;     
    }
}


/***********************************************************************
*  Name        : GPIOD_Handler
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void GPIOD_Handler(void)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[3].base;
    int isr = gpiox->PIO_ISR;
    for(int i = 0; i <32; i++)
    {
        if((isr&0x01)&&(irq_cbs[96+i]!= 0))
        {
            irq_cbs[96+i]();
        }
        isr >>= 1;     
    }
}


/***********************************************************************
*  Name        : GPIOE_Handler
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
void GPIOE_Handler(void)
{
    GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[4].base;
    int isr = gpiox->PIO_ISR;
    for(int i= 0; i <32; i++)
    {
        if((isr&0x01)&&(irq_cbs[128+i]!= 0))
        {
            irq_cbs[128+i]();
        }
        isr >>= 1;     
    }
}


/***********************************************************************
*  Name        : GPIOE_Handler
*  Description :
*  Parameter   : None
*  Returns     : None
***********************************************************************/
static void mcu_gpio_Init_level0(void)
{
    for(int i = 0; i<5; i++)
    {
        GPIO_Typedef* gpiox = (GPIO_Typedef*)gpioDesc[i].base;
        gpiox->PIO_ISR;
        gpiox->PIO_MDDR = 0xFFFFFFFF;//disable all OD 
        gpiox->PIO_PPDDR = 0xFFFFFFFF;//disable all pull down
        gpiox->PIO_PUDR = 0xFFFFFFFF;//disable all pull up
        gpiox->PIO_SCDR = 0x00;//Slow Clock Divider Selection for Debouncing 2tslk
    }
}

mcu_initcall(mcu_gpio_Init_level0);
