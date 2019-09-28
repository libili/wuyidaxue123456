/*****************************************************************************/
/**
*  @file      MCU.C
*  @brief     <b> MCU C File </b>
*  @details   File functionality description:
*  This file is to implement MCU core functions.
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
#include "../INC/WDG.H"
#include "core_cm7.h"
#include "../../CONFIG/INC/MP2017_CONFIG.H"
#include "../../CONFIG/INC/MP2017_GPIOS_CONFIG.H"
#include "../INC/FLASH.H"
#include "../INC/GPIO.H"
#include "../INC/RTT.H"
#include <string.h>
static void fpu_init(void);
static void irq_init(void);
static void mcu_config_and_enable_mpu(void);
void irq_disable_all( void);
void mcu_disable_all_peri(void);
static void mcu_gpio_reset_init(void);
static void mcu_config_info_init(void);
static void mcu_hardware_initcall(void);
static int mcu_wakeup_sources;
static unsigned short mcu_wakeup_triggers;
static unsigned short mcu_wakeup_levers;
typedef struct {
  volatile   		int PMC_SCER;       /**< \brief (Pmc Offset: 0x0000) System Clock Enable Register */
  volatile   		int PMC_SCDR;       /**< \brief (Pmc Offset: 0x0004) System Clock Disable Register */
  volatile 	const  	int PMC_SCSR;       /**< \brief (Pmc Offset: 0x0008) System Clock Status Register */
  volatile 	const  	int Reserved1[1];
  volatile   		int PMC_PCER0;      /**< \brief (Pmc Offset: 0x0010) Peripheral Clock Enable Register 0 */
  volatile   		int PMC_PCDR0;      /**< \brief (Pmc Offset: 0x0014) Peripheral Clock Disable Register 0 */
  volatile 	const  	int PMC_PCSR0;      /**< \brief (Pmc Offset: 0x0018) Peripheral Clock Status Register 0 */
  volatile 			int CKGR_UCKR;      /**< \brief (Pmc Offset: 0x001C) UTMI Clock Register */
  volatile 			int CKGR_MOR;       /**< \brief (Pmc Offset: 0x0020) Main Oscillator Register */
  volatile 			int CKGR_MCFR;      /**< \brief (Pmc Offset: 0x0024) Main Clock Frequency Register */
  volatile 			int CKGR_PLLAR;     /**< \brief (Pmc Offset: 0x0028) PLLA Register */
  volatile const  	int Reserved2[1];
  volatile 			int PMC_MCKR;       /**< \brief (Pmc Offset: 0x0030) Master Clock Register */
  volatile const  	int Reserved3[1];
  volatile 			int PMC_USB;        /**< \brief (Pmc Offset: 0x0038) USB Clock Register */
  volatile const  	int Reserved4[1];
  volatile 			int PMC_PCK[7];     /**< \brief (Pmc Offset: 0x0040) Programmable Clock 0 Register */
  volatile const  	int Reserved5[1];
  volatile   		int PMC_IER;        /**< \brief (Pmc Offset: 0x0060) Interrupt Enable Register */
  volatile   		int PMC_IDR;        /**< \brief (Pmc Offset: 0x0064) Interrupt Disable Register */
  volatile const  	int PMC_SR;         /**< \brief (Pmc Offset: 0x0068) Status Register */
  volatile const  	int PMC_IMR;        /**< \brief (Pmc Offset: 0x006C) Interrupt Mask Register */
  volatile 			int PMC_FSMR;       /**< \brief (Pmc Offset: 0x0070) Fast Startup Mode Register */
  volatile 			int PMC_FSPR;       /**< \brief (Pmc Offset: 0x0074) Fast Startup Polarity Register */
  volatile   		int PMC_FOCR;       /**< \brief (Pmc Offset: 0x0078) Fault Output Clear Register */
  volatile const  	int Reserved6[26];
  volatile 			int PMC_WPMR;       /**< \brief (Pmc Offset: 0x00E4) Write Protection Mode Register */
  volatile const  	int PMC_WPSR;       /**< \brief (Pmc Offset: 0x00E8) Write Protection Status Register */
  volatile const  	int Reserved7[5];
  volatile   		int PMC_PCER1;      /**< \brief (Pmc Offset: 0x0100) Peripheral Clock Enable Register 1 */
  volatile   		int PMC_PCDR1;      /**< \brief (Pmc Offset: 0x0104) Peripheral Clock Disable Register 1 */
  volatile const  	int PMC_PCSR1;      /**< \brief (Pmc Offset: 0x0108) Peripheral Clock Status Register 1 */
  volatile 			int PMC_PCR;        /**< \brief (Pmc Offset: 0x010C) Peripheral Control Register */
  volatile 			int PMC_OCR;        /**< \brief (Pmc Offset: 0x0110) Oscillator Calibration Register */
  volatile   		int PMC_SLPWK_ER0;  /**< \brief (Pmc Offset: 0x0114) SleepWalking Enable Register 0 */
  volatile   		int PMC_SLPWK_DR0;  /**< \brief (Pmc Offset: 0x0118) SleepWalking Disable Register 0 */
  volatile const  	int PMC_SLPWK_SR0;  /**< \brief (Pmc Offset: 0x011C) SleepWalking Status Register 0 */
  volatile const  	int PMC_SLPWK_ASR0; /**< \brief (Pmc Offset: 0x0120) SleepWalking Activity Status Register 0 */
  volatile const  	int Reserved8[4];
  volatile   		int PMC_SLPWK_ER1;  /**< \brief (Pmc Offset: 0x0134) SleepWalking Enable Register 1 */
  volatile   		int PMC_SLPWK_DR1;  /**< \brief (Pmc Offset: 0x0138) SleepWalking Disable Register 1 */
  volatile const  	int PMC_SLPWK_SR1;  /**< \brief (Pmc Offset: 0x013C) SleepWalking Status Register 1 */
  volatile const  	int PMC_SLPWK_ASR1; /**< \brief (Pmc Offset: 0x0140) SleepWalking Activity Status Register 1 */
  volatile const  	int PMC_SLPWK_AIPR; /**< \brief (Pmc Offset: 0x0144) SleepWalking Activity In Progress Register */
} PmcType;
typedef struct {
  volatile  int 	SUPC_CR;   /**< \brief (Supc Offset: 0x00) Supply Controller Control Register */
  volatile 	int  	SUPC_SMMR; /**< \brief (Supc Offset: 0x04) Supply Controller Supply Monitor Mode Register */
  volatile 	int 	SUPC_MR;   /**< \brief (Supc Offset: 0x08) Supply Controller Mode Register */
  volatile 	int  	SUPC_WUMR; /**< \brief (Supc Offset: 0x0C) Supply Controller Wake-up Mode Register */
  volatile 	int 	SUPC_WUIR; /**< \brief (Supc Offset: 0x10) Supply Controller Wake-up Inputs Register */
  volatile const  int SUPC_SR;   /**< \brief (Supc Offset: 0x14) Supply Controller Status Register */
} Supc;


typedef struct {
  volatile int EEFC_FMR;      /**< \brief (Efc Offset: 0x00) EEFC Flash Mode Register */
  volatile int EEFC_FCR;      /**< \brief (Efc Offset: 0x04) EEFC Flash Command Register */
  volatile const int EEFC_FSR;      /**< \brief (Efc Offset: 0x08) EEFC Flash Status Register */
  volatile const int EEFC_FRR;      /**< \brief (Efc Offset: 0x0C) EEFC Flash Result Register */
  volatile const int Reserved1[1];
  volatile const int EEFC_VERSION;  /**< \brief (Efc Offset: 0x14) EEFC Version Register */
  volatile const int Reserved2[51];
 volatile int EEFC_WPMR;     /**< \brief (Efc Offset: 0xE4) Write Protection Mode Register */
} Efc;

#define PMC			((PmcType*)0x400E0600U)
#define SUPC   		((Supc   *)0x400E1810U) 
#define EFC    ((Efc    *)0x400E0C00U)
#define CCFG_SYSIO    (*(volatile int*)0x40088114)
#define MOSCXTEN	0x01   /*  Main Crystal Oscillator Enable */
#define MOSCXTBY	0x01<<1
#define WAITMODE    0x01<<2  /*   Puts the device in Wait mode */
#define MOSCRCEN    0x01<<3  /*  The main on-chip RC oscillator is enabled. */
#define MOSCRCF(x)   x<<4    /* Main On-Chip RC Oscillator Frequency Selection */ 
#define MOSCXTST(x)  x<<8    /*  Main Crystal Oscillator Start-up Time */   
#define MOR_KEY      0x37<<16
#define MOSCSEL      (0x01u<<24)  /* The main crystal oscillator is selected */
#define CFDEN		 0x01<<25  /* The clock failure detector is enabled.*/
#define XT32KFME     0x01<<26  /* The 32768 Hz crystal oscillator frequency monitoring is enabled */

#define MOSCXTS       0x01
#define LOCKA         (0x01<<1) /* PLLA is locked */
#define MCKRDY        0x01<<3    /* Master Clock is ready */
#define LOCKU         0x01<<6    /* UTMI PLL is locked */
#define OSCSELS       0x01<<7   /* External slow clock 32 kHz oscillator is selected*/
#define PCKRDY0       0x01<<8   /* Programmable Clock x is ready*/
#define PCKRDY1       0x01<<9   /* Programmable Clock x is ready*/
#define PCKRDY2       0x01<<10   /* Programmable Clock x is ready*/
#define PCKRDY3       0x01<<11   /* Programmable Clock x is ready*/
#define PCKRDY4       0x01<<12   /* Programmable Clock x is ready*/
#define PCKRDY5       0x01<<13   /* Programmable Clock x is ready*/
#define PCKRDY6       0x01<<14   /* Programmable Clock x is ready*/
#define MOSCSELS      0x01<<16   /*  Main Oscillator Selection Status_Selection is done */
#define MOSCRCS       (0x01<<17)   /* Main On-Chip RC Oscillator Status  RC oscillator is stabilized*/
#define CFDEV         0x01<<18   /*  At least one clock failure detection of the fast crystal oscillator clock has occurred since the last read of PMC_SR */
#define CFDS		  0x01<<19	 /*  Clock Failure Detector Status*/
#define FOS			  0x01<<20	 /*  Clock Failure Detector Fault Output Status*/
#define XT32KERR	  0x01<<21	 /* Slow Crystal Oscillator Error */
#define   PMC_FSMR_FLPM_FLASH_DEEP_POWERDOWN (0x1u << 21)
#define SYSIO12         (0x01<<12)
#define SYSIO7         (0x01<<7)
#define SYSIO6         (0x01<<6)
#define SYSIO5         (0x01<<5)
#define SYSIO4         (0x01<<4)
#define CAN1DMABA       0x20400000
#define PMC_MCKR_CSS(o,x)	 	((o&0xFFFFFFFC)|x)
#define PMC_MCKR_MDIV(o,x)		((o&0xFFFFFCFF)|(x<<8))
#define PCK_CSS_MCK  (4)
#define PCK_CSS_UPLL  (3)
#define PCK_PRES(x)  ((x-1)<<4)
#define SCER_PCK5    (1<<13)
/* -------- CKGR_UCKR : (PMC Offset: 0x001C) UTMI Clock Register -------- */
#define CKGR_UCKR_UPLLEN (0x1u << 16) /**< \brief (CKGR_UCKR) UTMI PLL Enable */
#define CKGR_UCKR_UPLLCOUNT_Pos 20
#define CKGR_UCKR_UPLLCOUNT_Msk (0xfu << CKGR_UCKR_UPLLCOUNT_Pos) /**< \brief (CKGR_UCKR) UTMI PLL Start-up Time */
#define CKGR_UCKR_UPLLCOUNT(value) ((CKGR_UCKR_UPLLCOUNT_Msk & ((value) << CKGR_UCKR_UPLLCOUNT_Pos)))
#define PMC_SR_LOCKU   (0x1u << 6)
#define     IRQ_LEVEL0_0        0x00
#define     IRQ_LEVEL0_1        0x10
#define     IRQ_LEVEL0_2        0x20
#define     IRQ_LEVEL0_3        0x30
#define     IRQ_LEVEL1_0        0x40
#define     IRQ_LEVEL1_1        0x50
#define     IRQ_LEVEL1_2        0x60
#define     IRQ_LEVEL1_3        0x70
#define     IRQ_LEVEL2_0        0x80
#define     IRQ_LEVEL2_1        0x90
#define     IRQ_LEVEL2_2        0xA0
#define     IRQ_LEVEL2_3        0xB0
#define     IRQ_LEVEL3_0        0xC0
#define     IRQ_LEVEL3_1        0xD0
#define     IRQ_LEVEL3_2        0xE0
#define     IRQ_LEVEL3_3        0xF0
#define     IRQ_DEFAULT_LEVEL   IRQ_LEVEL3_0
typedef struct
{
    volatile int RSTC_CR;
    volatile const int RSTC_SR;
    volatile int RSTC_MR;
}RstcTypedef;
#define RSTC  ((RstcTypedef*)0x400E1800)
#define RSTC_KEY    (0xA5u<<24)
#define EXTRST      (0x01<<3)
#define PROCRST      (0x01<<0)
#define URSTEN      0x01
typedef enum{
   GENERAL_RST,
   BACKUP_RST,
   WDT_RST,
   SOFT_RST,
   USER_RST,     
}RSTTYP_TYPE;


extern void flash_init(void);
#define IRQ_NUM_MAX     64
static const unsigned char irqs_priority[IRQ_NUM_MAX]={
        /*0                1                2                   3                  4 */
        IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
        IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
/*1x*/  IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
        IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
/*2x*/  IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
        IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
/*3x*/  IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
        IRQ_LEVEL1_0,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
/*4x*/  IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_LEVEL0_0,     IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
        IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
/*5x*/  IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,
        IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_LEVEL1_0,IRQ_DEFAULT_LEVEL,
/*6x*/  IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL,IRQ_DEFAULT_LEVEL
};
const static unsigned char wakeup_pin_map[14]={
    GPIO_PA0,
    GPIO_PA1,
    GPIO_PA2,
    GPIO_PA4,/*WKUP3*/
    GPIO_PA5,
    GPIO_PD28,
    GPIO_PA9,
    GPIO_PA11,
    GPIO_PA14,
    GPIO_PA19,
    GPIO_PA20,
    GPIO_PA30,
    GPIO_PB3,
    GPIO_PB5,
};

/** 
 * @fn void mcu_init(void)
 * @brief   this function to init the mcu ips
 * @return none
 */
void mcu_init(void)
{
/*first disable WDG*/
#ifndef POWER_WATCHDOG_OPTION
	WATCHDOG_Disable();
#endif    
/**/
    irq_init();    
/*config the Flash waitTime*/
	flash_init();  
/*Initial FPU IP */
	fpu_init();
/*switch to high clock*/   
	mcu_setup_high_clock();
/*config and enable mpu */
    mcu_config_and_enable_mpu();    
/*enable  cache*/
    SCB_EnableICache();
/* Init low level hardward*/ 
    mcu_hardware_initcall();
/* init gpios */ 
    mcu_gpio_reset_init();
/* */    
     
/*config TCM*/  

/* init UARTS */    
 
/* init SPIs*/    

/* init I2Cs */ 

/* init EEPROM driver*/

/* init free_timer for conter*/

}    


/** 
 * @fn void mcu_hardware_initcall(void)
 * @brief   this function to make mcu to init low level hardware
 * @return none
 */
void mcu_hardware_initcall(void)
{
   extern unsigned int Image$$MCU_INIT_CODE$$Limit;
   extern unsigned int Image$$MCU_INIT_CODE$$Base;
   mcu_initcall_t* mcu_init_fun = (mcu_initcall_t*)&Image$$MCU_INIT_CODE$$Base; 
   while(mcu_init_fun<(mcu_initcall_t*)&Image$$MCU_INIT_CODE$$Limit)
   {
       (*mcu_init_fun)();
       mcu_init_fun++;
   }     
}    

/** 
 * @fn void mcu_gpio_reset_init(void)
 * @brief   this function to init gpios
 * @return none
 */
void mcu_gpio_reset_init(void)
{
   gpio_set_all(&mcu_configs.pinResetSetting); 
} 


/** 
 * @fn void mcu_io_init_standby(void)
 * @brief   this function to init gpios before enter standby
 * @return none
 */
void mcu_io_init_standby(void)
{
   gpio_free_all_irqs(); 
   //important!! first to set the PB7/6/5/4/12 as normal GPIO
   CCFG_SYSIO = CAN1DMABA|SYSIO12|SYSIO7|SYSIO6|SYSIO5|SYSIO4;//PB7,PB6,PB5,PB4,P12 as normal IO
   //then setting the gpio to the sleep state 
   gpio_set_all(&mcu_configs.pinSleepSetting); 
}


/** 
 * @fn void mcu_config_info_init(void)
 * @brief   this function to init mcu configs info
 * @return none
 */
void mcu_config_info_init(void)
{
   extern int get_crc32_result(const unsigned char *buf, int size);
   MCU_ConfigInfoType m_config;
   flash_read_user_signature((unsigned int*)&m_config,sizeof(MCU_ConfigInfoType)/sizeof(unsigned int));
   if(0 != memcmp(&m_config,&mcu_configs,sizeof(MCU_ConfigInfoType)-sizeof(int)))
   {
        memcpy(&m_config,&mcu_configs,sizeof(MCU_ConfigInfoType)-sizeof(int));
        m_config.crc = get_crc32_result((unsigned char *)&mcu_configs, sizeof(MCU_ConfigInfoType)-sizeof(int));
        flash_erase_user_signature();
        flash_write_user_signature(&m_config,sizeof(MCU_ConfigInfoType)/sizeof(unsigned int));
   }    
}

/** 
 * @fn void mcu_config_and_enable_mpu(void)
 * @brief   this function to config mpu
 * @return none
 */
void mcu_config_and_enable_mpu(void)
{
    MPU->CTRL = 0x00;
    /*Region 0:Embedded internel flash memory- 1M*/
    MPU->RBAR = 0x00400000;
    MPU->RASR = 0x06230027;/*Read Only and use Cache*/
    
    /*Region 1: Internal SRAM -16K(4k for mcan and 12k for ram_run_function)*/
    MPU->RBAR = 0x20400011;
    MPU->RASR = 0x030C001B;
    /*Region 2: Internal SRAM -16K*/
    MPU->RBAR = 0x20404012;
    MPU->RASR = 0x0323001B;
    /*Region 3: Internal SRAM -32K*/
    MPU->RBAR = 0x20408013;
    MPU->RASR = 0x0323001D;
    /*Region 4: Internal SRAM -64K*/
    MPU->RBAR = 0x20410014;
    MPU->RASR = 0x0323001F;
    /*Region 5: Internal SRAM -256K*/
    MPU->RBAR = 0x20420015;
    MPU->RASR = 0x03230023;
    
    /*Region6: Controllers - 1MB*/
    MPU->RBAR = 0x40000016;
    MPU->RASR = 0x13010027;
    
    /*Region7: EBI SDRAM mapping space - 512M */
    MPU->RBAR = 0x60000017;
    MPU->RASR = 0x03000039;
  
    /*Region 8: QSPI  mapping space - 512MB */
    MPU->RBAR = 0x80000018;
    MPU->RASR = 0x03000039;
  
    /* Region9: system mapping space - 256M*/
    MPU->RBAR = 0xE0000019;
    MPU->RASR = 0x03000037;
   
    //enable mpu
    MPU->CTRL = 0x01;
}


/** 
 * @fn unsigned short mcu_get_wakeup_pin_lever(void)
 * @brief   this function to get wakeup pin current lever
 * @return unsigned short
 */
unsigned short mcu_get_wakeup_pin_lever(void)
{
    unsigned short new_lever = 0;
    for(int i = 0; i < 14; i++)
    {
        if(mcu_wakeup_triggers&(1<<i))
        {
           new_lever |= gpio_get_value(wakeup_pin_map[i])<<i;
        }    
    }
    return new_lever;
}

/** 
 * @fn void mcu_enable_os_sched(void)
 * @brief   this function to enable os schedule
 * @return none
 */
void mcu_enable_os_sched(void)
{
    SysTick->CTRL |= (SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk);
} 

/** 
 * @fn void mcu_disable_os_sched(void)
 * @brief   this function to disable os schedule
 * @return none
 */
void mcu_disable_os_sched(void)
{
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk);
}

/** 
 * @fn  void mcu_use_rc_for_mainclk(void)
 * @brief   setup main clock to 4M RC clock
 * @return none
 */
void mcu_use_rc_for_mainclk(void)
{
    //enable main cr clock and wait ready
    if((PMC->PMC_SR&MOSCRCS)!= MOSCRCS)
    {
        PMC->CKGR_MOR = MOR_KEY | MOSCRCEN |MOSCXTEN;
        while((PMC->PMC_SR&MOSCRCS)!=MOSCRCS)
		{	
		}
    }
    //switch to main cr clock
    PMC->CKGR_MOR = MOR_KEY|  MOSCRCEN |MOSCXTEN;
    while((PMC->PMC_SR&MOSCSELS)!= MOSCSELS)
	{	
	}
    //switch off osc clock
    PMC->CKGR_MOR = MOR_KEY|  MOSCRCEN;
    //swith to main clock
   // PMC->PMC_MCKR = PMC_MCKR_CSS(PMC->PMC_MCKR,1 );
	//while((PMC->PMC_SR&MCKRDY)!=MCKRDY)
	//{
	//}
}  
/** 
 * @fn  void mcu_to_stop_mode( void )
 * @brief   this function to make mcu to lowcost mode(backup mode)
 * @return none
 */
 void mcu_to_stop_mode( void )
{
    mcu_disable_os_sched();
    mcu_io_init_standby();
    mcu_use_rc_for_mainclk();
    unsigned short target_trigger;
    SUPC->SUPC_SR;//clear last wakeup source register
    irq_disable_all();
    //mcu_disable_all_peri();
    target_trigger = ~mcu_wakeup_levers;
#ifdef RTT_WAKE_OPTION	
	    RTT_ConfigTy config ={
        .interval = 100
    };
    rtt_init(&config);
    rtt_setAlarm(50,0);
    SUPC->SUPC_WUMR = ( 1<<2);//setting WKUPDBC=0 and enable RTT wakeup
#else
	SUPC->SUPC_WUMR = 0<<12;//setting WKUPDBC=0 
#endif	  
    SUPC->SUPC_WUIR = ((unsigned int)target_trigger)<<16 | mcu_wakeup_triggers; 
    SCB->SCR |= 0x04; // setting SLEEPDEEP to 1;
    SUPC->SUPC_CR = (0xA5UL<<24)|0x04;
    while(1);
   
    
}


static int indexOf(const unsigned char* array,const unsigned char array_size,unsigned char element)
{
    for(int i = 0; i < array_size; i++ )
    {
        if(array[i]==element)
        {
            return i;
        }    
    }
    return -1;    
}

/** 
 * @fn  int mcu_pin2WakeupSrc(int pin,int current_lever)
 * @brief   change pin to wakeup source
 * @return none
 */
int mcu_pin2WakeupSrc(int pin,int current_lever)
{
    int wpin = indexOf(wakeup_pin_map,14,pin);
    if(wpin!=-1)
    {
       return  ((1<<wpin)|((current_lever&0x01)<<(wpin+16)));
    }
    return 0;    
}

/** 
 * @fn  void mcu_set_WakeupSources( unsigned int wakeupScr)
 * @brief   set mcu wake up sources in all
 * @return none
 */
void mcu_set_WakeupSources( unsigned int wakeupScr)
{
    mcu_wakeup_triggers = wakeupScr&0xFFFF;
    mcu_wakeup_levers = (wakeupScr >> 16)&0xFFFF;
}


/**
 * @brief to check whether the pin is the wakeup source or not.
 *
 * @param pin  the pin to check
 * @return 1 is the wakeup source, 0 is not
 */
int mcu_is_wakeup_from(unsigned char pin)
{
    int wpin = indexOf(wakeup_pin_map,14,pin);
    int rst  = (SUPC->SUPC_SR>>8)&0x07;
    if((wpin!=-1)&&(rst==1))
    {
        return (((mcu_wakeup_sources>>16)&(1<<wpin))==(1<<wpin));
    }
    else if((pin == RTT)&&(rst==1)&&((mcu_wakeup_sources>>16)==0x00))
    {
        return 1;
    }    
    return 0;    
}

/**
 * @brief to init mcu variable.
 *
 * @return none
 */
void mcu_variable_init(void)
{
    mcu_wakeup_sources = SUPC->SUPC_SR;
    mcu_config_info_init();
}    


/** 
 * @fn  void mcu_sw_reset( void )
 * @brief   make MCU core and all prei reset
 * @return none
 */
void mcu_sw_reset( void )
{
    RSTC->RSTC_CR = RSTC_KEY|PROCRST;
    for(;;);
} 


/** 
 * @fn  MCU_RESET_TYPE mcu_get_reset_reason( void )
 * @brief   this function return a flag to indicate reset reason
 * @return none
 */
MCU_RESET_TYPE mcu_get_reset_reason( void )
{
    MCU_RESET_TYPE type;
    int rsttyp = (RSTC->RSTC_SR>>8)&0x07;
    switch(rsttyp)
    {
        case BACKUP_RST:
            type = MCU_RESET_BY_TRG;
            break;
        case WDT_RST:
            type = MCU_RESET_BY_ERR;
            break;
        case SOFT_RST:
            type = MCU_RESET_BY_SW;
            break;
        case USER_RST:
        case GENERAL_RST:    
        default:
            type = MCU_RESET_BY_PWR;
        break;
    }    
    return type;
}



/** 
 * @fn  void mcu_setup_high_clock(void)
 * @brief   setup mcu clock in high speed mode,here use the main oscillation and pll circuit cpu->300M
 * @return none
 */
void mcu_setup_high_clock(void)
{
    extern const McuDrvConfigsTy   mcu_drv_configs;
	const McuTimingConfigTy* pconf= mcu_drv_configs.mcu_timing;
    /*Initialize Main OSX*/
	if((PMC->CKGR_MOR&MOSCSEL)!= MOSCSEL)
	{
		PMC->CKGR_MOR = MOR_KEY | MOSCRCEN |MOSCXTEN|MOSCXTST(pconf->mosx_wait_time);
		while((PMC->PMC_SR&MOSCXTS)!=MOSCXTS)
		{	
		}	
	}	
	/*Switch to Xtal oscillator*/
	PMC->CKGR_MOR = MOR_KEY|  MOSCRCEN |MOSCXTEN| MOSCSEL | MOSCXTST(pconf->mosx_wait_time);
	while((PMC->PMC_SR&MOSCSELS)!= MOSCSELS)
	{	
	}
	
	PMC->PMC_MCKR = PMC_MCKR_CSS(PMC->PMC_MCKR,1 );
	while((PMC->PMC_SR&MCKRDY)!=MCKRDY)
	{
	}	
	/*Initialize PLLA*/
	PMC->CKGR_PLLAR = (1<<29)|(int)(pconf->pll_wait_time<<8)|pconf->pll_diva|(int)(pconf->pll_mula<<16);
	while((PMC->PMC_SR&LOCKA)!=LOCKA)
	{
	}	
	/*Switch to PLLA*/
    PMC->PMC_MCKR = PMC_MCKR_MDIV(PMC->PMC_MCKR,1);
	while ( (PMC->PMC_SR & MCKRDY)!= MCKRDY )
	{
	}
	PMC->PMC_MCKR =  PMC_MCKR_CSS(PMC->PMC_MCKR,2 );
    while ( (PMC->PMC_SR & MCKRDY)!= MCKRDY )
	{
	}
    PMC->PMC_MCKR =  PMC_MCKR_MDIV(PMC->PMC_MCKR,3/*300M/3=100M(MCLK)*/ );//DIV3-3 DIV4-2 DIV2-1 DIV1-0
    while ( (PMC->PMC_SR & MCKRDY)!= MCKRDY )
	{
	}
    
	/*for 32K osx*/
    
    
	/*for usb and pcks */ 
    /* Enable PLL 480 MHz */
	PMC->CKGR_UCKR = CKGR_UCKR_UPLLEN | CKGR_UCKR_UPLLCOUNT(0xF);
	/* Wait that PLL is considered locked by the PMC */
	while (!(PMC->PMC_SR & PMC_SR_LOCKU))
    {
    } 
    /*Select 16M for MCan*/
    PMC->PMC_PCK[5]= PCK_CSS_UPLL|PCK_PRES(24);
    while ( (PMC->PMC_SR & PCKRDY5)!= PCKRDY5 )
	{
        
	}
    PMC->PMC_SCER = SCER_PCK5;
    /*for Enable default Peripheral Clock*/
    PMC->PMC_PCER0 = 0x00031C00;//only enable GPIOS clock
    PMC->PMC_PCER1 = 0x04000028;//enable CAN clock and DMA clock
}


/** 
 * @fn  void mcu_peri_clk_enable(unsigned char id)
 * @brief   to enable one peripheral
 * @param id the peripheral id
 * @return none
 */
void mcu_peri_clk_enable(unsigned char id)
{
   if(id < 32)
   {
       PMC->PMC_PCER0 = 1<<id;
   }
   else if(id <64)
   {
       PMC->PMC_PCER1 = 1<<(id-32); 
   }    
       
}    


/** 
 * @fn  void mcu_peri_clk_disable(unsigned char id)
 * @brief   to disable one peripheral
 * @param id the peripheral id
 * @return none
 */
void mcu_peri_clk_disable(unsigned char id)
{
   if(id < 32)
   {
       PMC->PMC_PCDR0 = 1<<id;
   }
   else if(id <64)
   {
       PMC->PMC_PCDR1 = 1<<(id-32); 
   } 
}

/** 
 * @fn  void mcu_disable_all_peri(void)
 * @brief   to disable all peripherals
 * @return none
 */
void mcu_disable_all_peri(void)
{
    PMC->PMC_PCDR0 = 0xFFFFFF80;
    PMC->PMC_PCDR1 = 0x1F3FFFAF;
    while((PMC->PMC_PCSR0&0xFFFFFF80)!= 0x00000000);
    while((PMC->PMC_PCSR1&0x1F3FFFAF)!= 0x00000000);
}



/** 
 * @fn  void fpu_init(void)
 * @brief   to init the fpu unit
 * @return none
 */
void fpu_init(void)
{
#if (__FPU_USED == 1)                   // Keil
  /* enable FPU if available and used */
  SCB->CPACR |= ((3UL << 10*2) |       /* set CP10 Full Access */
                 (3UL << 11*2)  );     /* set CP11 Full Access */
#endif 
    
}    

/** 
 * @fn  void irq_init(void)
 * @brief   to init the irq and setting irq priority
 * @return none
 */
#define SCB_AIRCR_PRIGROUP    0x05FA0500
void irq_init(void)
{
    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICER[1] = 0xFFFFFFFF;
    SCB->AIRCR = SCB_AIRCR_PRIGROUP;
    /*config  the prority */
    for(int i = 0; i < IRQ_NUM_MAX; i++)
    {
       NVIC->IP[i] =  irqs_priority[i];
    }
}

/** 
 * @fn  void irq_enable( unsigned char irq_num )
 * @brief   to enable the irq interrupt
 * @param irq_num
 * @return none
 */
void irq_enable( unsigned char irq_num )
{
	if(irq_num <= 68)
	{
		NVIC->ISER[irq_num >> 5] =  0x1<<(irq_num&0x1F);
	}
    
}

/** 
 * @fn  void irq_disable( unsigned char irq_num )
 * @brief   to disable the irq interrupt
 * @param irq_num
 * @return none
 */
void irq_disable( unsigned char irq_num )
{
	if(irq_num <= 68)
	{
		NVIC->ICER[irq_num >> 5] =  0x1<<(irq_num&0x1F);
	}
}

/** 
 * @fn  void irq_disable_all( void)
 * @brief   
 * @return none
 */
void irq_disable_all( void)
{
    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICER[1] = 0xFFFFFFFF;
    NVIC->ICER[2] = 0xFFFFFFFF;
}  

/** 
 * @fn  void mcu_reset_init_level0(void)
 * @brief   set NRST pin can be able to occur reset 
 * @return none
 */
static void mcu_reset_init_level0(void)
{
    RSTC->RSTC_MR = URSTEN;
    /* Setting System I/O and CAN1 DMA Base Address*/
    CCFG_SYSIO = CAN1DMABA|SYSIO12|SYSIO5|SYSIO4;//PB5,PB4,P12 as normal IO
}

mcu_initcall(mcu_reset_init_level0);
