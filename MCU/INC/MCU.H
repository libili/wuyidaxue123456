/*****************************************************************************/
/**
*  @file      MCU.H
*  @brief     <b> MCU H File </b>
*  @details   File functionality description:
*  This file provides the mcu core  interface.
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
#ifndef SV_MCU_H
#define SV_MCU_H
#include "../INC/GPIO.H"
typedef void(* mcu_initcall_t)(void);
#define mcu_initcall(fn)  static const mcu_initcall_t __mcu_initcall_##fn\
    __attribute__((section("mcu_init_func_level_0")))\
    __attribute__((used))=(fn)
                                                                           
typedef enum
{
    MCU_RESET_BY_SW =  0x00,
    MCU_RESET_BY_PWR = 0x01,
    MCU_RESET_BY_PIN = 0x02,
    MCU_RESET_BY_ERR = 0x03,
    MCU_RESET_BY_TRG = 0x04
}MCU_RESET_TYPE;

typedef struct
{
    GpiosConfDef pinResetSetting;
    GpiosConfDef pinSleepSetting;
    int flag;
    int crc;  
}MCU_ConfigInfoType;

typedef enum
{
	WAKEUP_EDGE_FALLING,
	WAKEUP_EDGE_RISING,
}WAKEUP_EDGE_TYPE;

/** 
 * @fn void mcu_init(void)
 * @brief   this function to init the mcu ips
 * @return none
 */
void mcu_init(void);



/** 
 * @fn  void mcu_to_stop_mode( void )
 * @brief   this function to make mcu to lowcost mode(backup mode)
 * @return none
 */
void mcu_to_stop_mode( void );


/** 
 * @fn  int mcu_pin2WakeupSrc(int pin,int current_lever)
 * @brief   change pin to wakeup source
 * @return none
 */
int mcu_pin2WakeupSrc(int pin,int current_lever);

/** 
 * @fn  void mcu_set_WakeupSources( unsigned int wakeupScr)
 * @brief   set mcu wake up sources in all
 * @return none
 */
void mcu_set_WakeupSources( unsigned int wakeupScr);


/**
 * @brief to check whether the pin is the wakeup source or not.
 *
 * @param pin  the pin to check
 * @return 1 is the wakeup source, 0 is not
 */
int mcu_is_wakeup_from(unsigned char pin);

/**
 * @brief to init mcu variable.
 *
 * @return none
 */
void mcu_variable_init(void);

/** 
 * @fn  void mcu_sw_reset( void )
 * @brief   make MCU core and all prei reset
 * @return none
 */
void mcu_sw_reset( void );

/** 
 * @fn  MCU_RESET_TYPE mcu_get_reset_reason( void )
 * @brief   this function return a flag to indicate reset reason
 * @return none
 */
MCU_RESET_TYPE mcu_get_reset_reason( void );


/** 
 * @fn  void mcu_setup_high_clock(void)
 * @brief   setup mcu clock in high speed mode,here use the main oscillation and pll circuit cpu->300M
 * @return none
 */
void mcu_setup_high_clock(void);


/** 
 * @fn  void mcu_peri_clk_enable(unsigned char id)
 * @brief   to enable one peripheral
 * @param id the peripheral id
 * @return none
 */
void mcu_peri_clk_enable(unsigned char id);  


/** 
 * @fn  void mcu_peri_clk_disable(unsigned char id)
 * @brief   to disable one peripheral
 * @param id the peripheral id
 * @return none
 */
void mcu_peri_clk_disable(unsigned char id);

/** 
 * @fn  void mcu_disable_all_peri(void)
 * @brief   to disable all peripherals
 * @return none
 */
void mcu_disable_all_peri(void);

/** 
 * @fn  void mcu_setup_low_clock(void)
 * @brief   
 * @return none
 */
void mcu_setup_low_clock(void);   


/** 
 * @fn  void fpu_init(void)
 * @brief   to init the fpu unit
 * @return none
 */
void fpu_init(void);   

/** 
 * @fn  void irq_init(void)
 * @brief   to init the irq and setting irq priority
 * @return none
 */
#define SCB_AIRCR_PRIGROUP    0x05FA0500
void irq_init(void);

/** 
 * @fn  void irq_enable( unsigned char irq_num )
 * @brief   to enable the irq interrupt
 * @param irq_num
 * @return none
 */
void irq_enable( unsigned char irq_num );

/** 
 * @fn  void irq_disable( unsigned char irq_num )
 * @brief   to disable the irq interrupt
 * @param irq_num
 * @return none
 */
void irq_disable( unsigned char irq_num );

/** 
 * @fn  void irq_disable_all( void)
 * @brief   
 * @return none
 */
void irq_disable_all( void); 
#endif /* _SV_MCU_H */
