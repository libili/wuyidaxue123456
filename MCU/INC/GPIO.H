/*****************************************************************************/
/**
*  @file      GPIO.H
*  @brief     <b> GPIO H File </b>
*  @details   File functionality description:
*  This file provides the gpio driver interface.
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
#ifndef __SV_GPIO_H
#define __SV_GPIO_H

typedef enum
{
	GPIO_PA0,
	GPIO_PA1,
	GPIO_PA2,
	GPIO_PA3,
	GPIO_PA4,
	GPIO_PA5,
	GPIO_PA6,
	GPIO_PA7,
	GPIO_PA8,
	GPIO_PA9,
	GPIO_PA10,
	GPIO_PA11,
	GPIO_PA12,
	GPIO_PA13,
	GPIO_PA14,
	GPIO_PA15,
	GPIO_PA16,
	GPIO_PA17,
	GPIO_PA18,
	GPIO_PA19,
	GPIO_PA20,
	GPIO_PA21,
	GPIO_PA22,
	GPIO_PA23,
	GPIO_PA24,
	GPIO_PA25,
	GPIO_PA26,
	GPIO_PA27,
	GPIO_PA28,
	GPIO_PA29,
	GPIO_PA30,
	GPIO_PA31,
	GPIO_PB0,
	GPIO_PB1,
	GPIO_PB2,
	GPIO_PB3,
	GPIO_PB4,
	GPIO_PB5,
	GPIO_PB6,
	GPIO_PB7,
	GPIO_PB8,
	GPIO_PB9,
	GPIO_PB10,
	GPIO_PB11,
	GPIO_PB12,
	GPIO_PB13,
	GPIO_PB14,
	GPIO_PB15,
	GPIO_PB16,
	GPIO_PB17,
	GPIO_PB18,
	GPIO_PB19,
	GPIO_PB20,
	GPIO_PB21,
	GPIO_PB22,
	GPIO_PB23,
	GPIO_PB24,
	GPIO_PB25,
	GPIO_PB26,
	GPIO_PB27,
	GPIO_PB28,
	GPIO_PB29,
	GPIO_PB30,
	GPIO_PB31,
	GPIO_PC0,
	GPIO_PC1,
	GPIO_PC2,
	GPIO_PC3,
	GPIO_PC4,
	GPIO_PC5,
	GPIO_PC6,
	GPIO_PC7,
	GPIO_PC8,
	GPIO_PC9,
	GPIO_PC10,
	GPIO_PC11,
	GPIO_PC12,
	GPIO_PC13,
	GPIO_PC14,
	GPIO_PC15,
	GPIO_PC16,
	GPIO_PC17,
	GPIO_PC18,
	GPIO_PC19,
	GPIO_PC20,
	GPIO_PC21,
	GPIO_PC22,
	GPIO_PC23,
	GPIO_PC24,
	GPIO_PC25,
	GPIO_PC26,
	GPIO_PC27,
	GPIO_PC28,
	GPIO_PC29,
	GPIO_PC30,
	GPIO_PC31,
	GPIO_PD0,
	GPIO_PD1,
	GPIO_PD2,
	GPIO_PD3,
	GPIO_PD4,
	GPIO_PD5,
	GPIO_PD6,
	GPIO_PD7,
	GPIO_PD8,
	GPIO_PD9,
	GPIO_PD10,
	GPIO_PD11,
	GPIO_PD12,
	GPIO_PD13,
	GPIO_PD14,
	GPIO_PD15,
	GPIO_PD16,
	GPIO_PD17,
	GPIO_PD18,
	GPIO_PD19,
	GPIO_PD20,
	GPIO_PD21,
	GPIO_PD22,
	GPIO_PD23,
	GPIO_PD24,
	GPIO_PD25,
	GPIO_PD26,
	GPIO_PD27,
	GPIO_PD28,
	GPIO_PD29,
	GPIO_PD30,
	GPIO_PD31,
	GPIO_PE0,
	GPIO_PE1,
	GPIO_PE2,
	GPIO_PE3,
	GPIO_PE4,
	GPIO_PE5,
	GPIO_PE6,
	GPIO_PE7,
	GPIO_PE8,
	GPIO_PE9,
	GPIO_PE10,
	GPIO_PE11,
	GPIO_PE12,
	GPIO_PE13,
	GPIO_PE14,
	GPIO_PE15,
	GPIO_PE16,
	GPIO_PE17,
	GPIO_PE18,
	GPIO_PE19,
	GPIO_PE20,
	GPIO_PE21,
	GPIO_PE22,
	GPIO_PE23,
	GPIO_PE24,
	GPIO_PE25,
	GPIO_PE26,
	GPIO_PE27,
	GPIO_PE28,
	GPIO_PE29,
	GPIO_PE30,
	GPIO_PE31,
    GPIO_P_MAX,
    RTT
}GPIO_PinTy;


typedef enum
{
    GPIO_DATA_LOW,
    GPIO_DATA_HIGH
} GPIO_DataTy;


typedef enum
{
    GPIO_MODE_IO,
    GPIO_MODE_ALT_FUNCTION_A,
    GPIO_MODE_ALT_FUNCTION_B,
    GPIO_MODE_ALT_FUNCTION_C,
    GPIO_MODE_ALT_FUNCTION_D,
    GPIO_MODE_AF_OD,
} GPIO_ModeTy;


typedef enum
{
    GPIO_TRIG_BOTH_EDGES,
    GPIO_TRIG_RISING_EDGE,
    GPIO_TRIG_FALLING_EDGE,
    GPIO_TRIG_LOW_LEVEL,
    GPIO_TRIG_HIGH_LEVEL, 
} GPIO_TrigTy;

typedef enum
{
    GPIO_PULL_NONE,
    GPIO_PULL_DOWN,
    GPIO_PULL_UP,
}GPIO_PullTy;

typedef struct
{
    int data;
    int dir;
    int sel_a;
    int sel_b;
    int per;
}PortInfoTy;

typedef struct
{
    PortInfoTy port[5];
}GpiosConfDef;   

/**
 * @fn void gpio_set_all(const GpiosConfDef* gpio)
 * @brief setting all gpios with the configurations
 *  
 * @param [in] gpio
 *  
 */
void gpio_set_all(const GpiosConfDef* gpio);
/**
 * @fn void gpio_set_cfg(unsigned gpio,GPIO_ModeTy mode)
 * @brief selecting the gpio mode
 *  
 * @param [in] gpio
 * @param [in] mode
 *  
 */
void gpio_set_cfg(unsigned gpio,GPIO_ModeTy mode);

/**
 * @fn gpio_set_filter(unsigned gpio, int enable)
 * @brief  set input filter slow clock enable/disable register
 *  
 * @param [in] gpio
 * @param [in] enable
 *  
 */
void gpio_set_filter(unsigned gpio, int enable);

/**
 * @fn void gpio_set_value(unsigned gpio, int value)
 * @brief  setting the output pin to high or low
 *  
 * @param [in] gpio
 * @param [in] value
 *  
 */
void gpio_set_value(unsigned gpio, int value);


/**
 * @fn int gpio_get_value(unsigned gpio)
 * @brief  getting the input pin to know it's high or low
 *  
 * @param [in] gpio
 * @return int 
 *  
 */
int gpio_get_value(unsigned gpio);

/**
 * @fn void gpio_direction_output(unsigned gpio, int value)
 * @brief  setting the pin to output with  value high or low 
 *  
 * @param [in] gpio
 * @param [in] value 
 *  
 */
void gpio_direction_output(unsigned gpio, int value);

/**
 * @fn void gpio_direction_input(unsigned gpio)
 * @brief  setting the gpio to input mode
 *  
 * @param [in] gpio
 *  
 */
void gpio_direction_input(unsigned gpio);


/**
 * @fn void gpio_setpull(unsigned gpio,GPIO_PullTy pull)
 * @brief  setting the gpio pull mode(down,up,none)
 *  
 * @param [in] gpio
 * @param [in] pull
 *  
 */
void gpio_setpull(unsigned gpio,GPIO_PullTy pull);
    
/**
 * @fn void gpio_request_irq(unsigned gpio,GPIO_TrigTy type,void(*gpio_irq_cb)(void))
 * @brief  request the gpio interrupt
 *  
 * @param [in] gpio
 * @param [in] type
 * @param [in] gpio_irq_cb
 *  
 */
void gpio_request_irq(unsigned gpio,GPIO_TrigTy type,void(*gpio_irq_cb)(void));

/**
 * @fn void gpio_free_irq(unsigned gpio)
 * @brief  free the gpio interrupt
 *  
 * @param [in] gpio
 *  
 */
void gpio_free_irq(unsigned gpio);

/**
 * @fn void gpio_request_irq(unsigned gpio,GPIO_TrigTy type,void(*gpio_irq_cb)(void))
 * @brief  free all  gpios interrupt
 *  
 */
void gpio_free_all_irqs(void);

#endif /* __SV_GPIO_H */
