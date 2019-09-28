/*****************************************************************************/
/**
*  @file      MAIN.C
*  @brief     <b> MAIN C File </b>
*  @details   File functionality description:
*  This file provides main function.
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
#include <cmsis_os.h>  
#include "../INC/MCU.H"
/*@brief init thread function*/
extern void init_thread(void const *argument);  
/*@brief define init_thread id*/
osThreadId tid_init_thread;    
/*@brief define init thread object */
osThreadDef(init_thread, osPriorityRealtime, 1, 400);

/**
 * @fn int main(void)
 * @brief  mainThread function
 *  
 * @return int
 *  
 */
int main(void)
{
/*init mcu variables*/    
    mcu_variable_init();
/* init OS Kernel first*/    
    osKernelInitialize(); 
/* creat init_thread */    
    tid_init_thread  = osThreadCreate (osThread(init_thread), NULL);
    if(!tid_init_thread) 
    {
        mcu_sw_reset();
    }
 /*start OS Kernel*/   
    osKernelStart (); 
     return 0;
}

