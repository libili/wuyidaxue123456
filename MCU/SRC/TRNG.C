/*****************************************************************************/
/**
*  @file      TRNG.C
*  @brief     <b> TRNG C File </b>
*  @details   File functionality description:
*  This file is to implement the TRNG driver.
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
#include "../INC/TRNG.H"
typedef struct{
	volatile 		int TRNG_CR;
                    int Reversed[3];        
	volatile 		int TRNG_IER;
	volatile 		int TRNG_IDR;
	volatile const 	int	TRNG_IMR;
	volatile const 	int	TRNG_ISR;
                    int Reversed2[(0x50-0x20)/4]; 
	volatile const 	int	TRNG_ODATA;
}TrngTypedef;
#define KEY	(0x524E47<<8)
#define CR_ENABLE	0x01
#define DATRDY     0x01
typedef struct
{
    unsigned int base;
    unsigned int irq;
}TrngDescTypedef;

TrngDescTypedef trngDesc[1]={
	{.base = 0x40070000,.irq = 57}
};



/**
 * @fn void trng_init(void)
 * @brief true random hardware init
 *  
 * @return none
 *  
 */
void trng_init(void)
{
	TrngTypedef* trng = (TrngTypedef*)trngDesc[0].base;
	mcu_peri_clk_enable(trngDesc[0].irq);
	trng->TRNG_ISR;
	trng->TRNG_CR = KEY|CR_ENABLE;
}



/**
 * @fn unsigned int trng_getValue(void)
 * @brief getting true random 32bits value
 *  
 * @return unsigned int
 *  
 */
unsigned int trng_getValue(void)
{
	TrngTypedef* trng = (TrngTypedef*)trngDesc[0].base;
	int sr; 
	do{
		sr = trng->TRNG_ISR;
	}while((sr&DATRDY)!= DATRDY);
	return trng->TRNG_ODATA;
}

/**
 * @fn void trng_deint(void)
 * @brief deint the true random hardware
 *  
 * @return none
 *  
 */
void trng_deint(void)
{
	TrngTypedef* trng = (TrngTypedef*)trngDesc[0].base;
	trng->TRNG_ISR;
	trng->TRNG_CR = KEY;
	mcu_peri_clk_disable(trngDesc[0].irq);
}

