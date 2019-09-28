/*****************************************************************************/
/**
*  @file      FLASH.C
*  @brief     <b> FLASH C File </b>
*  @details   File functionality description:
*  This file is to implement the Flash driver.
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
#include "core_cm7.h"
#include "../INC/FLASH.H"
#include <string.h>
#include <stdlib.h>
typedef struct {
  volatile 			int EEFC_FMR;      /**< \brief (Efc Offset: 0x00) EEFC Flash Mode Register */
  volatile  		int EEFC_FCR;      /**< \brief (Efc Offset: 0x04) EEFC Flash Command Register */
  volatile 	const  	int EEFC_FSR;      /**< \brief (Efc Offset: 0x08) EEFC Flash Status Register */
  volatile 	const  	int EEFC_FRR;      /**< \brief (Efc Offset: 0x0C) EEFC Flash Result Register */
  volatile 	const  	int Reserved1[1];
  volatile 	const  	int EEFC_VERSION;  /**< \brief (Efc Offset: 0x14) EEFC Version Register */
  volatile 	const  	int Reserved2[51];
  volatile 			int EEFC_WPMR;     /**< \brief (Efc Offset: 0xE4) Write Protection Mode Register */
}EfcTypeDef;
#define EFC    ((EfcTypeDef    *)0x400E0C00U) /**< \brief (EFC   ) Base Address */
#define EEFC_FSR_FRDY (0x1u << 0) /**< \brief (EEFC_FSR) Flash Ready Status (cleared when Flash is busy) */
#define EEFC_FSR_FCMDE (0x1u << 1) /**< \brief (EEFC_FSR) Flash Command Error Status (cleared on read or by writing EEFC_FCR) */
#define EEFC_FSR_FLOCKE (0x1u << 2) /**< \brief (EEFC_FSR) Flash Lock Error Status (cleared on read) */
#define EEFC_FSR_FLERR (0x1u << 3) /**< \brief (EEFC_FSR) Flash Error Status (cleared when a programming operation starts) */
#define EEFC_FSR_UECCELSB (0x1u << 16) /**< \brief (EEFC_FSR) Unique ECC Error on LSB Part of the Memory Flash Data Bus (cleared on read) */
#define EEFC_FSR_MECCELSB (0x1u << 17) /**< \brief (EEFC_FSR) Multiple ECC Error on LSB Part of the Memory Flash Data Bus (cleared on read) */
#define EEFC_FSR_UECCEMSB (0x1u << 18) /**< \brief (EEFC_FSR) Unique ECC Error on MSB Part of the Memory Flash Data Bus (cleared on read) */
#define EEFC_FSR_MECCEMSB (0x1u << 19) /**< \brief (EEFC_FSR) Multiple ECC Error on MSB Part of the Memory Flash Data Bus (cleared on read) */


/* Define memory barrier for tool chains */
#define memory_barrier()        __dmb(15);


/* EFC command */
#define EFC_FCMD_GETD    0x00 /* Get Flash Descriptor */
#define EFC_FCMD_WP      0x01 /* Write page */
#define EFC_FCMD_WPL     0x02 /* Write page and lock */
#define EFC_FCMD_EWP     0x03 /* Erase page and write page */
#define EFC_FCMD_EWPL    0x04 /* Erase page and write page then lock */
#define EFC_FCMD_EA      0x05 /* Erase all */
#define EFC_FCMD_EPA     0x07 /* Erase pages */
#define EFC_FCMD_SLB     0x08 /* Set Lock Bit */
#define EFC_FCMD_CLB     0x09 /* Clear Lock Bit */
#define EFC_FCMD_GLB     0x0A /* Get Lock Bit */
#define EFC_FCMD_SFB     0x0B /* Set GPNVM Bit */
#define EFC_FCMD_CFB     0x0C /* Clear GPNVM Bit */
#define EFC_FCMD_GFB     0x0D /* Get GPNVM Bit */
#define EFC_FCMD_STUI    0x0E /* Start unique ID */
#define EFC_FCMD_SPUI    0x0F /* Stop unique ID */
#define EFC_FCMD_GCALB   0x10 /* Get CALIB Bit */
#define EFC_FCMD_ES      0x11 /* Erase Sector */
#define EFC_FCMD_WUS     0x12 /* Write User Signature */
#define EFC_FCMD_EUS     0x13 /* Erase User Signature */
#define EFC_FCMD_STUS    0x14 /* Start Read User Signature */
#define EFC_FCMD_SPUS    0x15 /* Stop Read User Signature */


/*----------------------------------------------------------------------------
 *        Macro 
 *----------------------------------------------------------------------------*/
#define EEFC_FCR_FKEY_PASSWD (0x5Au << 24)
#define GPNVM_NUM_MAX   9
#define IFLASH_ADDR   (0x00400000u) /**< Internal Flash base address */
#define IFLASH_PAGE_SIZE        (512u)
#define IFLASH_SIZE             (0x100000u)
#define FLASH_ADDR_CHECK(addr)  ((addr >=IFLASH_ADDR)&&(addr < IFLASH_ADDR+IFLASH_SIZE)) 
#define PAGE_TO_ADDR(page)		((page*IFLASH_PAGE_SIZE)+IFLASH_ADDR)	
#define ADDR_TO_PAGE(addr)		((addr - IFLASH_ADDR)/IFLASH_PAGE_SIZE)
#define ADDR_TO_OFFSET(addr)	((addr - IFLASH_ADDR)%IFLASH_PAGE_SIZE)	
#define MIN(a,b)				(a>b?b:a)
static unsigned int _pdwPageBuffer[IFLASH_PAGE_SIZE/sizeof(unsigned int)] ;
/**
 * @fn void EFC_EnableIt( void)
 * @brief Enables the flash ready interrupt source on the EEFC peripheral.
 *
 * @param efc  Pointer to a Efc instance
 */
void EFC_EnableIt( void)
{
	unsigned int dwFmr;

	dwFmr = EFC->EEFC_FMR|0x01;
	EFC->EEFC_FMR = dwFmr;
}

/**
 * @fn void EFC_DisableIt( void )
 * @brief Disables the flash ready interrupt source on the EEFC peripheral.
 *
 * @param efc  Pointer to a Efc instance
 */
void EFC_DisableIt( void )
{
	unsigned int dwFmr;

	dwFmr = EFC->EEFC_FMR & (~0x01);
	EFC->EEFC_FMR = dwFmr;
}


/**
 * @fn void EFC_SetWaitState(unsigned char ucCycles )
 * @brief Set read/write wait state on the EEFC peripheral.
 *
 * @param efc  Pointer to a Efc instance
 * @param cycles  the number of wait states in cycle.
 */
void EFC_SetWaitState(unsigned char ucCycles )
{
	unsigned int dwFmr ;
#define EEFC_FMR_FWS(v)  ((unsigned int)(0xF&v)<<8)
#define EEFC_FMR_FWS_Msk  0xF00
	dwFmr = EFC->EEFC_FMR ;
	dwFmr &= ~((unsigned int)EEFC_FMR_FWS_Msk) ;
	dwFmr |= EEFC_FMR_FWS(ucCycles);
	EFC->EEFC_FMR = dwFmr;
}


/**
 * @fn unsigned int EFC_GetResult(void)
 * @brief Returns the result of the last executed command.
 *
 * @param efc  Pointer to a Efc instance
 */
unsigned int EFC_GetResult(void)
{
	return EFC->EEFC_FRR ;
}


   
/**
 * @fn unsigned int EFC_PerformCommand(unsigned int dwCommand,unsigned int dwArgument)
 * @brief Performs the given command and wait until its completion (or an error).
 *
 * @param efc  Pointer to a Efc instance
 * @param command  Command to perform.
 * @param argument  Optional command argument.
 *
 * @return 0 if successful, otherwise returns an error code.
 */
unsigned int EFC_PerformCommand(unsigned int dwCommand,unsigned int dwArgument)
{
	volatile unsigned int dwStatus ;
    #define EEFC_FCR_FARG(x)  ((x&0xFFFF)<<8)
    #define EEFC_FCR_FCMD(x)  (x&0xFF) 
    #define EEFC_FSR_FRDY (0x1u << 0)    
	EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG(dwArgument) 
				| EEFC_FCR_FCMD(dwCommand) ;
	do{
		dwStatus = EFC->EEFC_FSR ;
	}
	while ( (dwStatus & EEFC_FSR_FRDY) != EEFC_FSR_FRDY ) ;
	return ( dwStatus & (EEFC_FSR_FLOCKE | EEFC_FSR_FCMDE | EEFC_FSR_FLERR) ) ;
	
}


unsigned int efc_desc[7];
void EFC_GetDesc(void)
{
   EFC_PerformCommand(EFC_FCMD_GETD,0);
   for(int i = 0; i <7; i++)
   {
        efc_desc[i] = EFC_GetResult();
   } 
} 
/**
 * @fn void flash_init(void)
 * @brief Perform Flash init 
 *  
 */
void flash_init(void)
{
	EFC_DisableIt();
	EFC_SetWaitState(6);
}



/**
 * @fn void flash_erase_all(void)
 * @brief Erases the entire flash
 *  
 */
void flash_erase_all(void)
{
	EFC_PerformCommand(EFC_FCMD_EA, 0);
}


/**
 * @fn unsigned int flash_erase_sector(unsigned int address)
 * @brief erase flash sector
 *  
 * @param [in] address memory address
 * @return erase result status
 *  
 */
unsigned int flash_erase_sector(unsigned int address)
{
	unsigned int dwError = 0xFF;
	if(FLASH_ADDR_CHECK(address))
	{
		dwError = EFC_PerformCommand(EFC_FCMD_ES, ADDR_TO_PAGE(address));
	}	
	return dwError;
}


/**
 * @fn void flash_erase_pages(unsigned int address,unsigned int length)
 * @brief flash_erase_pages
 *  
 * @param [in] address memory address
 * @param [in] length  number of bytes
 * @return erase result status
 *  
 */
void flash_erase_pages(unsigned int address,unsigned int length)
{
	//todo later
}


/**
 * @fn unsigned int flash_write(unsigned int address,const void *pvBuffer, unsigned int length)
 * @brief flash_write
 *  
 * @param [in] address memory address
 * @param [in] pvBuffer point to target datas
 * @param [in] length  number of bytes
 * @return 0: all is ok,otherwise something is error
 *  
 */
unsigned int flash_write(unsigned int address,const void *pvBuffer, unsigned int length)
{	
	unsigned short page ;
	unsigned short offset ;
	unsigned int writeSize ;
	unsigned int pageAddress ;
	unsigned short padding ;
	unsigned int dwError = FLASH_RC_OK;
	unsigned int dwIdx;
	unsigned int *pAlignedDestination ;
	unsigned char  *pucPageBuffer = (unsigned char *)_pdwPageBuffer;

	/* Translate write address */
	page = ADDR_TO_PAGE(address);
	offset = ADDR_TO_OFFSET(address);
	/* Write all pages */
	while ( length > 0 ) {
		/* Copy data in temporary buffer to avoid alignment problems */
		writeSize = MIN((unsigned int)IFLASH_PAGE_SIZE - offset, length );
		pageAddress = PAGE_TO_ADDR(page);
		padding = IFLASH_PAGE_SIZE - offset - writeSize ;

		/* Pre-buffer data */
		memcpy( pucPageBuffer, (void *) pageAddress, offset);

		/* Buffer data */
		memcpy( pucPageBuffer + offset, pvBuffer, writeSize);

		/* Post-buffer data */
		memcpy( pucPageBuffer + offset + writeSize, 
			(void *) (pageAddress + offset + writeSize), padding);

		/* Write page
		 * Writing 8-bit and 16-bit data is not allowed and may 
			lead to unpredictable data corruption
		 */
		pAlignedDestination = (unsigned int*)pageAddress ;
		for (dwIdx = 0; dwIdx < (IFLASH_PAGE_SIZE / sizeof(unsigned int)); ++ dwIdx) {
			*pAlignedDestination++ = _pdwPageBuffer[dwIdx];
			memory_barrier()
		}

		/* Note: It is not possible to use Erase and write Command (EWP) on all Flash
		(this command is available on the First 2 Small Sector, 16K Bytes). 
		For the next block, Erase them first then use Write page command. */

		/* Send writing command */
		dwError = EFC_PerformCommand(EFC_FCMD_WP, page) ;
		if ( dwError ) {
			return dwError ;
		}

		/* Progression */
		pvBuffer = (void *)((unsigned int) pvBuffer + writeSize) ;
		length -= writeSize ;
		page++;
		offset = 0;
	}
    return dwError;
}

/**
 * @fn unsigned int  flash_lock( unsigned int startAddr, unsigned int endAddr)
 * @brief flash_lock
 *  
 * @param [in] startAddr memory start address
 * @param [in] endAddr memory end address
 * @return 0: all is ok,otherwise something is error
 *  
 */
unsigned int  flash_lock( unsigned int startAddr, unsigned int endAddr)
{
	unsigned short page_start = ADDR_TO_PAGE(startAddr);
    unsigned short page_end = ADDR_TO_PAGE(endAddr);
    unsigned int dwError = 0;
    unsigned short page = page_start;
    while(page <= page_end)
    {
        dwError = EFC_PerformCommand(EFC_FCMD_SLB, page++) ;
    }    
    return dwError; 					
}					

/**
 * @fn unsigned int flash_unlock(unsigned int startAddr, unsigned int endAddr)
 * @brief flash_unlock
 *  
 * @param [in] startAddr memory start address
 * @param [in] endAddr memory end address
 * @return 0: all is ok,otherwise something is error
 *  
 */
unsigned int flash_unlock(unsigned int startAddr, unsigned int endAddr)
{
	unsigned short page_start = ADDR_TO_PAGE(startAddr);
    unsigned short page_end = ADDR_TO_PAGE(endAddr);
    unsigned int dwError = 0;
    unsigned short page = page_start;
    while(page <= page_end)
    {
        dwError = EFC_PerformCommand(EFC_FCMD_CLB, page++) ;
    }    
    return dwError; 					
}


/**
 * @fn int flash_islock( unsigned int startAddr, unsigned int endAddr )
 * @brief check the memory is lock or not
 *  
 * @param [in] startAddr memory start address
 * @param [in] endAddr memory end address
 * @return 1: lock,otherwise is unlock
 *  
 */
unsigned int flash_islock( unsigned int startAddr, unsigned int endAddr )
{
    return 1;
}

/*************************************************************
GPNVM Bit       Function
0				Security bit
1				0:ROM(default) 1:Flash
5:2				Free
6				Reserved
8:7             TCM configuration 	00(0K 	D 	0k	I	)
									01(32k 	D	32k	I	)
									10(64k	D 	64k	I	)
									11(128k	D   128kI	)		
**************************************************************/
/**
 * @fn unsigned int flash_isGPNVMSet( unsigned char ucGPNVM )
 * @brief Check if the given GPNVM bit is set or not.
 *
 * @param gpnvm  GPNVM bit index.
 * @returns 1 if the given GPNVM bit is currently set; otherwise returns 0.
 */
unsigned int flash_isGPNVMSet( unsigned char ucGPNVM )
{
	unsigned int dwStatus ;
	if(ucGPNVM >= GPNVM_NUM_MAX)
		return 2;
	/* Get GPNVMs status */
	EFC_PerformCommand(EFC_FCMD_GFB, 0) ;
	dwStatus = EFC_GetResult();

	/* Check if GPNVM is set */
	if ( (dwStatus & (1 << ucGPNVM)) != 0 ) {
		return 1 ;
	} else {
		return 0 ;
	}
}

/**
 * @fn unsigned int flash_setGPNVM( unsigned char ucGPNVM )
 * @brief Sets the selected GPNVM bit.
 *
 * @param gpnvm  GPNVM bit index.
 * @returns 0 if successful; otherwise returns an error code.
 */
unsigned int flash_setGPNVM( unsigned char ucGPNVM )
{
	if(ucGPNVM >= GPNVM_NUM_MAX)
		return 2;
	if ( !flash_isGPNVMSet( ucGPNVM ) ) {
		return EFC_PerformCommand(EFC_FCMD_SFB, ucGPNVM);
	} else {
		return 0 ;
	}
}

/**
 * @fn unsigned int flash_clearGPNVM( unsigned char ucGPNVM )
 * @brief Clears the selected GPNVM bit.
 *
 * @param gpnvm  GPNVM bit index.
 * @returns 0 if successful; otherwise returns an error code.
 */
unsigned int flash_clearGPNVM( unsigned char ucGPNVM )
{
	if(ucGPNVM >= GPNVM_NUM_MAX)
		return 2;
	if ( flash_isGPNVMSet( ucGPNVM )==1 ) {
		return EFC_PerformCommand(EFC_FCMD_CFB, ucGPNVM);
	} else {
		return 0 ;
	}
}

/**
 * @fn unsigned int flash_erase_user_signature(void)
 * @brief Erase the flash user signature.
 *
 * @return 0 if successful; otherwise returns an error code.
 */
unsigned int flash_erase_user_signature(void)
{
	/* Perform the erase user signature command */
	return EFC_PerformCommand(EFC_FCMD_EUS, 0);
}


/**
 * @fn unsigned int flash_write_user_signature(const void *p_buffer, unsigned int ul_size)
 * @brief Write the flash user signature.
 *
 * @param p_data Pointer to a data buffer to store info for the user signature.
 * @param ul_size Data buffer size in 32 bit words.
 *
 * @return 0 if successful; otherwise returns an error code.
 */
unsigned int flash_write_user_signature(const void *p_buffer, unsigned int ul_size)
{
	unsigned int ul_idx;
	unsigned int *p_dest;
    
	/* The user signature should be no longer than 512 bytes */
	if (ul_size > (IFLASH_PAGE_SIZE / sizeof(unsigned int))) {
		return FLASH_RC_INVALID;
	}
    unsigned int *gs_ul_page_buffer = (unsigned int *)malloc(IFLASH_PAGE_SIZE);
    if(!gs_ul_page_buffer)
        return FLASH_RC_ERROR;
	/* Copy Buffer data */
	memcpy((unsigned char *) gs_ul_page_buffer, p_buffer, 
			ul_size * sizeof(unsigned int));

	/* Write page buffer.
	* Writing 8-bit and 16-bit data is not allowed and may lead to
	* unpredictable data corruption.
	*/
    /*backup mpu control status and disable mpu*/
    int ctrl = MPU->CTRL;
    MPU->CTRL = 0x00;
	p_dest = (unsigned int *)IFLASH_ADDR;
	for (ul_idx = 0; ul_idx < (IFLASH_PAGE_SIZE / sizeof(unsigned int)); 
			ul_idx++) {
		*p_dest++ = gs_ul_page_buffer[ul_idx];
	}
    free(gs_ul_page_buffer);
    /* restore mpu control*/
    MPU->CTRL = ctrl;
	/* Send the write signature command */
	if (FLASH_RC_OK != EFC_PerformCommand(EFC_FCMD_WUS, 0)) {
		return FLASH_RC_ERROR;
	}

	return FLASH_RC_OK;
}

/**
 * @fn unsigned int flash_read_user_signature(unsigned int *p_data, unsigned int ul_size)
 * @brief Read the flash user signature.
 *
 * @param p_data Pointer to a data buffer to store 512 bytes of user signature.
 * @param ul_size Data buffer size in 32 bit words.
 *
 * @return 0 if successful; otherwise returns an error code.
 */
#pragma arm section code ="ram_run_function"
unsigned int flash_read_user_signature(unsigned int *p_data, unsigned int ul_size)
{
    volatile unsigned int dwStatus ;
    unsigned int ul_cnt;
    unsigned int* p_ul_data = (unsigned int *)IFLASH_ADDR;
	if (ul_size > (IFLASH_PAGE_SIZE / sizeof(unsigned int))) {
		/* Only 512 byte to store user signature */
		ul_size = IFLASH_PAGE_SIZE / sizeof(unsigned int);
	}
    EFC->EEFC_FMR |= (0x1u << 16);
    EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG(0) 
				| EEFC_FCR_FCMD(EFC_FCMD_STUS) ;
	do{
		dwStatus = EFC->EEFC_FSR ;
	}
	while ( (dwStatus & EEFC_FSR_FRDY) == EEFC_FSR_FRDY ) ;
    
    for (ul_cnt = 0; ul_cnt < ul_size; ul_cnt++) {
		p_data[ul_cnt] = p_ul_data[ul_cnt];
	}
    EFC->EEFC_FCR = EEFC_FCR_FKEY_PASSWD | EEFC_FCR_FARG(0) 
				| EEFC_FCR_FCMD(EFC_FCMD_SPUS) ;
	do{
		dwStatus = EFC->EEFC_FSR ;
	}
	while ( (dwStatus & EEFC_FSR_FRDY) != EEFC_FSR_FRDY ) ;
    EFC->EEFC_FMR &= ~(0x1u << 16);
	return FLASH_RC_OK;
}
#pragma arm section
