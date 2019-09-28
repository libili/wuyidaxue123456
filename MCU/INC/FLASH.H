/*****************************************************************************/
/**
*  @file      FLASH.H
*  @brief     <b> FLASH H File </b>
*  @details   File functionality description:
*  This file provides the Flash driver interface.
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
#ifndef SV_FLASH_H
#define SV_FLASH_H

/*! \name Flash driver return codes */
//! @{
typedef enum flash_rc {
	FLASH_RC_OK = 0,        //!< Operation OK
	FLASH_RC_YES = 1,       //!< Yes
	FLASH_RC_NO = 0,        //!< No
	FLASH_RC_ERROR = 0x10,  //!< General error
	FLASH_RC_INVALID,       //!< Invalid argument input
	FLASH_RC_NOT_SUPPORT = (int)0xFFFFFFFF    //!< Operation is not supported
} flash_rc_t;


/**
 * @fn void flash_init(void)
 * @brief Enables the flash ready interrupt source on the EEFC peripheral.
 *
 * @param efc  Pointer to a Efc instance
 */
void flash_init(void);


/**
 * @fn void flash_erase_all(void)
 * @brief Erases the entire flash
 *  
 */
void flash_erase_all(void);

/**
 * @fn unsigned int flash_erase_sector(unsigned int address)
 * @brief erase flash sector
 *  
 * @param [in] address memory address
 * @return erase result status
 *  
 */
unsigned int flash_erase_sector(unsigned int address);

/**
 * @fn void flash_erase_pages(unsigned int address,unsigned int length)
 * @brief flash_erase_pages
 *  
 * @param [in] address memory address
 * @param [in] length  number of bytes
 * @return erase result status
 *  
 */
void flash_erase_pages(unsigned int address,unsigned int length);

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
unsigned int flash_write(unsigned int address,const void *pvBuffer, unsigned int length);


/**
 * @fn unsigned int  flash_lock( unsigned int startAddr, unsigned int endAddr)
 * @brief flash_lock
 *  
 * @param [in] startAddr memory start address
 * @param [in] endAddr memory end address
 * @return 0: all is ok,otherwise something is error
 *  
 */
unsigned int  flash_lock( unsigned int startAddr, unsigned int endAddr);

/**
 * @fn unsigned int flash_unlock(unsigned int startAddr, unsigned int endAddr)
 * @brief flash_unlock
 *  
 * @param [in] startAddr memory start address
 * @param [in] endAddr memory end address
 * @return 0: all is ok,otherwise something is error
 *  
 */
unsigned int flash_unlock(unsigned int startAddr, unsigned int endAddr);


/**
 * @fn int flash_islock( unsigned int startAddr, unsigned int endAddr )
 * @brief check the memory is lock or not
 *  
 * @param [in] startAddr memory start address
 * @param [in] endAddr memory end address
 * @return 1: lock,otherwise is unlock
 *  
 */
unsigned int flash_islock( unsigned int startAddr, unsigned int endAddr );
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
extern unsigned int flash_isGPNVMSet( unsigned char ucGPNVM );

/**
 * @fn unsigned int flash_setGPNVM( unsigned char ucGPNVM )
 * @brief Sets the selected GPNVM bit.
 *
 * @param gpnvm  GPNVM bit index.
 * @returns 0 if successful; otherwise returns an error code.
 */
unsigned int flash_setGPNVM( unsigned char ucGPNVM );

/**
 * @fn unsigned int flash_clearGPNVM( unsigned char ucGPNVM )
 * @brief Clears the selected GPNVM bit.
 *
 * @param gpnvm  GPNVM bit index.
 * @returns 0 if successful; otherwise returns an error code.
 */
unsigned int flash_clearGPNVM( unsigned char ucGPNVM );

/**
 * @fn unsigned int flash_erase_user_signature(void)
 * @brief Erase the flash user signature.
 *
 * @return 0 if successful; otherwise returns an error code.
 */
unsigned int flash_erase_user_signature(void);


/**
 * @fn unsigned int flash_write_user_signature(const void *p_buffer, unsigned int ul_size)
 * @brief Write the flash user signature.
 *
 * @param p_data Pointer to a data buffer to store info for the user signature.
 * @param ul_size Data buffer size in 32 bit words.
 *
 * @return 0 if successful; otherwise returns an error code.
 */
unsigned int flash_write_user_signature(const void *p_buffer, unsigned int ul_size);

/**
 * @fn unsigned int flash_read_user_signature(unsigned int *p_data, unsigned int ul_size)
 * @brief Read the flash user signature.
 *
 * @param p_data Pointer to a data buffer to store 512 bytes of user signature.
 * @param ul_size Data buffer size in 32 bit words.
 *
 * @return 0 if successful; otherwise returns an error code.
 */
unsigned int flash_read_user_signature(unsigned int *p_data, unsigned int ul_size);
#endif
