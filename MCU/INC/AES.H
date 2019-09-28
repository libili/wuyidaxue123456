/*****************************************************************************/
/**
*  @file      AES.H
*  @brief     <b> AES H File </b>
*  @details   File functionality description:
* This file provides the AES driver interface.
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
#ifndef _SV_AES
#define _SV_AES




/**
 * @brief AES Key Mode 
 * 
 */
typedef enum
{
    AES_KeyMode_128,
    AES_KeyMode_192,
    AES_KeyMode_256,
}AES_KeyModeType;




/**
 * @fn void aes_ecb_encrypt(AES_KeyModeType type,const unsigned char *pKey,const unsigned char* pIdata,unsigned char* pOdata)
 * @brief to encrypt AES text with AES key
 * 
 * @param type key mode can be AES_KeyMode_128/AES_KeyMode_192/AES_KeyMode_256
 * @param pKey the point to AES Keys which can be 16/24/32bytes
 * @param pIdata the point to data for encryption
 * @param pOdata the point to data output
 */
void aes_ecb_encrypt(AES_KeyModeType type,const unsigned char *pKey,const unsigned char* pIdata,unsigned char* pOdata);


/**
 * @fn void aes_ecb_decrypt(AES_KeyModeType type,const unsigned char *pKey,const unsigned char* pIdata,unsigned char* pOdata)
 * @brief to decrypt AES ciphertext with AES key
 * 
 * @param type key mode can be AES_KeyMode_128/AES_KeyMode_192/AES_KeyMode_256
 * @param pKey the point to AES Keys which can be 16/24/32bytes
 * @param pIdata the point to data for decryption
 * @param pOdata the point to data output
 */
void aes_ecb_decrypt(AES_KeyModeType type,const unsigned char *pKey,const unsigned char* pIdata,unsigned char* pOdata);


#endif /* _SV_AES */
