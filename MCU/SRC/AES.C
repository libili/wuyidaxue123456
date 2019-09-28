/*****************************************************************************/
/**
*  @file      AES.C
*  @brief     <b> AES C File </b>
*  @details   File functionality description:
*  This file is to implement the AES driver.
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
#include "../INC/AES.H"
#include "../INC/SAMV70.H"

/*register description */
typedef struct {
  volatile          unsigned int AES_CR;        /**< \brief (Aes Offset: 0x00) Control Register */
  volatile          unsigned int AES_MR;        /**< \brief (Aes Offset: 0x04) Mode Register */
  volatile  const   unsigned int Reserved1[2];
  volatile          unsigned int AES_IER;       /**< \brief (Aes Offset: 0x10) Interrupt Enable Register */
  volatile          unsigned int AES_IDR;       /**< \brief (Aes Offset: 0x14) Interrupt Disable Register */
  volatile  const   unsigned int AES_IMR;       /**< \brief (Aes Offset: 0x18) Interrupt Mask Register */
  volatile  const   unsigned int AES_ISR;       /**< \brief (Aes Offset: 0x1C) Interrupt Status Register */
  volatile          unsigned int AES_KEYWR[8];  /**< \brief (Aes Offset: 0x20) Key Word Register */
  volatile          unsigned int AES_IDATAR[4]; /**< \brief (Aes Offset: 0x40) Input Data Register */
  volatile  const   unsigned int AES_ODATAR[4]; /**< \brief (Aes Offset: 0x50) Output Data Register */
  volatile          unsigned int AES_IVR[4];    /**< \brief (Aes Offset: 0x60) Initialization Vector Register */
  volatile          unsigned int AES_AADLENR;   /**< \brief (Aes Offset: 0x70) Additional Authenticated Data Length Register */
  volatile          unsigned int AES_CLENR;     /**< \brief (Aes Offset: 0x74) Plaintext/Ciphertext Length Register */
  volatile          unsigned int AES_GHASHR[4]; /**< \brief (Aes Offset: 0x78) GCM Intermediate Hash Word Register */
  volatile  const   unsigned int AES_TAGR[4];   /**< \brief (Aes Offset: 0x88) GCM Authentication Tag Word Register */
  volatile  const   unsigned int AES_CTRR;      /**< \brief (Aes Offset: 0x98) GCM Encryption Counter Value Register */
  volatile          unsigned int AES_GCMHR[4];  /**< \brief (Aes Offset: 0x9C) GCM H Word Register */
  volatile  const   unsigned int Reserved2[20];
  volatile  const   unsigned int AES_VERSION;   /**< \brief (Aes Offset: 0xFC) Version Register */
} AesTypedef;


typedef struct
{
	unsigned int base;
	unsigned int irq;
}AesDescTypedef; 


AesDescTypedef aesDesc={
    .base = 0x4006C000UL,
    .irq = 56,
};
#define AES ((AesTypedef*)(aesDesc.base))
#define AES_CR_START (0x1u << 0) /**< \brief (AES_CR) Start Processing */
#define AES_CR_SWRST (0x1u << 8) /**< \brief (AES_CR) Software Reset */
#define AES_CR_LOADSEED (0x1u << 16) /**< \brief (AES_CR) Random Number Generator Seed Loading */
/* -------- AES_MR : (AES Offset: 0x04) Mode Register -------- */
#define AES_MR_KEYSIZE_AES128 (0x0u << 10) /**< \brief (AES_MR) AES Key Size is 128 bits */
#define AES_MR_KEYSIZE_AES192 (0x1u << 10) /**< \brief (AES_MR) AES Key Size is 192 bits */
#define AES_MR_KEYSIZE_AES256 (0x2u << 10) /**< \brief (AES_MR) AES Key Size is 256 bits */
#define AES_MR_CIPHER (0x1u << 0) /**< \brief (AES_MR) Processing Mode */
#define AES_MR_GTAGEN (0x1u << 1) /**< \brief (AES_MR) GCM Automatic Tag Generation Enable */
#define AES_MR_DUALBUFF (0x1u << 3) /**< \brief (AES_MR) Dual Input Buffer */
#define AES_MR_DUALBUFF_INACTIVE (0x0u << 3) /**< \brief (AES_MR) AES_IDATARx cannot be written during processing of previous block. */
#define AES_MR_DUALBUFF_ACTIVE (0x1u << 3) /**< \brief (AES_MR) AES_IDATARx can be written during processing of previous block when SMOD = 2. It speeds up the overall runtime of large files. */
#define AES_MR_PROCDLY_Pos 4
#define AES_MR_PROCDLY_Msk (0xfu << AES_MR_PROCDLY_Pos) /**< \brief (AES_MR) Processing Delay */
#define AES_MR_PROCDLY(value) ((AES_MR_PROCDLY_Msk & ((value) << AES_MR_PROCDLY_Pos)))
#define AES_MR_SMOD_Pos 8
#define AES_MR_SMOD_Msk (0x3u << AES_MR_SMOD_Pos) /**< \brief (AES_MR) Start Mode */
#define AES_MR_SMOD(value) ((AES_MR_SMOD_Msk & ((value) << AES_MR_SMOD_Pos)))
#define AES_MR_SMOD_MANUAL_START (0x0u << 8) /**< \brief (AES_MR) Manual Mode */
#define AES_MR_SMOD_AUTO_START (0x1u << 8) /**< \brief (AES_MR) Auto Mode */
#define AES_MR_SMOD_IDATAR0_START (0x2u << 8) /**< \brief (AES_MR) AES_IDATAR0 access only Auto Mode (DMA) */
#define AES_MR_KEYSIZE_Pos 10
#define AES_MR_KEYSIZE_Msk (0x3u << AES_MR_KEYSIZE_Pos) /**< \brief (AES_MR) Key Size */
#define AES_MR_KEYSIZE(value) ((AES_MR_KEYSIZE_Msk & ((value) << AES_MR_KEYSIZE_Pos)))
#define AES_MR_KEYSIZE_AES128 (0x0u << 10) /**< \brief (AES_MR) AES Key Size is 128 bits */
#define AES_MR_KEYSIZE_AES192 (0x1u << 10) /**< \brief (AES_MR) AES Key Size is 192 bits */
#define AES_MR_KEYSIZE_AES256 (0x2u << 10) /**< \brief (AES_MR) AES Key Size is 256 bits */
#define AES_MR_OPMOD_Pos 12
#define AES_MR_OPMOD_Msk (0x7u << AES_MR_OPMOD_Pos) /**< \brief (AES_MR) Operation Mode */
#define AES_MR_OPMOD(value) ((AES_MR_OPMOD_Msk & ((value) << AES_MR_OPMOD_Pos)))
#define AES_MR_OPMOD_ECB (0x0u << 12) /**< \brief (AES_MR) ECB: Electronic Code Book mode */
#define AES_MR_OPMOD_CBC (0x1u << 12) /**< \brief (AES_MR) CBC: Cipher Block Chaining mode */
#define AES_MR_OPMOD_OFB (0x2u << 12) /**< \brief (AES_MR) OFB: Output Feedback mode */
#define AES_MR_OPMOD_CFB (0x3u << 12) /**< \brief (AES_MR) CFB: Cipher Feedback mode */
#define AES_MR_OPMOD_CTR (0x4u << 12) /**< \brief (AES_MR) CTR: Counter mode (16-bit internal counter) */
#define AES_MR_OPMOD_GCM (0x5u << 12) /**< \brief (AES_MR) GCM: Galois/Counter mode */
#define AES_MR_LOD (0x1u << 15) /**< \brief (AES_MR) Last Output Data Mode */
#define AES_MR_CFBS_Pos 16
#define AES_MR_CFBS_Msk (0x7u << AES_MR_CFBS_Pos) /**< \brief (AES_MR) Cipher Feedback Data Size */
#define AES_MR_CFBS(value) ((AES_MR_CFBS_Msk & ((value) << AES_MR_CFBS_Pos)))
#define AES_MR_CFBS_SIZE_128BIT (0x0u << 16) /**< \brief (AES_MR) 128-bit */
#define AES_MR_CFBS_SIZE_64BIT (0x1u << 16) /**< \brief (AES_MR) 64-bit */
#define AES_MR_CFBS_SIZE_32BIT (0x2u << 16) /**< \brief (AES_MR) 32-bit */
#define AES_MR_CFBS_SIZE_16BIT (0x3u << 16) /**< \brief (AES_MR) 16-bit */
#define AES_MR_CFBS_SIZE_8BIT (0x4u << 16) /**< \brief (AES_MR) 8-bit */
#define AES_MR_CKEY_Pos 20
#define AES_MR_CKEY_Msk (0xfu << AES_MR_CKEY_Pos) /**< \brief (AES_MR) Countermeasure Key */
#define AES_MR_CKEY(value) ((AES_MR_CKEY_Msk & ((value) << AES_MR_CKEY_Pos)))
#define AES_MR_CKEY_PASSWD (0xEu << 20) /**< \brief (AES_MR) This field must be written with 0xE to allow CMTYPx bit configuration changes. Any other values will abort the write operation in CMTYPx bits.Always reads as 0. */
#define AES_MR_CMTYP1 (0x1u << 24) /**< \brief (AES_MR) Countermeasure Type 1 */
#define AES_MR_CMTYP1_NOPROT_EXTKEY (0x0u << 24) /**< \brief (AES_MR) Countermeasure type 1 is disabled. */
#define AES_MR_CMTYP1_PROT_EXTKEY (0x1u << 24) /**< \brief (AES_MR) Countermeasure type 1 is enabled. */
#define AES_MR_CMTYP2 (0x1u << 25) /**< \brief (AES_MR) Countermeasure Type 2 */
#define AES_MR_CMTYP2_NO_PAUSE (0x0u << 25) /**< \brief (AES_MR) Countermeasure type 2 is disabled. */
#define AES_MR_CMTYP2_PAUSE (0x1u << 25) /**< \brief (AES_MR) Countermeasure type 2 is enabled. */
#define AES_MR_CMTYP3 (0x1u << 26) /**< \brief (AES_MR) Countermeasure Type 3 */
#define AES_MR_CMTYP3_NO_DUMMY (0x0u << 26) /**< \brief (AES_MR) Countermeasure type 3 is disabled. */
#define AES_MR_CMTYP3_DUMMY (0x1u << 26) /**< \brief (AES_MR) Countermeasure type 3 is enabled. */
#define AES_MR_CMTYP4 (0x1u << 27) /**< \brief (AES_MR) Countermeasure Type 4 */
#define AES_MR_CMTYP4_NO_RESTART (0x0u << 27) /**< \brief (AES_MR) Countermeasure type 4 is disabled. */
#define AES_MR_CMTYP4_RESTART (0x1u << 27) /**< \brief (AES_MR) Countermeasure type 4 is enabled. */
#define AES_MR_CMTYP5 (0x1u << 28) /**< \brief (AES_MR) Countermeasure Type 5 */
#define AES_MR_CMTYP5_NO_ADDACCESS (0x0u << 28) /**< \brief (AES_MR) Countermeasure type 5 is disabled. */
#define AES_MR_CMTYP5_ADDACCESS (0x1u << 28) /**< \brief (AES_MR) Countermeasure type 5 is enabled. */
#define AES_MR_CMTYP6 (0x1u << 29) /**< \brief (AES_MR) Countermeasure Type 6 */
#define AES_MR_CMTYP6_NO_IDLECURRENT (0x0u << 29) /**< \brief (AES_MR) Countermeasure type 6 is disabled. */
#define AES_MR_CMTYP6_IDLECURRENT (0x1u << 29) /**< \brief (AES_MR) Countermeasure type 6 is enabled. */
#define AES_ISR_DATRDY (0x1u << 0) /**< \brief (AES_ISR) Data Ready (cleared by setting bit START or bit SWRST in AES_CR or by reading AES_ODATARx) */
/**
 * @fn void aes_start(void)
 * @brief Starts Manual encryption/decryption process.
 */
void aes_start(void)
{
	AES->AES_CR = AES_CR_START;
}

/**
 * @fn void aes_softReset(void)
 * @brief Resets the AES. A software triggered hardware reset of the AES
 *  interface is performed.
 */
void aes_softReset(void)
{
	AES->AES_CR = AES_CR_SWRST;
}


/**
 * @fn void aes_configure(unsigned int mode)
 * @brief Configures an AES peripheral with the specified parameters.
 * @param mode  Desired value for the AES mode register (see the datasheet).
 */
void aes_configure(unsigned int mode)
{
	AES->AES_MR = mode; 
}

/**
 * @fn void aes_enableIt(unsigned int sources)
 * @brief Enables the selected interrupts sources on a AES peripheral.
 * @param sources  Bitwise OR of selected interrupt sources.
 */
void aes_enableIt(unsigned int sources)
{
	AES->AES_IER = sources;
}

/**
 * @fn void aes_disableIt(unsigned int sources)
 * @brief Disables the selected interrupts sources on a AES peripheral.
 * @param sources  Bitwise OR of selected interrupt sources.
 */
void aes_disableIt(unsigned int sources)
{
	AES->AES_IDR = sources;
}

/**
 * @fn unsigned int aes_getStatus(void)
 * @brief Get the current status register of the given AES peripheral.
 * @return  AES status register.
 */
unsigned int aes_getStatus(void)
{
	return AES->AES_ISR;
}

/**
 * @fn void aes_writeKey(const unsigned int *pKey, unsigned int keyLength)
 * @brief Set the 128-bit/192-bit/256-bit cryptographic key used for 
 * encryption/decryption.
 * @param pKey Pointer to a 16/24/32 bytes cipher key.
 * @param keyLength length of key
 */
void aes_writeKey(const unsigned int *pKey, unsigned int keyLength)
{
	AES->AES_KEYWR[0] = pKey[0];
	AES->AES_KEYWR[1] = pKey[1];
	AES->AES_KEYWR[2] = pKey[2];
	AES->AES_KEYWR[3] = pKey[3];

	if( keyLength >= 24 ) {
		AES->AES_KEYWR[4] = pKey[4];
		AES->AES_KEYWR[5] = pKey[5];
	}
	if( keyLength == 32 ) {
		AES->AES_KEYWR[6] = pKey[6];
		AES->AES_KEYWR[7] = pKey[7];
	}
}

/**
 * @fn void aes_setInput(const unsigned int *data)
 * @brief Set the for 32-bit input Data allow to set the 128-bit data block
 * used for encryption/decryption.
 * @param data Pointer to the 16-bytes data to cipher/decipher.
 */
void aes_setInput(const unsigned int *data)
{
	unsigned char i;
	for (i = 0; i< 4; i++)
		AES->AES_IDATAR[i] = data[i];
}

/**
 * @fn void aes_getOutput(unsigned int *data)
 * @brief Get the four 32-bit data contain the 128-bit data block which 
 * has been encrypted/decrypted.
 * @param data pointer to the word that has been encrypted/decrypted..
 */
void aes_getOutput(unsigned int *data)
{
	unsigned char i;
	for (i = 0; i< 4; i++) 
		data[i] = AES->AES_ODATAR[i];
}

/**
 * @fn void AES_Handler(void)
 * @brief AES interrupt handler 
 * @param none
 */
void AES_Handler(void)
{
      
}


/**
 * @fn void aes_ecb_start_compute(char cipher,const unsigned char type,const unsigned int *pKey,const unsigned int* pIdata,unsigned int* pOdata)
 * @brief to encrypt/decrypt AES text with AES key
 * @param cipher encrypt/decrypt
 * @param type key mode can be AES_KeyMode_128/AES_KeyMode_192/AES_KeyMode_256
 * @param pKey the point to AES Keys which can be 16/24/32bytes
 * @param pIdata the point to data for encryption
 * @param pOdata the point to data output
 */
void aes_ecb_start_compute(char cipher,const unsigned char type,const unsigned int *pKey,const unsigned int* pIdata,unsigned int* pOdata)
{
    /*First to enable AES module clock*/
    int keyLength;
    keyLength = 128+(type>>(AES_MR_KEYSIZE_Pos-6));        
    mcu_peri_clk_enable(aesDesc.irq);
    aes_softReset();
    if(cipher)
    {
        aes_configure(type|AES_MR_CKEY_PASSWD|AES_MR_OPMOD_ECB|AES_MR_CIPHER);
    }    
    else
    { 
        aes_configure(type|AES_MR_CKEY_PASSWD|AES_MR_OPMOD_ECB);  
    }
    aes_writeKey(pKey,keyLength);
    aes_setInput(pIdata);
    aes_start();
    while((AES_ISR_DATRDY&aes_getStatus())!=AES_ISR_DATRDY);
    aes_getOutput(pOdata);
    mcu_peri_clk_disable(aesDesc.irq);
}

/**
 * @fn aes_ecb_encrypt(AES_KeyModeType type,const unsigned char *pKey,const unsigned char* pIdata,unsigned char* pOdata)
 * @brief to encrypt AES text with AES key
 * 
 * @param type key mode can be AES_KeyMode_128/AES_KeyMode_192/AES_KeyMode_256
 * @param pKey the point to AES Keys which can be 16/24/32bytes
 * @param pIdata the point to data for encryption
 * @param pOdata the point to data output
 */
void aes_ecb_encrypt(AES_KeyModeType type,const unsigned char *pKey,const unsigned char* pIdata,unsigned char* pOdata)
{
    if(type <= 2)
    {
        aes_ecb_start_compute(1,type<<AES_MR_KEYSIZE_Pos,(const unsigned int *)pKey,(const unsigned int*) pIdata,(unsigned int*) pOdata);
    }      
}



/**
 * @fn void aes_ecb_decrypt(AES_KeyModeType type,const unsigned char *pKey,const unsigned char* pIdata,unsigned char* pOdata)
 * @brief to decrypt AES ciphertext with AES key
 * 
 * @param type key mode can be AES_KeyMode_128/AES_KeyMode_192/AES_KeyMode_256
 * @param pKey the point to AES Keys which can be 16/24/32bytes
 * @param pIdata the point to data for decryption
 * @param pOdata the point to data output
 */
void aes_ecb_decrypt(AES_KeyModeType type,const unsigned char *pKey,const unsigned char* pIdata,unsigned char* pOdata)
{
    if(type <= 2)
    {
        aes_ecb_start_compute(0,type<<AES_MR_KEYSIZE_Pos,(const unsigned int *)pKey,(const unsigned int*) pIdata,(unsigned int*) pOdata);
    } 
}

