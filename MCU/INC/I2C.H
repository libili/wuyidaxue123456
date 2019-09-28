/*****************************************************************************/
/**
*  @file      I2C.H
*  @brief     <b> I2C H File </b>
*  @details   File functionality description:
*  This file provides the i2c driver interface.
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
#ifndef SV_I2C_H
#define SV_I2C_H

typedef enum
{
	I2C_SLAVE_MODE,
	I2C_MASTER_MODE,
}I2C_OPERATER_MODE_TY;

typedef enum
{
	I2C_SPEED_STD,
	I2C_SPEED_FAST,
	I2C_SPEED_PLUSFAST,
}I2C_SPEED_MODE_TY;



typedef enum
{
	I2C_CH_0,
	I2C_CH_1,
	I2C_CH_2,
    I2C_CH_MAX,
}I2C_CH_TY;


typedef struct
{
    unsigned int speed_maxHZ;
	unsigned int operate_mode:2;/*master=1 or slave = 0*/
	unsigned int slave_addr:7;/*only support 7-bit mode addr*/
}I2C_ConfigTy;




typedef struct  {
    unsigned char addr;     // Slave Address
#define I2C_M_WE	0
#define I2C_M_RD 	1
    unsigned char flags;    //
    unsigned int 	len;      //transfer length
    unsigned char *buf;      //
}I2C_MsgTy;




/** 
 * @fn  void i2c_init(I2C_CH_TY ch,I2C_ConfigTy* config)
 * @brief init and configurate i2c driver
 * @param[in] ch the i2c channel
 * @param[in] config the i2c channel config parameter
 * @return none
*/
void i2c_init(I2C_CH_TY ch,I2C_ConfigTy* config);

/** 
 * @fn  int i2c_transfer(I2C_CH_TY ch, I2C_MsgTy *msgs,int num)
 * @brief to transfer i2c messages
 * @param[in] ch the i2c channel
 * @param[in] megs the i2c messages
 * @param[in] num the number of messages
 * @return int 
*/
int i2c_transfer(I2C_CH_TY ch, I2C_MsgTy *msgs, int num);


#endif /*SV_I2C_H */

