/***************************************************************************//**
 * @file
 * @brief i2c driver for the Si1147
 * @version 3.20.5
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/



#include "i2cdrv.h"
#include "si1147_i2c.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/


/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/



/**************************************************************************//**
 * @brief
  *  Reads register from the Si1147 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[out] data
 *   The data read from the sensor.
 * @param[in] reg
 *   The register address to read from in the sensor.
 * @return
 *   Returns number of bytes read on success. Otherwise returns error codes
 *   based on the I2CDRV.
 *****************************************************************************/
int Si1147_Read_Register (I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t *data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[1];

  /* Unused parameter */
  (void) i2c;

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select register to start reading from */
  i2c_write_data[0] = reg;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 1;
  /* Select length of data to be read */
  seq.buf[1].data = data;
  seq.buf[1].len  = 1;

  ret = I2CDRV_Transfer(&seq);
  if (ret != i2cTransferDone)
  {
    *data = 0xff;
    return((int) ret);
  }

  return((int) 1);
}

/**************************************************************************//**
 * @brief
  *  Writes register in the Si1147 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[in] data
 *   The data to write to the sensor.
 * @param[in] reg
 *   The register address to write to in the sensor.
 * @return
 *   Returns zero on success. Otherwise returns error codes
 *   based on the I2CDRV.
 *****************************************************************************/
int Si1147_Write_Register (I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[2];
  uint8_t i2c_read_data[1];
  /* Unused parameter */
  (void) i2c;

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE;
  /* Select register and data to write */
  i2c_write_data[0] = reg;
  i2c_write_data[1] = data;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 2;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CDRV_Transfer(&seq);
  if (ret != i2cTransferDone)
  {
    return((int) ret);
  }

  return((int) 0);
}

/**************************************************************************//**
 * @brief
  *  Writes a block of data to the Si1147 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[in] data
 *   The data to write to the sensor.
 * @param[in] length
 *   The number of bytes to write to the sensor.
 * @param[in] reg
 *   The first register to begin writing to.
 * @return
 *   Returns zero on success. Otherwise returns error codes
 *   based on the I2CDRV.
 *****************************************************************************/
int Si1147_Write_Block_Register (I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t length, uint8_t const *data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[10];
  uint8_t i2c_read_data[1];
  int i;
  /* Unused parameter */
  (void) i2c;

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE;
  /* Select register to start writing to*/
  i2c_write_data[0] = reg;
  for (i=0; i<length;i++)
  {
    i2c_write_data[i+1] = data[i];
  }
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 1+length;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 0;

  ret = I2CDRV_Transfer(&seq);
  if (ret != i2cTransferDone)
  {
    return((int) ret);
  }

  return((int) 0);
}

/**************************************************************************//**
 * @brief
  *  Reads a block of data from the Si1147 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[out] data
 *   The data read from the sensor.
 * @param[in] length
 *   The number of bytes to write to the sensor.
 * @param[in] reg
 *   The first register to begin reading from.
 * @return
 *   Returns number of bytes read on success. Otherwise returns error codes
 *   based on the I2CDRV.
 *****************************************************************************/
int Si1147_Read_Block_Register (I2C_TypeDef *i2c,uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[1];
  /* Unused parameter */
  (void) i2c;

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select register to start reading from */
  i2c_write_data[0] = reg;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len  = 1;
  /* Select length of data to be read */
  seq.buf[1].data = data;
  seq.buf[1].len  = length;
  ret = I2CDRV_Transfer(&seq);
  if (ret != i2cTransferDone)
  {
    return((int) ret);
  }

  return((int) length);
}

