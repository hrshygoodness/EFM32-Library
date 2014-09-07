/***************************************************************************//**
 * @file
 * @brief Driver for the Si7013 Temperature / Humidity sensor
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


#include "si7013.h"
#include "i2cdrv.h"

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/** Si7013 Read Temperature Command */
#define SI7013_READ_TEMP     0xE0 /* Read previous T data from RH measurement
                                     command*/
/** Si7013 Read RH Command */
#define SI7013_READ_RH       0xE5 /* Perform RH (and T) measurement. */
/** Si7013 Read ID */
#define SI7013_READ_ID1_1    0xFA
#define SI7013_READ_ID1_2    0x0F
#define SI7013_READ_ID2_1    0xFc
#define SI7013_READ_ID2_2    0xc9

/** @endcond */

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief
  *  Reads data from the Si7013 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use (not used).
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[out] data
 *   The data read from the sensor.
 * @param[in] command
 *   The command to send to device. See the \#define's for details.
 * @return
 *   Returns number of bytes read on success. Otherwise returns error codes
 *   based on the I2CDRV.
 *****************************************************************************/
static int Si7013_Measure(I2C_TypeDef *i2c, uint8_t addr, uint32_t *data,
                          uint8_t command)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[2];
  uint8_t                    i2c_write_data[1];

  /* Unused parameter */
  (void) i2c;

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = command;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 1;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 2;

  ret = I2CDRV_Transfer(&seq);

  if (ret != i2cTransferDone)
  {
    *data = 0;
    return((int) ret);
  }

  *data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);

  return((int) 2);
}


/**************************************************************************//**
 * @brief
  *  Reads relative humidity and temperature from a Si7013 sensor.
 * @param[in] i2c
 *   The I2C peripheral to use.
 * @param[in] addr
 *   The I2C address of the sensor.
 * @param[out] rhData
 *   The relative humidity in percent (multiplied by 1000).
 * @param[out] tData
 *   The temperature in milli-Celsius.
 * @return
 *   Returns zero on OK, non-zero otherwise.
 *****************************************************************************/
int Si7013_MeasureRHAndTemp(I2C_TypeDef *i2c, uint8_t addr, uint32_t *rhData,
                        int32_t *tData)
{
  int ret = Si7013_Measure(i2c, addr, rhData, SI7013_READ_RH);

  if (ret == 2)
  {
    /* convert to milli-percent */
    *rhData = (((*rhData) * 15625L) >> 13) - 6000;
  }
  else
  {
    return -1;
  }

  ret = Si7013_Measure(i2c, addr, (uint32_t *) tData, SI7013_READ_TEMP);

  if (ret == 2)
  {
    *tData = (((*tData) * 21965L) >> 13) - 46850; /* convert to milli-degC */
  }
  else
  {
    return -1;
  }

  return 0;
}

/**************************************************************************//**
 * @brief
 *   Checks if a Si7013 is present on the I2C bus or not.
 * @param[in] i2c
 *   The I2C peripheral to use (Not used).
 * @param[in] addr
 *   The I2C address to probe.
 * @return
 *   True if a Si7013 is detected, false otherwise.
 *****************************************************************************/
bool Si7013_Detect(I2C_TypeDef *i2c, uint8_t addr)
{
  I2C_TransferSeq_TypeDef    seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t                    i2c_read_data[8];
  uint8_t                    i2c_write_data[2];

  /* Unused parameter */
  (void) i2c;

  seq.addr  = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select command to issue */
  i2c_write_data[0] = SI7013_READ_ID1_1;
  i2c_write_data[1] = SI7013_READ_ID1_2;
  seq.buf[0].data   = i2c_write_data;
  seq.buf[0].len    = 2;
  /* Select location/length of data to be read */
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len  = 8;

  ret = I2CDRV_Transfer(&seq);
  if (ret != i2cTransferDone)
  {
    return(false);
  }

  return(true);
}
