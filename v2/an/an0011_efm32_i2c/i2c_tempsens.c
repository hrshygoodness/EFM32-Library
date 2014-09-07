/***************************************************************************//**
 * @file i2c_tempsens.c
 * @brief Temperature sensor driver for DS75 temperature sensor compatible
 *   device on the DVK.
 * @author Silicon Labs
 * @version 1.07
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/


#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "bsp.h"
#include "i2c_tempsens.h"


/*******************************************************************************
 **************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/
I2C_TransferReturn_TypeDef I2C_Status;

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C0_IRQHandler(void)
{
  /* Just run the I2C_Transfer function that checks interrupts flags and returns */
  /* the appropriate status */
  I2C_Status = I2C_Transfer(I2C0);
}

/***************************************************************************//**
 * @brief
 *   Initalize basic I2C master mode driver for use on the DVK.
 *
 * @details
 *   This driver only supports master mode, single bus-master. In addition
 *   to configuring the EFM32 I2C peripheral module, it also configures DVK
 *   specific setup in order to use the I2C bus.
 *
 * @param[in] init
 *   Pointer to I2C initialization structure.
 ******************************************************************************/
void I2C_Tempsens_Init(void)
{
  int i;
  
  /* Initialize I2C driver for the tempsensor on the DK, using standard rate. */
  /* Devices on DK itself supports fast mode, */
  /* but in case some slower devices are added on */
  /* prototype board, we use standard mode. */
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
  
  /* Initialize DVK board register access */
  BSP_Init(BSP_INIT_DEFAULT);
  
  BSP_PeripheralAccess(BSP_I2C, true);

  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_I2C0, true);

  /* Use location 3: SDA - Pin D14, SCL - Pin D15 */
  /* Output value must be set to 1 to not drive lines low... We set */
  /* SCL first, to ensure it is high before changing SDA. */
  GPIO_PinModeSet(gpioPortD, 15, gpioModeWiredAnd, 1);
  GPIO_PinModeSet(gpioPortD, 14, gpioModeWiredAnd, 1);

  /* In some situations (after a reset during an I2C transfer), the slave */
  /* device may be left in an unknown state. Send 9 clock pulses just in case. */
  for (i = 0; i < 9; i++)
  {
    /*
     * TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler
     * optimization when running at 14MHz. A bit high for standard mode devices,
     * but DVK only has fast mode devices. Need however to add some time
     * measurement in order to not be dependable on frequency and code executed.
     */
    GPIO_PinModeSet(gpioPortD, 15, gpioModeWiredAnd, 0);
    GPIO_PinModeSet(gpioPortD, 15, gpioModeWiredAnd, 1);
  }

  /* Enable pins at location 3 (which is used on the DVK) */
  I2C0->ROUTE = I2C_ROUTE_SDAPEN |
                I2C_ROUTE_SCLPEN |
                (3 << _I2C_ROUTE_LOCATION_SHIFT);

  I2C_Init(I2C0, &i2cInit);
  
  /* Clear and enable interrupt from I2C module */
  NVIC_ClearPendingIRQ(I2C0_IRQn);
  NVIC_EnableIRQ(I2C0_IRQn);
}

/***************************************************************************//**
 * @brief
 *   Convert temperature from Celsius to Fahrenheit.
 *
 * @param[in,out] temp
 *   Reference to structure to holding temperature in Celsius to convert. Upon
 *   return, it has been changed to Fahrenheit.
 ******************************************************************************/
void TEMPSENS_Celsius2Fahrenheit(TEMPSENS_Temp_TypeDef *temp)
{
  int32_t convert;

  /* Use integer arithmetic. Values are sufficiently low, so int32 will */
  /* not overflow. */
  convert = (int32_t)(temp->i) * 10000;
  convert += (int32_t)(temp->f);
  convert = ((convert * 9) / 5) + (32 * 10000);

  temp->i = (int16_t)(convert / 10000);
  temp->f = (int16_t)(convert % 10000);
}


/***************************************************************************//**
 * @brief
 *   Read sensor register content.
 *
 * @details
 *   If reading the temperature register, when a measurement is completed inside
 *   the sensor device, the new measurement may not be stored. For this reason,
 *   the temperature should not be polled with a higher frequency than the
 *   measurement conversion time for a given resolution configuration. Please
 *   refer to sensor device datasheet.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] addr
 *   I2C address for temperature sensor, in 8 bit format, where LSB is reserved
 *   for R/W bit.
 *
 * @param[in] reg
 *   Register to read.
 *
 * @param[out] val
 *   Reference to place register read.
 *
 * @return
 *   Returns 0 if register read, <0 if unable to read register.
 ******************************************************************************/
int TEMPSENS_RegisterGet(I2C_TypeDef *i2c,
                         uint8_t addr,
                         TEMPSENS_Register_TypeDef reg,
                         uint16_t *val)
{
  I2C_TransferSeq_TypeDef seq;
  uint8_t regid[1];
  uint8_t data[2];

  seq.addr = addr;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select register to be read */
  regid[0] = ((uint8_t)reg) & 0x3;
  seq.buf[0].data = regid;
  seq.buf[0].len = 1;
  /* Select location/length to place register */
  if (reg == tempsensRegConfig)
  {
    /* Only 1 byte reg, clear upper 8 bits */
    data[0] = 0;
    seq.buf[1].data = data + 1;
    seq.buf[1].len = 1;
  }
  else
  {
    seq.buf[1].data = data;
    seq.buf[1].len = 2;
  }

  /* Do a polled transfer */
  I2C_Status = I2C_TransferInit(i2c, &seq);
  while (I2C_Status == i2cTransferInProgress)
  {
    /* Enter EM1 while waiting for I2C interrupt */
    EMU_EnterEM1();
    /* Could do a timeout function here. */
  }
  
  if (I2C_Status != i2cTransferDone)
  {
    return((int)I2C_Status);
  }

  *val = (((uint16_t)(data[0])) << 8) | data[1];

  return(0);
}


/***************************************************************************//**
 * @brief
 *   Write to sensor register.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] addr
 *   I2C address for temperature sensor, in 8 bit format, where LSB is reserved
 *   for R/W bit.
 *
 * @param[in] reg
 *   Register to write (temperature register cannot be written).
 *
 * @param[in] val
 *   Value used when writing to register.
 *
 * @return
 *   Returns 0 if register written, <0 if unable to write to register.
 ******************************************************************************/
int TEMPSENS_RegisterSet(I2C_TypeDef *i2c,
                         uint8_t addr,
                         TEMPSENS_Register_TypeDef reg,
                         uint16_t val)
{
  I2C_TransferSeq_TypeDef seq;
  uint8_t data[3];

  if (reg == tempsensRegTemp)
  {
    return(-1);
  }

  seq.addr = addr;
  seq.flags = I2C_FLAG_WRITE;
  /* Select register to be written */
  data[0] = ((uint8_t)reg) & 0x3;
  seq.buf[0].data = data;
  if (reg == tempsensRegConfig)
  {
    /* Only 1 byte reg */
    data[1] = (uint8_t)val;
    seq.buf[0].len = 2;
  }
  else
  {
    data[1] = (uint8_t)(val >> 8);
    data[2] = (uint8_t)val;
    seq.buf[0].len = 3;
  }

  /* Do a polled transfer */
  I2C_Status = I2C_TransferInit(i2c, &seq);
  while (I2C_Status == i2cTransferInProgress)
  {
    /* Enter EM1 while waiting for I2C interrupt */
    EMU_EnterEM1();
    /* Could do a timeout function here. */
  }
  
  return(I2C_Status);
}


/***************************************************************************//**
 * @brief
 *   Fetch current temperature from temperature sensor (in Celsius).
 *
 * @details
 *   If reading the temperature, when a measurement is completed inside the
 *   sensor device, the new measurement may not be stored. For this reason, the
 *   temperature should not be polled with a higher frequency than the
 *   measurement conversion time for a given resolution configuration. Please
 *   refer to sensor device datasheet.
 *
 * @note
 *   The sensor on the the DVK, STDS75, does not seem to have any check on max
 *   temperature limit, ie when exceeding specified max 125 C limit, it will keep
 *   increasing 9 bit counter to max representable 9 bit value of 127.5 C and
 *   then flip over to negative numbers. There is no limit check in this
 *   implementation on such issues.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] addr
 *   I2C address for temperature sensor, in 8 bit format, where LSB is reserved
 *   for R/W bit.
 *
 * @param[out] temp
 *   Reference to structure to place temperature (in Celsius).
 *
 * @return
 *   Returns 0 if temperature read, <0 if unable to read temperature.
 ******************************************************************************/
int TEMPSENS_TemperatureGet(I2C_TypeDef *i2c,
                            uint8_t addr,
                            TEMPSENS_Temp_TypeDef *temp)
{
  int ret;
  uint32_t tmp;
  uint16_t val = 0;

  ret = TEMPSENS_RegisterGet(i2c, addr, tempsensRegTemp, &val);
  if (ret < 0)
  {
    return(ret);
  }

  /* Get all 12 bits potentially used */
  tmp = (uint32_t)(val >> 4);

  /* If negative number, convert using 2s complement */
  if (tmp & 0x800)
  {
    tmp = (~tmp + 1) & 0xfff;
    temp->i = - (int16_t)(tmp >> 4);
    temp->f = - (int16_t)((tmp & 0xf) * 625);
  }
  else
  {
    temp->i = (int16_t)(tmp >> 4);
    temp->f = (int16_t)((tmp & 0xf) * 625);
  }

  return(0);
}
