//-----------------------------------------------------------------------------
// Si114x_functions.c
//-----------------------------------------------------------------------------
// Copyright 2013 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// File Description:
//
// This file contains low-level routines that access the i2c controller
// The code has been ported from the generic Si114x API to the EFM32
// platform.
//
// Target:         Si114x
//
//-----------------------------------------------------------------------------
#include "si114x_functions.h"


#ifndef DELETE_PORTING_SECTION
//-----------------------------------------------------------------------------
#include "rtcdriver.h"
#include "em_device.h"
#include "si1147_i2c.h"

s16 Si114xWriteToRegister(HANDLE si114x_handle, u8 address, u8 data)
{

  return Si1147_Write_Register (((si114x_i2c_t*)si114x_handle)->i2c,((si114x_i2c_t*)si114x_handle)->addr, address, data);

}

s16 Si114xReadFromRegister(HANDLE si114x_handle, u8 address)
{
    uint8_t data;
    Si1147_Read_Register (((si114x_i2c_t*)si114x_handle)->i2c,((si114x_i2c_t*)si114x_handle)->addr, address, &data);
    return data;
}

s16 Si114xBlockWrite(HANDLE si114x_handle,
                        u8 address, u8 length, u8 const *values)
{

     return Si1147_Write_Block_Register (((si114x_i2c_t*)si114x_handle)->i2c,((si114x_i2c_t*)si114x_handle)->addr, address, length, values);

}

s16 Si114xBlockRead(HANDLE si114x_handle,
                        u8 address, u8 length, u8 *values)
{
    return Si1147_Read_Block_Register (((si114x_i2c_t*)si114x_handle)->i2c,((si114x_i2c_t*)si114x_handle)->addr, address, length, values);
}

extern volatile uint32_t msTicks;
void delay_10ms()
{
    // This is needed immediately after a reset command to the Si114x
    // In the PGM_Toolkit, there is sufficient latency, so none is added
    // here. This is a reminder that when porting code, that this must
    // be implemented.
    RTCDRV_Delay(10);

}

//-----------------------------------------------------------------------------
//------ End of Porting section. No need to modify anything else below --------
//-----------------------------------------------------------------------------
#endif // DELETE_PORTING_SECTION


//-----------------------------------------------------------------------------
// Following functions reads/writes i2c registers that will not wake up
// the Si114x's internal MCU
//
static s16 _waitUntilSleep(HANDLE si114x_handle)
{
    s16 retval;
    // This loops until the Si114x is known to be in its sleep state
    // or if an i2c error occurs
    while (1)
    {
        retval = Si114xReadFromRegister(si114x_handle, REG_CHIP_STAT);
        if (retval == 1) break;
        if (retval <  0) return retval;
    }
    return 0;
}

//-----------------------------------------------------------------------------
// The following functions writes to the CMD register and can therefore
// wake up the internal MCU

s16 Si114xReset(HANDLE si114x_handle)
{
    s32 retval = 0;

    retval+=Si114xWriteToRegister(si114x_handle, REG_MEAS_RATE,  0x00);
    retval+=Si114xWriteToRegister(si114x_handle, REG_ALS_RATE, 0x0);
    retval+=Si114xPauseAll(si114x_handle);
    // The clearing of the registers could be redundant, but it is okay.
    // This is to make sure that these registers are cleared.
    retval+=Si114xWriteToRegister(si114x_handle, REG_MEAS_RATE,  0x00);
    retval+=Si114xWriteToRegister(si114x_handle, REG_IRQ_ENABLE, 0x00);
    retval+=Si114xWriteToRegister(si114x_handle, REG_IRQ_MODE1,  0x00);
    retval+=Si114xWriteToRegister(si114x_handle, REG_IRQ_MODE2,  0x00);
    retval+=Si114xWriteToRegister(si114x_handle, REG_INT_CFG  ,  0x00);
    // retval+=Si114xWriteToRegister(si114x_handle, REG_COMMAND,    0x01);
    retval+=Si114xWriteToRegister(si114x_handle, REG_IRQ_STATUS, 0xFF);

    // Perform the Reset Command
    retval+=Si114xWriteToRegister(si114x_handle, REG_COMMAND, 1);

    // Delay for 10 ms. This delay is needed to allow the Si114x
    // to perform internal reset sequence.
    delay_10ms();

    // Write Hardware Key
    retval+=Si114xWriteToRegister(si114x_handle, REG_HW_KEY, HW_KEY_VAL0);

    return retval;
}


static s16 _sendCmd(HANDLE si114x_handle, u8 command)
{
    s16  response;
    s16  retval;

    // Get the response register contents
    if ((response=Si114xReadFromRegister(si114x_handle, REG_RESPONSE))<0)
        return response;

    // Double-check the response register is consistent
    while(1)
    {
        if((retval=_waitUntilSleep(si114x_handle)) != 0) return retval;

        if(command==0) break; // Skip if the command is NOP

        retval=Si114xReadFromRegister(si114x_handle, REG_RESPONSE);
        if(retval==response) break;
        else if(retval<0) return retval;
        else response = retval;
    }

    // Send the Command
    if ((retval=Si114xWriteToRegister(si114x_handle, REG_COMMAND, command)) !=0)
        return retval;

    // Expect a change in the response register
    while(1)
    {

        if(command==0) break; // Skip if the command is NOP

        retval= Si114xReadFromRegister(si114x_handle, REG_RESPONSE);
        if (retval != response) break;
        else if(retval<0) return retval;
    }
    return 0;
}

s16 Si114xParamRead(HANDLE si114x_handle, u8 address)
{
    // returns Parameter[address]
    s16 retval;
    u8 cmd = 0x80 + (address & 0x1F);
    if((retval=_sendCmd(si114x_handle, cmd ))!=0) return retval;
    return Si114xReadFromRegister(si114x_handle, REG_PARAM_RD);
}

s16 Si114xNop       (HANDLE si114x_handle) {return _sendCmd(si114x_handle,0x00);}
s16 Si114xPsForce   (HANDLE si114x_handle) {return _sendCmd(si114x_handle,0x05);}
s16 Si114xAlsForce  (HANDLE si114x_handle) {return _sendCmd(si114x_handle,0x06);}
s16 Si114xPsAlsForce(HANDLE si114x_handle) {return _sendCmd(si114x_handle,0x07);}
s16 Si114xPsAlsAuto (HANDLE si114x_handle) {return _sendCmd(si114x_handle,0x0F);}

//-----------------------------------------------------------------------------
// Si114xParamSet writes to the PARAM_WR and CMD register
//
s16 Si114xParamSet(HANDLE si114x_handle, u8 address, u8 data)
{
    s16     retval;
    u8      buffer[2];
    //s16     response;

    if((retval = _waitUntilSleep(si114x_handle))!=0) return retval;
    buffer[0]= data;
    buffer[1]= 0xA0 + (address & 0x1F);
    retval=Si114xBlockWrite(si114x_handle, REG_PARAM_WR, 2, (const u8 *)buffer);
    if(retval!=0) return retval;
    while(1)
    {
        retval=Si114xReadFromRegister(si114x_handle, REG_PARAM_RD);
        if (retval==data) break;
    }
    return 0;
}


//-----------------------------------------------------------------------------
// use this to pause measurements
static s16 _PsAlsPause (HANDLE si114x_handle)
{
    return _sendCmd(si114x_handle,0x0B);
}

s16 Si114xPauseAll(HANDLE si114x_handle)
{
    while (1)
    {
        // Keep sending nops until the response is zero
        while (1)
        {
            if ((Si114xReadFromRegister(si114x_handle, REG_RESPONSE))==0)
                break;
            else
                Si114xNop(si114x_handle);
        }

        // Pause the device
        _PsAlsPause(si114x_handle);

        // Wait for response
        while(1)
        {
            if ((Si114xReadFromRegister(si114x_handle, REG_RESPONSE))!=0)
                break;
        }

        // When the PsAlsPause() response is good, we expect it to be a '1'.
        if ((Si114xReadFromRegister(si114x_handle, REG_RESPONSE))==1)
            break;  // otherwise, start over.
    }
    return 0;
}


s16 si114x_get_calibration( HANDLE si114x_handle, SI114X_CAL_S *si114x_cal, char security)
{
   (void) si114x_handle;
   (void) si114x_cal;
   (void) security;
   // although the SI114x_CAL_S structure is not filled up properly, the set_ucoef() function will not use it.
   return 0;
}

s16 si114x_set_ucoef( HANDLE si114x_handle, u8 ref_ucoef[], SI114X_CAL_S *si114x_cal )
{
   (void) ref_ucoef;
   (void) si114x_cal;
   s16 response;
   u8 ucoef[4] = { 0x29, 0x89, 0x02, 0x00 } ;

   // This will write 4 bytes starting with I2C address 0x13
    response = Si114xBlockWrite( si114x_handle, REG_UCOEF0, 4, &ucoef[0] );
    return response;
}

