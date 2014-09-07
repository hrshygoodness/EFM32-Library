/**************************************************************************//**
 * @file utils.c
 * @brief Various utility functions for the debug interface
 * @author Silicon Labs
 * @version 1.03
 ******************************************************************************
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
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "dap.h"
#include "utils.h"
#include "errors.h"
#include "delay.h"


/**********************************************************
 * Reads the unique ID of the target from the DI page.
 * 
 * @returns
 *    The unique ID of target
 **********************************************************/
uint64_t readUniqueId(void)
{
  uint64_t uniqueId;
  
  /* Retrive high part of unique ID */
  uniqueId = readMem(UNIQUE_ID_HIGH_ADDR);
  uniqueId = uniqueId << 32;
  
  /* Retrive low part of unique ID */
  uniqueId |= readMem(UNIQUE_ID_LOW_ADDR);  

  return uniqueId;
}


/**********************************************************
 * Halts the target CPU
 **********************************************************/
void haltTarget(void)
{
  int timeout = DEBUG_EVENT_TIMEOUT;
  writeAP(AP_TAR, (uint32_t)&(CoreDebug->DHCSR));
  writeAP(AP_DRW, STOP_CMD);
  
  uint32_t dhcrState;
  do {
    writeAP(AP_TAR, (uint32_t)&(CoreDebug->DHCSR));
    readAP(AP_DRW, &dhcrState);
    readDP(DP_RDBUFF, &dhcrState);
    timeout--;
  } while ( !(dhcrState & CoreDebug_DHCSR_S_HALT_Msk) && timeout > 0 ); 
  
  if ( !(dhcrState & CoreDebug_DHCSR_S_HALT_Msk) ) {
    RAISE(SWD_ERROR_TIMEOUT_HALT);
  }
}


/**********************************************************
 * Lets the target CPU run freely (stops halting)
 **********************************************************/
void runTarget(void)
{
  writeAP(AP_TAR, (uint32_t)&(CoreDebug->DHCSR));
  writeAP(AP_DRW, RUN_CMD);
}

/**********************************************************
 * Single steps the target
 **********************************************************/
void stepTarget(void)
{
  writeAP(AP_TAR, (uint32_t)&(CoreDebug->DHCSR));
  writeAP(AP_DRW, STEP_CMD);
}


/**********************************************************
 * Retrieves total flash size from the DI page of the target
 * 
 * @returns
 *    The flash size in bytes
 **********************************************************/
int getFlashSize(void)
{  
  /* Read memory size from the DI page */
  uint32_t msize = readMem((uint32_t)&(DEVINFO->MSIZE));
  
  /* Retrieve flash size (in kB) */
  uint32_t flashSize = (msize & _DEVINFO_MSIZE_FLASH_MASK) >> _DEVINFO_MSIZE_FLASH_SHIFT;
  
  /* Return value in bytes */
  return flashSize * 1024;
}


/**********************************************************
 * Retrieves page size from the DI page of the target
 * 
 * @returns
 *    The page size in bytes
 **********************************************************/
int getPageSize(void)
{  
  uint32_t part = readMem((uint32_t)&(DEVINFO->PART));
  uint32_t msize = readMem((uint32_t)&(DEVINFO->MSIZE));
  
  uint32_t prodRev = (part & _DEVINFO_PART_PROD_REV_MASK) >> _DEVINFO_PART_PROD_REV_SHIFT;
  uint32_t family = (part &  _DEVINFO_PART_DEVICE_FAMILY_MASK) >> _DEVINFO_PART_DEVICE_FAMILY_SHIFT;
  
  uint32_t pageSize;
  
  /* Figure out the size of the flash pages. */
  switch(family)
  {
  case _DEVINFO_PART_DEVICE_FAMILY_GG:          /* Giant Gecko   */
    if (prodRev < 13)
    {
      /* All Giant Gecko rev B, with prod rev. < 13 use 2048 as page size, not 4096 */
      pageSize = 2048;
    } 
    else
    {
      pageSize = 4096;
    }
    break;
    
  case _DEVINFO_PART_DEVICE_FAMILY_LG:          /* Leopard Gecko */
  case _DEVINFO_PART_DEVICE_FAMILY_WG:          /* Wonder Gecko  */  
    pageSize = 2048;
    break;

  case _DEVINFO_PART_DEVICE_FAMILY_G:
  case _DEVINFO_PART_DEVICE_FAMILY_TG:
    pageSize = 512;
    break;
    
  case _DEVINFO_PART_DEVICE_FAMILY_ZG:
    pageSize = 1024;
    break;

  default:
    pageSize = 512;
    break;
  }
  
  return pageSize;
}

/**********************************************************
 * Retrieve the device name from the DI page of the target
 * 
 * @param deviceName[out]
 *    Device name is stored in this buffer when 
 *    the function returns. The calling function is
 *    responsible for allocating memory for the string
 **********************************************************/
void getDeviceName(char deviceName[])
{
  char familyCode[3];
  
  uint32_t part = readMem((uint32_t)&(DEVINFO->PART));
  uint32_t msize = readMem((uint32_t)&(DEVINFO->MSIZE));
  
  uint32_t flashSize = (msize & _DEVINFO_MSIZE_FLASH_MASK) >> _DEVINFO_MSIZE_FLASH_SHIFT;
  uint32_t family = (part &  _DEVINFO_PART_DEVICE_FAMILY_MASK) >> _DEVINFO_PART_DEVICE_FAMILY_SHIFT;  
  uint32_t partNum = (part &  _DEVINFO_PART_DEVICE_NUMBER_MASK) >> _DEVINFO_PART_DEVICE_NUMBER_SHIFT;  
  
  switch (family)
  {
  case _DEVINFO_PART_DEVICE_FAMILY_GG:          /* Giant Gecko   */
    sprintf(familyCode, "%s", "GG");
    break;
  case _DEVINFO_PART_DEVICE_FAMILY_LG:          /* Leopard Gecko */
    sprintf(familyCode, "%s", "LG");
    break;
  case _DEVINFO_PART_DEVICE_FAMILY_WG:          /* Wonder Gecko  */
    sprintf(familyCode, "%s", "WG");
    break;
  case _DEVINFO_PART_DEVICE_FAMILY_G:           /* Gecko */
    sprintf(familyCode, "%s", "G");
    break;
  case _DEVINFO_PART_DEVICE_FAMILY_TG:          /* Tiny Gecko */
    sprintf(familyCode, "%s", "TG");
    break;
  case _DEVINFO_PART_DEVICE_FAMILY_ZG:          /* Zero Gecko */
    sprintf(familyCode, "%s", "ZG");
    break;
  default:
    sprintf(familyCode, "%s", "XX");            /* Unknown family */
    break;
  }
  
  sprintf(deviceName, "EFM32%s%dF%d", familyCode, partNum, flashSize);
}

/**********************************************************
 * Get the wrap-around period for the TAR register. This
 * is device dependent and affects the burst write
 * algorithms. This function hard-codes the values
 * based on information from the DI-page. 
 * 
 * @returns
 *   The wrap-around period of the TAR register
 **********************************************************/
uint32_t getTarWrap(void)
{
  uint32_t part = readMem((uint32_t)&(DEVINFO->PART));
  uint32_t family = (part &  _DEVINFO_PART_DEVICE_FAMILY_MASK) >> _DEVINFO_PART_DEVICE_FAMILY_SHIFT;
  
  /* Hard-code result based on device family. ZG has 1kB 
   * wrap. G/TG/LG/WG/GG has 4kB. Default to 1 kB on unknown
   * devices */
  switch (family)
  {
  case _DEVINFO_PART_DEVICE_FAMILY_GG:          /* Giant Gecko   */
  case _DEVINFO_PART_DEVICE_FAMILY_LG:          /* Leopard Gecko */
  case _DEVINFO_PART_DEVICE_FAMILY_WG:          /* Wonder Gecko  */
  case _DEVINFO_PART_DEVICE_FAMILY_G:           /* Gecko */
  case _DEVINFO_PART_DEVICE_FAMILY_TG:          /* Tiny Gecko */
    return 0xFFF;
  case _DEVINFO_PART_DEVICE_FAMILY_ZG:          /* Zero Gecko */
    return 0x3FF;
  default:                                      /* Unknown family */
    return 0x3FF;
  }
}


/**********************************************************
 * Resets the target CPU by using the AIRCR register. 
 * The target will be halted immediately when coming
 * out of reset. Does not reset the debug interface.
 **********************************************************/
void resetAndHaltTarget(void)
{
  uint32_t dhcsr;
  int timeout = DEBUG_EVENT_TIMEOUT;
  
  /* Halt target first. This is necessary before setting
   * the VECTRESET bit */
  haltTarget();
  
  /* Set halt-on-reset bit */
  writeMem((uint32_t)&(CoreDebug->DEMCR), CoreDebug_DEMCR_VC_CORERESET_Msk);
  
  /* Clear exception state and reset target */
  writeAP(AP_TAR, (uint32_t)&(SCB->AIRCR));
  writeAP(AP_DRW, (0x05FA << SCB_AIRCR_VECTKEY_Pos) |
                  SCB_AIRCR_VECTCLRACTIVE_Msk |
                  SCB_AIRCR_VECTRESET_Msk);
    
  /* Wait for target to reset */
  do { 
    delayUs(10);
    timeout--;
    dhcsr = readMem((uint32_t)&(CoreDebug->DHCSR));
  } while ( dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk );
  
  
  /* Check if we timed out */
  dhcsr = readMem((uint32_t)&(CoreDebug->DHCSR));
  if ( dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk ) 
  {
    RAISE(SWD_ERROR_TIMEOUT_WAITING_RESET);
  }
  
  /* Verify that target is halted */
  if ( !(dhcsr & CoreDebug_DHCSR_S_HALT_Msk) ) 
  {
    RAISE(SWD_ERROR_TARGET_NOT_HALTED);
  }
}

/**********************************************************
 * Resets the target CPU by using the AIRCR register. 
 * Does not reset the debug interface
 **********************************************************/
void resetTarget(void)
{  
  uint32_t dhcsr;
  int timeout = DEBUG_EVENT_TIMEOUT;
  
  /* Clear the VC_CORERESET bit */
  writeMem((uint32_t)&(CoreDebug->DEMCR), 0);
  
  /* Do a dummy read of sticky bit to make sure it is cleared */
  readMem((uint32_t)&(CoreDebug->DHCSR));
  dhcsr = readMem((uint32_t)&(CoreDebug->DHCSR));
  
  /* Reset CPU */
  writeMem((uint32_t)&(SCB->AIRCR), AIRCR_RESET_CMD);
  
  /* Wait for reset to complete */
  
  
  /* First wait until sticky bit is set. This means we are
   * or have been in reset */
  delayUs(100);
  do { 
    delayUs(10);
    dhcsr = readMem((uint32_t)&(CoreDebug->DHCSR));
    timeout--;
  } while ( !(dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk) && timeout > 0 );
    
  /* Throw error if sticky bit is never set */
  if ( !(dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk) ) {
    RAISE(SWD_ERROR_TIMEOUT_WAITING_RESET);
  }
    
  /* Wait for sticky bit to be cleared. When bit is cleared are we out of reset */
  timeout = DEBUG_EVENT_TIMEOUT;
  do { 
    delayUs(10);
    dhcsr = readMem((uint32_t)&(CoreDebug->DHCSR));
    timeout--;
  } while ( dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk && timeout > 0 );
  
  /* Throw error if bit is never cleared */
  if ( dhcsr & CoreDebug_DHCSR_S_RESET_ST_Msk ) {
    RAISE(SWD_ERROR_TIMEOUT_WAITING_RESET);
  }
  
}

/**********************************************************
 * Performs a pin reset on the target
 **********************************************************/
void hardResetTarget(void)
{
  GPIO_PinOutClear((GPIO_Port_TypeDef)RESET_PORT, RESET_PIN);
  
  delayMs(50);
  
  GPIO_PinOutSet((GPIO_Port_TypeDef)RESET_PORT, RESET_PIN);
}

/**********************************************************
 * Reads one word from internal memory
 * 
 * @param addr 
 *    The address to read from
 * 
 * @returns 
 *    The value at @param addr
 **********************************************************/
uint32_t readMem(uint32_t addr)
{
  uint32_t ret;
  writeAP(AP_TAR, addr);
  readAP(AP_DRW, &ret);
  readDP(DP_RDBUFF, &ret);
  return ret;
}

/**********************************************************
 * Writes one word to internal memory
 * 
 * @param addr 
 *    The address to write to 
 *
 * @param data
 *    The value to write
 * 
 * @returns 
 *    The value at @param addr
 **********************************************************/
void writeMem(uint32_t addr, uint32_t data)
{
  writeAP(AP_TAR, addr);
  writeAP(AP_DRW, data);
}
 
/**********************************************************
 * Waits for the REGRDY bit in DCRSR. This bit indicates
 * that the DCRDR/DCRSR registers are ready to accept
 * new data. 
 **********************************************************/
void waitForRegReady(void)
{
  uint32_t dhcsr;
  do {
    dhcsr = readMem((uint32_t)&CoreDebug->DHCSR);
  } while ( !(dhcsr & CoreDebug_DHCSR_S_REGRDY_Msk) );
}


/**********************************************************
 * Verifies the current firmware against the locally
 * stored original. This function assumes the firmware
 * has been written to target starting at address 0x00. 
 *
 * @param fwImage
 *    Pointer to locally stored copy of firmware image
 * 
 * @param size
 *    Size (in bytes) of firmware image
 * 
 * @returns 
 *    True if target firmware matches local copy. 
 *    False otherwise. 
 **********************************************************/
bool verifyFirmware(uint32_t *fwImage, uint32_t size)
{
  int i;
  int numWords = size / 4;
  bool ret = true;
  uint32_t value;
  uint32_t addr;
  uint32_t tarWrap =  getTarWrap();
  
  printf("Verifying firmware\n");
  
  /* Set autoincrement on TAR */
  writeAP(AP_CSW, AP_CSW_DEFAULT | AP_CSW_AUTO_INCREMENT);
  
  for ( i=0; i<numWords; i++ ) 
  {   
    /* Get current address */
    addr = i*4;
        
    /* TAR must be initialized at every TAR wrap boundary
     * because the autoincrement wraps around at these */
    if ( (addr & tarWrap) == 0 )
    {
      writeAP(AP_TAR, addr);
      
      /* Do one dummy read. Subsequent reads will return the 
       * correct result. */
      readAP(AP_DRW, &value);
    }
    
    /* Read the value from addr */
    readAP(AP_DRW, &value);
    
    /* Verify that the read value matches what is expected */
    if ( value != fwImage[i] ) 
    {
      printf("Verification failed at address 0x%.8x\n", addr);
      printf("Value is 0x%.8x, should have been 0x%.8x\n", value, fwImage[i]);
      ret = false;
      break;
    }
  }
  
  /* Disable autoincrement on TAR */
  writeAP(AP_CSW, AP_CSW_DEFAULT);
     
  return ret;
}

/**********************************************************
 * Returns true if the @param dpId is a valid
 * IDCODE value. 
 **********************************************************/
bool verifyDpId(uint32_t dpId)
{
  if ( dpId == EFM32_DPID_1 ) 
  {
    return true; // Valid for G, LG, GG, TG, WG
  }
  else if ( dpId == EFM32_DPID_2 )
  {
    return true; // Valid for ZG
  } 
  else if ( dpId == EFM32_DPID_3 )
  {
    return true; // Valid for SWMD
  }   
  else 
  {
    return false;
  }
}

/**********************************************************
 * Returns true if the @param apId is a valid
 * IDR value for the AHB-AP. 
 **********************************************************/
bool verifyAhbApId(uint32_t apId)
{
  if ( apId == EFM32_AHBAP_ID_1 ) 
  {
    return true; // Valid for G, LG, GG, TG, WG
  }
  else if ( apId == EFM32_AHBAP_ID_2 )
  {
    return true; // Valid for ZG
  } 
  else 
  {
    return false;
  }
}

/**********************************************************
 * This function will check if a Zero Gecko device is
 * locked. The method is different from other MCUs,
 * we have to read the AAP registers from the internal
 * memory space.
 * 
 * This process can fail (we receive a FAULT response) on 
 * other devices so wee need to check for failure on the AP 
 * transaction and clear the STICKYERR flag in CTRL/STAT 
 * before continuing. 
 **********************************************************/
void checkIfZeroGeckoIsLocked(void)
{
  int readError = SWD_ERROR_OK;
  uint32_t readVal;
  uint32_t apId;
  
  /* Try reading the AAP_IDR register on Zero. 
   * Do in a separate TRY/CATCH block in case to allow
   * failure in this transaction.
   */
  TRY 
    apId = readMem(AAP_IDR_ZERO);
  CATCH
    /* If transaction failed. Store error code */
    readError = errorCode;
  ENDTRY
  
  /* If the transaction was OK we check if we got
   * access to the AAP registers. If we do, the device
   * is locked. 
   */
  if ( readError == SWD_ERROR_OK )
  {
    if ( apId == EFM32_AAP_ID ) 
    {
      RAISE(SWD_ERROR_MCU_LOCKED);
    }
  } 
  /* We received a FAULT or WAIT error. This is normal on non-ZG devices. 
   * If this happens we have to clear the STICKYERR flag before continuing. 
   * If we do not do this all subsequent AP transactions will fail. 
   */
  else if ( readError == SWD_ERROR_FAULT || readError == SWD_ERROR_WAIT )
  {
    /* Read CTRL/STAT register */
    readDP(DP_CTRL, &readVal);
    
    /* See if STICKYERR is set */
    if ( readVal & (1 << 5) )
    {
      /* Clear sticky error */
      writeDP(DP_ABORT, (1 << 2));
    } 
  } 
  /* We received another error, e.g. protocol error. 
   * Report the error back to the calling function */
  else  
  {
    RAISE(readError);
  }
}


/**********************************************************
 * Writes a value to a CPU register in the target.
 * 
 * @param reg
 *   The register number to write to
 * 
 * @param value
 *   The value to write to the register
 **********************************************************/
void writeCpuReg(int reg, uint32_t value)
{
  /* Wait until debug register is ready to accept new data */
  waitForRegReady();
  
  /* Write value to Data Register */
  writeAP(AP_TAR, (uint32_t)&(CoreDebug->DCRDR));
  writeAP(AP_DRW, value);
  
  /* Write register number ot Selector Register. 
   * This will update the CPU register */
  writeAP(AP_TAR, (uint32_t)&(CoreDebug->DCRSR));
  writeAP(AP_DRW, 0x10000 | reg); 
}


/**********************************************************
 * Performs the initialization sequence on the SW-DP. 
 * After this completes the debug interface can be used. 
 * Raises an exception on any error during connection. 
 **********************************************************/
void connectToTarget(void)
{
  uint32_t dpId,apId;
  
  delayUs(500);
  
  printf("Connecting to target...\n");
  
  dpId = initDp();
  
  /* Verify that the DP returns the correct ID */
  if ( !verifyDpId(dpId) ) 
  {
    printf("Read IDCODE = 0x%.8x\n", dpId);
    RAISE(SWD_ERROR_INVALID_IDCODE);
  }
  
  /* Verify that the AP returns the correct ID */
  int retry = AHB_IDR_RETRY_COUNT;
  while ( retry > 0 )
  {
    apId = readApId();
    if ( verifyAhbApId(apId) )
    {
      /* Success. AHB-AP registers found */
      break;
    }
    
    retry--;
  }
  
  /* In case we did NOT find the AHB-AP registers check 
   * if the chip is locked or if some other error ocurred. */  
  if ( !verifyAhbApId(apId) ) 
  {
    /* If the ID is from AAP, the MCU is locked. 
     * Debug access is not available. */
    if ( apId == EFM32_AAP_ID ) 
    {
      RAISE(SWD_ERROR_MCU_LOCKED);
    } 
    else
    {
      /* The AP ID was not from AAP nor AHB-AP. This is an error. */ 
      printf("Read IDR = 0x%.8x\n", apId);
      RAISE(SWD_ERROR_INVALID_IDR);  
    }
  } 
    
  /* Set up parameters for AHB-AP. This must be done before accessing
   * internal memory. */
  initAhbAp();
  
  /*
   * On Zero Gecko it is possible that the device is locked
   * even though we reach this point. We have to see if 
   * we can access the AAP registers which are memory mapped 
   * on these devices. The function will raise an exception
   * if the device is a Zero Gecko and is currently locked.
   */
  if ( apId == EFM32_AHBAP_ID_2 )
  {
    checkIfZeroGeckoIsLocked();
  }
  
  haltTarget();
   
  /* Get device name */
  char deviceName[30];
  getDeviceName(deviceName);
  printf("Target is %s\n", deviceName);
  
  /* Read the unique ID of the MCU */
  uint64_t uniqueId = readUniqueId();
  printf("Unique ID = 0x%.16llx\n", uniqueId);
}

