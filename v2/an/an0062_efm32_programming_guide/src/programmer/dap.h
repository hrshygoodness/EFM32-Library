/*******************************************************************************
 * @file dap.h
 * @brief Low level SWD interface functions. 
 * @author Silicon Labs
 * @version 1.03
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
#ifndef _DAP_H_
#define _DAP_H_

/* ACK responses */
#define ACK_OK     1
#define ACK_WAIT   2
#define ACK_FAULT  4

/* Address of DP read registers */
#define DP_IDCODE  0
#define DP_CTRL    1
#define DP_RESEND  2
#define DP_RDBUFF  3

/* Adressses of DP write registers */
#define DP_ABORT   0
#define DP_STAT    1
#define DP_SELECT  2

/* AHB-AP registers */
#define AP_CSW 0
#define AP_TAR 1
#define AP_DRW 3
#define AP_IDR 3  /* In bank 0xf */

/* AAP registers */
#define AAP_CMD    0 
#define AAP_CMDKEY 1
#define AAP_STATUS 2
#define AAP_IDR    3  /* In bank 0xf */

#define AAP_CMD_ZERO      0xF0E00000
#define AAP_CMDKEY_ZERO   0xF0E00004
#define AAP_STATUS_ZERO   0xF0E00008
#define AAP_IDR_ZERO      0xF0E000FC

/* Valid values for the DP IDCODE register */
#define EFM32_DPID_1      0x2BA01477  // G, LG, GG, TG, WG
#define EFM32_DPID_2      0x0BC11477  // ZG
#define EFM32_DPID_3      0x0bc12477  // SWMD


/* Valid value for the AAP IDR register */
#define EFM32_AAP_ID    0x16e60001

/* Valid values for the AHB-AP IDR register */
#define EFM32_AHBAP_ID_1  0x24770011 // G, LG, GG, TG, WG
#define EFM32_AHBAP_ID_2  0x04770031 // ZG

/* Key which must be written to AAP_CMDKEY before
 * writing to AAP_CMD */
#define AAP_UNLOCK_KEY 0xcfacc118
   
/* Value to write to AIRCR in order to do a soft
 * reset of the target */
#define AIRCR_RESET_CMD 0x05FA0006

/* Bit fields for the CSW register */
#define AP_CSW_32BIT_TRANSFER   0x02
#define AP_CSW_AUTO_INCREMENT   0x10
#define AP_CSW_MASTERTYPE_DEBUG (1 << 29)
#define AP_CSW_HPROT            (1 << 25)
#define AP_CSW_DEFAULT (AP_CSW_32BIT_TRANSFER | AP_CSW_MASTERTYPE_DEBUG | AP_CSW_HPROT)

/* Powerup request and acknowledge bits in CTRL/STAT */
#define DP_CTRL_CDBGPWRUPREQ  (1 << 28)
#define DP_CTRL_CDBGPWRUPACK  (1 << 29)
#define DP_CTRL_CSYSPWRUPREQ  (1 << 30)
#define DP_CTRL_CSYSPWRUPACK  (1 << 31)

/* Commands to run/step and let CPU run. 
 * Write these to DHCSR */
#define RUN_CMD  0xA05F0001
#define STOP_CMD 0xA05F0003
#define STEP_CMD 0xA05F0005

/* Flash addresses to retrieve the device unique id */
#define UNIQUE_ID_HIGH_ADDR 0x0FE081F4
#define UNIQUE_ID_LOW_ADDR  0x0FE081F0
   

/* AAP bit fields */
#define AAP_CMD_DEVICEERASE 1
#define AAP_CMD_SYSRESETREQ 2
#define AAP_STATUS_ERASEBUSY 1


/* Default pin configuration is provided
 * for EFM32GG-STK3700 and EFM32GG-DK3750.
 * Change these defines to select other pins. 
 * Note that in order to perform the 
 * Gecko Unlock Sequence, the pins PE8,PE9
 * MUST be used for SWCLK,SWDIO.
 */
#if defined(STK)

#define SWCLK_PORT 3
#define SWCLK_PIN  0
#define SWDIO_PORT 3
#define SWDIO_PIN  1
#define RESET_PORT 2
#define RESET_PIN  0

#else

#define SWCLK_PORT 4
#define SWCLK_PIN  8
#define SWDIO_PORT 4
#define SWDIO_PIN  9
#define RESET_PORT 2
#define RESET_PIN  0

#endif



/* Number of times to retry an SWD operation when receiving 
 * a WAIT response */
#define SWD_RETRY_COUNT 200

/* Number of times to retry the connection sequence */
#define CONNECT_RETRY_COUNT 3
   
/* Number of times to retry reading the AHB-IDR register when connecting */
#define AHB_IDR_RETRY_COUNT 20

/* Number of times to retry the AAP window expansion
 * sequence (with longer and longer delays) */   
#define AAP_EXPANSION_RETRY_COUNT 50
   
/* Number of times to retry reading the CTLR/STAT
 * register while waiting for powerup acknowledge */
#define PWRUP_TIMEOUT 100

/* Number of times to retry reading the ERASEBUSY flag
 * while waiting for Mass Erase (Debug Unlock) to complete */
#define UNLOCK_RETRY_COUNT 5
   
/* Number of times to retry reading status registers while
 * waiting for a debug event (such as a halt of soft reset) */
#define DEBUG_EVENT_TIMEOUT 200

/* Number of times to wait for flashloader */
#define FLASHLOADER_RETRY_COUNT 1000
   
/* Number of times to wait for MSC operation to complete */
#define MSC_TIMEOUT 100
   

/* JTAG to SWD bit sequence, transmitted LSB first */
#define JTAG2SWD 0xE79E

    


#if (SWDIO_PIN < 8)
#define SWDIO_SET_INPUT() { \
    GPIO->P[SWDIO_PORT].MODEL = (GPIO->P[SWDIO_PORT].MODEL & ~(0xf << 4 * SWDIO_PIN)) | (0x1 << (4 * SWDIO_PIN)); \
    GPIO->P[SWDIO_PORT].DOUT &=  ~(1 << SWDIO_PIN); }
#else 
#define SWDIO_SET_INPUT() { \
    GPIO->P[SWDIO_PORT].MODEH = (GPIO->P[SWDIO_PORT].MODEH & ~(0xf << 4 * (SWDIO_PIN-8))) | (0x1 << (4 * (SWDIO_PIN-8))); \
    GPIO->P[SWDIO_PORT].DOUT &=  ~(1 << SWDIO_PIN); }
#endif

#if (SWDIO_PIN < 8)
#define SWDIO_SET_OUTPUT() { \
    GPIO->P[SWDIO_PORT].MODEL = (GPIO->P[SWDIO_PORT].MODEL & ~(0xf << 4 * SWDIO_PIN)) | (0x4 << (4 * SWDIO_PIN)); \
    GPIO->P[SWDIO_PORT].DOUT &=  ~(1 << SWDIO_PIN); }    
#else
#define SWDIO_SET_OUTPUT() { \
    GPIO->P[SWDIO_PORT].MODEH = (GPIO->P[SWDIO_PORT].MODEH & ~(0xf << 4 * (SWDIO_PIN-8))) | (0x4 << (4 * (SWDIO_PIN-8))); \
    GPIO->P[SWDIO_PORT].DOUT &=  ~(1 << SWDIO_PIN); }    
#endif


#define SWDIO_SET() GPIO->P[SWDIO_PORT].DOUT |= (1 << SWDIO_PIN)
#define SWDIO_CLR() GPIO->P[SWDIO_PORT].DOUT &= ~(1 << SWDIO_PIN)
#define SWCLK_SET() GPIO->P[SWCLK_PORT].DOUT |= (1 << SWCLK_PIN)
#define SWCLK_CLR() GPIO->P[SWCLK_PORT].DOUT &= ~(1 << SWCLK_PIN)

#define SWDIO_OUT(bit) {if ( bit ) SWDIO_SET(); else SWDIO_CLR();}
#define SWDIO_IN() ((GPIO->P[SWDIO_PORT].DIN >> SWDIO_PIN) & 0x1)

#define SWCLK_CYCLE()   \
  SWCLK_CLR();          \
  SWCLK_SET()

#define SWDIO_CYCLE()   \
  SWDIO_SET();          \
  SWDIO_CLR()


/* If SWD_FAST is defined we compile faster versions
 * of the write and read bit macros. 
 * Note that the fast macros require that 
 * SWCLK and SWDIO is on the same GPIO port
 * and also that the RESET line is NOT connected
 * to this port since the macros write to the 
 * entire port at once.
 */
#ifdef SWD_FAST

#define WRITE_BIT(bit) \
if ( bit ) { \
  GPIO->P[SWDIO_PORT].DOUT = (1 << SWDIO_PIN); \
  GPIO->P[SWDIO_PORT].DOUT = (1 << SWDIO_PIN) | (1 << SWCLK_PIN); \
} else { \
  GPIO->P[SWDIO_PORT].DOUT = 0;  \
  GPIO->P[SWDIO_PORT].DOUT = 1 << SWCLK_PIN; \
}

#define READ_BIT(bit)                           \
  GPIO->P[SWCLK_PORT].DOUT = 0;                 \
  bit = SWDIO_IN();                             \
  GPIO->P[SWCLK_PORT].DOUT = 1 << SWCLK_PIN  
     
#else

#define WRITE_BIT(bit)  \
  SWDIO_OUT(bit)        \
  SWCLK_CLR();          \
  SWCLK_SET();          

#define READ_BIT(bit)   \
  SWCLK_CLR();          \
  bit = SWDIO_IN();     \
  SWCLK_SET()          

#endif

      

void JTAG_to_SWD_Sequence(void);
void writeAP(int reg, uint32_t data);
void writeDP(int reg, uint32_t data);
void readAP(int reg, uint32_t *data);
void readDP(int reg, uint32_t *data);
uint32_t initDp(void);
uint32_t readApId(void);
void aapExtensionSequence(void);


#endif