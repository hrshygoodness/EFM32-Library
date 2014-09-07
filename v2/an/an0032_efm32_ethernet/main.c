/**************************************************************************//**
 * @file main.c
 * @brief lwIP web server demo for EFM32_Gxxx_STK
 * @author Silicon Labs
 * @version 1.09
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

#include <string.h>
#include <stdlib.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_assert.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_lcd.h"
#include "em_rtc.h"
#include "em_system.h"
#include "segmentlcd.h"
#include "math.h"
#include "lwip/memp.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/netifapi.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/tcpip.h"
#include "lwip/sockets.h"
#include "lwip/mem.h"
#include "lwip/init.h"
#include "lwip/dhcp.h"
#include "lwip/autoip.h"
#include "netif/etharp.h"
#include "spi.h"
#include "netif/AX88796C.h"
#include "netif/AX88796C_if.h"

#define IPADDR_USE_STATIC    0
#define IPADDR_USE_DHCP      1
#define IPADDR_USE_AUTOIP    2


#define IPADDR(a, b, c, d)    ((a << 24) | (b << 16) | (c << 8) | d)

/* ---------------------------------------------------- */
/* Definitions and variables to write IP on the STK LCD */
/* ---------------------------------------------------- */
#define IP_ADDR_FIRST_TEXT     "X1 1 1 "
#define IP_ADDR_SECOND_TEXT    " 1X1 1 "
#define IP_ADDR_THIRD_TEXT     " 1 1X1 "
#define IP_ADDR_FOURTH_TEXT    " 1 1 1X"
#define RTC_FREQUENCY          32768
#define RTC_DELAY_SECONDS      3
#define RTC_COMP_VALUE         RTC_FREQUENCY * RTC_DELAY_SECONDS
#define RTC_COMP               0

/* MACROS for masking and shifting the IP values to the correct place */
#define IP_ADDR_FIRST_NUM(x)     ((x) & 0x000000FF)
#define IP_ADDR_SECOND_NUM(x)    (((x) & 0x0000FF00) >> 8)
#define IP_ADDR_THIRD_NUM(x)     (((x) & 0x00FF0000) >> 16)
#define IP_ADDR_FOURTH_NUM(x)    (((x) & 0xFF000000) >> 24)

static uint8_t ip_field     = 1;     /*  Start writing the first IP field */
static bool    updateIpFlag = false; /* Flag to indicate when LCD should be updated */
/* ---------------------------------------------------- */
/* ---------------------------------------------------- */

extern void httpd_init(void);
extern unsigned long tick_count;


uint32_t temperature, read_temperature = 1;

/* Function prototypes */
void lwIPInit(const unsigned char *pucMAC, unsigned long ulIPAddr,
              unsigned long ulNetMask, unsigned long ulGWAddr, unsigned long ulIPMode);
void setupSensor(void);
float convertToCelsius(uint32_t adcSample);
void gpioSetup(void);

/**************************************************************************//**
 * The lwIP network interface structure for the AX88796C Ethernet MAC.
 *****************************************************************************/
struct netif   lwip_netif;
struct netif   *ethif;
struct ip_addr ipaddr, netmask, gw;
struct pbuf    *p = NULL, *q = NULL;

/**************************************************************************//**
 * The local time for the lwIP Library Abstraction layer, used to support the
 * Host and lwIP periodic callback functions.
 *****************************************************************************/
static unsigned long g_ulLocalTimer = 0;

/**************************************************************************//**
 * The local time when the TCP timer was last serviced.
 *****************************************************************************/
static unsigned long g_ulTCPTimer = 0;

/**************************************************************************//**
 * The local time when the HOST timer was last serviced.
 *****************************************************************************/
#if HOST_TMR_INTERVAL
static unsigned long g_ulHostTimer = 0;
#endif

/**************************************************************************//**
 * The local time when the ARP timer was last serviced.
 *****************************************************************************/
#if LWIP_ARP
static unsigned long g_ulARPTimer = 0;
#endif

/**************************************************************************//**
 * The local time when the AutoIP timer was last serviced.
 *****************************************************************************/
#if LWIP_AUTOIP
static unsigned long g_ulAutoIPTimer = 0;
#endif

/**************************************************************************//**
 *The local time when the DHCP Coarse timer was last serviced.
 *****************************************************************************/
#if LWIP_DHCP
static unsigned long g_ulDHCPCoarseTimer = 0;
#endif

/**************************************************************************//**
 *The local time when the DHCP Fine timer was last serviced.
 *****************************************************************************/
#if LWIP_DHCP
static unsigned long g_ulDHCPFineTimer = 0;
#endif

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter.
 *****************************************************************************/
void SysTick_Handler(void)
{
  ++g_ulLocalTimer;
  ++tick_count;

  /*Service the host timer.*/
#if HOST_TMR_INTERVAL
  if ((g_ulLocalTimer - g_ulHostTimer) >= HOST_TMR_INTERVAL)
  {
    g_ulHostTimer = g_ulLocalTimer;
  }
#endif

  /* Service the ARP timer.*/
#if LWIP_ARP
  if ((g_ulLocalTimer - g_ulARPTimer) >= ARP_TMR_INTERVAL)
  {
    g_ulARPTimer = g_ulLocalTimer;
    etharp_tmr();
  }
#endif

  /* Service the TCP timer.*/
  if ((g_ulLocalTimer - g_ulTCPTimer) >= TCP_TMR_INTERVAL)
  {
    g_ulTCPTimer = g_ulLocalTimer;
    tcp_tmr();
  }

  /* Service the AutoIP timer.*/
#if LWIP_AUTOIP
  if ((g_ulLocalTimer - g_ulAutoIPTimer) >= AUTOIP_TMR_INTERVAL)
  {
    g_ulAutoIPTimer = g_ulLocalTimer;
    autoip_tmr();
  }
#endif

  /* Service the DCHP Coarse Timer. */
#if LWIP_DHCP
  if ((g_ulLocalTimer - g_ulDHCPCoarseTimer) >= DHCP_COARSE_TIMER_MSECS)
  {
    g_ulDHCPCoarseTimer = g_ulLocalTimer;
    dhcp_coarse_tmr();
  }
#endif

  /* Service the DCHP Fine Timer.*/
#if LWIP_DHCP
  if ((g_ulLocalTimer - g_ulDHCPFineTimer) >= DHCP_FINE_TIMER_MSECS)
  {
    g_ulDHCPFineTimer = g_ulLocalTimer;
    dhcp_fine_tmr();
  }
#endif
}

/**************************************************************************//**
 * @brief Initializes the lwIP TCP/IP stack.
 * \param pucMAC is a pointer to a six byte array containing the MAC
 * address to be used for the interface.
 * \param ulIPAddr is the IP address to be used (static).
 * \param ulNetMask is the network mask to be used (static).
 * \param ulGWAddr is the Gateway address to be used (static).
 * \param ulIPMode is the IP Address Mode.  \b IPADDR_USE_STATIC will force
 * static IP addressing to be used, \b IPADDR_USE_DHCP will force DHCP with
 * fallback to Link Local (Auto IP), while \b IPADDR_USE_AUTOIP will force
 * Link Local only.
 *
 * This function performs initialization of the lwIP TCP/IP stack for the
 * AX88796 Ethernet MAC, including DHCP and/or AutoIP, as configured.
 * \return None.
 *****************************************************************************/
void
lwIPInit(const unsigned char *pucMAC, unsigned long ulIPAddr,
         unsigned long ulNetMask, unsigned long ulGWAddr,
         unsigned long ulIPMode)
{
  (void) pucMAC;

  struct ip_addr ip_addr;
  struct ip_addr net_mask;
  struct ip_addr gw_addr;


/* Check the parameters.*/
#if LWIP_DHCP && LWIP_AUTOIP
  EFM_ASSERT((ulIPMode == IPADDR_USE_STATIC) ||
             (ulIPMode == IPADDR_USE_DHCP) ||
             (ulIPMode == IPADDR_USE_AUTOIP));
#elif LWIP_DHCP
  EFM_ASSERT((ulIPMode == IPADDR_USE_STATIC) ||
             (ulIPMode == IPADDR_USE_DHCP));
#elif LWIP_AUTOIP
  EFM_ASSERT((ulIPMode == IPADDR_USE_STATIC) ||
             (ulIPMode == IPADDR_USE_AUTOIP));
#else
  EFM_ASSERT(ulIPMode == IPADDR_USE_STATIC);
#endif

  /* Initialize lwIP library modules */
  lwip_init();

  /* Setup the network address values.*/
  if (ulIPMode == IPADDR_USE_STATIC)
  {
    ip_addr.addr  = htonl(ulIPAddr);
    net_mask.addr = htonl(ulNetMask);
    gw_addr.addr  = htonl(ulGWAddr);
  }
#if LWIP_DHCP || LWIP_AUTOIP
  else
  {
    ip_addr.addr  = 0;
    net_mask.addr = 0;
    gw_addr.addr  = 0;
  }
#endif

  /* Create, configure and add the Ethernet controller interface with
   * default settings.*/
  netif_add(&lwip_netif, &ip_addr, &net_mask, &gw_addr, NULL,
            AX88796Cif_init, ip_input);

  netif_set_default(&lwip_netif);


  /* Start DHCP, if enabled.*/
#if LWIP_DHCP
  if (ulIPMode == IPADDR_USE_DHCP)
  {
    dhcp_start(&lwip_netif);
  }
#endif

  /* Start AutoIP, if enabled and DHCP is not.*/
#if LWIP_AUTOIP
  if (ulIPMode == IPADDR_USE_AUTOIP)
  {
    autoip_start(&lwip_netif);
  }
#endif

  /* Bring the interface up.*/
  netif_set_up(&lwip_netif);
}

/**************************************************************************//**
 * @brief ADC0 interrupt handler. Simply clears interrupt flag.
 *****************************************************************************/
void ADC0_IRQHandler(void)
{
  ADC_IntClear(ADC0, ADC_IF_SINGLE);
  /* Set temperature flag */
  read_temperature = 1;
}

/**************************************************************************//**
 * @brief Initialize ADC for temperature sensor readings in single poin
 *****************************************************************************/
void setupSensor(void)
{
  /* Base the ADC configuration on the default setup. */
  ADC_Init_TypeDef       init  = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;

  /* Initialize timebases */
  init.timebase = ADC_TimebaseCalc(0);
  init.prescale = ADC_PrescaleCalc(400000, 0);
  ADC_Init(ADC0, &init);

  /* Set input to temperature sensor. Reference must be 1.25V */
  sInit.reference = adcRef1V25;
  sInit.input     = adcSingleInpTemp;
  ADC_InitSingle(ADC0, &sInit);

  /* Setup interrupt generation on completed conversion. */
  ADC_IntEnable(ADC0, ADC_IF_SINGLE);
  NVIC_EnableIRQ(ADC0_IRQn);
}

/**************************************************************************//**
 * @brief Convert ADC sample values to celsius.
 * @note See section 2.3.4 in the reference manual for details on this
 *       calculatoin
 * @param adcSample Raw value from ADC to be converted to celsius
 * @return The temperature in degrees Celsius.
 *****************************************************************************/
float convertToCelsius(uint32_t adcSample)
{
  float temp;
  /* Factory calibration temperature from device information page. */
  float cal_temp_0 = (float)((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
                             >> _DEVINFO_CAL_TEMP_SHIFT);
  /* Factory calibration value from device information page. */

  float cal_value_0 = (float)((DEVINFO->ADC0CAL2 & 0xFFF00000) >> 20);


  /* Temperature gradient (from datasheet) */
  float t_grad = -3.85;

  temp = (cal_temp_0 - ((cal_value_0 - adcSample) / t_grad));

  return temp;
}

/**************************************************************************//**
 * @brief Setup GPIO interrupt to change temp. display
 *****************************************************************************/
void gpioSetup(void)
{
  /* Enable GPIO clock */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

  /* Configure PB12 as input and enable interrupt  */
  GPIO_PinModeSet(gpioPortB, 12, gpioModeInputPull, 1);
  GPIO_IntConfig(gpioPortB, 12, false, true, true);

  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

/**************************************************************************//**
 * @brief GPIO Interrupt handler (PB10) Fahrenheit
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Acknowledge interrupt */
  GPIO_IntClear(1 << 12);

  evn_router_interrupt0_handler();
}


/***************************************************************************//**
 * @brief RTC Interrupt Handler
 ******************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear COMP0 interrupt */
  RTC_IntClear(0x2);

  updateIpFlag = true;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  unsigned char pucMACArray[8];
  uint32_t      temp;

  /* Chip revision alignment and errata fixes */
  CHIP_Init();

  /* Ensure core frequency has been updated */
  SystemCoreClockUpdate();
  CMU_ClockEnable(cmuClock_ADC0, true);

  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(SystemCoreClock / 1000)) while (1) ;

  /* Setup ADC for sampling internal temperature sensor. */
  setupSensor();

  /* Spi init*/
  spiInit();


#if LWIP_DHCP
  /* Initialze the lwIP library, using DHCP.*/
  lwIPInit(pucMACArray, 0, 0, 0, IPADDR_USE_DHCP);
#else
  /* Initialze the lwIP library, using Static IP.*/
  lwIPInit(pucMACArray, IPADDR(192, 168, 79, 160), IPADDR(255, 255, 255, 0), \
           IPADDR(192, 168, 79, 1), IPADDR_USE_STATIC);
#endif

  /* Initialize a sample httpd server.*/
  httpd_init();

  /* Start one ADC sample */
  ADC_Start(ADC0, adcStartSingle);

  /* Enable board control interrupts */
  gpioSetup();

  axspi_write_reg(~IMR_RXPKT, P0_IMR);

  /* Start LCD  without boost */
  SegmentLCD_Init(false);

  CMU_ClockEnable(cmuClock_RTC, true);
  /* RTC configuration structure */
  RTC_Init_TypeDef rtcInit = {
    .enable   = false,
    .debugRun = false,
    .comp0Top = true
  };

  /* Initialize RTC */
  RTC_Init(&rtcInit);

  /* Set COMP0 value which will be the top value as well */
  RTC_CompareSet(RTC_COMP, RTC_COMP_VALUE);

  /* Clear all pending interrupts */
  RTC_IntClear(0x7);

  /* Enable COMP0 interrupts */
  RTC_IntEnable(0x2);

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(RTC_IRQn);
  NVIC_EnableIRQ(RTC_IRQn);

  RTC_Enable(true);

  while (1)
  {
    /* check temperature flag*/
    if (read_temperature)
    {
      temp = ADC_DataSingleGet(ADC0);

      /* Show Celsius on numeric part of display */
      temperature = (int)(convertToCelsius(temp));

      /* Start a new conversion */
      ADC_Start(ADC0, adcStartSingle);

      /* Reset temperature flag */
      read_temperature = 0;
    }

    if (updateIpFlag)
    {
      switch (ip_field)
      {
      case 1:
      {
        SegmentLCD_Write(IP_ADDR_FIRST_TEXT);
        SegmentLCD_Number(IP_ADDR_FIRST_NUM(lwip_netif.ip_addr.addr));
        ip_field++;
      }
      break;

      case 2:
      {
        SegmentLCD_Write(IP_ADDR_SECOND_TEXT);
        SegmentLCD_Number(IP_ADDR_SECOND_NUM(lwip_netif.ip_addr.addr));
        ip_field++;
      }
      break;

      case 3:
      {
        SegmentLCD_Write(IP_ADDR_THIRD_TEXT);
        SegmentLCD_Number(IP_ADDR_THIRD_NUM(lwip_netif.ip_addr.addr));
        ip_field++;
      }
      break;

      case 4:
      {
        SegmentLCD_Write(IP_ADDR_FOURTH_TEXT);
        SegmentLCD_Number(IP_ADDR_FOURTH_NUM(lwip_netif.ip_addr.addr));
        ip_field = 1;
      }
      break;

      default: break;
      }

      updateIpFlag = false;
    }
  }
}


