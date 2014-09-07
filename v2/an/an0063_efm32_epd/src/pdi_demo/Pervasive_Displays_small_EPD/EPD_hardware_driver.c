/**
 * \file
 *
 * \brief The initialization and configuration of COG hardware driver
 *
 * Copyright (c) 2012-2013 Pervasive Displays Inc. All rights reserved.
 *
 *  Authors: Pervasive Displays Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <math.h>
#include "EPD_hardware_driver.h"

#define __Adcoffset			278
static volatile uint32_t EPD_Counter;

/**
 * \brief Set up EPD Timer for 1 mSec interrupts
 *
 * \note
 * desired value: 1mSec
 * actual value:  1.000mSec
 */
static void initialize_EPD_timer(void) {
	 /* Setup SysTick Timer for 1 msec interrupts  */
    if (SysTick_Config(SystemCoreClock / 1000)) while (1) ;
    EPD_Counter=	0;
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	EPD_Counter = 0;
}

/**
 * \brief Start Timer
 */
void start_EPD_timer(void) {
	initialize_EPD_timer();
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

/**
 * \brief Stop Timer
 */
void stop_EPD_timer(void) {
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
 * \brief Get current Timer after starting a new one
 */
uint32_t get_current_time_tick(void) {
	return EPD_Counter;
}

/**
 * \brief Interrupt Service Routine for system tick counter
 */
void SysTick_Handler(void) {
	EPD_Counter++;
}


/**
 * \brief Delay mini-seconds
 * \param ms The number of mini-seconds
 */
void delay_ms(unsigned int ms) {
	 uint32_t curTicks;
	 start_EPD_timer();
  	 curTicks = EPD_Counter;
  	 while ((EPD_Counter - curTicks) < ms) __WFI();	
	 stop_EPD_timer();
}

/**
 * \brief Delay mini-seconds
 * \param ms The number of mini-seconds
 */
void sys_delay_ms(unsigned int ms) {
	delay_ms(ms);
}

static void Wait_10us(void) {
	uint16_t i;
	for (i = 0; i < 4; i++){
	
	}
}

//******************************************************************
//* PWM  Configuration/Control //PWM output : PD3
//******************************************************************

/**
 * \brief The PWM signal starts toggling
 */
void PWM_start_toggle(void) {
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;

	/* Initalizing Timer0 */
	timerInit.enable = false;
	TIMER_Init(TIMER0, &timerInit);
	
	/* CC channel 2 */
	TIMER_InitCC_TypeDef timerInitCC = TIMER_INITCC_DEFAULT;
	timerInitCC.mode = timerCCModePWM;
	TIMER_InitCC(TIMER0, 2, &timerInitCC);	
	
	/* Pin PD3 is configured to Push-pull */
	GPIO_PinModeSet(gpioPortD, 3, gpioModePushPull, 1);
	/* Enable signal CC2 at location 3 */
	TIMER0->ROUTE |= TIMER_ROUTE_CC2PEN + TIMER_ROUTE_LOCATION_LOC3;
	
	/* Set duty cycle */
	TIMER0->TOP = PWMFREQ;
	TIMER0->CC[2].CCVB = PWMDUTY;
	
	TIMER0->CNT = 0;
	TIMER0->CMD = TIMER_CMD_START;
}

/**
 * \brief The PWM signal stops toggling.
 */
void PWM_stop_toggle(void) {
    TIMER0->CMD = TIMER_CMD_STOP;
}

/**
 * \brief PWM toggling.
 *
 * \param ms The interval of PWM toggling (mini seconds)
 */
void PWM_run(uint16_t ms) {
	PWM_start_toggle();
	 start_EPD_timer();
	while(ms>EPD_Counter)	{	
		__WFI();	 
	}
	PWM_stop_toggle();
	stop_EPD_timer();
}

//******************************************************************
//* SPI  Configuration
//******************************************************************

/**
 * \brief Configure SPI
 */
void spi_init(void) {
  USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

  /* MSB first, TX only */  
  init.msbf = true;
  init.enable = usartEnable;
  init.baudrate =SPI_baudrate;
 
   /* Enabling clock to USART */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_USART1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Use location 1: TX - Pin D0, RX - Pin D1 (no use), CLK - Pin D2 */
  config_gpio_dir_o(SPICLK_PORT,SPICLK_PIN);
  config_gpio_dir_o(SPIMOSI_PORT,SPIMOSI_PIN);
  config_gpio_dir_i(SPIMISO_PORT,SPIMISO_PIN);
 
  /* Ensure out of reset configuration */
  USART_Reset(SPI_device);

 /* Configure setting for basic sync operation */
  USART_InitSync(SPI_device, &init);
 /* Enabling pins and setting location, use manual control on CS */
  SPI_device->ROUTE =   USART_ROUTE_RXPEN | 
                        USART_ROUTE_TXPEN | 
                        USART_ROUTE_CLKPEN | 
                        USART_ROUTE_LOCATION_LOC1;
}

/**
 * \brief Initialize SPI
 */
void spi_attach(void) {
	if(SPI_device->ROUTE==0)spi_init();
	else USART_Enable(SPI_device,usartEnable);
}

/**
 * \brief Disable SPI and change to GPIO
 */
void spi_detach(void) {
	   USART_Enable(SPI_device,usartDisable);
	 SPI_device->ROUTE =0;
	 SPIMOSI_low();	
	 SPICLK_low();	 
	 SPIMISO_low();
}

static inline uint8_t SPI_RWData(uint8_t data)
{
      USART_Tx(SPI_device,data);
	return USART_Rx(SPI_device);
}
/**
 * \brief SPI synchronous write
 */
void SPI_write(unsigned char Data) {
	SPI_RWData(Data);
}
/**
 * \brief SPI synchronous read
 */
uint8_t SPI_read(unsigned char RDATA) {
	return SPI_RWData(RDATA);
}

/**
 * \brief Send data to SPI with time out feature
 *
 * \param data The data to be sent out
 */
uint8_t SPI_write_ex(unsigned char Data) {
	uint16_t cnt=50000;
	uint8_t flag=1;
    while (!(SPI_device->STATUS & USART_STATUS_TXBL)){
		if((cnt--)==0) break;
  	}
	 SPI_device->TXDATA = Data;
         cnt=50000;
    while (!(SPI_device->STATUS & USART_STATUS_RXDATAV)){
		if((cnt--)==0){
			flag=0;
			break;
		}
	 }		 
	  cnt=(uint8_t)SPI_device->RXDATA;
	 return flag;
}

#if (defined COG_V110_G2) || (defined COG_V230)
/**
* \brief SPI command
*
* \param register_index The Register Index as SPI Data to COG
* \param register_data The Register Data for sending command data to COG
* \return the SPI read value
*/
uint8_t SPI_R(uint8_t Register, uint8_t Data) {
	uint8_t result;
	EPD_cs_low ();
	SPI_write (0x70); // header of Register Index
	SPI_write (Register);

	EPD_cs_high ();
	Wait_10us ();
	EPD_cs_low ();

	SPI_write (0x73); // header of Register Data of read command
	result=SPI_read (Data);

	EPD_cs_high ();

	return result;
}
#endif

/**
* \brief SPI command if register data is larger than two bytes
*
* \param register_index The Register Index as SPI command to COG
* \param register_data The Register Data for sending command data to COG
* \param length The number of bytes of Register Data which depends on which
* Register Index is selected.
*/
void SPI_send (unsigned char register_index, unsigned char *register_data,
               unsigned length) {
	EPD_cs_low ();
	SPI_write (0x70); // header of Register Index
	SPI_write (register_index);

	EPD_cs_high ();
	Wait_10us ();
	EPD_cs_low ();

	SPI_write (0x72); // header of Register Data of write command
	while(length--) {
		SPI_write (*register_data++);
	}
	EPD_cs_high ();
}

/**
* \brief SPI command
*
* \param register_index The Register Index as SPI command to COG
* \param register_data The Register Data for sending command data to COG
*/
void SPI_send_byte (uint8_t register_index, uint8_t register_data) {
	EPD_cs_low ();
	SPI_write (0x70); // header of Register Index
	SPI_write (register_index);

	EPD_cs_high ();
	Wait_10us ();
	EPD_cs_low ();
	SPI_write (0x72); // header of Register Data
	SPI_write (register_data);
	EPD_cs_high ();
}

//******************************************************************
//* Temperature sensor  Configuration
//******************************************************************

#ifdef __Internal_Temperature_Sensor
/**
 * \brief Get temperature value from ADC
 *
 * \return the Celsius temperature
 */
int16_t get_temperature(void) {

	return (int8_t)25;
}
#elif defined __External_Temperature_Sensor

/**
 * \brief Get temperature value from ADC
 *
 * \return the Celsius temperature
 */
int16_t get_temperature(void) {
   uint8_t i;
   long tmp=0;
   float IntDegC=0.0;
   const float DegCOffset=0.0;
/* Enable clocks required */
  //CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_ADC0, true);
  
  ADC_Init_TypeDef		 init		= ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
  
  /* Init common settings for both single conversion and scan mode */
  init.timebase = ADC_TimebaseCalc(0);
  /* Might as well finish conversion as quickly as possibly since polling */
  /* for completion. */
  /* Set ADC clock to 200kHz, use default HFPERCLK */
  init.prescale = ADC_PrescaleCalc(ADC_Convert_Speed, 0);
  
  /* WARMUPMODE must be set to Normal according to ref manual before */
  /* entering EM2. In this example, the warmup time is not a big problem */
  /* due to relatively infrequent polling. Leave at default NORMAL, */
  
  ADC_Init(ADC0, &init);
  
  /* Init for single conversion use, 2.5 reference. */
  singleInit.reference	= ADC_Reference;
  singleInit.input		= ADC_Input;
  singleInit.resolution = ADC_Resolution;
  
 
  /* 32 cycles should be safe for all ADC clock frequencies */
  singleInit.acqTime = adcAcqTime64;
  
  ADC_InitSingle(ADC0, &singleInit);
  
   for(i=0;i<2;i++){   
	   /* Start ADC convert by Software trigger */
	   ADC_Start(ADC0, adcStartSingle);
	   /* Wait while conversion is active */
	   while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;  
	   /* Get ADC result */
	   tmp += ADC_DataSingleGet(ADC0);
   }
   tmp=0;
  for(i=0;i<8;i++){   
	  /* Start ADC convert by Software trigger */
	  ADC_Start(ADC0, adcStartSingle);
	  /* Wait while conversion is active */
	  while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;  
	  /* Get ADC result */
	  tmp += ADC_DataSingleGet(ADC0);
  }
   tmp=tmp/8;
   CMU_ClockEnable(cmuClock_ADC0, false);
      
  // IntDegC=(float)(211.3-DegCOffset)-(float)((tmp/8)+(tmp/64)+(tmp/1024)));//100-((Temperature voltage-1.199)*1000)/10.77=IntDegC
   IntDegC=(203.7-DegCOffset)-(float)((tmp/8)+(tmp/128)+(tmp/256)+(tmp/2048));//100-((Temperature voltage-1.145)*1000)/11.04=IntDegC  
 
   return   (int16_t)IntDegC;
}

#endif

/**
 * \brief Initialize the temperature sensor
 */
void initialize_temperature(void){

}

/**
 * \brief Initialize the EPD hardware setting
 */
void EPD_display_hardware_init(void) {
	EPD_initialize_gpio();
	EPD_Vcc_turn_off();
	spi_init();
	initialize_temperature();
	EPD_cs_low();
	EPD_pwm_low();
	EPD_rst_low();
	EPD_discharge_low();
	EPD_border_low();
	//initialize_EPD_timer();
}

