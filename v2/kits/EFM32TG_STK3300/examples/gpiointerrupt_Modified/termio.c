#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include "termio.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_int.h"

/* Receive buffer */
#define RXBUFSIZE    8                          /**< Buffer size for RX */
static volatile int     rxReadIndex  = 0;       /**< Index in buffer to be read */
static volatile int     rxWriteIndex = 0;       /**< Index in buffer to be written to */
static volatile int     rxCount      = 0;       /**< Keeps track of the num bytes stored in buffer */
static volatile uint8_t rxBuffer[RXBUFSIZE];    /**< Buffer to store data */
static uint8_t          LFtoCRLF    = 0;        /**< LF to CRLF conversion disabled */
static bool             initialized = false;    /**< Initialize UART/LEUART */

int dprintf(const char *fmt, ...)
{
#ifdef DEBUG
    va_list args;
    int count = 0;
    
    va_start(args, fmt);
    count = vprintf(fmt, args);
    va_end(args);
    return count;
#else
    return 0;
#endif
}

uint8_t TERMIO_GetChar(void) {
    return RETARGET_RX(RETARGET_UART);
}

void TERMIO_PutChar(uint8_t ch) {
	/* sends a character to the terminal channel */
	RETARGET_TX(RETARGET_UART, ch);
}

/**************************************************************************//**
 * @brief Receive a byte from USART/LEUART and put into global buffer
 * @return -1 on failure, or positive character integer on sucesss
 *****************************************************************************/
int RETARGET_ReadChar(void)
{
  int c = -1;

  if (initialized == false)
  {
    TERMIO_Init();
  }

  INT_Disable();
  if (rxCount > 0)
  {
    c = rxBuffer[rxReadIndex];
    rxReadIndex++;
    if (rxReadIndex == RXBUFSIZE)
    {
      rxReadIndex = 0;
    }
    rxCount--;
  }
  INT_Enable();

  return c;
}

/**************************************************************************//**
 * @brief Transmit single byte to USART/LEUART
 * @param c Character to transmit
 * @return Transmitted character
 *****************************************************************************/
uint8_t RETARGET_WriteChar(uint8_t c)
{
  if (initialized == false)
  {
    TERMIO_Init();
  }

  /* Add CR or LF to CRLF if enabled */
  if (LFtoCRLF && (c == '\n'))
  {
    RETARGET_TX(RETARGET_UART, '\r');
  }
  RETARGET_TX(RETARGET_UART, c);

  if (LFtoCRLF && (c == '\r'))
  {
    RETARGET_TX(RETARGET_UART, '\n');
  }

  return c;
}

int TERMIO_TxBuf(uint8_t *buffer, int nbytes)
{
  int i;

  for (i = 0; i < nbytes; i++) {
    RETARGET_WriteChar(*buffer++);
  }
  return nbytes;
}

void TERMIO_Init(void) {

  /* Configure GPIO pins */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* To avoid false start, configure output as high */
  GPIO_PinModeSet(RETARGET_TXPORT, RETARGET_TXPIN, gpioModePushPull, 1);
  GPIO_PinModeSet(RETARGET_RXPORT, RETARGET_RXPIN, gpioModeInput, 0);
  
 #if defined(RETARGET_UART)
  USART_TypeDef *usart = RETARGET_UART;
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

  /* Enable peripheral clocks */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(RETARGET_CLK, true);

  /* Configure USART for basic async operation */
  init.enable = usartDisable;
  USART_InitAsync(usart, &init);

  /* Enable pins at correct location */
  usart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | RETARGET_LOCATION;
  
  /*
  // Clear previous RX interrupts 
  USART_IntClear(RETARGET_UART, USART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(RETARGET_IRQn);

  // Enable RX interrupts
  USART_IntEnable(RETARGET_UART, USART_IF_RXDATAV);
  NVIC_EnableIRQ(RETARGET_IRQn);
  */
  
  /* Finally enable it */
  USART_Enable(usart, usartEnable);
  
#elif defined(RETARGET_LEUART)
  LEUART_TypeDef *leuart = RETARGET_UART;
  LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;

  /* Enable CORE LE clock in order to access LE modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Select LFXO for LEUARTs (and wait for it to stabilize) */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

  CMU_ClockEnable(RETARGET_CLK, true);

  /* Do not prescale clock */
  CMU_ClockDivSet(RETARGET_CLK, cmuClkDiv_1);

  /* Configure LEUART to 9600 baud*/
  init.enable = leuartDisable;
  LEUART_Init(leuart, &init);
  /* Enable pins at default location */
  leuart->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | RETARGET_LOCATION;

  /*
  // Clear previous RX interrupts 
  LEUART_IntClear(RETARGET_UART, LEUART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(RETARGET_IRQn);

  // Enable RX interrupts
  LEUART_IntEnable(RETARGET_UART, LEUART_IF_RXDATAV);
  NVIC_EnableIRQ(RETARGET_IRQn);
  */
  
  /* Finally enable it */
  LEUART_Enable(leuart, leuartEnable);  
#endif
  
  initialized = true;
  TERMIO_SerialCrLf(1);

}

// void RETARGET_SerialInit

int kbhit(void) {
	/* checks for a character from the terminal channel */
	if(RETARGET_UART->STATUS & USART_STATUS_RXDATAV)
		return 1;
	else
		return 0;
}


/**************************************************************************//**
 * @brief UART/LEUART IRQ Handler
 *****************************************************************************/
void RETARGET_IRQ_NAME(void)
{
#if defined(RETARGET_UART)
  if (RETARGET_UART->STATUS & USART_STATUS_RXDATAV)
  {
#else
  if (RETARGET_UART->IF & LEUART_IF_RXDATAV)
  {
#endif

    /* Store Data */
    rxBuffer[rxWriteIndex] = RETARGET_RX(RETARGET_UART);
    rxWriteIndex++;
    rxCount++;
    if (rxWriteIndex == RXBUFSIZE)
    {
      rxWriteIndex = 0;
    }
    /* Check for overflow - flush buffer */
    if (rxCount > RXBUFSIZE)
    {
      rxWriteIndex = 0;
      rxCount      = 0;
      rxReadIndex  = 0;
    }
  }
}

/**************************************************************************//**
 * @brief UART/LEUART toggle LF to CRLF conversion
 * @param on If non-zero, automatic LF to CRLF conversion will be enabled
 *****************************************************************************/
void TERMIO_SerialCrLf(int on)
{
  if (on)
    LFtoCRLF = 1;
  else
    LFtoCRLF = 0;
}


#if defined(ccs)

#include <file.h>

#define PORT_NUM			0
#define UART_BASE			UART0_BASE

int uart_open(const char *path, unsigned flags, int llv_fd)
{
	return llv_fd;
}

int uart_close(int dev_fd)
{
	return dev_fd;
}

off_t uart_lseek(int dev_fd, off_t offset, int origin)
{
	return -1;
}

int uart_unlink(const char *path)
{
	return -1;
}

int uart_rename(const char * old_name, const char * new_name)
{
	return -1;
}


int uart_write(int dev_fd, const char *buf, unsigned count)
{
	int retVal = count;
	const char *pch = buf;
	if (buf == NULL)
		return -1;
	while(count) {
		UARTCharPut(UART_BASE, *pch++);
		count--;
//		if (UARTCharsAvail(UART_BASE)) {
//			UARTCharPutNonBlocking(UART_BASE, *pch++);
//			count--;
//		}
	}
	return retVal;
}

int uart_read(int dev_fd, char *buf, unsigned count)
{
	if (!count)
		return 0;
	if (buf == NULL)
		return -1;
	char *pch = buf;
	unsigned numRead = 0;
	int retVal = 0;
	while (count) {
		retVal = UARTCharGet(UART_BASE);
		if (retVal == -1)
			return 0;
		*pch++ = retVal;
		numRead++;
		count--;
//		if (UARTCharsAvail(UART_BASE)) {
//			retVal = UARTCharGetNonBlocking(UART_BASE);
//			// if character available is EOF, return 0
//			if (retVal == -1)
//				return 0;
//			*pch++ = retVal;
//			numRead++;
//		}
	}

	return numRead;
}



int mapStdioToUart(void)
{
	// _MSA indicates device supports multiple streams open at a time
	// _SSA indicates device supports only one open stream at a time
	int retVal = add_device("uart", _MSA, uart_open, uart_close, uart_read, uart_write,
				uart_lseek, uart_unlink, uart_rename);
	FILE *fid = fopen("uart", "w");

	if (fid == NULL) {
		puts(" Failed top open uart for C I/O.");
		return -1;
	}

	// Reopen stdout as a uart file
	if (!freopen("uart:", "w", stdout)) {
		puts("Failed to freopen stdout");
		return -2;
	}

	// Turn off buffering for stdout by setting to 0
	// TIRTOS uses line buffering IOLBF (typical for stdout) at 128
    //freopen("UART:0", "w", stdout);
    //setvbuf(stdout, NULL, _IOLBF, 128);
	if (setvbuf(stdout, NULL, _IONBF, 0)) {
		puts("Failed to setvbuf stdout");
		return -3;
	}

	printf("This goes to stdout\n\r");
	puts("puts test to stdout");

	if (!freopen("uart:", "r", stdin)) {
		puts("Failed to freopen stdout");
		return -4;
	}

	if (setvbuf(stdin, NULL, _IONBF, 0)) {
		puts("Failed to setvbuf stdout");
		return -5;
	}

	return 0;
}

#endif

#if defined(__CC_ARM)

#include <stdio.h>

/* #pragma import(__use_no_semihosting_swi) */

struct __FILE
{
  int handle;
};

/**Standard output stream*/
FILE __stdout;

/**************************************************************************//**
 * @brief
 *  Writes character to file
 *
 * @param[in] f
 *  File
 *
 * @param[in] ch
 *  Character
 *
 * @return
 *  Written character
 *****************************************************************************/
int fputc(int ch, FILE *f)
{
  return(TERMIO_WriteChar(ch));
}

/**************************************************************************//**
 * @brief
 *  Reads character from file
 *
 * @param[in] f
 *  File
 *
 * @return
 *  Character
 *****************************************************************************/
int fgetc(FILE *f)
{
  return(TERMIO_ReadChar());
}

/**************************************************************************//**
 * @brief
 *  Tests the error indicator for the stream pointed
 *  to by file
 *
 * @param[in] f
 *  File
 *
 * @return
 *  Returns non-zero if it is set
 *****************************************************************************/
int ferror(FILE *f)
{
  /* Your implementation of ferror */
  return EOF;
}

/**************************************************************************//**
 * @brief
 *  Writes a character to the console
 *
 * @param[in] ch
 *  Character
 *****************************************************************************/
void _ttywrch(int ch)
{
  TERMIO_WriteChar(ch);
}

/**************************************************************************//**
 * @brief
 *  Library exit function. This function is called if stack
 *  overflow occurs.
 *
 * @param[in] return_code
 *  Return code
 *****************************************************************************/
void _sys_exit(int return_code)
{
  while (1)
    ;   /* forever loop */
}
#endif /* defined( __CC_ARM ) */
