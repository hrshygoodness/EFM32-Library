/*! @file main.c
 * @brief The main.c file of the Fixed packet length Custom RX demo
 * for Si446X devices.
 *
 * Contains the initialization of the MCU & the radio.
 * @n The main loop controls the program flow & the radio.
 *
 * @b CREATED
 * @n Silicon Laboratories Ltd.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 *
 */

#include "bsp_def.h"


/*------------------------------------------------------------------------*/
/*                          Global variables                              */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/*                              Defines                                   */
/*------------------------------------------------------------------------*/

// COMMAND_HEADER = | Command | PROPERTY GROUP | LENGTH | PROPERTY NUMBER | 
#define COMMAND_HEADER_LENGTH 0x04

#define ERROR_HOOK                while (TRUE) \
    { \
  for (rollingCounter = 0u; \
  rollingCounter < 0xFFFF; \
  rollingCounter++)  ; \
  BSP_LedToggle(0); \
    }
/*------------------------------------------------------------------------*/
/*                             Enumeration                                */
/*------------------------------------------------------------------------*/
enum
{
  SEARCH_FIELD_ERROR,
  SEARCH_FIELD_SUCCESS
};
/*------------------------------------------------------------------------*/
/*                             Typedefs                                   */
/*------------------------------------------------------------------------*/
typedef struct
{
    // Length of FIELD [1..5]
    uint16_t fieldLength[5];
    //  Length Field in the packet
    uint8_t  fieldContainsLength;
    //  Variable Field in the packet
    uint8_t  fieldVaryInLength;
    //  Length byte is skipped or not
    uint8_t  lengthStoredInFifo;
    //  Length Field is one or two bytes
    uint8_t  lengthFieldInByte;
    // Length endian
    uint8_t lengthEndian;

} tSearchFieldParameters;

typedef enum
{
  ANALYZE_STATE_IS_LENGTH_FIELD,
  ANALYZE_STATE_IS_LENGTH_FIELD_INCLUDED,
  ANALYZE_STATE_IS_VARIABLE_FIELD,
  ANALYZE_STATE_IS_DATA_REMAINING,
  ANALYZE_STATE_WRONG_PACKET
}eAnalyzeFieldState;

/*------------------------------------------------------------------------*/
/*                          Local variables                               */
/*------------------------------------------------------------------------*/
uint16_t rollingCounter;
tSearchFieldParameters fieldParameters;
/*------------------------------------------------------------------------*/
/*                      Local function prototypes                         */
/*------------------------------------------------------------------------*/
void eADesigner_Init(void);
void DemoApp_Pollhandler (void);
uint8_t  SearchFieldProperties(const uint8_t* pSetPropCmd,tSearchFieldParameters* fieldParams);
uint8_t  gSampleCode_StringCompare(uint8_t* pbiPacketContent, uint8_t* pbiString, uint8_t lenght);
uint8_t  gSampleCode_StringCompareVariable(uint8_t* pbiPacketContent, uint8_t* pbiString, uint8_t lenght, tSearchFieldParameters* fieldParams);

/*------------------------------------------------------------------------*/
/*                          Function implementations                      */
/*------------------------------------------------------------------------*/

/** \fn void main(void)
 * \brief The main function of the demo.
 *
 * \todo Create description
 */
int main(void)
{
  uint32_t tick;

  /* Chip errata */
  CHIP_Init();

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  // Initialize the MCU peripherals
  eADesigner_Init();
  
  /* Setup SysTick Timer for 1 msec interrupts  */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;

  // Start the push button handler
  vHmi_InitPbHandler();

  // Start the Led handler
  vHmi_InitLedHandler();
  
  /* Initialize LED driver */
  BSP_LedsInit();
  BSP_LedSet(0);
#ifdef EFM32LG990F256
  BSP_LedSet(1);
#endif
  
  // Initialize the Radio
  vRadio_Init();  

  //Clear the Led to show that the initlization of the radio is finished.
  BSP_LedClear(0);
#ifdef EFM32LG990F256
  BSP_LedClear(1);
#endif
  
  // Search Packet handler Fields parameters
  if( SEARCH_FIELD_ERROR == SearchFieldProperties(pRadioConfiguration->Radio_ConfigurationArray, &fieldParameters))
  {
    ERROR_HOOK;
  } 	

  // Start RX
  vRadio_StartRX(pRadioConfiguration->Radio_ChannelNumber);  
  tick = SysTick_GetTick();

  while (TRUE)
  {
    if (tick != SysTick_GetTick())
    {
      tick = SysTick_GetTick();

      vHmi_PbHandler();
      vHmi_LedHandler();
    }

    // Demo Application Poll-Handler function
    DemoApp_Pollhandler();
  }
}

/**
 *  Demo Application Poll-Handler
 *
 *  @note This function must be called periodically.
 *
 */
void DemoApp_Pollhandler()
{
  // Check if radio packet received
  if (TRUE == gRadio_CheckReceived_VariablePacket())
  {
    switch (fieldParameters.fieldVaryInLength)
    {
      case 0x00:

        // Check if  the radio packet does not contain the variable custom payload
        if (gSampleCode_StringCompare((uint8_t *) &fixRadioPacket[0u], pRadioConfiguration->Radio_CustomPayload,
            pRadioConfiguration->Radio_PacketLength) == TRUE )
        {
#ifdef EFM32LG990F256
          /* There is not enough LED on EFM32LG-STK3600 kit, only two LEDs */ 
          vHmi_ChangeLedState(eHmi_Led2_c, eHmi_LedBlinkOnce_c);
#endif
          
#ifdef EFM32TG840F32
          /* There is not enough LED on EFM32TG-STK3300 kit, only one LED */ 
          vHmi_ChangeLedState(eHmi_Led1_c, eHmi_LedBlinkOnce_c);
#endif
        }

        break;

      case 0x02:
      case 0x03:
      case 0x04:
      case 0x05:

        // Check if the radio packet contains the variable custom payload
        if (gSampleCode_StringCompareVariable((uint8_t *) &fixRadioPacket[0u], pRadioConfiguration->Radio_CustomPayload,
            pRadioConfiguration->Radio_PacketLength, &fieldParameters) == TRUE)
        {
          /* There is not enough LED on EFM32LG-STK3600 kit, only two LEDs */ 
          vHmi_ChangeLedState(eHmi_Led2_c, eHmi_LedBlinkOnce_c);
        }

        break;

      default:
        break;
    }
  }
}


/*!
 * This function is used to search for the lengths of the Field1, Field2, Field3, Field4 and Field5
 * and which field contains the varaible length Field and how long this field is.
 * Before this function @si446x_reset should be called.
 */

uint8_t SearchFieldProperties(const uint8_t* pSetPropCmd, tSearchFieldParameters* fieldParams)
{
  uint8_t col;
  uint8_t numOfBytes;

  // Search a property in the interval e.g [leftPropertyInterval,rightPropertyInterval]
  uint16_t leftPropertyInterval;
  uint16_t rightPropertyInterval;

  //property position from the end of COMMAND_HEADER
  uint16_t positionInPro2CmdLine;

  // element of the Prp2Cmd array
  uint8_t elementPro2CmdLine;

  // PKT_FIELD_X_LENGTH_12_8 where X is [1..5], high bytes
  uint16_t field_X_HighBytes[5] =
      {
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_1_LENGTH_12_8),
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_2_LENGTH_12_8),
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_3_LENGTH_12_8),
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_4_LENGTH_12_8),
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_5_LENGTH_12_8),
      };

  // PKT_FIELD_X_LENGTH_7_0 where X is [1..5], high bytes
  uint16_t field_X_LowBytes[5] =
      {
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_1_LENGTH_7_0),
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_2_LENGTH_7_0),
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_3_LENGTH_7_0),
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_4_LENGTH_7_0),
          ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_FIELD_5_LENGTH_7_0),
      };

  // Iterator for fieldLength array
  uint8_t numOfFields;

  // PKT_LEN_FIELD_SOURCE property
  uint16_t property_PKT_LEN_FIELD_SOURCE =
      ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_LEN_FIELD_SOURCE);

  // PKT_LEN property
  uint16_t property_PKT_LEN =
      ((uint16_t)SI446X_PROP_GRP_ID_PKT << 8 ) | ((uint16_t)SI446X_PROP_GRP_INDEX_PKT_LEN);


  fieldParams->fieldLength[0] = 0u;
  fieldParams->fieldLength[1] = 0u;
  fieldParams->fieldLength[2] = 0u;
  fieldParams->fieldLength[3] = 0u;
  fieldParams->fieldLength[4] = 0u;

  fieldParams->fieldContainsLength = 0u;
  fieldParams->fieldVaryInLength = 0u; 
  fieldParams->lengthStoredInFifo = 0u;    
  fieldParams->lengthFieldInByte = 0u;
  fieldParams->lengthEndian = 0u;


  // While cycle as far as the pointer points to a command
  while (*pSetPropCmd != 0x00)
  {
    // Commands structure in the array:
    // --------------------------------
    // LEN | <LEN length of data>


    numOfBytes = *pSetPropCmd++;

    if (numOfBytes > 16u)
    {
      // Number of command bytes exceeds maximal allowable length
      return SEARCH_FIELD_ERROR;
    }

    // Parse a command line
    for (col = 0u; col < numOfBytes; col++)
    {
      Pro2Cmd[col] = *pSetPropCmd;
      pSetPropCmd++;
    }

    if(Pro2Cmd[0] == 0x11)
    {
      // Calculate the interval that properties cover in a command line
      //[leftPropertyValue, rightPropertyValue] is the interval
      //
      leftPropertyInterval  =  (uint16_t) ( ((uint16_t)Pro2Cmd[1] << 8) | ((uint16_t)Pro2Cmd[3]) );
      rightPropertyInterval =  (uint16_t) ( ((uint16_t)Pro2Cmd[1] << 8) | ((uint16_t)Pro2Cmd[2] + (uint16_t)Pro2Cmd[3] - (uint16_t)0x01)) ;

      // Determine which one of the fields [FIELD1,FIELD2, FIELD3, FIELD4] contains the length information
      if( (leftPropertyInterval 		   <=  property_PKT_LEN_FIELD_SOURCE) &&
          (property_PKT_LEN_FIELD_SOURCE <=   rightPropertyInterval))
      {
        // Calculate the difference in properties
        positionInPro2CmdLine = property_PKT_LEN_FIELD_SOURCE - leftPropertyInterval;

        // Position to the right value from the beginning of the command line
        elementPro2CmdLine = Pro2Cmd[COMMAND_HEADER_LENGTH + (uint8_t)positionInPro2CmdLine];

        // Determine which field contains the length information
        switch(elementPro2CmdLine & SI446X_PROP_PKT_LEN_FIELD_SOURCE_SRC_FIELD_MASK)
        {
          case 0x00:
          case 0x01:
            // FIELD1 says the lenght of the variable FIELD
            fieldParams->fieldContainsLength = 0x01;
            break;

          case 0x02:
            // FIELD2 says the length of the variable FIELD
            fieldParams->fieldContainsLength = 0x02;
            break;

          case 0x03:
            // FIELD3 says the length of the variable FIELD
            fieldParams->fieldContainsLength = 0x03;
            break;

          case 0x04:
            // FIELD4 says the length of the variable FIELD
            fieldParams->fieldContainsLength = 0x04;
            break;

          default:
            break;
        }
      }

      // Determine which one of the fields [FIELD2,FIELD3, FIELD4, FIELD5] is variable part in the packet
      if  ( (leftPropertyInterval  <=  property_PKT_LEN     ) &&
          (property_PKT_LEN      <=  rightPropertyInterval))
      {
        // Calculate the difference in properties
        positionInPro2CmdLine = property_PKT_LEN - leftPropertyInterval;

        // Position to the right value from the beginning of the command line
        elementPro2CmdLine = Pro2Cmd[COMMAND_HEADER_LENGTH + (uint8_t)(positionInPro2CmdLine & 0xFF)];

        // Determine if lenght bytes(s) is stored in FiFO
        if  ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_IN_FIFO_MASK) == 0x00)
        {
          // Length byte is not stored in FIFO
          fieldParams->lengthStoredInFifo = 0x00;
        }
        else if ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_IN_FIFO_MASK) == 0x08)
        {
          // Lenght byte is stored in FIFO
          fieldParams->lengthStoredInFifo = 0x01;
        }

        if  ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_SIZE_MASK) == 0x00)
        {
          // Lenght field is one byte in length
          fieldParams->lengthFieldInByte = 0x01;
        }
        else if ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_SIZE_MASK) == 0x10)
          // length field is two bytes in length
          fieldParams->lengthFieldInByte = 0x02;
      }

      if  ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_ENDIAN_MASK) == 0x00 )
      {
        // Length field is little endian
        fieldParams->lengthEndian = 0x00;
      }
      else if((elementPro2CmdLine & SI446X_PROP_PKT_LEN_ENDIAN_MASK) == 0x20)
      {
        fieldParams->lengthEndian = 0x01;
      }

      // Determine which field is variable
      if  ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_DST_FIELD_MASK) == 0x00)
      {
        // Variable packet mode not used
        fieldParams->fieldVaryInLength = 0x00;
      }
      else if ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_DST_FIELD_MASK) == 0x02)
      {
        //FIELD2 is variable in the packet
        fieldParams->fieldVaryInLength = 0x02;
      }
      else if ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_DST_FIELD_MASK) == 0x03)
      {
        // FIELD3 is variable in the packet
        fieldParams->fieldVaryInLength = 0x03;
      }
      else if ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_DST_FIELD_MASK) == 0x04)
      {
        // FIELD4 is variable in the packet
        fieldParams->fieldVaryInLength = 0x04;
      }
      else if ((elementPro2CmdLine & SI446X_PROP_PKT_LEN_DST_FIELD_MASK) == 0x05)
      {
        //FIELD5 is variable in the packet
        fieldParams->fieldVaryInLength = 0x05;
      }
    }


    for(numOfFields = 0u; numOfFields < 5u; numOfFields = numOfFields+1)
    {
      // Determine if field lenght property is in the interval
      if( (leftPropertyInterval       <= field_X_HighBytes[numOfFields]) &&
          (field_X_HighBytes[numOfFields] <= rightPropertyInterval))
      {
        positionInPro2CmdLine = field_X_HighBytes[numOfFields] - leftPropertyInterval;

        if (Pro2Cmd[COMMAND_HEADER_LENGTH + (uint8_t)(positionInPro2CmdLine & 0xFF)] != 0u)
        {
          fieldParams->fieldLength[numOfFields] = ((uint16_t) Pro2Cmd[COMMAND_HEADER_LENGTH + (uint8_t)(positionInPro2CmdLine & 0xFF)] << 8 );
        }
        else
        {
          break;
        }

      }
    }

    // Search for the lenghts of Field1, Field2, Field3, Field4, Field5
    for(numOfFields = 0u; numOfFields < 5u;  numOfFields = numOfFields+1)
    {
      // Determine if field lenght property is in the interval
      if( (leftPropertyInterval          <= field_X_LowBytes[numOfFields]) &&
          (field_X_LowBytes[numOfFields] <= rightPropertyInterval))
      {
        positionInPro2CmdLine = field_X_LowBytes[numOfFields] - leftPropertyInterval;

        if (Pro2Cmd[COMMAND_HEADER_LENGTH + (uint8_t)(positionInPro2CmdLine & 0xFF)] != 0u)
        {
          fieldParams->fieldLength[numOfFields] |= (uint16_t)Pro2Cmd[COMMAND_HEADER_LENGTH + (uint8_t)(positionInPro2CmdLine & 0xFF)];
        }
        else
        {
          break;
        }

      }
    }
  }

  // If any field lenght is zero means that the next fields also must be zero
  // Store the position of the first zero
  for(numOfFields = 0u; numOfFields < 5u; numOfFields++ )
  {
    if (fieldParams->fieldLength[numOfFields] == 0u){
      break;
    }
  }
  // Set the remaining fields to zero
  for(/*numOfFields + 1u*/; numOfFields < 5u; numOfFields++){
    fieldParams->fieldLength[numOfFields] = 0u;
  }

  // All fields are zero means PH is configured incorrectly
  if( (fieldParams->fieldLength[0u] == 0u) &&
      (fieldParams->fieldLength[1u] == 0u) &&
      (fieldParams->fieldLength[2u] == 0u) &&
      (fieldParams->fieldLength[3u] == 0u) &&
      (fieldParams->fieldLength[4u] == 0u))
  {
    return SEARCH_FIELD_ERROR;
  }

  return SEARCH_FIELD_SUCCESS;	
}



/*!
 * This function is used to compare the content of the received packet to a string.
 *
 * @return  None.
 */

uint8_t gSampleCode_StringCompare(uint8_t* pbiPacketContent, uint8_t* pbiString, uint8_t lenght)
{
  while ((*pbiPacketContent++ == *pbiString++) && (lenght > 0u))
  {
    if( (--lenght) == 0u )
    {
      return TRUE;
    }
  }

  return FALSE;
}

/*!
 * This function is used to compare the content of the received packet to a string.
 *
 * @return  None.
 */
uint8_t gSampleCode_StringCompareVariable(uint8_t* pbiPacketContent, uint8_t* pbiString, uint8_t length, tSearchFieldParameters* fParams)
{


  //TODO: Variable field length info shall contain only the length information yet! Improvement requested in future!
  eAnalyzeFieldState analyzeFieldState = ANALYZE_STATE_IS_LENGTH_FIELD;

  uint16_t lengthToCheck;
  uint16_t realVariableLength;
  uint16_t indexInPacket = 0u;
  uint8_t fieldExamined = 0u;


  while (TRUE)
  {
    switch (analyzeFieldState)
    {
      case ANALYZE_STATE_IS_LENGTH_FIELD:

        // Is the examined field the length field?
        if (fieldExamined == (fParams->fieldContainsLength - 1u))
        {
          analyzeFieldState = ANALYZE_STATE_IS_LENGTH_FIELD_INCLUDED;
        }
        else
        {
          // Not the length field

          // Get the field length
          lengthToCheck = fParams->fieldLength[fieldExamined];

          // Store the index in the packet
          indexInPacket = indexInPacket + lengthToCheck;

          // Compare the content
          while (lengthToCheck > 0u)
          {
            if (*pbiPacketContent++ != *pbiString++)
            {
              // Content mismatch
              return FALSE;
            }

            lengthToCheck = lengthToCheck - 1u;
          }

          // Next field will be examined
          fieldExamined = fieldExamined + 1u;
        }
        break;

      case ANALYZE_STATE_IS_LENGTH_FIELD_INCLUDED:

        pbiString += fParams->lengthFieldInByte;


        // Calculate the variable field length (depends on Si446xCmd union!!)
        lengthToCheck = fParams->fieldLength[fParams->fieldVaryInLength - 1u] - (length - Si446xCmd.FIFO_INFO.RX_FIFO_COUNT);

        if (fParams->lengthStoredInFifo == 0x00)
        {
          // Length not included in FIFO

          // Next field will be examined
          fieldExamined = fieldExamined + 1u;

          analyzeFieldState =	ANALYZE_STATE_IS_VARIABLE_FIELD;
        }
        else
        {
          // Length included in FIFO

          if (fParams->lengthFieldInByte == 0x01)
          {
            // If the length is one byte

            if (*pbiPacketContent++ == (lengthToCheck & 0xFF))
            {
              // Next field will be examined
              fieldExamined = fieldExamined + 1u;

              // Store the index in the packet
              indexInPacket = indexInPacket + 1u;
            }
            else
            {
              // Content mismatch
              return FALSE;
            }
          }
          else if (fParams->lengthFieldInByte == 0x02)
          {
            if (fParams->lengthEndian == 0x01)
            {
              // Length in big endian

              // Variable filed supposed to be always less than 64 bytes
              if(*pbiPacketContent++ == 0x00)
              {
                if(*pbiPacketContent++ == (lengthToCheck & 0xFF))
                {
                  // Next field will be examined
                  fieldExamined = fieldExamined + 1u;

                  // Store the index in the packet
                  indexInPacket = indexInPacket + 2u;
                }
                else
                {
                  // Content mismatch
                  return FALSE;
                }
              }
              else
              {
                // Content mismatch
                return FALSE;
              }
            }
            else if(fParams->lengthEndian == 0x00)
            {
              // Length in little endian
              if(*pbiPacketContent++ == (lengthToCheck & 0xFF))
              {
                if(*pbiPacketContent++ == 0x00)
                {
                  // Next field will be examined
                  fieldExamined = fieldExamined + 1u;

                  // Store the index in the packet
                  indexInPacket = indexInPacket + 2u;
                }
                else
                {
                  //Content mismatch
                  return FALSE;
                }
              }
              else
              {
                // Content mismatch
                return FALSE;
              }
            }
          }
          else{
            // Never happens (error in config)
            return FALSE;
          }

          analyzeFieldState =	ANALYZE_STATE_IS_VARIABLE_FIELD;
        }

        break;

      case ANALYZE_STATE_IS_VARIABLE_FIELD:
        if (fieldExamined == fParams->fieldVaryInLength - 1u)
        {
          // Length not stored in Fifo
          if (fParams->lengthStoredInFifo == 0x00)
          {
            lengthToCheck = fParams->fieldLength[fParams->fieldVaryInLength - 1u] - (length - Si446xCmd.FIFO_INFO.RX_FIFO_COUNT - fParams->lengthFieldInByte);
          }
          else
          {
            // Calculate the variable field length (depends on Si446xCmd union!!)
            lengthToCheck = fParams->fieldLength[fParams->fieldVaryInLength - 1u] - (length - Si446xCmd.FIFO_INFO.RX_FIFO_COUNT);
          }

          // Store the real variable length of the packet
          realVariableLength = lengthToCheck;

          // Store the index in the packet
          indexInPacket = indexInPacket + lengthToCheck;

          while (lengthToCheck > 0u)
          {
            if(*pbiPacketContent++ != *pbiString++)
            {
              // Content mismatch
              return FALSE;
            }

            lengthToCheck = lengthToCheck - 1u;
          }

          // Content is correct
          analyzeFieldState = ANALYZE_STATE_IS_DATA_REMAINING;
        }
        else
        {
          // Get the field length
          lengthToCheck = fParams->fieldLength[fieldExamined];

          // Store the index in the packet
          indexInPacket = indexInPacket + lengthToCheck;

          // Compare the content
          while (lengthToCheck > 0u)
          {
            if (*pbiPacketContent++ != *pbiString++)
            {
              // Content mismatch
              return FALSE;
            }

            lengthToCheck = lengthToCheck - 1u;
          }

          // Next field will be examined
          fieldExamined = fieldExamined + 1u;
        }

        break;

      case ANALYZE_STATE_IS_DATA_REMAINING:

        // Step through those bytes in the variable field pattern that are not received
        pbiString += fParams->fieldLength[fParams->fieldVaryInLength - 1u] - realVariableLength;

        while (Si446xCmd.FIFO_INFO.RX_FIFO_COUNT > indexInPacket)
        {
          // Compare the content
          if (*pbiPacketContent++ != *pbiString++)
          {
            // Content mismatch
            return FALSE;
          }

          indexInPacket++;
        }

        return TRUE;

//        break;


      default:

        return FALSE;
//        break;

    } //swicth
  } //while

  // Never reach here, so we delete it for no warning while compile the code. 
//  return FALSE;
}


/** \fn void eADesigner_Init(void)
 *  \brief Initializes ports of the MCU.
 *
 *  \return None
 *
 *  \note It has to be called from the Initialization section.
 *  \todo Create description
 */

/*************************************************************************//**
 * @brief energyAware Designer MCU initialization
 *
 * This code is generated by the energyAware Designer appliction to configure
 * the MCU for application specific operation. 
 *
 * The generated code is a starting point, which might require adjustment for
 * correct operation. Call this function at early initialization.
 *****************************************************************************/
void eADesigner_Init(void)
{
#ifdef EFM32LG990F256
  /* Using HFRCO at 14MHz as high frequency clock, HFCLK */
  
  /* No LE clock source selected */
  
  /* Enable GPIO clock */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  
  /* Pin PB9 is configured to Input enabled */
  GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE9_MASK) | GPIO_P_MODEH_MODE9_INPUT;
  /* Pin PB10 is configured to Input enabled */
  GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE10_MASK) | GPIO_P_MODEH_MODE10_INPUT;
  /* Pin PB11 is configured to Input enabled */
  GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE11_MASK) | GPIO_P_MODEH_MODE11_INPUT;
  /* Pin PB12 is configured to Input enabled */
  GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE12_MASK) | GPIO_P_MODEH_MODE12_INPUT;
  /* Pin PC0 is configured to Input enabled */
  GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | GPIO_P_MODEL_MODE0_INPUT;
  /* Pin PC3 is configured to Push-pull */
  GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE3_MASK) | GPIO_P_MODEL_MODE3_PUSHPULL;
  /* Pin PC4 is configured to Input enabled */
  GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK) | GPIO_P_MODEL_MODE4_INPUT;
  /* Pin PC5 is configured to Input enabled */
  GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE5_MASK) | GPIO_P_MODEL_MODE5_INPUT;
  /* To avoid false start, configure output US1_TX as high on PD0 */
  GPIO->P[3].DOUT |= (1 << 0);
  /* Pin PD0 is configured to Push-pull */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | GPIO_P_MODEL_MODE0_PUSHPULL;
  /* Pin PD1 is configured to Input enabled */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE1_MASK) | GPIO_P_MODEL_MODE1_INPUT;
  /* Pin PD2 is configured to Push-pull */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_PUSHPULL;
  /* To avoid false start, configure output US1_CS as high on PD3 */
  GPIO->P[3].DOUT |= (1 << 3);
  /* Pin PD3 is configured to Push-pull */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE3_MASK) | GPIO_P_MODEL_MODE3_PUSHPULL;
  /* Pin PE2 is configured to Push-pull */
  GPIO->P[4].MODEL = (GPIO->P[4].MODEL & ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_PUSHPULL;
  /* Pin PE3 is configured to Push-pull */
  GPIO->P[4].MODEL = (GPIO->P[4].MODEL & ~_GPIO_P_MODEL_MODE3_MASK) | GPIO_P_MODEL_MODE3_PUSHPULL;

  /* Enable clock for USART1 */
  CMU_ClockEnable(cmuClock_USART1, true);
  /* Custom initialization for USART1 */
  vSpiInitialize();
  /* Module USART1 is configured to location 1 */
  USART1->ROUTE = (USART1->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | USART_ROUTE_LOCATION_LOC1;
  /* Enable signals TX, RX, CLK, CS not enable */
  USART1->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;

  /* Enabling TX and RX */
  USART1->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
  /* Clear previous interrupts */
  USART1->IFC = _USART_IFC_MASK;
#endif
  
#ifdef EFM32TG840F32
  /* Using HFRCO at 14MHz as high frequency clock, HFCLK */
  
  /* No LE clock source selected */
  
  /* Enable GPIO clock */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;
  
  /* Pin PB11 is configured to Input enabled */
  GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE11_MASK) | GPIO_P_MODEH_MODE11_INPUT;
  /* Pin PB12 is configured to Input enabled */
  GPIO->P[1].MODEH = (GPIO->P[1].MODEH & ~_GPIO_P_MODEH_MODE12_MASK) | GPIO_P_MODEH_MODE12_INPUT;
  /* Pin PC4 is configured to Input enabled */
  GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK) | GPIO_P_MODEL_MODE4_INPUT;
  /* Pin PC5 is configured to Push-pull */
  GPIO->P[2].MODEL = (GPIO->P[2].MODEL & ~_GPIO_P_MODEL_MODE5_MASK) | GPIO_P_MODEL_MODE5_PUSHPULL;
  /* Pin PC12 is configured to Input enabled */
  GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE12_MASK) | GPIO_P_MODEH_MODE12_INPUT;
  /* Pin PC13 is configured to Input enabled */
  GPIO->P[2].MODEH = (GPIO->P[2].MODEH & ~_GPIO_P_MODEH_MODE13_MASK) | GPIO_P_MODEH_MODE13_INPUT;
  /* To avoid false start, configure output US1_TX as high on PD0 */
  GPIO->P[3].DOUT |= (1 << 0);
  /* Pin PD0 is configured to Push-pull */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE0_MASK) | GPIO_P_MODEL_MODE0_PUSHPULL;
  /* Pin PD1 is configured to Input enabled */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE1_MASK) | GPIO_P_MODEL_MODE1_INPUT;
  /* Pin PD2 is configured to Push-pull */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_PUSHPULL;
  /* Pin PD3 is configured to Push-pull */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE3_MASK) | GPIO_P_MODEL_MODE3_PUSHPULL;
  /* Pin PD7 is configured to Push-pull */
  GPIO->P[3].MODEL = (GPIO->P[3].MODEL & ~_GPIO_P_MODEL_MODE7_MASK) | GPIO_P_MODEL_MODE7_PUSHPULL;
  /* Pin PD8 is configured to Input enabled */
  GPIO->P[3].MODEH = (GPIO->P[3].MODEH & ~_GPIO_P_MODEH_MODE8_MASK) | GPIO_P_MODEH_MODE8_INPUT;
  
  /* Enable clock for USART1 */
  CMU_ClockEnable(cmuClock_USART1, true);
  /* Custom initialization for USART1 */
  vSpiInitialize();
  /* Module USART1 is configured to location 1 */
  USART1->ROUTE = (USART1->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | USART_ROUTE_LOCATION_LOC1;
  /* Enable signals TX, RX, CLK, CS not enable */
  USART1->ROUTE |= USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;  
  
  /* Enabling TX and RX */
  USART1->CMD = USART_CMD_TXEN | USART_CMD_RXEN;
  /* Clear previous interrupts */
  USART1->IFC = _USART_IFC_MASK;  
#endif
}
