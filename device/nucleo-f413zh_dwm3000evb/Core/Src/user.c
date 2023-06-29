/*
 * user.c
 *
 *  Created on: May 23, 2023
 *      Author: Johannes Arenander
 *
 * Ranging based on example code by DecaWave:
 * https://www.qorvo.com/products/d/da007992
 */

/*————————————————————————————————————————————————————————————*/

#include <stdlib.h>

#include <main.h>
#include <user.h>
#include <math.h>

// Don't know exactly which ones are required for DW3000 TWR.
#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>

#define LED_ON 1
#define LED_OFF 0
#define LED_GREEN 1
#define LED_BLUE 2
#define LED_RED 3
#define LED_EXT 4
#define UART_TIMEOUT_MS 10

#define FRAME_COUNT_INDEX 2
#define FRAME_SOURCE_INDEX 5
#define FRAME_DESTINATION_INDEX 7
#define FRAME_POLL_RX_TIMESTAMP_INDEX 10
#define FRAME_RESPONSE_TX_TIMESTAMP_INDEX 14
#define FRAME_POLL_LENGTH 12
#define FRAME_RESPONSE_LENGTH 20

/*————————————————————————————————————————————————————————————*/

extern dwt_txconfig_t txconfig_options;
extern TIM_HandleTypeDef *phCurrentTimer;
extern UART_HandleTypeDef *phCurrentUART;

int iUserButtonInterruptFlag = 0;
char *pczUserLogAppendix = "";
unsigned uUserLogAppendixLength = 0;
unsigned long elapsedTimeMilliseconds = 0;

dwt_config_t config = {
  5,
  DWT_PLEN_128,
  DWT_PAC8,
  9,
  9,
  1,
  DWT_BR_6M8,
  DWT_PHRMODE_STD,
  DWT_PHRRATE_STD,
  129,
  DWT_STS_MODE_OFF,
  DWT_STS_LEN_64,
  DWT_PDOA_M0
};

uint8_t pu8PollMessage[FRAME_POLL_LENGTH] = {
  0x41, 0x88, // Frame control (0x8841)
  0,          // Sequence number
  0xCA, 0xDE, // PAN ID (0xDECA)
  'W', 'A',   // Destination ID ("AW")
  'V', 'E',   // Source ID ("EV")
  0xE0,       // Function code
  0, 0        // Checksum
};

uint8_t pu8ResponseMessage[FRAME_RESPONSE_LENGTH] = {
  0x41, 0x88, // Frame control (0x8841)
  0,          // Sequence number
  0xCA, 0xDE, // PAN ID (0xDECA)
  'V', 'E',   // Destination ID ("EV")
  'W', 'A',   // Source ID ("AW")
  0xE1,       // Function code
  0, 0, 0, 0, // Poll timestamp
  0, 0, 0, 0, // Response timestamp
  0, 0        // Checksum
};

char *userPollFrame = (char *)pu8PollMessage;
char *userResponseFrame = (char *)pu8ResponseMessage;

uint8_t pu8Buffer[20];

/*————————————————————————————————————————————————————————————*/

static int iGetLed(int, GPIO_TypeDef **, uint16_t *);
static int iValidateFrame(void *, void *);

/*————————————————————————————————————————————————————————————*/

/*
 * Set SPI rate, reset and then initialize transceiver.
 */
int userInitializeDevice(void)
{
  port_set_dw_ic_spi_fastrate();
  reset_DWIC();
  Sleep(2);
  dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

  while (!dwt_checkidlerc())
  {
    ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    return -1;
  }

  return 0;
}

/*
 * Configure transmit and receive antenna delay,
 * LEDs, general device configuration,
 * and some other things under the hood
 * which I have no clue about
 * (strictly following DecaWave's example code).
 */
int userConfigureDevice(int iNodeMode)
{
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  if (dwt_configure(&config) != DWT_SUCCESS)
  {
    return -1;
  }

  dwt_configuretxrf(&txconfig_options);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  if (iNodeMode == USER_MODE_ANCHOR)
  {
    dwt_setrxaftertxdelay(240);
    dwt_setrxtimeout(400);
  }
  else if (iNodeMode == USER_MODE_TAG)
  {
    ;
  }

  // Set default antenna delay.
  dwt_setrxantennadelay(16385);
  dwt_settxantennadelay(16385);

  return 0;
}

/*
 * Calculate distance
 */
double userCalculateDistance()
{
  uint8_t *pu8Frame = pu8Buffer;
  uint32_t u32PollTXTime = 0;
  uint32_t u32PollRXTime = 0;
  uint32_t u32ResponseTXTime = 0;
  uint32_t u32ResponseRXTime = 0;
  float fClockOffsetRatio = 0;
  double dTimeOfFlight = 0;

  u32PollTXTime = dwt_readtxtimestamplo32();
  u32ResponseRXTime = dwt_readrxtimestamplo32();
  fClockOffsetRatio =
    ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
  resp_msg_get_ts(&pu8Frame[10], &u32PollRXTime);
  resp_msg_get_ts(&pu8Frame[14], &u32ResponseTXTime);
  dTimeOfFlight = DWT_TIME_UNITS * 0.5 * (
    (u32ResponseRXTime - u32PollTXTime) -
    (u32ResponseTXTime - u32PollRXTime) *
    (1 - fClockOffsetRatio));
  return dTimeOfFlight * SPEED_OF_LIGHT;
}

/*
 * Start a ranging exchange.
 */
int userPoll(double *dDistance)
{
  uint32_t u32Mask = DWT_INT_RXFCG_BIT_MASK |
    SYS_STATUS_ALL_RX_TO |
    SYS_STATUS_ALL_RX_ERR;
  uint32_t u32Mode = DWT_START_TX_IMMEDIATE |
    DWT_RESPONSE_EXPECTED;
  uint32_t u32Status = 0;
  uint16_t u16FrameLength = 0;

  dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof (pu8PollMessage), pu8PollMessage, 0);
  dwt_writetxfctrl(sizeof (pu8PollMessage), 0, 1);

  if (dwt_starttx(u32Mode) != DWT_SUCCESS)
  {
    return -1;
  }

  waitforsysstatus(&u32Status, NULL, u32Mask, 0);

  if (!(u32Status & DWT_INT_RXFCG_BIT_MASK))
  {
    dwt_writesysstatuslo(
      SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    return -1;
  }

  dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
  u16FrameLength = dwt_getframelength();

  if (u16FrameLength > sizeof (pu8Buffer))
  {
    return -1;
  }

  dwt_readrxdata(pu8Buffer, u16FrameLength, 0);

  if (iValidateFrame(pu8Buffer, pu8ResponseMessage) == -1)
  {
    return -1;
  }

  *dDistance = userCalculateDistance();
  return 0;
}

/*
 * Wait for a ranging request.
 */
int userRespond(void)
{
  uint32_t u32Mask = DWT_INT_RXFCG_BIT_MASK |
    SYS_STATUS_ALL_RX_ERR;
  uint32_t u32Mode = DWT_START_TX_DELAYED;
  uint32_t u32Status = 0;
  uint16_t u16FrameLength = 0;
  uint64_t u64PollRXTime = 0;
  uint64_t u64ResponseTXTime = 0;

  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  waitforsysstatus(&u32Status, NULL, u32Mask, 0);

  if (!(u32Status & DWT_INT_RXFCG_BIT_MASK))
  {
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
    return -1;
  }

  dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
  u16FrameLength = dwt_getframelength();

  if (u16FrameLength > sizeof (pu8Buffer))
  {
    return -1;
  }

  dwt_readrxdata(pu8Buffer, u16FrameLength, 0);

  if (iValidateFrame(pu8Buffer, pu8PollMessage) == -1)
  {
    return -1;
  }

  u64PollRXTime = get_rx_timestamp_u64();
  u64ResponseTXTime =
    (u64PollRXTime + 650 * UUS_TO_DWT_TIME) >> 8;
  dwt_setdelayedtrxtime(u64ResponseTXTime);
  u64ResponseTXTime =
    ((u64ResponseTXTime & 0xFFFFFFFEUL) << 8) +
    dwt_gettxantennadelay();
  resp_msg_set_ts(
    &pu8ResponseMessage[FRAME_POLL_RX_TIMESTAMP_INDEX],
    u64PollRXTime);
  resp_msg_set_ts(
    &pu8ResponseMessage[FRAME_RESPONSE_TX_TIMESTAMP_INDEX],
    u64ResponseTXTime);
  dwt_writetxdata(
    sizeof (pu8ResponseMessage), pu8ResponseMessage, 0);
  dwt_writetxfctrl(sizeof (pu8ResponseMessage), 0, 1);

  if (dwt_starttx(u32Mode) != DWT_SUCCESS)
  {
    return -1;
  }

  waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
  dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
  return 0;
}

/*
 * Enable logging.
 */
void userDisableLogging(void)
{
  if (HAL_UART_DeInit(phCurrentUART) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * Disable logging.
 */
void userEnableLogging(void)
{
  char *pcz = USER_NEWLINE;
  uUserLogAppendixLength = 0;
  for (char *pc = pcz; *pc; pc++) uUserLogAppendixLength++;
  if (HAL_UART_Init(phCurrentUART) != HAL_OK) Error_Handler();
}

/*
 * Log a message.
 */
void userLog(char *pczMessage)
{
  char *pczAppendix = USER_NEWLINE;
  uint16_t u16Len = uUserLogAppendixLength;
  for (char *pc = pczMessage; *pc; pc++) u16Len++;
  uint8_t pu8Data[u16Len];
  uint8_t *pu8 = pu8Data;

  while (*pczMessage)
  {
    *pu8 = *pczMessage;
     pu8++;
     pczMessage++;
  }
  
  while (*pczAppendix)
  {
    *pu8 = *pczAppendix;
     pu8++;
     pczAppendix++;
  }

  HAL_UART_Transmit(
    phCurrentUART,
    pu8Data,
    u16Len,
    UART_TIMEOUT_MS);
}

/*
 * Set the state of an LED.
 */
void userSetLed(int iLed, int iState)
{
  GPIO_TypeDef *pPort = NULL;
  uint16_t u16Pin = 0;
  GPIO_PinState eState = (iState == LED_ON) ?
      GPIO_PIN_SET : GPIO_PIN_RESET;
  if (iGetLed(iLed, &pPort, &u16Pin) == -1) return;
  HAL_GPIO_WritePin(pPort, u16Pin, eState);
}

/*
 * Toggle the state of an LED.
 */
void userToggleLed(int iLed)
{
  GPIO_TypeDef *pPort = NULL;
  uint16_t u16Pin = 0;
  if (iGetLed(iLed, &pPort, &u16Pin) == -1) return;
  HAL_GPIO_TogglePin(pPort, u16Pin);
}

/*
 * Disable interrupts.
 */
void userDisableInterrupts(void)
{
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}

/*
 * Button interrupt.
 */
void userButtonInterruptHandler(void)
{
  if (iUserButtonInterruptFlag == 0)
  {
    userSetLed(LED_GREEN, LED_ON);
    userSetLed(LED_EXT, LED_ON);
    userEnableLogging();
    userDisableInterrupts();

    if (HAL_TIM_Base_Start_IT(phCurrentTimer) != HAL_OK)
    {
      userSetLed(LED_GREEN, LED_OFF);
      userSetLed(LED_EXT, LED_OFF);
      userSetLed(LED_RED, LED_ON);
      Error_Handler();
    }

    iUserButtonInterruptFlag = 1;
  }
}

/*
 * Timer interrupt.
 */
void userTimerInterruptHandler(void)
{
  elapsedTimeMilliseconds++;

  /*
  if (elapsedTimeMilliseconds % 1000 == 0)
  {
    userToggleLed(LED_BLUE);
    userToggleLed(LED_EXT);
  }
  */
}

/*————————————————————————————————————————————————————————————*/

/*
 * Timer interrupt callback.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *phTimer)
{
  if (phTimer == phCurrentTimer) userTimerInterruptHandler();
}

/*————————————————————————————————————————————————————————————*/

/*
 * Get the port and pin of a specific LED.
 */
static int iGetLed(
    int iLed,
    GPIO_TypeDef **ppPort,
    uint16_t *pu16Pin)
{
  switch (iLed)
  {
    case LED_GREEN:
      *ppPort = LD1_GPIO_Port;
      *pu16Pin = LD1_Pin;
      break;
    case LED_BLUE:
      *ppPort = LD2_GPIO_Port;
      *pu16Pin = LD2_Pin;
      break;
    case LED_RED:
      *ppPort = LD3_GPIO_Port;
      *pu16Pin = LD3_Pin;
      break;
    case LED_EXT:
      *ppPort = EXT_LED_GPIO_Port;
      *pu16Pin = EXT_LED_Pin;
      break;
    default:
      return -1;
  }

  return 0;
}

/*
 * Validate frame.
 */
static int iValidateFrame(void *pFrame, void *pReference)
{
  int iResult = -1;
  iResult = memcmp(pFrame, pReference, 10) == 0 ? 0 : -1;
  return iResult == 0 ? 0 : -1;
}

/*————————————————————————————————————————————————————————————*/

/*
 * Notes
 *
 * References
 *
 * [1] "APS312 APPLICATION NOTE"; DecaWave; 2021.
 * [2] "APS014 APPLICATION NOTE"; DecaWave; 2018.
 *
 */
