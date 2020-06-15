/******************************************************************************
* LOCAL INCLUDE FILES
******************************************************************************/
#include <stdio.h>
#include "main.h"
#include "hlw8032.h"
#include "cmsis_os.h"

/******************************************************************************
* LOCAL MACROS AND DEFINITIONS
******************************************************************************/
#define HLW8032_REG_NUMBER          (uint8_t)(24)
#define HLW8032_UART_RX_BUFFER      (uint32_t)(HLW8032_REG_NUMBER * 3)
#define HLW8032_UART_TIMEOUT        (uint8_t)(50)
#define HLW8032_UART                huart2

/* Data register index */
#define IND_STATE                   0
#define IND_CHECK                   1
#define IND_VPARAM                  2
#define IND_VOLTAGE                 5
#define IND_CPARAM			        8
#define IND_CURRENT                 11
#define IND_PPARAM                  14
#define IND_POWER                   17
#define IND_UPDATE                  20
#define IND_PF                      21
#define IND_CHECKSUM                23

/* Register default value */
#define CHECK_REG_VALUE             (uint8_t)(0x5A)
#define CHIP_ERROR                  (uint8_t)(0xAA)
#define REG_NOT_OVERFLOW            (uint8_t)(0x55)
#define REG_OVERFLOW_TIME           (uint32_t)(2000000)

/* Coefficients */
#define CURRENT_COEFFICIENT         (0.001f)
#define VOLTAGE_COEFFICIENT         (1.0f)

/* Check define */
#define PF_VALUE(reg)               ((reg[IND_UPDATE] & 0x80) >> 7)
#define IS_VOLTAGE_UPDATE(reg)      ((reg[IND_UPDATE] & 0x40) >> 6)
#define IS_CURRENT_UPDATE(reg)      ((reg[IND_UPDATE] & 0x20) >> 5)
#define IS_POWER_UPDATE(reg)        ((reg[IND_UPDATE] & 0x10) >> 4)

#define IS_VOLTAGE_OVERFLOW(reg)    ((reg[IND_STATE] & 0x08) >> 3)
#define IS_CURRENT_OVERFLOW(reg)    ((reg[IND_STATE] & 0x04) >> 2)
#define IS_POWER_OVERFLOW(reg)      ((reg[IND_STATE] & 0x02) >> 1)

/******************************************************************************
* LOCAL TYPE DECLARATIONS
******************************************************************************/
typedef struct TS_USART
{
  uint8_t *pRbuf;
  uint16_t rbufCount;
  uint16_t rbufWriteIndex;
  uint16_t rbufReadIndex;
} TS_USART;

typedef struct TS_POWERMETER_DATA
{
  float voltage;
  float current;
  float activePower;
  float apparentPower;
  float powerFactor;
  float Kwh;
  uint16_t pfNegationCount;
  uint8_t pfFlag;
} TS_POWERMETER_DATA;

typedef enum TE_HLW8032_STATE
{
  HLW8032_OK = 0,
  HLW8032_CRC_ERROR,
  HLW8032_CHECK_ERROR,
  HLW8032_CHIP_ERROR,
  HLW8032_REG_NOT_OVERFLOW,
  HLW8032_REG_OVERFLOW,
} TE_HLW8032_STATE;

/******************************************************************************
* LOCAL CONSTANTS
******************************************************************************/

/******************************************************************************
* LOCAL VARIABLES
******************************************************************************/
/* Definitions for HLW8032Task */
static osThreadId HLW8032TaskHandle;

/* Definitions for HLW8032Queue */
static osMessageQId HLW8032QueueHandle;

/* Definitions for HLW8032Timer */
static osTimerId HLW8032TimerHandle;

static TS_USART m_hlw8032Uart;
static uint8_t m_hlw8032Reg[HLW8032_REG_NUMBER];
static TS_POWERMETER_DATA m_powerMeterData;

/******************************************************************************
* LOCAL FUNCTION DECLARATIONS
******************************************************************************/
static void HLW8032_Task(void const *argument);
static void HLW8032_Timer_Callback(void const *argument);
static uint16_t nextIndex(uint16_t index);
static uint8_t getChar(void);
static void copyReceivedData(void);
static void runPowerMetering(void);
static TE_HLW8032_STATE checkReg(void);
static TE_HLW8032_STATE calculateCRC(void);
static TE_HLW8032_STATE checkState(void);
static void calculatePower(TE_HLW8032_STATE state);

/******************************************************************************
* EXPORTED FUNCTIONS
******************************************************************************/
void HLW8032_Init(void)
{
  /* init uart data */
  m_hlw8032Uart.pRbuf = (uint8_t *)pvPortMalloc(HLW8032_UART_RX_BUFFER);
  m_hlw8032Uart.rbufReadIndex = 0;
  m_hlw8032Uart.rbufWriteIndex = 0;
  m_hlw8032Uart.rbufCount = 0;

  /* definition and creation of HLW8032Task */
  osThreadDef(HLW8032Task, HLW8032_Task, osPriorityNormal, 0, 128);
  HLW8032TaskHandle = osThreadCreate(osThread(HLW8032Task), NULL);

  /* definition and creation of HLW8032Queue */
  osMessageQDef(HLW8032Queue, 10, uint16_t);
  HLW8032QueueHandle = osMessageCreate(osMessageQ(HLW8032Queue), NULL);

  /* definition and creation of HLW8032Timer */
  osTimerDef(HLW8032Timer, HLW8032_Timer_Callback);
  HLW8032TimerHandle = osTimerCreate(osTimer(HLW8032Timer), osTimerOnce, NULL);
}

void HLW8032_DeInit(void)
{
  if(m_hlw8032Uart.pRbuf != NULL)
  {
	  vPortFree(m_hlw8032Uart.pRbuf);
  }

  /* delete of HLW8032Task */
  osThreadTerminate(HLW8032TaskHandle);

  /* delete of HLW8032_Queue */
  osMessageDelete(HLW8032QueueHandle);

  /* delete of HLW8032Timer */
  osTimerDelete(HLW8032TimerHandle);
}
/*
void HLW8032_Print(void)
{
  printf("-> Voltage:%.2fV\n", m_powerMeterData.voltage);
  printf("-> Current:%.2fA\n", m_powerMeterData.current);
  printf("-> Active Power:%.2fW\n", m_powerMeterData.activePower);
  printf("-> Apparent Power:%.2fW\n", m_powerMeterData.apparentPower);
  printf("-> Power Factor:%.4f\n", m_powerMeterData.powerFactor);
  printf("-> KWH:%f\n", m_powerMeterData.Kwh);
}*/

void HLW8032_UART_RxCallback(void)
{
  /* receive one byte data */
  m_hlw8032Uart.rbufCount++;

  /* Move write index */
  m_hlw8032Uart.rbufWriteIndex = nextIndex(m_hlw8032Uart.rbufWriteIndex);

  /* Check buffer is overflow */
  if(m_hlw8032Uart.rbufWriteIndex == m_hlw8032Uart.rbufReadIndex)
  {
    m_hlw8032Uart.rbufWriteIndex = 0;
    m_hlw8032Uart.rbufReadIndex = 0;
    //printf("HLW8032 uart rx buffer is overflow!\n");
  }

  /* Start receive next data */
  HAL_UART_Receive_IT(&HLW8032_UART,
                      m_hlw8032Uart.pRbuf + m_hlw8032Uart.rbufWriteIndex,
                      1);

  /* start software timer to wait data receive finish */
  osTimerStart(HLW8032TimerHandle, HLW8032_UART_TIMEOUT);
}

/******************************************************************************
* LOCAL FUNCTIONS
******************************************************************************/
/**
  * @brief  Function implementing the HLW8032Task thread.
  * @param  argument: Not used
  * @retval None
  */
static void HLW8032_Task(void const *argument)
{
  osEvent event;

  /* USER CODE BEGIN 5 */
  HAL_UART_Receive_IT(&HLW8032_UART, m_hlw8032Uart.pRbuf, 1);

  /* Infinite loop */
  for(;;)
  {
    event = osMessageGet(HLW8032QueueHandle, osWaitForever);
    if(event.status == osEventMessage)
    {
      /* Parse data from uart receive buffer (24 bytes) */
      copyReceivedData();

      /* Check data and calcuate power */
      runPowerMetering();

      printf("received\r\n");
    }
  }
  /* USER CODE END 5 */
}

static void HLW8032_Timer_Callback(void const *argument)
{
  if(m_hlw8032Uart.rbufCount == HLW8032_REG_NUMBER)
  {
    /* Send message for complete receive data */
	osMessagePut(HLW8032QueueHandle, m_hlw8032Uart.rbufCount, 10);
  }
  else
  {
     /* Data length is not 24 bytes, ignore it */
     m_hlw8032Uart.rbufReadIndex = m_hlw8032Uart.rbufWriteIndex;
  }

  m_hlw8032Uart.rbufCount = 0;
}

static uint16_t nextIndex(uint16_t index)
{
  index++;
  if(index == HLW8032_UART_RX_BUFFER)
  {
    index = 0;
  }
  return index;
}

static uint8_t getChar(void)
{
  uint8_t buf;

  buf = *(m_hlw8032Uart.pRbuf + m_hlw8032Uart.rbufReadIndex);

  m_hlw8032Uart.rbufReadIndex = nextIndex(m_hlw8032Uart.rbufReadIndex);

  return buf;
}

static void copyReceivedData(void)
{
  uint8_t index = 0;

  for(index = 0; index < HLW8032_REG_NUMBER; index++)
  {
    m_hlw8032Reg[index] = getChar();
  }
}

static void runPowerMetering(void)
{
  TE_HLW8032_STATE res;

  /* calculate CRC */
  res = calculateCRC();
  if(res != HLW8032_OK)
  {
    return;
  }

  /* Check reg */
  res = checkReg();
  if(res != HLW8032_OK)
  {
    return;
  }

  /* Check state */
  res = checkState();
  if(res == HLW8032_CHIP_ERROR)
  {
    return;
  }

  calculatePower(res);
}

static TE_HLW8032_STATE checkReg(void)
{
  if(m_hlw8032Reg[IND_CHECK] == CHECK_REG_VALUE)
  {
    return HLW8032_OK;
  }
  else
  {
    return HLW8032_CHECK_ERROR;
  }
}

static TE_HLW8032_STATE calculateCRC(void)
{
  uint8_t index = 0;
  uint16_t crc = 0;

  for(index = IND_VPARAM; index < IND_CHECKSUM; index++)
  {
    crc += m_hlw8032Reg[index];
  }

  if(m_hlw8032Reg[IND_CHECKSUM] == (uint8_t)(crc & 0xFF))
  {
    return HLW8032_OK;
  }
  else
  {
    return HLW8032_CRC_ERROR;
  }
}

static TE_HLW8032_STATE checkState(void)
{
  if(m_hlw8032Reg[IND_STATE] == CHIP_ERROR)
  {
    return HLW8032_CHIP_ERROR;
  }
  else if(m_hlw8032Reg[IND_STATE] == REG_NOT_OVERFLOW)
  {
    return HLW8032_REG_NOT_OVERFLOW;
  }
  else
  {
    return HLW8032_REG_OVERFLOW;
  }
}

static void calculatePower(TE_HLW8032_STATE state)
{
  uint32_t voltage = (m_hlw8032Reg[IND_VOLTAGE] << 16) |
                     (m_hlw8032Reg[IND_VOLTAGE + 1] << 8) |
                     (m_hlw8032Reg[IND_VOLTAGE + 2]);
  uint32_t current = (m_hlw8032Reg[IND_CURRENT] << 16) |
                     (m_hlw8032Reg[IND_CURRENT + 1] << 8) |
                     (m_hlw8032Reg[IND_CURRENT + 2]);
  uint32_t power   = (m_hlw8032Reg[IND_POWER] << 16) |
                     (m_hlw8032Reg[IND_POWER + 1] << 8) |
                     (m_hlw8032Reg[IND_POWER + 2]);
  uint32_t voltageParam = (m_hlw8032Reg[IND_VPARAM] << 16) |
                          (m_hlw8032Reg[IND_VPARAM + 1] << 8) |
                          (m_hlw8032Reg[IND_VPARAM + 2]);
  uint32_t currentParam = (m_hlw8032Reg[IND_CPARAM] << 16) |
                          (m_hlw8032Reg[IND_CPARAM + 1] << 8) |
                          (m_hlw8032Reg[IND_CPARAM + 2]);
  uint32_t powerParam   = (m_hlw8032Reg[IND_PPARAM] << 16) |
                          (m_hlw8032Reg[IND_PPARAM + 1] << 8) |
                          (m_hlw8032Reg[IND_PPARAM + 2]);
  uint16_t pf = (m_hlw8032Reg[IND_PF] << 8) | (m_hlw8032Reg[IND_PF + 1]);
  uint32_t pfCount;
  float oneKwhCount;

  if(state == HLW8032_REG_NOT_OVERFLOW)
  {
    /* Calculate voltage */
    if(IS_VOLTAGE_UPDATE(m_hlw8032Reg))
    {
      m_powerMeterData.voltage = (voltageParam * 100 * VOLTAGE_COEFFICIENT) / voltage;
      m_powerMeterData.voltage = m_powerMeterData.voltage * 10;
    }
    /* Calculate current */
    if(IS_CURRENT_UPDATE(m_hlw8032Reg))
    {
      m_powerMeterData.current = (currentParam * 100 * CURRENT_COEFFICIENT) / current;
      m_powerMeterData.current = m_powerMeterData.current *10;
    }
    /* Calculate activePower */
    if(IS_POWER_UPDATE(m_hlw8032Reg))
    {
      m_powerMeterData.activePower = (powerParam * 100 * VOLTAGE_COEFFICIENT * CURRENT_COEFFICIENT) / power;
      m_powerMeterData.activePower = m_powerMeterData.activePower *10;
    }
  }
  else if(state == HLW8032_REG_OVERFLOW)
  {
    /* Check voltage */
    if(IS_VOLTAGE_OVERFLOW(m_hlw8032Reg))
    {
      m_powerMeterData.voltage = 0.0;
    }
    else
    {
      if(voltage > REG_OVERFLOW_TIME)
      {
        m_powerMeterData.voltage = 0.0;
      }
    }

    /* Check current */
    if(IS_CURRENT_OVERFLOW(m_hlw8032Reg))
    {
      m_powerMeterData.current = 0.0;
    }
    else
    {
      if(current > REG_OVERFLOW_TIME)
      {
        m_powerMeterData.current = 0.0;
      }
    }

    /* Check activePower */
    if(IS_POWER_OVERFLOW(m_hlw8032Reg))
    {
      m_powerMeterData.activePower = 0.0;
    }
    else
    {
      if(power > REG_OVERFLOW_TIME)
      {
        m_powerMeterData.activePower = 0.0;
      }
    }
  }

  /* Calculate apparent power */
  m_powerMeterData.apparentPower = (m_powerMeterData.voltage * m_powerMeterData.current) / 1000.0f;

  /* Calculate power factor */
  m_powerMeterData.powerFactor = m_powerMeterData.activePower / m_powerMeterData.apparentPower;

  /* Calculate electrical quantity */
  if(m_powerMeterData.pfFlag != PF_VALUE(m_hlw8032Reg))
  {
    /* Update pf negation count */
    m_powerMeterData.pfFlag = PF_VALUE(m_hlw8032Reg);
    m_powerMeterData.pfNegationCount++;
  }

  pfCount = m_powerMeterData.pfNegationCount * 65536 + pf;
  oneKwhCount = (powerParam * VOLTAGE_COEFFICIENT * CURRENT_COEFFICIENT);
  oneKwhCount = 1000000000.0f / oneKwhCount;
  oneKwhCount = oneKwhCount * 3600.0f;
  m_powerMeterData.Kwh = pfCount / oneKwhCount;
}
