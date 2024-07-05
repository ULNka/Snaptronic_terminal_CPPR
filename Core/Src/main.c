/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "init.h"
#include "lcd1602.h"
#include "usb_modbus.h"
#include "slaveDevices.h"
#include "Modbus.h"
#include "terminal_util_DG.h"
#include "GyverMotor.h"
#include "wiegand.h"
#include "ledUART.h"
//#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REMOTE 1
#define MASTER_KEY 6393037			//Код мастер ключа
#define FIRST_KEY 6393038			//Код первого ключа
#define D0        D0_Pin
#define D1        D1_Pin
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SysUtil */
osThreadId_t SysUtilHandle;
const osThreadAttr_t SysUtil_attributes = {
  .name = "SysUtil",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IOUtil */
osThreadId_t IOUtilHandle;
const osThreadAttr_t IOUtil_attributes = {
  .name = "IOUtil",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UsbSlave */
osThreadId_t UsbSlaveHandle;
const osThreadAttr_t UsbSlave_attributes = {
  .name = "UsbSlave",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotProcess */
osThreadId_t RobotProcessHandle;
const osThreadAttr_t RobotProcess_attributes = {
  .name = "RobotProcess",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ClimatControl */
osThreadId_t ClimatControlHandle;
const osThreadAttr_t ClimatControl_attributes = {
  .name = "ClimatControl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for SecurityTask */
osThreadId_t SecurityTaskHandle;
const osThreadAttr_t SecurityTask_attributes = {
  .name = "SecurityTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LedStrip */
osThreadId_t LedStripHandle;
const osThreadAttr_t LedStrip_attributes = {
  .name = "LedStrip",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ModbusMaster */
osThreadId_t ModbusMasterHandle;
const osThreadAttr_t ModbusMaster_attributes = {
  .name = "ModbusMaster",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for usbLedTimer */
osTimerId_t usbLedTimerHandle;
const osTimerAttr_t usbLedTimer_attributes = {
  .name = "usbLedTimer"
};
/* Definitions for usbSem */
osSemaphoreId_t usbSemHandle;
const osSemaphoreAttr_t usbSem_attributes = {
  .name = "usbSem"
};
/* USER CODE BEGIN PV */
uint8_t lightEffect=1;		//0-нет связи, 1-свободно, 2-занято, 3-техобслуживание, 4-заезд авто, 5-открывание дверки, 6-двкерка открыта
uint8_t brightness = 205, doorMotorState = 0, alarm = 0, noUsbConnect = 0, rty;
uint8_t dataUartBuffer[8];
uint16_t dataUartSize;
volatile uint8_t adcDataIsReady=0;
volatile uint8_t wig_flag_inrt = 1;
uint16_t adc[2];
uint16_t ain1, ain2;
float temper, hum; 					//Переменные влажности и температуры встроенного датчика
char bufTemp[4];
uint8_t bufHum[4];
modbusHandler_t ModbusH2;
modbus_t robot2[6], gateIN[2], gateOUT[2], remote[2]; //Структуры для Modbus Master
uint16_t ModbusDATA[128]; //Буфер даннх для Modbus Master
//uint16_t robotInputs[1], gateINInputs[1], gateOUTInputs[1], remoteInputs[1], robotOutputs[1], gateINOutputs[1], gateOUTOutnputs[1], remoteOutputs[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);
void StartSysUtil(void *argument);
void StartIOUtil(void *argument);
void StartUsbSlave(void *argument);
void StartRobotProcess(void *argument);
void StartClimatControl(void *argument);
void StartSecurityTask(void *argument);
void StartLedStrip(void *argument);
void StartModbusMaster(void *argument);
void usbLedCallback(void *argument);

/* USER CODE BEGIN PFP */
int16_t map(int16_t input, int16_t inMin, int16_t inMax, int16_t outMin,
		int16_t outMax) {
	int16_t result;
	result = (((input - inMin) * (outMax - outMin)) / (inMax - inMin)) + outMin;
	return result;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//int _write(int file, char *ptr, int len){
//	int i=0;
//	for(i=0; i<len; i++){
//		ITM_SendChar((*ptr++));
//		return len;
//	}
//}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
        if(wig_flag_inrt && GPIO_Pin == D0)
        {
                ReadD0();
        }
        else if(wig_flag_inrt && GPIO_Pin == D1)
        {
                ReadD1();
        }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
    	adcDataIsReady = 1;
    }
}
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ){
	printf("we have a problem...");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
struct dataMain settings;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  usbReInit();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
//  HAL_GPIO_WritePin(WP_GPIO_Port, WP_Pin, 1);
  ledInit();
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 2); // стартуем АЦП
  /* пока что так, затем реализовать чтение настроек из EEPROM */
  settings.timeToEntance=40;				//Время заезда
  settings.timeToExit=40;
  settings.startDelay=3;
  settings.gateMode=1;
  settings.coolerTemp=30;
  settings.heaterTemp = 5;
//  timeToEntance = settings.timeToEntance;
  timeToEntance = 30;
  timeToExit = settings.timeToExit;
  startDelay = settings.startDelay;
  gateMode = settings.gateMode;
  coolerTemp = settings.coolerTemp;
  heaterTemp = settings.heaterTemp;
  readyDelay = 5000;

  /* Master initialization */
  ModbusH2.uModbusType = MB_MASTER;
  ModbusH2.port =  &huart3;
  ModbusH2.u8id = 0; // For master it must be 0
  ModbusH2.u16timeOut = 200;
//  ModbusH2.EN_Port = NULL;
  ModbusH2.EN_Port = RE2_GPIO_Port;
  ModbusH2.EN_Pin = RE2_Pin;
  ModbusH2.u16regs = ModbusDATA;
  ModbusH2.u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
  ModbusH2.xTypeHW = USART_HW_DMA;
  //Initialize Modbus library
  ModbusInit(&ModbusH2);
  //Start capturing traffic on serial Port
  ModbusStart(&ModbusH2);
//  HAL_UARTEx_Receive_DMA(&huart4, dataUartBuffer, 5);
//  HAL_UART_Receive_IT(&huart4, dataUartBuffer, 5);
  HAL_UARTEx_ReceiveToIdle_IT(&huart4, dataUartBuffer, 5);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of usbSem */
  usbSemHandle = osSemaphoreNew(1, 0, &usbSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of usbLedTimer */
  usbLedTimerHandle = osTimerNew(usbLedCallback, osTimerOnce, NULL, &usbLedTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
    robotProcessingInit();
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SysUtil */
  SysUtilHandle = osThreadNew(StartSysUtil, NULL, &SysUtil_attributes);

  /* creation of IOUtil */
  IOUtilHandle = osThreadNew(StartIOUtil, NULL, &IOUtil_attributes);

  /* creation of UsbSlave */
  UsbSlaveHandle = osThreadNew(StartUsbSlave, NULL, &UsbSlave_attributes);

  /* creation of RobotProcess */
  RobotProcessHandle = osThreadNew(StartRobotProcess, NULL, &RobotProcess_attributes);

  /* creation of ClimatControl */
  ClimatControlHandle = osThreadNew(StartClimatControl, NULL, &ClimatControl_attributes);

  /* creation of SecurityTask */
  SecurityTaskHandle = osThreadNew(StartSecurityTask, NULL, &SecurityTask_attributes);

  /* creation of LedStrip */
  LedStripHandle = osThreadNew(StartLedStrip, NULL, &LedStrip_attributes);

  /* creation of ModbusMaster */
  ModbusMasterHandle = osThreadNew(StartModbusMaster, NULL, &ModbusMaster_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_9;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 2300;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 89;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
  /* DMA2_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BUZZER_Pin|QOUT1_Pin|OUT1_Pin|OUT2_Pin
                          |OUT3_Pin|OUT4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT5_Pin|OUT6_Pin|WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RE2_Pin|LED5_Pin|LED4_Pin|LED3_Pin
                          |LED2_Pin|LED1_Pin|PWRLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RE1_GPIO_Port, RE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_Pin QOUT1_Pin OUT1_Pin OUT2_Pin
                           OUT3_Pin OUT4_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|QOUT1_Pin|OUT1_Pin|OUT2_Pin
                          |OUT3_Pin|OUT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DCOK_Pin */
  GPIO_InitStruct.Pin = DCOK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DCOK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT5_Pin OUT6_Pin */
  GPIO_InitStruct.Pin = OUT5_Pin|OUT6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN8_Pin IN7_Pin VUSB_Pin */
  GPIO_InitStruct.Pin = IN8_Pin|IN7_Pin|VUSB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D0_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RE2_Pin */
  GPIO_InitStruct.Pin = RE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RE2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED5_Pin LED3_Pin LED2_Pin LED1_Pin
                           PWRLED_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|LED3_Pin|LED2_Pin|LED1_Pin
                          |PWRLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_Pin IN3_Pin IN2_Pin IN1_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|IN3_Pin|IN2_Pin|IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RE1_Pin */
  GPIO_InitStruct.Pin = RE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RE1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WP_Pin */
  GPIO_InitStruct.Pin = WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSysUtil */
/**
* @brief Function implementing the SysUtil thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSysUtil */
void StartSysUtil(void *argument)
{
  /* USER CODE BEGIN StartSysUtil */
  /* Infinite loop */
  for(;;)
  {	HAL_GPIO_TogglePin(PWRLED_GPIO_Port, PWRLED_Pin);
	HAL_IWDG_Refresh(&hiwdg);

//	printf("Hello from SWO 2");
osDelay(800);
  }
  /* USER CODE END StartSysUtil */
}

/* USER CODE BEGIN Header_StartIOUtil */
/**
* @brief Function implementing the IOUtil thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIOUtil */
void StartIOUtil(void *argument)
{
  /* USER CODE BEGIN StartIOUtil */

	/* Infinite loop */
	for (;;)
  {
		if (adcDataIsReady) {
			adcDataIsReady = 0;
			HAL_ADC_Stop_DMA(&hadc1); // это необязательно
			ain1 = adc[0];
			ain2 = adc[1];
//			snprintf(trans_str, 63, "ADC %d %d\n", (uint16_t) adc[0], (uint16_t) adc[1]);
//			HAL_UART_Transmit(&huart1, (uint8_t*) trans_str, strlen(trans_str),	1000);
			adc[0] = 0;
			adc[1] = 0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc, 2);
		}
//		if (getCarIsideStatus()) {
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, !isReady);
//		} else {
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
//		};
//		if (getPhotoInSensor()) {
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
//		} else {
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
//		};
//		if (getPhotoOutSensor()) {
//			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
//		} else {
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, !controllerError);
//		};

    osDelay(100); // Поменять на 10
  }
  /* USER CODE END StartIOUtil */
}

/* USER CODE BEGIN Header_StartUsbSlave */
/**
* @brief Function implementing the UsbSlave thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsbSlave */
void StartUsbSlave(void *argument)
{
  /* USER CODE BEGIN StartUsbSlave */
  /* Infinite loop */
  for(;;)
  {
	osSemaphoreAcquire(usbSemHandle, portMAX_DELAY);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 0);
	osTimerStart(usbLedTimerHandle, 80);
	usbModbusProcessing();
    osDelay(1);
  }
  /* USER CODE END StartUsbSlave */
}

/* USER CODE BEGIN Header_StartRobotProcess */
/**
* @brief Function implementing the RobotProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotProcess */
void StartRobotProcess(void *argument)
{
  /* USER CODE BEGIN StartRobotProcess */
//	uint8_t   value;

  /* Infinite loop */
  for(;;)
  {
	  dataFromUsbProcessing();
	  gatePhotoHandler();
	  robotHandler();
#ifdef REMOTE
	  remoteHandler();
#endif

	  usbDiscreteRegister[6] = isReady;
	  usbDiscreteRegister[7] = controllerError;
	  usbDiscreteRegister[8] = techBreack; // технический перерыв

    osDelay(1);
  }
  /* USER CODE END StartRobotProcess */
}

/* USER CODE BEGIN Header_StartClimatControl */
/**
* @brief Function implementing the ClimatControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartClimatControl */
void StartClimatControl(void *argument)
{
  /* USER CODE BEGIN StartClimatControl */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	uint16_t power;
	const uint8_t reqReset = 0xFE;				//Команда сброса
	const uint8_t reqTemp = 0xE3;				//Команда запроса температуры
	const uint8_t reqHum = 0xE5;				//Команда запроса влажности
	const uint8_t sens_addr = (0x40 << 1);
	HAL_I2C_Master_Transmit(&hi2c1, sens_addr, &reqReset, 1, 200); // Сброс встроенного датчика
	uint16_t varTemp = 0;
	uint16_t varHum = 0;
	uint8_t data[3];
//	uint16_t tempVarArr[2];
	I2C_send(0b00110000, 0);   // 8ми битный интерфейс
	I2C_send(0b00000010, 0);   // установка курсора в начале строки
	I2C_send(0b00001100, 0);   // нормальный режим работы, выкл курсор
	I2C_send(0b00000001, 0);   // очистка дисплея
	/* Infinite loop */
	for (;;) {
		HAL_I2C_Master_Transmit(&hi2c1, sens_addr, (uint8_t[]){reqTemp}, 1, 200); //Запрос температуры
		HAL_I2C_Master_Receive(&hi2c1, sens_addr, data, 3, 200);

		data[1] &= 0xFC;  //Зануляем последние два бита
		varTemp = data[1] | (data[0] << 8);
		temper = (((float) varTemp / 65536) * 175.72) - 46.85; //Преобразуем в градусы по формуле из даташита

		HAL_I2C_Master_Transmit(&hi2c1, sens_addr, &reqHum, 1, 200); //Запрос влажности
		HAL_I2C_Master_Receive(&hi2c1, sens_addr, data, 3, 200);

		data[1] &= 0xFC; //Зануляем последние два бита
		varHum = data[1] | (data[0] << 8);
		hum = (((float) varHum / 65536) * 125.0) - 6.0; //Преобразуем в проценты по формуле из даташита
				/*
				 memcpy(tempVarArr, &temper, sizeof(temper)); //Разбиваем float32 на два uint16 и заносим во временный буфер
				 usbHoldingRegister[10] = tempVarArr[1];
				 usbHoldingRegister[11] = tempVarArr[0];
				 int16_t rtu = (int16_t)hum;
				 rtu *=10;

				 memcpy(tempVarArr, &rtu, sizeof(rtu)); //Разбиваем float32 на два uint16 и заносим во временный буфер
				 usbHoldingRegister[12] = tempVarArr[1];
				 usbHoldingRegister[13] = tempVarArr[0];*/

				 sprintf((char*)bufTemp, "%.1f", temper); //Преобразуем float32 в строку(массив символов)
				 sprintf((char*)bufHum, "%.1f", hum); // @suppress("Float formatting support")

		if ((int16_t)temper <= heaterTemp) {
			HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET);
		} else if ((int16_t)temper >= heaterTemp+1) {
			HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);
		}

		if (temper > 50.0) {
			power = 500;
		} else if (temper > 40.0f) {
			power = 400;
		} else if (temper > 32.0f ){
			power = 300;
		} else if (temper > 28.0f ){
			power = 200;
		} else power = 0;

		TIM4->CCR3 = power;

//		I2C_send(0b10000000, 0);   // переход на 1 строку
//			LCD_SendString("   AquaRobot ");
//			I2C_send(0b11000000, 0);   // переход на 2 строку
//			LCD_SendString((char*)bufTemp);
//			LCD_SendString("`C     ");
//			LCD_SendString((char*)bufHum);
//			LCD_SendString("%");
		osDelay(2000);
	}
  /* USER CODE END StartClimatControl */
}

/* USER CODE BEGIN Header_StartSecurityTask */
/**
* @brief Function implementing the SecurityTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSecurityTask */
void StartSecurityTask(void *argument)
{
  /* USER CODE BEGIN StartSecurityTask */
	enum direction{
		OPENING,
		CLOSING,
		INITIALIZATION
	};
	enum position{
		UNKNOWN,
		OPEN,
		CLOSED,
		MIDDLE,
	};
	enum motor {
		STOP,
		RUN
	};
	uint8_t status=0, direction, doorPosition, limitSwitch, switchRTig, switchFlag, zeroCurrent, overCurrent;
	uint16_t speed;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	uint32_t timer, beepTimer;
	if (!HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin)) { //Чекаем концевик замка двЁрки
		doorPosition = CLOSED;
	} else doorPosition = UNKNOWN;
	char trans_str[13];
	const uint16_t maxCurrent = 680;
	const uint16_t nullCurrent = 480;
	const uint16_t maxSpeed = 500;
  /* Infinite loop */
  for(;;)
  {

	  limitSwitch = !HAL_GPIO_ReadPin(IN1_GPIO_Port, IN1_Pin); // Положение коцевика замка
	  if(!switchFlag && limitSwitch) {
		  switchFlag = 1;
		  switchRTig = 1;
	  }
	  else if(switchFlag && !limitSwitch){
		  switchFlag = 0;
		  switchRTig = 0;
	  }
	  /*------------*/
	if (wig_available() ) {
		wig_flag_inrt = 0;
		uint32_t wcode = getCode();

		/*----ОТЛАДКА----*/
		/*int16_t wtype = getWiegandType();
		wig_flag_inrt = 1;
		char str[64] = { 0, };
		snprintf(str, 64, "HEX=0x%lX DEC=%lu, Protokol Wiegand-%d\n", wcode,
				wcode, wtype);
		HAL_UART_Transmit(&huart4, (uint8_t*) str, strlen(str), 1000);*/
		/*----КОНЕЦ БЛОКА ОТЛАДК�?----*/

		if (wcode == MASTER_KEY || wcode == FIRST_KEY) {
			//Подадим предупреждающий сигнал
			if(status == STOP){
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
			osDelay(1000);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
			osDelay(500);
			}

			timer = HAL_GetTick();
			beepTimer = timer;
			speed = maxSpeed;
			if(doorPosition != UNKNOWN){
				if(getState() !=0) {
					status = STOP;
					doorPosition = MIDDLE;
				}else if (doorPosition == CLOSED){
					direction = OPENING;
					status = RUN;
					doorPosition = MIDDLE;
				} else if(doorPosition == OPEN){
					direction = CLOSING;
					status = RUN;
					doorPosition = MIDDLE;
				} else if(doorPosition == MIDDLE){
					speed=200;
					if(direction == CLOSING) direction = OPENING;
					else direction = CLOSING;
					status = RUN;
				}
			}else {
				direction = INITIALIZATION;
				status = RUN;
			}
		} else {
			// Сигнал неверного ключа
			for(int8_t i = 0; i<3; i++){
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
				osDelay(100);
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
				osDelay(100);
			}
		}
	}
	doorMotorState = status;
//---------Контроль тока мотора-------------//
	if(status == RUN && (HAL_GetTick()-timer >= 2000)){
		if(ain1 < nullCurrent) zeroCurrent = 1;
		else zeroCurrent = 0;

		if(ain1>=maxCurrent) overCurrent = 1;
		else overCurrent = 0;
	}
	if(status == RUN && overCurrent){
		status = STOP;
		doorPosition=MIDDLE;
		setSpeed(0);
		osDelay(350);
		if(direction == CLOSING) setMotorDirection(NORMAL);
		else setMotorDirection(REVERSE);
		overCurrent = 0;
		setSpeed(220);
		osDelay(3000);
		setSpeed(0);
	}

		if (status == RUN) {
			switch (direction) {
			case OPENING:
				openDoor(3200, timer, speed);
				break;
			case CLOSING:
				closeDoor(4000, timer, speed);
				break;
			case INITIALIZATION:
				initDoorPosition(190, 10000, timer);
				break;
			}
			if (direction == CLOSING && (!getState() || switchRTig || zeroCurrent)) {
				setSmoothSpeed(60);
				osDelay(300);
				smoothTick(0);
				status = STOP;
				switchRTig = 0;
				doorPosition = CLOSED;
			} else if (direction == OPENING && (!getState() || zeroCurrent)) {
				status = STOP;
				doorPosition = OPEN;
			} else if (direction == INITIALIZATION && (!getState() || zeroCurrent)) {
				status = STOP;
				doorPosition = OPEN;
			}
			if(HAL_GetTick() - beepTimer >= 600){
				beepTimer = HAL_GetTick();
				HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
			}
		} else if (status == STOP) {
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
			setSmoothSpeed(60);
			smoothTick(0);
			switchRTig = 0;
			zeroCurrent = 0;
			overCurrent = 0;
		}

		snprintf(trans_str, 13, "ADC %d   ", (uint16_t) ain1);
		I2C_send(0b10000000, 0);   // переход на 1 строку
		LCD_SendString((char*)trans_str);
//-----------------------------------------//

	  /*--------------*/
	//Закрытие дверки кнопкой
	  if(!HAL_GPIO_ReadPin(IN2_GPIO_Port, IN2_Pin) && doorPosition == OPEN){
		// toClose = 1;
	  }
	  if(doorPosition == OPEN) {
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0); // отладка
		  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, 1); // реле подсветки
	  } else {
		  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		  HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, 0);
	  }

	  if(doorPosition == CLOSED) HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	  else HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);


    osDelay(10);
  }
  /* USER CODE END StartSecurityTask */
}

/* USER CODE BEGIN Header_StartLedStrip */
/**
* @brief Function implementing the LedStrip thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedStrip */
void StartLedStrip(void *argument)
{
  /* USER CODE BEGIN StartLedStrip */

  /* Infinite loop */
	for (;;) {
		if (dataUartSize > 4) {
			if (dataUartBuffer[0] == 0x40 && checkLedCRC(dataUartBuffer, dataUartSize) ) {

				switch (dataUartBuffer[2]){
				case 1:
//					lightEffect = rty; // отладка
					if(noUsbConnect) lightEffect = 9;
					if(doorMotorState) lightEffect = 5;
					if(alarm) lightEffect = 8;
					sendEffect(lightEffect);
					break;
				case 2:
					if(alarm) brightness = 250;
					sendBrightness(brightness);
				}
			}
			dataUartSize = 0;

		}
		osDelay(1);
  }
  /* USER CODE END StartLedStrip */
}

/* USER CODE BEGIN Header_StartModbusMaster */
/**
* @brief Function implementing the ModbusMaster thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartModbusMaster */
void StartModbusMaster(void *argument)
{
  /* USER CODE BEGIN StartModbusMaster */
	robot2[0].u8id = 1;
	robot2[0].u8fct = 4;
	robot2[0].u16RegAdd = 0x00;
	robot2[0].u16CoilsNo = 1;
	robot2[0].u16reg = robotInputs;
	robot2[0].u32CurrentTask = ModbusMasterHandle;

	gateIN[0].u8id = 2;
	gateIN[0].u8fct = 3;
	gateIN[0].u16RegAdd = 0x00;
	gateIN[0].u16CoilsNo = 1;
	gateIN[0].u16reg = gateINInputs;
	gateIN[0].u32CurrentTask = ModbusMasterHandle;

	gateOUT[0].u8id = 3;
	gateOUT[0].u8fct = 3;
	gateOUT[0].u16RegAdd = 0x00;
	gateOUT[0].u16CoilsNo = 1;
	gateOUT[0].u16reg = gateOUTInputs;
	gateOUT[0].u32CurrentTask = ModbusMasterHandle;

	remote[0].u8id = 10;
	remote[0].u8fct = 3;
	remote[0].u16RegAdd = 0x00;
	remote[0].u16CoilsNo = 1;
	remote[0].u16reg = remoteInputs;
	remote[0].u32CurrentTask = ModbusMasterHandle;

	robot2[1].u8id = 1;
	robot2[1].u8fct = 6;
	robot2[1].u16RegAdd = 0x01;
	robot2[1].u16CoilsNo = 1;
	robot2[1].u16reg = robotOutputs;
	robot2[1].u32CurrentTask = ModbusMasterHandle;

	gateIN[1].u8id = 2;
	gateIN[1].u8fct = 6;
	gateIN[1].u16RegAdd = 0x01;
	gateIN[1].u16CoilsNo = 1;
	gateIN[1].u16reg = gateINOutputs;
	gateIN[1].u32CurrentTask = ModbusMasterHandle;

	gateOUT[1].u8id = 3;
	gateOUT[1].u8fct = 6;
	gateOUT[1].u16RegAdd = 0x01;
	gateOUT[1].u16CoilsNo = 1;
	gateOUT[1].u16reg = gateOUTOutnputs;
	gateOUT[1].u32CurrentTask = ModbusMasterHandle;

	remote[1].u8id = 10;
	remote[1].u8fct = 6;
	remote[1].u16RegAdd = 0x01;
	remote[1].u16CoilsNo = 1;
	remote[1].u16reg = remoteOutputs;
	remote[1].u32CurrentTask = ModbusMasterHandle;
//	osSemaphoreRelease(ModbusH2.ModBusSphrHandle);
  /* Infinite loop */
	for (;;) {
//		osSemaphoreAcquire(ModbusH2.ModBusSphrHandle, 300);
//		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
			ModbusQuery(&ModbusH2, robot2[0]);
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
		osDelay(30);
//			osSemaphoreRelease(ModbusH2.ModBusSphrHandle);


//	osSemaphoreAcquire(ModbusH2.ModBusSphrHandle, 300);

			ModbusQuery(&ModbusH2, robot2[1]);
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
//			osSemaphoreRelease(ModbusH2.ModBusSphrHandle);
			osDelay(30);

//		if (osSemaphoreAcquire(ModbusH2.ModBusSphrHandle, 300) == pdTRUE) {

			ModbusQuery(&ModbusH2, gateIN[0]);
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
//		osDelay(delay);
			osDelay(30);
//			osSemaphoreRelease(ModbusH2.ModBusSphrHandle);
//		}

//		if (osSemaphoreAcquire(ModbusH2.ModBusSphrHandle, 300) == pdTRUE) {
			ModbusQuery(&ModbusH2, gateIN[1]);
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
			osDelay(30);
//			osSemaphoreRelease(ModbusH2.ModBusSphrHandle);
//		}
/*
//		if (osSemaphoreAcquire(ModbusH2.ModBusSphrHandle, 300) == pdTRUE) {
			ModbusQuery(&ModbusH2, gateOUT[0]);
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
			osDelay(30);
//			osSemaphoreRelease(ModbusH2.ModBusSphrHandle);
//		}

//		if (osSemaphoreAcquire(ModbusH2.ModBusSphrHandle, 300) == pdTRUE) {
			ModbusQuery(&ModbusH2, gateOUT[1]);
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
//			osSemaphoreRelease(ModbusH2.ModBusSphrHandle);
//		}
			osDelay(30);
*/
#ifdef REMOTE
			ModbusQuery(&ModbusH2, remote[0]);
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
			osDelay(30);

			ModbusQuery(&ModbusH2, remote[1]);
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
			osDelay(30);

#endif
	}
  /* USER CODE END StartModbusMaster */
}

/* usbLedCallback function */
void usbLedCallback(void *argument)
{
  /* USER CODE BEGIN usbLedCallback */
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, 1);
  /* USER CODE END usbLedCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
