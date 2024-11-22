/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "DACx1416_dep.h"
#include "DACx1416.h"
#include "DACx1416_port.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "fifo_queue.h"
#include "slip.h"
//#include "cmdParser.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;
/* USER CODE BEGIN PV */
DACx1416 dacUnit;
volatile uint8_t dacUseCRC = 0;
volatile uint8_t dacTransferComplete = 1;

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
FIFOQueue *cdcRxQueue;
SLIP_HandleTypeDef slip;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





// Function to receive a single character from the CDC Rx queue
// Function to receive a single character from the CDC Rx queue (blocking)
char cdc_recv_char(void) {
    while (isEmpty(cdcRxQueue)) {
        // Wait for data to be available in the queue
        HAL_Delay(1); // Sleep for 1 ms (adjust based on your system requirements)
    }

    uint8_t *value = (uint8_t *)dequeue(cdcRxQueue);  // Dequeue a uint8_t
    if (!value) {
        //printf("Failed to dequeue from CDC Rx queue.\n");
        return 0xFF;  // Return an error value if dequeue fails
    }

    uint8_t result = *value;  // Dereference the pointer to get the uint8_t value
    //*value = NULL;
    //free(value);              // Free the allocated memory
    return result;            // Return the dequeued value
}

void cdc_send_char(char ch) {
    // Cast the char to uint8_t* by passing the address of ch
    while (CDC_Transmit_FS((uint8_t*)&ch, 1) != USBD_OK) {
        // Send the character as a byte over USB
    }
}
// Function called when data is received via USB
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
	//HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
	//HAL_NVIC_EnableIRQ(SPI1_IRQn);
	dacUnit.port.SPI_transmit = DACx1416_SPI_transmit;
	dacUnit.port.SPI_receive = DACx1416_SPI_receive;
	dacUnit.port.SPI_transmitReceive = DACx1416_SPI_transmitReceive;
	dacUnit.port.nCS = DACx1416_nCS;
	dacUnit.port.nLDAC = DACx1416_nLDAC;
	dacUnit.port.nRESET = DACx1416_nRESET;
	dacUnit.port.nCLR = DACx1416_nCLR;
	dacUnit.port.TGL = DACx1416_tgl;
	dacUnit.port.calculate_crc8 = DACx1416_calculate_crc8;

	DACx1416_initialize(&dacUnit);
//
//	uint16_t temp;
//	DACx1416_read_register(&dacUnit, DACx1416_REG_SPICONFIG, &temp);
//	//printf("SPICONFIG : %X\r\n", temp);
//
//	DACx1416_read_register(&dacUnit, DACx1416_REG_SPICONFIG, &temp);
//	//printf("SPICONFIG : %X\r\n", temp);
//
//	DACx1416_read_register(&dacUnit, DACx1416_REG_SPICONFIG, &temp);
//	//printf("SPICONFIG : %X\r\n", temp);
//
//	temp = 0x1A2A;
//	DACx1416_write_register(&dacUnit, DACx1416_REG_DACPWDWN, &temp);
//	DACx1416_read_register(&dacUnit, DACx1416_REG_DACPWDWN, &temp);
//	//printf("AHA 1 : %X\r\n", temp);
//
//	temp = 0x2A3A;
//	DACx1416_write_register(&dacUnit, DACx1416_REG_DACPWDWN, &temp);
//	DACx1416_read_register(&dacUnit, DACx1416_REG_DACPWDWN, &temp);
//	//printf("AHA 2 : %X\r\n", temp);
//
//	DACx1416_devicePowerDown(&dacUnit, 1);
//	DACx1416_devicePowerDown(&dacUnit, 0);
//
//	DACx1416_read_register(&dacUnit, DACx1416_REG_SPICONFIG, &temp);
//	//printf("SPI CONFIG : %X\r\n", temp);
//
//	temp = 0x3A4A;
//	DACx1416_write_register(&dacUnit, DACx1416_REG_DACPWDWN, &temp);
//	DACx1416_read_register(&dacUnit, DACx1416_REG_DACPWDWN, &temp);
//	//printf("AHA 3 : %X\r\n", temp);
//
//	//DACx1416_devicePowerDown(&dacUnit, 0);
//	DACx1416_channelRange(&dacUnit, DACx1416_DAC0, DACx1416_DACRANGE_n5_to_p5);
//	DACx1416_channelEnable(&dacUnit, DACx1416_DAC0, 1);
//
//	DACx1416_crcEnable(&dacUnit, 1);
//
//	DACx1416_channelValue(&dacUnit, DACx1416_DAC0, 1023);
//	DACx1416_channelValue(&dacUnit, DACx1416_DAC0, 16383);
//	DACx1416_channelValue(&dacUnit, DACx1416_DAC0, 44000);
//	DACx1416_channelValue(&dacUnit, DACx1416_DAC0, 60000);
//	DACx1416_channelValue(&dacUnit, DACx1416_DAC0, 65535);




	cdcRxQueue = createQueue();
	if (!cdcRxQueue) {
		//printf("Failed to initialize the global queue.\n");
		return 1;
	}

	slip.recv_char = cdc_recv_char;
	slip.send_char = cdc_send_char;
//
//
//	uint16_t temp = 0;
//	DACx1416_read_register(&dac, DACx1416_REG_DEVICEID, &temp);
//	DACx1416_read_register(&dac, DACx1416_REG_STATUS, &temp);
//	DACx1416_read_register(&dac, DACx1416_REG_SPICONFIG, &temp);
//	DACx1416_read_register(&dac, DACx1416_REG_GENCONFIG, &temp);
//
//	temp = 0x0AA4;
//	DACx1416_write_register(&dac, DACx1416_REG_SPICONFIG, &temp);
//
//	DACx1416_crcAlarmEnable(&dac, 1);
//
//	DACx1416_read_register(&dac, DACx1416_REG_DEVICEID, &temp);
//	DACx1416_read_register(&dac, DACx1416_REG_STATUS, &temp);
//	DACx1416_read_register(&dac, DACx1416_REG_SPICONFIG, &temp);
//	DACx1416_read_register(&dac, DACx1416_REG_GENCONFIG, &temp);
//
//	DACx1416_deviceID_t deviceID;
//	DACx1416_get_device_id(&dac, &deviceID);
//
//	asm("nop");
//	asm("nop");
//	asm("nop");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t slipRxData[1024];
	uint8_t slipTxData[1024] = "ABCDE";




	while (1) {

		//char c = cdc_recv_char();
		//cdc_send_char(c);

		uint32_t len = slip_recv_packet(&slip, slipRxData, 1024);
		parseCmd(&slip, slipRxData, len);


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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 7;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
  hcrc.Init.InitValue = 0;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma1_channel1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Request = DMA_REQUEST_0;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DAC_nCS_GPIO_Port, DAC_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DAC_TOGGLE0_Pin|DAC_TOGGLE1_Pin|DAC_TOGGLE2_Pin|DAC_nLDAC_Pin
                          |DAC_nRESET_Pin|DAC_nCLR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DAC_nCS_Pin */
  GPIO_InitStruct.Pin = DAC_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DAC_nCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_TOGGLE0_Pin DAC_TOGGLE1_Pin DAC_TOGGLE2_Pin DAC_nLDAC_Pin
                           DAC_nRESET_Pin DAC_nCLR_Pin */
  GPIO_InitStruct.Pin = DAC_TOGGLE0_Pin|DAC_TOGGLE1_Pin|DAC_TOGGLE2_Pin|DAC_nLDAC_Pin
                          |DAC_nRESET_Pin|DAC_nCLR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DAC_nALMOUT_Pin */
  GPIO_InitStruct.Pin = DAC_nALMOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DAC_nALMOUT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
	// Write character to ITM ch.0
	ITM_SendChar(ch);
	return (ch);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
	while (1) {
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
