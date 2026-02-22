/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sx127x.h"
#include "lora_protocol.h"
#include <stdio.h>
#include <string.h>
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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
SX127x_t lora;
LoRa_Protocol_t protocol;
char uart_buf[256];
char uart_rx_buffer[256];
volatile uint16_t uart_rx_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Helper function to send debug messages via UART
void UART_Print(const char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// Callback when data is received from LoRa
void OnDataReceived(char *msg, uint8_t len) {
    UART_Print("Utente 2: ");  // Fixed: Arduino is User 2
    UART_Print(msg);
    UART_Print("\r\n");
    
    // Debug info
    snprintf(uart_buf, sizeof(uart_buf), "[DEBUG] Ricevuto %d byte via LoRa\r\n", len);
    UART_Print(uart_buf);
}

// Callback when ACK is received (optional, silent)
void OnAckReceived(uint8_t seq) {
    // ACKs are received silently, no print
}

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
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Longer delay for system and LoRa module to stabilize
  HAL_Delay(500);

  // Simple test message
  const char *test_msg = "\r\n\r\n*** STM32 LoRa Chat System ***\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)test_msg, strlen(test_msg), 1000);

  // Initialize LoRa structure
  lora.hspi = &hspi2;
  lora.nss_port = NSS_GPIO_Port;
  lora.nss_pin = NSS_Pin;
  lora.rst_port = RST_GPIO_Port;
  lora.rst_pin = RST_Pin;
  lora.dio0_port = DI0_GPIO_Port;
  lora.dio0_pin = DI0_Pin;

  UART_Print("Inizializzazione LoRa...\r\n");

  // Proper reset sequence with longer delays
  HAL_GPIO_WritePin(lora.rst_port, lora.rst_pin, GPIO_PIN_RESET);
  HAL_Delay(50);  // Hold reset longer
  HAL_GPIO_WritePin(lora.rst_port, lora.rst_pin, GPIO_PIN_SET);
  HAL_Delay(200); // Wait longer for module to boot

  // Check version
  uint8_t version = SX127x_ReadRegister(&lora, REG_VERSION);
  snprintf(uart_buf, sizeof(uart_buf), "Chip SX127x: 0x%02X ", version);
  UART_Print(uart_buf);
  
  if (version == 0x12) {
      UART_Print("OK!\r\n");
  } else {
      UART_Print("ERRORE!\r\n");
      UART_Print("Verifica connessioni hardware.\r\n");
      Error_Handler();
  }

  // Initialize LoRa at 433 MHz
  if (SX127x_Begin(&lora, 433E6)) {
      UART_Print("LoRa 433 MHz: OK\r\n");
  } else {
      UART_Print("LoRa init: ERRORE\r\n");
      Error_Handler();
  }

  // Set sync word to match Arduino (0xF3)
  SX127x_SetSyncWord(&lora, 0xF3);
  UART_Print("Sync Word: 0xF3\r\n");

  // Initialize protocol
  LoRa_Protocol_Init(&protocol, &lora);
  protocol.data_received_callback = OnDataReceived;
  protocol.ack_received_callback = OnAckReceived;
  
  UART_Print("\r\n=== SISTEMA PRONTO ===\r\n");
  UART_Print("Digita un messaggio e premi INVIO.\r\n\r\n");

  // Start receiving
  SX127x_Receive(&lora);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // 1. ASCOLTO DALLA RADIO (framed packets)
    LoRa_Protocol_ProcessIncoming(&protocol);
    
    // 2. INVIO DA SERIALE (framed packets)
    // Read bytes one by one in a tight loop to catch burst transmissions
    uint8_t inChar;
    uint32_t start_time = HAL_GetTick();
    
    // Try to read for up to 100ms to catch all bytes in a burst
    while ((HAL_GetTick() - start_time) < 100) {
        if (HAL_UART_Receive(&huart2, &inChar, 1, 1) == HAL_OK) {
            // Got a byte
            start_time = HAL_GetTick(); // Reset timer on each byte
            
            if (inChar == '\n' || inChar == '\r') {
                if (uart_rx_index > 0) {
                    // Null-terminate
                    uart_rx_buffer[uart_rx_index] = '\0';
                    
                    // Debug
                    snprintf(uart_buf, sizeof(uart_buf), "\r\n[DEBUG] Invio '%s' (%d byte) via LoRa\r\n", 
                             uart_rx_buffer, uart_rx_index);
                    UART_Print(uart_buf);
                    
                    // Send via LoRa
                    bool sent = LoRa_Protocol_SendData(&protocol, ADDR_ARDUINO, uart_rx_buffer);
                    
                    if (sent) {
                        // Echo - STM32 is User 1
                        UART_Print("Utente 1: ");
                        UART_Print(uart_rx_buffer);
                        UART_Print("\r\n\r\n");
                    } else {
                        UART_Print("[ERRORE] Invio fallito\r\n\r\n");
                    }
                    
                    // Reset buffer
                    uart_rx_index = 0;
                    
                    // Back to RX mode
                    SX127x_Receive(&lora);
                    break; // Exit read loop after sending
                }
            } else if (inChar >= 32 && inChar < 127) {
                // Accumulate printable character
                if (uart_rx_index < sizeof(uart_rx_buffer) - 1) {
                    uart_rx_buffer[uart_rx_index++] = inChar;
                }
            }
        } else {
            // No byte available, check if we have data to send
            if (uart_rx_index > 0 && (HAL_GetTick() - start_time) > 50) {
                // 50ms timeout with partial data - might be missing newline
                uart_rx_buffer[uart_rx_index] = '\0';
                
                snprintf(uart_buf, sizeof(uart_buf), "\r\n[DEBUG] Invio (timeout) '%s' (%d byte)\r\n", 
                         uart_rx_buffer, uart_rx_index);
                UART_Print(uart_buf);
                
                LoRa_Protocol_SendData(&protocol, ADDR_ARDUINO, uart_rx_buffer);
                
                UART_Print("Utente 1: ");
                UART_Print(uart_rx_buffer);
                UART_Print("\r\n\r\n");
                
                uart_rx_index = 0;
                SX127x_Receive(&lora);
                break;
            }
        }
    }
    
    // Small delay
    HAL_Delay(10);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RST_Pin|DI0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin RST_Pin DI0_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|RST_Pin|DI0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
