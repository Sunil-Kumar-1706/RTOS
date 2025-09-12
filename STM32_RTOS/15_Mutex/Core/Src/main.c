#include "main.h"          // Main HAL header
#include "cmsis_os.h"      // CMSIS-RTOS header

#include "semphr.h"        // FreeRTOS semaphore/mutex support
#include <string.h>        // For string functions like strlen()

UART_HandleTypeDef huart2; // UART2 handle

osThreadId defaultTaskHandle; // Default task handle

SemaphoreHandle_t xUartMutex; // UART mutex handle

// Configure system clock
void SystemClock_Config(void);
// Initialize GPIO pins
static void MX_GPIO_Init(void);
// Initialize USART2
static void MX_USART2_UART_Init(void);
// Default task function
void StartDefaultTask(void const * argument);

// Send a string over UART
void uart_send(char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

// Task1 function
void Task1(void *argument)
{
    while(1)
    {
        // Take UART mutex
        if(xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE)
        {
            uart_send("Task 1: Hello from Task 1\r\n"); // Send message
            xSemaphoreGive(xUartMutex);                 // Release mutex
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
    }
}

// Task2 function
void Task2(void *argument)
{
    while(1)
    {
        // Take UART mutex
        if(xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE)
        {
            uart_send("Task 2: Hello from Task 2\r\n"); // Send message
            xSemaphoreGive(xUartMutex);                 // Release mutex
        }
        vTaskDelay(pdMS_TO_TICKS(1500)); // Wait 1.5 seconds
    }
}

int main(void)
{
  HAL_Init();             // Initialize HAL
  SystemClock_Config();   // Setup system clock
  MX_GPIO_Init();         // Initialize GPIO
  MX_USART2_UART_Init();  // Initialize UART2

  // Create mutex for UART
  xUartMutex = xSemaphoreCreateMutex();

  if(xUartMutex != NULL)
  {
      // Create Task1
      xTaskCreate(Task1, "Task1", 128, NULL, 1, NULL);
      // Create Task2
      xTaskCreate(Task2, "Task2", 128, NULL, 1, NULL);
  }

  // Start CMSIS-RTOS kernel
  osKernelStart();

  // should not reach here
  while (1)
  {
  }
}

// Configure system clock
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // Oscillator config struct
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // Clock config struct

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler(); // Error if power scaling fails
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Use HSI oscillator
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // Enable HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default trim
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // Source HSI
  RCC_OscInitStruct.PLL.PLLM = 1;                            // PLL divider M
  RCC_OscInitStruct.PLL.PLLN = 10;                           // PLL multiplier N
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;                // PLL divider P
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                // PLL divider Q
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                // PLL divider R
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(); // Error if oscillator config fails
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; // Clock domains
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;             // Use PLL as SYSCLK
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;                    // AHB prescaler
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;                     // APB1 prescaler
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;                     // APB2 prescaler

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler(); // Error if clock config fails
  }
}

// Initialize USART2
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;                        // Use USART2
  huart2.Init.BaudRate = 115200;                   // Set baud rate
  huart2.Init.WordLength = UART_WORDLENGTH_8B;     // 8-bit data
  huart2.Init.StopBits = UART_STOPBITS_1;          // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE;           // No parity
  huart2.Init.Mode = UART_MODE_TX_RX;              // Enable TX and RX
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;     // No flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16; // Oversampling 16
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // No 1-bit sampling
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // No advanced features
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler(); // Error if UART init fails
  }
}

// Initialize GPIO
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0}; // GPIO config struct

  __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable GPIOC clock
  __HAL_RCC_GPIOH_CLK_ENABLE(); // Enable GPIOH clock
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA clock
  __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable GPIOB clock

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Reset LED2

  GPIO_InitStruct.Pin = B1_Pin;               // Button pin
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;// Interrupt on falling edge
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up/down
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;              // LED2 pin
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up/down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// Low speed
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

// Default RTOS task
void StartDefaultTask(void const * argument)
{
  for(;;)
  {
    osDelay(1); // Delay 1 tick
  }
}

// Error handler
void Error_Handler(void)
{
  __disable_irq(); // Disable interrupts
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
// Assert failure handler
void assert_failed(uint8_t *file, uint32_t line)
{
  // Print file and line number if needed
}
#endif
