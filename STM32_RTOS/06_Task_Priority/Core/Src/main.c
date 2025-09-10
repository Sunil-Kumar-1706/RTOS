#include "main.h"         // Main HAL header file
#include "cmsis_os.h"     // FreeRTOS CMSIS-OS wrapper

#include <string.h>       // For string functions
#include <stdio.h>        // For sprintf()
#include <stdint.h>       // For standard integer types

#define FALSE 0           // Define FALSE as 0
#define TRUE  1           // Define TRUE as 1

UART_HandleTypeDef huart2;  // UART2 handle

osThreadId defaultTaskHandle;   // Default RTOS task handle

TaskHandle_t xTaskHandle1 = NULL;  // Handle for task 1
TaskHandle_t xTaskHandle2 = NULL;  // Handle for task 2
uint8_t priority_switch = FALSE;   // Flag to track button press
char msg[50];                      // Buffer for UART messages

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

// ------------------- USER FUNCTIONS -------------------

// External interrupt handler (button on pin 13)
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13); // Clear EXTI interrupt
  priority_switch = TRUE;                // Set flag for switching priorities
}

// Send a string over UART2
void uart_msg(char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
}

// Simple delay using RTOS tick count
void rtos_delay(uint32_t delay_in_ms)
{
  uint32_t tick_count_local = xTaskGetTickCount();  // Current tick
  uint32_t delay_in_ticks = (delay_in_ms * configTICK_RATE_HZ) / 1000; // ms to ticks

  while (xTaskGetTickCount() < (tick_count_local + delay_in_ticks)); // Wait
}

// ------------------- TASK-1 -------------------
void task1(void *p)
{
  UBaseType_t p1, p2; // To store priorities

  uart_msg("TASK-1 RUNNING\r\n");
  sprintf(msg, "TASK-1 priority %ld\r\n", uxTaskPriorityGet(xTaskHandle1));
  uart_msg(msg);
  sprintf(msg, "TASK-2 priority %ld\r\n", uxTaskPriorityGet(xTaskHandle2));
  uart_msg(msg);

  while (1)
  {
    if (priority_switch) // If button pressed
    {
      priority_switch = FALSE;  // Reset flag
      uart_msg("Priority switch by TASK-1\r\n");

      p1 = uxTaskPriorityGet(xTaskHandle1); // Task1 priority
      p2 = uxTaskPriorityGet(xTaskHandle2); // Task2 priority

      vTaskPrioritySet(xTaskHandle1, p2);   // Swap priorities
      vTaskPrioritySet(xTaskHandle2, p1);
    }
    else
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
      rtos_delay(200);                       // 200 ms delay
    }
  }
}

// ------------------- TASK-2 -------------------
void task2(void *p)
{
  UBaseType_t p1, p2; // To store priorities

  uart_msg("TASK-2 RUNNING\r\n");
  sprintf(msg, "TASK-1 priority %ld\r\n", uxTaskPriorityGet(xTaskHandle1));
  uart_msg(msg);
  sprintf(msg, "TASK-2 priority %ld\r\n", uxTaskPriorityGet(xTaskHandle2));
  uart_msg(msg);

  while (1)
  {
    if (priority_switch) // If button pressed
    {
      priority_switch = FALSE;  // Reset flag
      uart_msg("Priority switch by TASK-2\r\n");

      p1 = uxTaskPriorityGet(xTaskHandle1); // Task1 priority
      p2 = uxTaskPriorityGet(xTaskHandle2); // Task2 priority

      vTaskPrioritySet(xTaskHandle1, p2);   // Swap priorities
      vTaskPrioritySet(xTaskHandle2, p1);
    }
    else
    {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
      rtos_delay(1000);                      // 1 second delay
    }
  }
}

// ------------------- MAIN FUNCTION -------------------
int main(void)
{
  HAL_Init();               // Initialize HAL
  SystemClock_Config();     // Configure system clock
  MX_GPIO_Init();           // Init GPIO
  MX_USART2_UART_Init();    // Init UART2

  // Create default task
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  // Create custom FreeRTOS tasks
  xTaskCreate(task1, "TASK1", configMINIMAL_STACK_SIZE, NULL, 2, &xTaskHandle1);
  xTaskCreate(task2, "TASK2", configMINIMAL_STACK_SIZE, NULL, 3, &xTaskHandle2);

  osKernelStart();          // Start RTOS scheduler

  while (1)
  {

  }             // Should never reach here
}

// ------------------- CLOCK CONFIG -------------------
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    Error_Handler();

  // Configure HSI + PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  // Configure AHB and APB clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    Error_Handler();
}

// ------------------- UART2 INIT -------------------
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();
}

// ------------------- GPIO INIT -------------------
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable clocks
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // LED pin (PA5) → Output
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  // Button pin (PC13) → External interrupt
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  // Enable EXTI interrupt for button
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

// ------------------- DEFAULT TASK -------------------
void StartDefaultTask(void const * argument)
{
  for (;;)
  {
    osDelay(1); // Small delay
  }
}

// ------------------- ERROR HANDLER -------------------
void Error_Handler(void)
{
  __disable_irq(); // Disable interrupts
  while (1) { }    // Stay here
}
