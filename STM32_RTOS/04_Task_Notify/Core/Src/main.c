#include "main.h"          // main header
#include "cmsis_os.h"      // FreeRTOS header
#include <stdio.h>         // standard I/O
#include <string.h>        // string functions

UART_HandleTypeDef huart2; // UART2 handle
osThreadId defaultTaskHandle; // default task handle

char msg[50];              // buffer for UART message
TaskHandle_t xTaskHandle1=NULL; // LED task handle
TaskHandle_t xTaskHandle2=NULL; // Button task handle

void SystemClock_Config(void);     // system clock setup
static void MX_GPIO_Init(void);    // GPIO setup
static void MX_USART2_UART_Init(void); // UART setup
void StartDefaultTask(void const * argument); // default task function

// LED task
void task_led(void * p)
{
    uint32_t current_notify_value = 0;  // store notification count
    while(1) // run forever
    {
        // wait for notification
        if (xTaskNotifyWait(0, 0, &current_notify_value, portMAX_DELAY) == pdTRUE)
        {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // toggle LED
            sprintf(msg, "Notification received : Button Press Count: %ld\r\n", current_notify_value); // format message
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000); // send via UART
        }
    }
}

// Button task
void task_button(void *p)
{
    while(1) // run forever
    {
        if (!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))) // check button press
        {
            vTaskDelay(1000);  // debounce delay
            xTaskNotify(xTaskHandle1, 0X0, eIncrement); // send notification to LED task
        }
    }
}

int main(void)
{
  HAL_Init();             // init HAL
  SystemClock_Config();   // set clock
  MX_GPIO_Init();         // init GPIO
  MX_USART2_UART_Init();  // init UART

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128); // define default task
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);      // create default task

  xTaskCreate(task_led, "LED", configMINIMAL_STACK_SIZE,NULL, 1, &xTaskHandle1);   // create LED task
  xTaskCreate(task_button, "BUTTON", configMINIMAL_STACK_SIZE, NULL, 1, &xTaskHandle2); // create button task

  osKernelStart();        // start scheduler

  while (1)
  {

  }            // infinite loop
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // oscillator config
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // clock config

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) // set voltage scale
  {
    Error_Handler(); // error
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // use HSI
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // turn on HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // default cal
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // PLL source HSI
  RCC_OscInitStruct.PLL.PLLM = 1;                            // divider M
  RCC_OscInitStruct.PLL.PLLN = 10;                           // multiplier N
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;                // divider P
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                // divider Q
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                // divider R
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)       // apply oscillator
  {
    Error_Handler(); // error
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; // select clocks
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;   // system clock = PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;          // AHB divider
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;           // APB1 divider
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           // APB2 divider

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) // apply clocks
  {
    Error_Handler(); // error
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;                        // UART2
  huart2.Init.BaudRate = 115200;                   // baud rate
  huart2.Init.WordLength = UART_WORDLENGTH_8B;     // 8-bit
  huart2.Init.StopBits = UART_STOPBITS_1;          // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE;           // no parity
  huart2.Init.Mode = UART_MODE_TX_RX;              // TX and RX
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;     // no flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16; // oversampling
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // no one-bit
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // no advanced
  if (HAL_UART_Init(&huart2) != HAL_OK)            // init UART
  {
    Error_Handler(); // error
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0}; // GPIO struct

  __HAL_RCC_GPIOC_CLK_ENABLE(); // enable GPIOC
  __HAL_RCC_GPIOH_CLK_ENABLE(); // enable GPIOH
  __HAL_RCC_GPIOA_CLK_ENABLE(); // enable GPIOA
  __HAL_RCC_GPIOB_CLK_ENABLE(); // enable GPIOB

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // reset LED

  GPIO_InitStruct.Pin = B1_Pin;                // button pin
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // falling edge
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // no pull
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct); // init button

  GPIO_InitStruct.Pin = LD2_Pin;               // LED pin
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // no pull
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // low speed
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct); // init LED
}

void StartDefaultTask(void const * argument)
{
  for(;;) // run forever
  {
    osDelay(1); // 1 ms delay
  }
}

void Error_Handler(void)
{
  __disable_irq(); // disable interrupts
  while (1) {}     // stay here
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) // assert failure
{
}
#endif
