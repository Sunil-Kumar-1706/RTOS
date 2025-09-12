#include "main.h"          // main header file
#include "cmsis_os.h"      // FreeRTOS API header
#include <string.h>        // for strlen()
#include <stdio.h>
#include "semphr.h"        // FreeRTOS semaphore functions

UART_HandleTypeDef huart2; // handle for UART2

osThreadId defaultTaskHandle; // handle for default task

SemaphoreHandle_t xcntsem;  // counting semaphore handle

// function prototypes
void SystemClock_Config(void);      // configure system clock
static void MX_GPIO_Init(void);     // init GPIO
static void MX_USART2_UART_Init(void); // init UART2
void StartDefaultTask(void const * argument); // default FreeRTOS task

// function to send string through UART2
void uart_send(char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 1000);
}

// task which waits for semaphore
static void handlertask(void *p)
{
    while(1)
    {
        // wait until semaphore is available
        if(xSemaphoreTake(xcntsem , portMAX_DELAY) == pdTRUE)
        {
            // print message if semaphore taken
            uart_send("Handler task - processing\r\n");
        }
    }
}

// task which gives semaphore periodically
static void periodictask(void *p)
{
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
        // delay 500ms

        uart_send("Periodic task giving semaphore\r\n");
        // print message

        xSemaphoreGive(xcntsem); // give semaphore
        xSemaphoreGive(xcntsem); // give semaphore
        xSemaphoreGive(xcntsem); // give semaphore
    }
}

int main(void)
{
  HAL_Init();              // initialize HAL library
  SystemClock_Config();    // set up system clock
  MX_GPIO_Init();          // init GPIO pins
  MX_USART2_UART_Init();   // init UART2

  // create default FreeRTOS task
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  // create counting semaphore
  xcntsem = xSemaphoreCreateCounting(5, 0);

  if (xcntsem != NULL)   // check if semaphore created successfully
  {
      // create handler task
      xTaskCreate(handlertask, "handler", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

      // create periodic task
      xTaskCreate(periodictask, "Periodic", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  }

  osKernelStart();  // start FreeRTOS scheduler

  while (1)
  {

  }      // should never reach here
}

// configure system clock
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // oscillator config struct
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // clock config struct

  // set regulator voltage scale
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  { Error_Handler(); }

  // configure HSI oscillator and PLL
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  // use HSI
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // turn on HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // default calibration
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // PLL source = HSI
  RCC_OscInitStruct.PLL.PLLM = 1;                            // PLLM = 1
  RCC_OscInitStruct.PLL.PLLN = 10;                           // PLLN = 10
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;                // PLLP divider = 7
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                // PLLQ divider = 2
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                // PLLR divider = 2
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { Error_Handler(); }

  // configure CPU, AHB, APB bus clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // use PLL as SYSCLK
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB = SYSCLK / 1
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;          // APB1 = HCLK / 1
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // APB2 = HCLK / 1

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  { Error_Handler(); }
}

// initialize UART2
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;                     // use USART2
  huart2.Init.BaudRate = 115200;                // baud rate = 115200
  huart2.Init.WordLength = UART_WORDLENGTH_8B;  // 8 data bits
  huart2.Init.StopBits = UART_STOPBITS_1;       // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE;        // no parity
  huart2.Init.Mode = UART_MODE_TX_RX;           // enable TX and RX
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;  // no flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16; // oversampling 16x
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // disable one-bit sampling
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // no advanced features
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

// initialize GPIO pins
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO config struct

  __HAL_RCC_GPIOC_CLK_ENABLE();  // enable GPIOC clock
  __HAL_RCC_GPIOH_CLK_ENABLE();  // enable GPIOH clock
  __HAL_RCC_GPIOA_CLK_ENABLE();  // enable GPIOA clock
  __HAL_RCC_GPIOB_CLK_ENABLE();  // enable GPIOB clock

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // reset LED2

  GPIO_InitStruct.Pin = B1_Pin;             // select pin B1
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // falling edge interrupt
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // no pull-up/down
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct); // init button pin

  GPIO_InitStruct.Pin = LD2_Pin;             // select LED2 pin
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // no pull-up/down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // low speed
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct); // init LED2 pin
}

// default FreeRTOS task
void StartDefaultTask(void const * argument)
{
  for(;;)  // run forever
  {
    osDelay(1);  // delay 1ms
  }
}

// error handler
void Error_Handler(void)
{
  __disable_irq(); // disable interrupts
  while (1)
  {

  }     // stay here forever
}
