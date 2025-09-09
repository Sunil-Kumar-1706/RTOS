#include "main.h"          // include main header
#include "cmsis_os.h"      // include FreeRTOS header

#define NOT_PRESSED 0      // button not pressed
#define PRESSED     1      // button pressed

UART_HandleTypeDef huart2; // UART2 handle
osThreadId defaultTaskHandle; // default task handle
uint8_t button_status = NOT_PRESSED; // variable to track button state

void SystemClock_Config(void);       // system clock function
static void MX_GPIO_Init(void);      // GPIO init function
static void MX_USART2_UART_Init(void); // UART init function
void StartDefaultTask(void const * argument); // default task function

// interrupt handler for button PC13
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13); // clear interrupt flag
  button_handler(NULL);                  // call button handler
}

// LED task
void led_task(void *p)
{
	while(1) // run forever
	{
		if(button_status == PRESSED)  // if pressed
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET); // LED ON
			vTaskDelay(1000); // wait 1 second
		}
		else  // if not pressed
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET); // LED OFF
		}
	}
}

// button handler
void button_handler(void *p)
{
	button_status ^= 1; // toggle button state
}

int main(void)
{
  HAL_Init();              // init HAL
  SystemClock_Config();    // set system clock
  MX_GPIO_Init();          // init GPIO
  MX_USART2_UART_Init();   // init UART

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128); // define default task
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);      // create default task

  xTaskCreate(led_task, "LED", configMINIMAL_STACK_SIZE, NULL, 1,NULL); // create LED task

  osKernelStart();         // start scheduler

  while (1)
  {

  }             // infinite loop
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // oscillator struct
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // clock struct

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) // voltage scale
  {
    Error_Handler(); // error handler
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // HSI oscillator
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // enable HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // default calibration
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // PLL source HSI
  RCC_OscInitStruct.PLL.PLLM = 1;                            // PLLM
  RCC_OscInitStruct.PLL.PLLN = 10;                           // PLLN
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;                // PLLP
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                // PLLQ
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                // PLLR
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)       // apply oscillator
  {
    Error_Handler(); // error handler
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; // clock types
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;   // PLL as system clock
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;          // AHB divider
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;           // APB1 divider
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           // APB2 divider

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) // apply clock
  {
    Error_Handler(); // error handler
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;                        // use USART2
  huart2.Init.BaudRate = 115200;                   // baud 115200
  huart2.Init.WordLength = UART_WORDLENGTH_8B;     // 8-bit
  huart2.Init.StopBits = UART_STOPBITS_1;          // 1 stop
  huart2.Init.Parity = UART_PARITY_NONE;           // no parity
  huart2.Init.Mode = UART_MODE_TX_RX;              // TX + RX
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;     // no flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16; // oversampling
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // no one-bit
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // no advanced
  if (HAL_UART_Init(&huart2) != HAL_OK)            // init UART
  {
    Error_Handler(); // error handler
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
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // output
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // no pull
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // low speed
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct); // init LED
}

void StartDefaultTask(void const * argument)
{
  for(;;) // run forever
  {
    osDelay(1); // delay 1ms
  }
}

void Error_Handler(void)
{
  __disable_irq(); // disable interrupts
  while (1) {}     // stay here
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) // assert fail
{
}
#endif
