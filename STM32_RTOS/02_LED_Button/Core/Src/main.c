#include "main.h"          // include main header file
#include "cmsis_os.h"      // include FreeRTOS CMSIS-OS wrapper

#define NOT_PRESSED 0      // define button not pressed
#define PRESSED     1      // define button pressed

UART_HandleTypeDef huart2; // handle for UART2
osThreadId defaultTaskHandle; // handle for default task
uint8_t button_status = NOT_PRESSED; // variable to store button state

// function declarations
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

// LED task function
void led_task(void *p)
{
	while(1)   // run forever
	{
		if(button_status == PRESSED)  // if button pressed
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);  // turn LED on
			vTaskDelay(1000);  // wait 1 second
		}
		else   // if button not pressed
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET); // turn LED off
		}
	}
}

// Button task function
void button_task(void *p)
{
	while(1)   // run forever
	{
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))  // read button pin
		{
			button_status = NOT_PRESSED; // if not pressed
		}
		else
		{
			button_status = PRESSED; // if pressed
		}
	}
}

// main function
int main(void)
{
  HAL_Init();               // initialize HAL library
  SystemClock_Config();     // configure system clock
  MX_GPIO_Init();           // initialize GPIO
  MX_USART2_UART_Init();    // initialize UART2

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);  // define default task
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);       // create default task

  xTaskCreate(led_task, "LED", configMINIMAL_STACK_SIZE, NULL, 1,NULL);     // create LED task
  xTaskCreate(button_task,"BUTTON",configMINIMAL_STACK_SIZE,NULL,1,NULL);  // create Button task

  osKernelStart();          // start FreeRTOS scheduler

  while (1)
  {

  }              // infinite loop (never reached)
}

// configure system clock
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // oscillator config struct
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // clock config struct

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1); // set voltage scaling

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // use HSI oscillator
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // turn HSI on
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // default calibration
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // enable PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // PLL source = HSI
  RCC_OscInitStruct.PLL.PLLM = 1;                            // PLLM divider
  RCC_OscInitStruct.PLL.PLLN = 10;                           // PLLN multiplier
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;                // PLLP divider
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                // PLLQ divider
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                // PLLR divider
  HAL_RCC_OscConfig(&RCC_OscInitStruct);                     // apply oscillator settings

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; // select clocks
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;   // use PLL as system clock
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;          // no division for AHB
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;           // no division for APB1
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;           // no division for APB2

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);   // apply clock config
}

// initialize UART2
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;                        // select USART2
  huart2.Init.BaudRate = 115200;                   // baudrate 115200
  huart2.Init.WordLength = UART_WORDLENGTH_8B;     // 8-bit data
  huart2.Init.StopBits = UART_STOPBITS_1;          // 1 stop bit
  huart2.Init.Parity = UART_PARITY_NONE;           // no parity
  huart2.Init.Mode = UART_MODE_TX_RX;              // enable TX and RX
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;     // no hardware flow control
  huart2.Init.OverSampling = UART_OVERSAMPLING_16; // oversampling by 16
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // disable one-bit sampling
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // no advanced features
  HAL_UART_Init(&huart2);                          // initialize UART
}

// initialize GPIO
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO config structure

  __HAL_RCC_GPIOC_CLK_ENABLE();   // enable GPIOC clock
  __HAL_RCC_GPIOH_CLK_ENABLE();   // enable GPIOH clock
  __HAL_RCC_GPIOA_CLK_ENABLE();   // enable GPIOA clock
  __HAL_RCC_GPIOB_CLK_ENABLE();   // enable GPIOB clock

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // turn off LED2

  GPIO_InitStruct.Pin = B1_Pin;                // button pin
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // falling edge interrupt
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // no pull-up or pull-down
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct); // initialize button pin

  GPIO_InitStruct.Pin = LD2_Pin;               // LED pin
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // push-pull output
  GPIO_InitStruct.Pull = GPIO_NOPULL;          // no pull-up or pull-down
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // low speed
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct); // initialize LED pin
}

// default task
void StartDefaultTask(void const * argument)
{
  for(;;)     // run forever
  {
    osDelay(1); // delay 1ms
  }
}

// error handler
void Error_Handler(void)
{
  __disable_irq();  // disable interrupts
  while (1) {}      // stay here forever
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) // assert failed handler
{
}
#endif
