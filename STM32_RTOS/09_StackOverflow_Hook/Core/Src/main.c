#include "main.h"          // main header file
#include "cmsis_os.h"      // FreeRTOS wrapper

#include <stdio.h>         // for sprintf
#include <string.h>        // for strlen

UART_HandleTypeDef huart2; // UART2 handle

osThreadId defaultTaskHandle; // default task handle

// system clock setup
void SystemClock_Config(void);
// GPIO setup
static void MX_GPIO_Init(void);
// UART2 setup
static void MX_USART2_UART_Init(void);
// default task
void StartDefaultTask(void const * argument);

// hook called on stack overflow
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    char msg[100]; // buffer for error message
    sprintf(msg, "STACK OVERFLOW in %s\r\n", pcTaskName); // format error
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); // send over UART

    while(1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // blink LED
        for (volatile uint32_t i = 0; i < 500000; i++); // software delay
    }
}

// task to deliberately use too much stack
void StackTestTask(void *argument)
{
    volatile uint32_t bigArray[200]; // large array to fill stack

    for(;;) // infinite loop
    {
        for(uint32_t i = 0; i < 200; i++) // fill array
        {
            bigArray[i] = i + 10;
        }
        osDelay(1); // yield to scheduler
    }
}

int main(void)
{
    HAL_Init();             // initialize HAL
    SystemClock_Config();   // configure system clock
    MX_GPIO_Init();         // initialize GPIO
    MX_USART2_UART_Init();  // initialize UART2

    // create CMSIS default task
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    // create stack test task with small stack (to force overflow)
    if(xTaskCreate(StackTestTask, "Stack_Task", 100, NULL, 1, NULL) != pdPASS)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)"FAIL\r\n", 6, 1000); // print error if task fails
    }

    osKernelStart(); // start FreeRTOS scheduler

    while (1)
    {

    }    // should never reach here
}

// configure system clock
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // oscillator config struct
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // clock config struct

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    // setup oscillator
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

    // setup clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

// initialize UART2
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

// initialize GPIO
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE(); // enable port C clock
    __HAL_RCC_GPIOH_CLK_ENABLE(); // enable port H clock
    __HAL_RCC_GPIOA_CLK_ENABLE(); // enable port A clock
    __HAL_RCC_GPIOB_CLK_ENABLE(); // enable port B clock

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // reset LED pin

    // configure button pin
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    // configure LED pin
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

// default task
void StartDefaultTask(void const * argument)
{
    for(;;) // infinite loop
    {
        osDelay(1); // delay 1 ms
    }
}

// error handler
void Error_Handler(void)
{
    __disable_irq(); // disable interrupts
    while (1) { }    // stay here
}

#ifdef USE_FULL_ASSERT
// assert failure report
void assert_failed(uint8_t *file, uint32_t line)
{
    // user can print file name and line number here
}
#endif
