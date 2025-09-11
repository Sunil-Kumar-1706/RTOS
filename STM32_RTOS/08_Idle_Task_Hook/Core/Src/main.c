#include "main.h"          // main header file
#include "cmsis_os.h"      // FreeRTOS CMSIS-OS wrapper

#include <string.h>        // string functions (for strlen)

UART_HandleTypeDef huart2; // UART2 handle

osThreadId defaultTaskHandle; // default task handle

TaskHandle_t xhandle1 = NULL; // handle for task1
TaskHandle_t xhandle2 = NULL; // handle for task2

void SystemClock_Config(void);     // system clock setup
static void MX_GPIO_Init(void);    // GPIO setup
static void MX_USART2_UART_Init(void); // UART2 setup
void StartDefaultTask(void const * argument); // default task

// Idle hook: runs when no tasks are ready
void vApplicationIdleHook(void)
{
    __WFI(); // put CPU in low-power until interrupt
}

// Task1: sends message over UART every 1s
void task1(void *p)
{
    while(1)
    {
        char *s = "TASK-1 Executing\r\n";  // message
        HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000); // send over UART
        vTaskDelay(pdMS_TO_TICKS(1000));    // wait 1000 ms
    }
}

// Task2: sends message over UART every 1s
void task2(void *p)
{
    while(1)
    {
        char *s = "TASK-2 Executing\r\n";  // message
        HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000); // send over UART
        vTaskDelay(pdMS_TO_TICKS(1000));    // wait 1000 ms
    }
}

int main(void)
{
    HAL_Init();              // initialize HAL
    SystemClock_Config();    // configure system clock
    MX_GPIO_Init();          // initialize GPIO
    MX_USART2_UART_Init();   // initialize UART2

    // create CMSIS default task
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    // create FreeRTOS tasks
    xTaskCreate(task1, "TASK-1", configMINIMAL_STACK_SIZE, NULL, 1, &xhandle1);
    xTaskCreate(task2, "TASK-2", configMINIMAL_STACK_SIZE, NULL, 1, &xhandle2);

    osKernelStart(); // start FreeRTOS scheduler

    while (1)
    {

    } // should never reach here
}

// system clock configuration
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // oscillator config struct
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // clock config struct

    // set voltage regulator scale
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    // configure oscillator
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

    // configure CPU, AHB, APB clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

// UART2 initialization
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

// GPIO initialization
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

// default task code
void StartDefaultTask(void const * argument)
{
    for(;;)
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
// report assert failure
void assert_failed(uint8_t *file, uint32_t line)
{
    // user can print file and line if needed
}
#endif
