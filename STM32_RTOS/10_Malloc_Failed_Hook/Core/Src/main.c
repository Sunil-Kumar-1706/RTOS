#include "main.h"          // main HAL header
#include "cmsis_os.h"      // FreeRTOS wrapper

#include <stdio.h>         // for sprintf
#include <string.h>        // for strlen

UART_HandleTypeDef huart2; // UART2 handle

osThreadId defaultTaskHandle;   // default task handle
TaskHandle_t xTask1 = NULL;     // handle for malloc test task

void SystemClock_Config(void);      // system clock setup
static void MX_GPIO_Init(void);     // GPIO setup
static void MX_USART2_UART_Init(void); // UART2 setup
void StartDefaultTask(void const * argument); // default task function

// called when FreeRTOS malloc fails
void vApplicationMallocFailedHook(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();   // get current task
    char msg[100];                                     // buffer for message
    sprintf(msg, "MALLOC FAILED in %s\r\n", pcTaskGetName(task)); // format text
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); // send text

    while(1)
    {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);         // blink LED
        for(volatile uint32_t i = 0; i < 500000; i++); // delay loop
    }
}

// task that keeps allocating memory to force malloc failure
void MallocTestTask(void *argument)
{
    void *p;                     // pointer for allocation
    for(;;)                      // infinite loop
    {
        p = pvPortMalloc(1000);  // allocate 1000 bytes
        HAL_Delay(10);           // small delay
    }
}

int main(void)
{
    HAL_Init();                  // initialize HAL
    SystemClock_Config();        // configure system clock
    MX_GPIO_Init();              // initialize GPIO
    MX_USART2_UART_Init();       // initialize UART2

    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128); // define default task
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);      // create default task

    xTaskCreate(MallocTestTask, "MALLOC_Task", 128, NULL, 1, &xTask1);         // create malloc test task

    osKernelStart();             // start scheduler

    while (1)
    {

    }                // should never reach here
}

// configure system clock
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // oscillator config struct
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // clock config struct

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();         // set voltage scaling

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // use HSI oscillator
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // enable HSI
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // calibration
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // enable PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // PLL source HSI
    RCC_OscInitStruct.PLL.PLLM = 1;                            // PLL divider M
    RCC_OscInitStruct.PLL.PLLN = 10;                           // PLL multiplier N
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;                // PLL P divider
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                // PLL Q divider
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                // PLL R divider
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();         // apply oscillator config

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // set clocks
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // system clock = PLL
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // AHB divider
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         // APB1 divider
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // APB2 divider

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();         // apply clock config
}

// initialize UART2
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;                        // set UART2
    huart2.Init.BaudRate = 115200;                   // baud rate
    huart2.Init.WordLength = UART_WORDLENGTH_8B;     // 8-bit word
    huart2.Init.StopBits = UART_STOPBITS_1;          // 1 stop bit
    huart2.Init.Parity = UART_PARITY_NONE;           // no parity
    huart2.Init.Mode = UART_MODE_TX_RX;              // enable TX/RX
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;     // no flow control
    huart2.Init.OverSampling = UART_OVERSAMPLING_16; // oversampling
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // one-bit sampling off
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // no advanced features
    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();                             // apply config
}

// initialize GPIO
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0}; // GPIO config struct

    __HAL_RCC_GPIOC_CLK_ENABLE();           // enable port C clock
    __HAL_RCC_GPIOH_CLK_ENABLE();           // enable port H clock
    __HAL_RCC_GPIOA_CLK_ENABLE();           // enable port A clock
    __HAL_RCC_GPIOB_CLK_ENABLE();           // enable port B clock

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // reset LED pin

    GPIO_InitStruct.Pin = B1_Pin;           // button pin
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // interrupt on falling edge
    GPIO_InitStruct.Pull = GPIO_NOPULL;     // no pull-up/down
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct); // init button

    GPIO_InitStruct.Pin = LD2_Pin;          // LED pin
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;     // no pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // low speed
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct); // init LED
}

// default task function
void StartDefaultTask(void const * argument)
{
    for(;;)              // infinite loop
    {
        osDelay(1);      // delay 1 ms
    }
}

// error handler
void Error_Handler(void)
{
    __disable_irq();     // disable interrupts
    while (1) { }        // stay here
}

#ifdef USE_FULL_ASSERT
// assert failed function
void assert_failed(uint8_t *file, uint32_t line)
{
    // can print file name and line number here
}
#endif
