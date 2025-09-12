#include "main.h"              // Include main header
#include "cmsis_os.h"          // Include CMSIS RTOS header

#include <string.h>            // For string handling functions
#include "semphr.h"            // For FreeRTOS semaphore functions

UART_HandleTypeDef huart2;     // UART2 handle

osThreadId defaultTaskHandle;  // Default RTOS task handle
xSemaphoreHandle xBinSem;      // Binary semaphore handle

void SystemClock_Config(void);          // Function to configure system clock
static void MX_GPIO_Init(void);         // Function to initialize GPIO
static void MX_USART2_UART_Init(void);  // Function to initialize UART2
void StartDefaultTask(void const * argument); // RTOS default task function

// Send string via UART2
void uart_send(char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000); // Transmit string
}

// Task 1
void task1(void *p)
{
    while(1)
    {
        xSemaphoreTake(xBinSem, portMAX_DELAY);   // Lock semaphore
        uart_send("task-1 running\r\n");          // Send message
        xSemaphoreGive(xBinSem);                  // Unlock semaphore
        vTaskDelay(pdMS_TO_TICKS(500));           // Delay 500ms
    }
}

// Task 2
void task2(void *p)
{
    while(1)
    {
        xSemaphoreTake(xBinSem, portMAX_DELAY);   // Lock semaphore
        uart_send("task-2 running\r\n");          // Send message
        xSemaphoreGive(xBinSem);                  // Unlock semaphore
        vTaskDelay(pdMS_TO_TICKS(500));           // Delay 500ms
    }
}

int main(void)
{
    HAL_Init();                     // Initialize HAL
    SystemClock_Config();           // Configure system clock
    MX_GPIO_Init();                 // Initialize GPIO
    MX_USART2_UART_Init();          // Initialize UART2

    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128); // Define default task
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);      // Create default task

    vSemaphoreCreateBinary(xBinSem);    // Create binary semaphore

    if(xBinSem != NULL)                 // Check if semaphore created successfully
    {
        xTaskCreate(task1, "Task-1", configMINIMAL_STACK_SIZE, NULL, 1, NULL); // Create Task 1
        xTaskCreate(task2, "Task-2", configMINIMAL_STACK_SIZE, NULL, 1, NULL); // Create Task 2
        xSemaphoreGive(xBinSem);        // Give semaphore initially
    }

    osKernelStart();    // Start RTOS scheduler

    while (1)           // Should never reach here
    {
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // Oscillator configuration struct
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // Clock configuration struct

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1); // Set voltage scaling

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;   // Use HSI oscillator
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;                     // Enable HSI
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                 // Enable PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;         // PLL source = HSI
    RCC_OscInitStruct.PLL.PLLM = 1;                              // PLLM division factor
    RCC_OscInitStruct.PLL.PLLN = 10;                             // PLLN multiplication factor
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;                  // PLLP division
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;                  // PLLQ division
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;                  // PLLR division
    HAL_RCC_OscConfig(&RCC_OscInitStruct);                       // Apply oscillator config

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // Set clock types
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;    // System clock source = PLL
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;           // AHB prescaler
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;            // APB1 prescaler
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;            // APB2 prescaler
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);    // Apply clock config
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;                       // Use USART2
    huart2.Init.BaudRate = 115200;                  // Baudrate = 115200
    huart2.Init.WordLength = UART_WORDLENGTH_8B;    // 8-bit word length
    huart2.Init.StopBits = UART_STOPBITS_1;         // 1 stop bit
    huart2.Init.Parity = UART_PARITY_NONE;          // No parity
    huart2.Init.Mode = UART_MODE_TX_RX;             // Enable TX and RX
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;    // No hardware flow control
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;// Oversampling 16
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // No one-bit sampling
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // No advanced features
    HAL_UART_Init(&huart2);                         // Initialize UART
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};     // GPIO init struct

    __HAL_RCC_GPIOC_CLK_ENABLE();               // Enable GPIOC clock
    __HAL_RCC_GPIOH_CLK_ENABLE();               // Enable GPIOH clock
    __HAL_RCC_GPIOA_CLK_ENABLE();               // Enable GPIOA clock
    __HAL_RCC_GPIOB_CLK_ENABLE();               // Enable GPIOB clock

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Reset LED

    GPIO_InitStruct.Pin = B1_Pin;               // Configure Button pin
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;// Falling edge interrupt
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up/pull-down
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct); // Init button

    GPIO_InitStruct.Pin = LD2_Pin;              // Configure LED pin
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull resistor
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;// Low speed
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct); // Init LED
}

void StartDefaultTask(void const * argument)
{
    for(;;)
    {
        osDelay(1);    // Delay 1 tick
    }
}

void Error_Handler(void)
{
    __disable_irq();   // Disable interrupts
    while (1)          // Infinite loop on error
    {
    }
}
