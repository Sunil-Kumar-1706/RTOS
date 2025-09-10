#include "main.h"        // Include main header file
#include "cmsis_os.h"    // Include CMSIS-RTOS API (FreeRTOS wrapper)

#include <stdio.h>       // For sprintf (string formatting)
#include <string.h>      // For strlen (string length function)

// UART handle for USART2
UART_HandleTypeDef huart2;

// Default RTOS task handle
osThreadId defaultTaskHandle;

// Task handles for custom FreeRTOS tasks
TaskHandle_t handle1 = NULL;  // Task 1 handle
TaskHandle_t handle2 = NULL;  // Task 2 handle

// Function declarations
void SystemClock_Config(void);             // Configure system clock
static void MX_GPIO_Init(void);            // Initialize GPIO
static void MX_USART2_UART_Init(void);     // Initialize UART2
void StartDefaultTask(void const * argument); // Default RTOS task

// Function to send a string over UART2
void uart_send(char *s)
{
    // Transmit the string through UART2
    HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 1000);
}

// Task1: Prints message and LED status
void task1_handler(void *p)
{
    char msg[50]; // Buffer to store messages
    while (1)     // Infinite loop
    {
        // Print the task name
        sprintf(msg, "%s\r\n", (char*)p);
        uart_send(msg);

        // Print the current LED status (ON=1, OFF=0)
        sprintf(msg, "LED status: %d\r\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));
        uart_send(msg);

        // Wait for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task2: Prints message and toggles LED
void task2_handler(void *p)
{
    char msg[50]; // Buffer to store messages
    while (1)     // Infinite loop
    {
        // Print the task name
        sprintf(msg, "%s\r\n", (char *)p);
        uart_send(msg);

        // Toggle LED
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        // Wait for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void)
{
    HAL_Init();              // Initialize HAL library

    SystemClock_Config();    // Configure system clock

    MX_GPIO_Init();          // Initialize GPIO
    MX_USART2_UART_Init();   // Initialize UART2

    // Create default RTOS task
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    // Create Task 1
    xTaskCreate(task1_handler, "task-1", configMINIMAL_STACK_SIZE, "TASK-1", 1, &handle1);

    // Create Task 2
    xTaskCreate(task2_handler, "task-2", configMINIMAL_STACK_SIZE, "TASK-2", 1, &handle2);

    osKernelStart();         // Start RTOS scheduler

    while (1)                // Should never reach here
    {

    }
}

// Configure system clock
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // Oscillator config structure
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // Clock config structure

    // Set voltage scaling
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler(); // If error, call error handler
    }

    // Configure HSI (High-Speed Internal oscillator) with PLL
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
    {
        Error_Handler();
    }

    // Configure CPU, AHB, APB bus clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

// Initialize USART2
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;                          // Select USART2
    huart2.Init.BaudRate = 115200;                     // Set baud rate
    huart2.Init.WordLength = UART_WORDLENGTH_8B;       // 8 data bits
    huart2.Init.StopBits = UART_STOPBITS_1;            // 1 stop bit
    huart2.Init.Parity = UART_PARITY_NONE;             // No parity
    huart2.Init.Mode = UART_MODE_TX_RX;                // Enable TX and RX
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;       // No flow control
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;   // Oversampling by 16
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // Disable 1-bit sampling
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // No advanced features
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler(); // If initialization fails
    }
}

// Initialize GPIO
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO config structure

    __HAL_RCC_GPIOC_CLK_ENABLE();  // Enable clock for GPIOC
    __HAL_RCC_GPIOH_CLK_ENABLE();  // Enable clock for GPIOH
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Enable clock for GPIOA
    __HAL_RCC_GPIOB_CLK_ENABLE();  // Enable clock for GPIOB

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Reset LED pin (turn OFF)

    // Configure push button (B1) pin as interrupt on falling edge
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    // Configure LED (LD2) pin as output
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

// Default RTOS task
void StartDefaultTask(void const * argument)
{
    for (;;)           // Infinite loop
    {
        osDelay(1);    // Delay for 1 tick
    }
}

// Error handler
void Error_Handler(void)
{
    __disable_irq();   // Disable interrupts
    while (1)          // Infinite loop
    {
    }
}

#ifdef USE_FULL_ASSERT
// Reports source file and line number if assert fails
void assert_failed(uint8_t *file, uint32_t line)
{
    // Example: printf("Wrong parameters: file %s line %d\r\n", file, line);
}
#endif
