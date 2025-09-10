#include "main.h"       // Main header file
#include "cmsis_os.h"   // FreeRTOS wrapper for CMSIS
#include <stdio.h>
#include <string.h>     // For string handling functions like strlen()

/* UART handle (used for communication via USART2) */
UART_HandleTypeDef huart2;

/* RTOS default task handle */
osThreadId defaultTaskHandle;

/* FreeRTOS task handles */
TaskHandle_t xTaskHandle1 = NULL;   // Task 1 handle
TaskHandle_t xTaskHandle2 = NULL;   // Task 2 handle

/* ---------- Function Prototypes ---------- */
void SystemClock_Config(void);          // Configure system clock
static void MX_GPIO_Init(void);         // Initialize GPIO
static void MX_USART2_UART_Init(void);  // Initialize UART2
void StartDefaultTask(void const * argument); // Default RTOS task

/* ---------- User Functions ---------- */

/* Function to send a string via UART2 */
void uart_msg(char *s)
{
    // Send string data to serial port
    HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), 1000);
}

/* Task 1: Blink LED periodically */
void task1(void *p)
{
    uart_msg("TASK-1 EXECUTING\r\n");  // Print start message

    while(1)   // Infinite loop
    {
        vTaskDelay(250);                        // Delay for 250 ticks
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Toggle LED on pin PA5
    }
}

/* Task 2: Monitor button and delete itself when pressed */
void task2(void *p)
{
    uart_msg("TASK-2 EXECUTING\r\n");  // Print start message

    while(1)   // Infinite loop
    {
        // Read button state (connected to PC13)
        if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) // Button not pressed (logic HIGH)
        {
            HAL_Delay(1000);                       // Delay 1 second (blocking delay)
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED
        }
        else // Button pressed (logic LOW)
        {
            uart_msg("BUTTON press detected!\r\n");
            uart_msg("TASK-2 going to be deleted\r\n");

            vTaskDelete(NULL);  // Delete this task itself

            // Below line will never execute (task already deleted)
            uart_msg("TASK-2 DELETED\r\n");
        }
    }
}

/* ---------- Main Function ---------- */
int main(void)
{
    HAL_Init();              // Initialize HAL library
    SystemClock_Config();    // Configure system clock
    MX_GPIO_Init();          // Initialize GPIO pins
    MX_USART2_UART_Init();   // Initialize UART2

    uart_msg("TASK DELETE DEMO\r\n");  // Print demo start message

    // Create FreeRTOS tasks
    xTaskCreate(task1, "TASK1", 128, NULL, 1, &xTaskHandle1);  // Create Task 1
    xTaskCreate(task2, "TASK2", 256, NULL, 2, &xTaskHandle2);  // Create Task 2

    // Create default task (CubeMX auto-generated)
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    osKernelStart();   // Start FreeRTOS scheduler (never returns)

    // Should never reach here
    while (1)
    {

    }
}

/* ---------- System Clock Configuration ---------- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // Clock settings structure
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // Bus clock settings

    // Configure voltage regulator
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    // Configure HSI (High Speed Internal) oscillator and PLL
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

    // Configure bus clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

/* ---------- UART2 Initialization ---------- */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;                          // Use USART2
    huart2.Init.BaudRate = 115200;                     // Baud rate
    huart2.Init.WordLength = UART_WORDLENGTH_8B;       // 8-bit data
    huart2.Init.StopBits = UART_STOPBITS_1;            // 1 stop bit
    huart2.Init.Parity = UART_PARITY_NONE;             // No parity
    huart2.Init.Mode = UART_MODE_TX_RX;                // Enable TX and RX
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;       // No hardware flow control
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;   // Oversampling
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart2) != HAL_OK)              // Initialize UART
        Error_Handler();
}

/* ---------- GPIO Initialization ---------- */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};  // Structure for GPIO config

    __HAL_RCC_GPIOC_CLK_ENABLE();   // Enable clock for GPIOC
    __HAL_RCC_GPIOH_CLK_ENABLE();   // Enable clock for GPIOH
    __HAL_RCC_GPIOA_CLK_ENABLE();   // Enable clock for GPIOA
    __HAL_RCC_GPIOB_CLK_ENABLE();   // Enable clock for GPIOB

    // Configure LED pin (PA5) as output
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    // Configure button pin (PC13) as input with falling edge interrupt
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/* ---------- Default RTOS Task ---------- */
void StartDefaultTask(void const * argument)
{
    while(1)
    {
        osDelay(1);   // Small delay
    }
}

/* ---------- Error Handler ---------- */
void Error_Handler(void)
{
    __disable_irq();   // Disable all interrupts
    while (1) {}       // Stay here forever
}
