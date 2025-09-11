#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

UART_HandleTypeDef huart2;              // UART handle for USART2
osThreadId defaultTaskHandle;           // Default task handle
xSemaphoreHandle binarySemaphore;       // Binary semaphore handle
xQueueHandle idQueue;                   // Queue handle for storing IDs

// Send string over UART
void UART_SendString(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 1000);
}

// Producer task: generates IDs and sends them to the queue
void ProducerTask(void *parameters)
{
    unsigned int generatedID;           // Randomly generated ID
    portBASE_TYPE queueStatus;          // Queue status return value

    xSemaphoreGive(binarySemaphore);    // Give semaphore at start

    while(1)
    {
        generatedID = (rand() & 0x1FF); // Generate random ID
        queueStatus = xQueueSend(idQueue, &generatedID, portMAX_DELAY); // Send ID to queue

        if(queueStatus != pdPASS)       // If failed to send
        {
            UART_SendString("Failed to send ID to queue\r\n");
        }
        else                            // If sent successfully
        {
            xSemaphoreGive(binarySemaphore); // Release semaphore
            taskYIELD();                    // Yield CPU to other tasks
        }
    }
}

// Process received ID
void ProcessID(unsigned char id)
{
    char message[50];                   // Buffer for message
    sprintf(message, "ID received: %d\r\n", id); // Format message
    UART_SendString(message);           // Send message
    vTaskDelay(id);                     // Delay for simulation
}

// Consumer task: receives IDs and processes them
void ConsumerTask(void *parameters)
{
    unsigned char receivedID;           // Received ID from queue
    portBASE_TYPE queueStatus;          // Queue status return value

    xSemaphoreGive(binarySemaphore);    // Give semaphore at start

    while(1)
    {
        xSemaphoreTake(binarySemaphore, 0);   // Try to take semaphore (non-blocking)
        queueStatus = xQueueReceive(idQueue, &receivedID, 0); // Try to receive ID

        if(queueStatus == pdPASS)       // If ID received
        {
            ProcessID(receivedID);      // Process the ID
        }
        else                            // If queue empty
        {
            UART_SendString("Queue is empty\r\n");
        }
    }
}

// Main function
int main(void)
{
    HAL_Init();                         // Initialize HAL library
    SystemClock_Config();               // Configure system clock
    MX_GPIO_Init();                     // Initialize GPIO
    MX_USART2_UART_Init();              // Initialize USART2

    vSemaphoreCreateBinary(binarySemaphore); // Create binary semaphore
    idQueue = xQueueCreate(1, sizeof(unsigned int)); // Create queue of length 1

    if((binarySemaphore != NULL) && (idQueue != NULL)) // Check if created
    {
        xTaskCreate(ProducerTask, "ProducerTask", configMINIMAL_STACK_SIZE, NULL, 3, NULL); // Create producer task
        xTaskCreate(ConsumerTask, "ConsumerTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL); // Create consumer task
    }
    else
    {
        UART_SendString("Queue/Semaphore creation failed\r\n");
    }

    osKernelStart();                    // Start FreeRTOS scheduler

    while (1)                           // Should never reach here
    {
    }
}

// System clock configuration
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                  RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}

// USART2 initialization
void MX_USART2_UART_Init(void)
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
    HAL_UART_Init(&huart2);
}

// GPIO initialization
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Set LED low

    GPIO_InitStruct.Pin = B1_Pin;       // Configure button pin
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LD2_Pin;      // Configure LED pin
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

// Default task (unused)
void StartDefaultTask(void const * argument)
{
    for(;;)
    {
        osDelay(1);                    // Delay 1 tick
    }
}

// Error handler
void Error_Handler(void)
{
    __disable_irq();                   // Disable interrupts
    while (1) {}                        // Stay here
}
