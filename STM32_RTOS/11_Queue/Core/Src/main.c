#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

RTC_HandleTypeDef hrtc;            // RTC handle
UART_HandleTypeDef huart2;         // UART2 handle
osThreadId defaultTaskHandle;      // Default task handle

// FreeRTOS task handles
TaskHandle_t taskhandle1 = NULL;
TaskHandle_t taskhandle2 = NULL;
TaskHandle_t taskhandle3 = NULL;
TaskHandle_t taskhandle4 = NULL;

// FreeRTOS queue handles
QueueHandle_t cmd_q = NULL;        // Queue for commands
QueueHandle_t uart_q = NULL;       // Queue for UART messages

// Command structure
typedef struct {
    uint8_t cmd_no;                // Command number
    uint8_t cmd_arg[10];           // Command arguments
} cmd_t;

uint8_t cmd_buf[20];               // UART input buffer
uint8_t cmd_len = 0;               // Current command length
uint8_t rx_byte;                   // Received UART byte

// Menu text
char menu[]="\r\nLED_ON ->1\r\nLED_OFF ->2\r\nLED_TOGGLE ->3\
		\r\nLED_READ_STATUS ->4\r\nRTC_PRINT ->5\
		\r\nEXIT ->0\r\nENTER YOUR OPTION:";

// Command codes
#define LED_ON 	        1
#define LED_OFF         2
#define LED_TOGGLE      3
#define LED_READ_STATUS 4
#define RTC_READ        5

// Function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);

uint8_t getCommandCode(uint8_t *buffer);
void make_led_on(void);
void make_led_off(void);
void led_toggle(void);
void read_led_status(char *task_msg);
void read_rtc_info(char *task_msg);
void print_error_message(char *task_msg);

// UART receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART2) // Check if from USART2
    {
        HAL_UART_Transmit_IT(&huart2, &rx_byte, 1);   // Echo byte back
        cmd_buf[cmd_len++] = rx_byte;                 // Store in buffer

        if (rx_byte == '\r')                          // If Enter pressed
        {
            cmd_len = 0;                              // Reset buffer
            vTaskNotifyGiveFromISR(taskhandle1, &xHigherPriorityTaskWoken); // Notify task1
            vTaskNotifyGiveFromISR(taskhandle2, &xHigherPriorityTaskWoken); // Notify task2
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);    // Enable next receive
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // Context switch if needed
    }
}

// Send string over UART (blocking)
void uart_send(char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), 1000);
}

// Task 1: show menu and wait for command
void task1_menu(void *p)
{
    char *data = menu;
    while(1)
    {
        xQueueSend(uart_q, &data, portMAX_DELAY); // Send menu
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); // Wait for input
    }
}

// Task 2: read command and forward it
void task2_command(void *p)
{
    uint8_t cmd_code = 0;
    cmd_t *new_cmd;

    while(1)
    {
        xTaskNotifyWait(0,0,NULL,portMAX_DELAY);      // Wait for command
        new_cmd = (cmd_t*)pvPortMalloc(sizeof(cmd_t));// Allocate memory

        taskENTER_CRITICAL();                         // Read safely
        cmd_code = getCommandCode(cmd_buf);           // Get code
        new_cmd->cmd_no = cmd_code;                   // Store code
        taskEXIT_CRITICAL();

        xQueueSend(cmd_q, &new_cmd, portMAX_DELAY);   // Send to queue
    }
}

// Task 3: process commands
void task3_command_process(void *p)
{
    cmd_t *new_cmd;
    char task_msg[50];

    while(1)
    {
        xQueueReceive(cmd_q, (void*)&new_cmd, portMAX_DELAY); // Wait for command

        if(new_cmd->cmd_no == LED_ON)
        	make_led_on();
        else if(new_cmd->cmd_no == LED_OFF)
        	make_led_off();
        else if(new_cmd->cmd_no == LED_TOGGLE)
        	led_toggle();
        else if(new_cmd->cmd_no == LED_READ_STATUS)
        	read_led_status(task_msg);
        else if(new_cmd->cmd_no == RTC_READ)
        	read_rtc_info(task_msg);
        else
        	print_error_message(task_msg);

        vPortFree(new_cmd); // Free memory
    }
}

// Task 4: send UART messages
void task4_uart_transmit(void *p)
{
    char *data = NULL;
    while(1)
    {
        xQueueReceive(uart_q, &data, portMAX_DELAY); // Wait for data
        uart_send(data);                             // Send over UART
    }
}

// Extract command code (ASCII to number)
uint8_t getCommandCode(uint8_t *buffer)
{
    return buffer[0] - 48;
}

// Turn LED on (PA5)
void make_led_on(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
}

// Turn LED off (PA5)
void make_led_off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
}

// Toggle LED
void led_toggle(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

// Read LED status
void read_led_status(char *task_msg)
{
    sprintf(task_msg , "\r\nLED status is : %d\r\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5));
    xQueueSend(uart_q,&task_msg,portMAX_DELAY);
}

// Read RTC info
void read_rtc_info(char *task_msg)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    if ((RTC->ISR & RTC_ISR_INITS) == 0) // If RTC not set
    {
        sTime.Hours = 5; sTime.Minutes = 15; sTime.Seconds = 30;
        sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
        sTime.StoreOperation = RTC_STOREOPERATION_RESET;
        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

        sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
        sDate.Month = RTC_MONTH_SEPTEMBER;
        sDate.Date = 11;
        sDate.Year = 25;
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
    }

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // Get time
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // Get date

    sprintf(task_msg, "Time: %02d:%02d:%02d | Date: %02d-%02d-20%02d\r\n",
              sTime.Hours, sTime.Minutes, sTime.Seconds,
              sDate.Date, sDate.Month, sDate.Year);

    xQueueSend(uart_q, &task_msg, portMAX_DELAY);
}

// Print error
void print_error_message(char *task_msg)
{
    sprintf(task_msg, "\r\nInvalid command received\r\n");
    xQueueSend(uart_q, &task_msg, portMAX_DELAY);
}

// Main function
int main(void)
{
  HAL_Init();                    // Initialize HAL
  SystemClock_Config();          // Configure clock
  MX_GPIO_Init();                // Init GPIO
  MX_USART2_UART_Init();         // Init UART2
  MX_RTC_Init();                 // Init RTC

  // Create queues
  cmd_q = xQueueCreate(10, sizeof(cmd_t));
  uart_q = xQueueCreate(10, 8);

  if((cmd_q != NULL) && (uart_q != NULL))
  {
      // Create tasks
      xTaskCreate(task1_menu, "MENU", configMINIMAL_STACK_SIZE, NULL, 1, &taskhandle1);
      xTaskCreate(task2_command, "command_handle", configMINIMAL_STACK_SIZE, NULL, 2, &taskhandle2);
      xTaskCreate(task3_command_process, "command_process", configMINIMAL_STACK_SIZE, NULL, 2, &taskhandle3);
      xTaskCreate(task4_uart_transmit, "uart_transmit", configMINIMAL_STACK_SIZE, NULL, 2, &taskhandle4);

      HAL_UART_Receive_IT(&huart2, &rx_byte, 1); // Start UART RX
  }
  else
  {
      uart_send("QUEUE creation failed\r\n"); // Error if queues not created
  }

  osKernelStart();  // Start RTOS scheduler

  while (1)
  {

  }      // Should never reach here
}

// System clock configuration
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    Error_Handler();

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    Error_Handler();
}

// RTC initialization
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK) Error_Handler();

  sTime.Hours = 0x0; sTime.Minutes = 0x0; sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();

  sDate.WeekDay = RTC_WEEKDAY_MONDAY; sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1; sDate.Year = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) Error_Handler();
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
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

// GPIO initialization
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

// Default task
void StartDefaultTask(void const * argument)
{
  for(;;) osDelay(1);
}

// Error handler
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
