#include<LPC21xx.h>
#include "Uart.h"
#include "FreeRTOS.h"
#include "task.h"

void task1(void* a);
void task2(void* b);

int main()
{
	UART0_CONFIG();
	
	xTaskCreate(task1,"Task1",128,NULL,1,NULL);
	xTaskCreate(task2,"Task2",128,NULL,2,NULL);
	
	vTaskStartScheduler();
}

void task1(void* a)
{
	while(1)
	{
		uart_string("Inside Task 1");
		uart_string("\r\n");
		
		vTaskDelay(1000);
	}
}

void task2(void* b)
{
	while(1)
	{
		uart_string("Inside Task 2");
		uart_string("\r\n");
		
		vTaskDelay(1000);
	}
}