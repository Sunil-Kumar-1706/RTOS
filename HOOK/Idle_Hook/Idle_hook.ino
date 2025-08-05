#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"

volatile unsigned long idleCounter = 0;

bool IdleHook()
{
  idleCounter=25;
  return true;
}

void Task1(void *pvParameters)
{
  while (1)
  {
    Serial.print("Idle Count: ");
    Serial.println(idleCounter);
    idleCounter = 0;
    vTaskDelay(pdMS_TO_TICKS(1000));
    
  }
}

void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("Starting...");

  esp_register_freertos_idle_hook_for_cpu(IdleHook, 0);
  esp_register_freertos_idle_hook_for_cpu(IdleHook, 1);

  xTaskCreate(Task1, "Task1", 2048, NULL, 1, NULL);
}

void loop() {}
