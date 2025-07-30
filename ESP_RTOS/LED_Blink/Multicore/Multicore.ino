static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

static const uint32_t task_0_delay = 500;

static SemaphoreHandle_t bin_sem;

void doTask0(void *parameters) {
  while (1) {
    xSemaphoreGive(bin_sem); 
    vTaskDelay(task_0_delay / portTICK_PERIOD_MS);
  }
}

void doTask1(void *parameters) {
  while (1) {
    Serial.println("[Task 1] Waiting for semaphore...");
    xSemaphoreTake(bin_sem, portMAX_DELAY); 
    Serial.println("[Task 1] Semaphore received!");
  }
}

void setup() {
  Serial.begin(9600);
  vTaskDelay(500 / portTICK_PERIOD_MS); /

  bin_sem = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(doTask0, "Task 0", 1024, NULL, 1, NULL, pro_cpu);
  xTaskCreatePinnedToCore(doTask1, "Task 1", 1024, NULL, 1, NULL, app_cpu);

  vTaskDelete(NULL); 
}

void loop() {}
