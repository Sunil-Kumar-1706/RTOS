#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

static SemaphoreHandle_t mutex_1;
static SemaphoreHandle_t mutex_2;

void doTaskA(void *parameters) {
  while (1) {
    xSemaphoreTake(mutex_1, portMAX_DELAY);
    Serial.println("Task A took mutex 1");
    vTaskDelay(1 / portTICK_PERIOD_MS);

    xSemaphoreTake(mutex_2, portMAX_DELAY);
    Serial.println("Task A took mutex 2");

    Serial.println("Task A doing some work");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    xSemaphoreGive(mutex_2);
    xSemaphoreGive(mutex_1);

    Serial.println("Task A going to sleep");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void doTaskB(void *parameters) {
  while (1) {
    xSemaphoreTake(mutex_1, portMAX_DELAY);
    Serial.println("Task B took mutex 1");
    vTaskDelay(1 / portTICK_PERIOD_MS);

    xSemaphoreTake(mutex_2, portMAX_DELAY);
    Serial.println("Task B took mutex 2");

    Serial.println("Task B doing some work");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    xSemaphoreGive(mutex_2);
    xSemaphoreGive(mutex_1);

    Serial.println("Task B going to sleep");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(9600);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();

  mutex_1 = xSemaphoreCreateMutex();
  mutex_2 = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(doTaskA, "Task A", 1024, NULL, 2, NULL, app_cpu);
  xTaskCreatePinnedToCore(doTaskB, "Task B", 1024, NULL, 1, NULL, app_cpu);

  vTaskDelete(NULL);
}

void loop() {}
