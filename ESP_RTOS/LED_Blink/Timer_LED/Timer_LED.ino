static const TickType_t dim_delay = 5000 / portTICK_PERIOD_MS;
static const int LED = 2;
static TimerHandle_t one_shot_timer = NULL;

void autoDimmerCallback(TimerHandle_t xTimer) {
  digitalWrite(LED, LOW);
}

void task(void *parameters) {
  char c;
  pinMode(LED, OUTPUT);
  while (1) {
    if (Serial.available() > 0) {
      c = Serial.read();
      Serial.print(c);
      digitalWrite(LED, HIGH);
      xTimerStart(one_shot_timer, portMAX_DELAY);
    }
  }
}

void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  one_shot_timer = xTimerCreate("One-shot timer", dim_delay, pdFALSE, (void *)0, autoDimmerCallback);

  xTaskCreatePinnedToCore(task, "task", 1024, NULL, 1, NULL, 1);
  vTaskDelete(NULL);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}