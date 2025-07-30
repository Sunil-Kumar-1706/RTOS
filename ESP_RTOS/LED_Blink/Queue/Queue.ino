static const uint8_t msg_queue_len = 5;
static QueueHandle_t msg_queue;

void Message(void *parameters) {
  int item;
  while (1) {
    if (xQueueReceive(msg_queue, (void *)&item, 0) == pdTRUE) {
          Serial.println(item);
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void setup() {

  Serial.begin(115200);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  msg_queue = xQueueCreate(msg_queue_len, sizeof(int));
  xTaskCreatePinnedToCore(Message, "Message", 1024, NULL, 1, NULL,1);

}

void loop() {

  static int num  = 0;
  if (xQueueSend(msg_queue, (void *)&num, 10) != pdTRUE) {
    Serial.println("Queue full");
  }
  num++;
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}