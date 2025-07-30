static const int LED = 2;
static SemaphoreHandle_t bin_sem;

void blink_led(void *parameters) {
  int num = *(int *)parameters;
  xSemaphoreGive(bin_sem);
  Serial.print("Received: ");
  Serial.println(num);
  pinMode(LED, OUTPUT);
  while (1) {
    digitalWrite(LED, HIGH);
    vTaskDelay(num / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(num / portTICK_PERIOD_MS);
  }
}

void setup() {
  long int delay_arg;
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Enter a number for delay (milliseconds)");
  while (Serial.available() <= 0);
  delay_arg = Serial.parseInt();
  Serial.print("Sending: ");
  Serial.println(delay_arg);
  bin_sem = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(blink_led,"blink_led",1024,(void *)&delay_arg,1,NULL,1);                   
  xSemaphoreTake(bin_sem, portMAX_DELAY);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}