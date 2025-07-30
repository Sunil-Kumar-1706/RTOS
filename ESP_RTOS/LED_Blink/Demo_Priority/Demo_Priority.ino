const char msg[] = "Welcome To ESP";

static TaskHandle_t t1 = NULL;
static TaskHandle_t t2 = NULL;


void startTask1(void *parameter) 
{
  int msg_len = strlen(msg);

  while (1) {
    Serial.println();
    for (int i = 0; i < msg_len; i++) {
      Serial.print(msg[i]);
    }
    Serial.println();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void startTask2(void *parameter) 
{
  while (1) {
    Serial.print('*');
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
 
void setup() {

  Serial.begin(300);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("RTOS");
  Serial.print("Setup and loop task running on core ");
  Serial.print(xPortGetCoreID());
  Serial.print(" With priority ");
  Serial.println(uxTaskPriorityGet(NULL));

  xTaskCreatePinnedToCore(startTask1,"Task 1",1024,NULL,1,&t1,1);
  xTaskCreatePinnedToCore(startTask2,"Task 2",1024,NULL,2,&t2,1);

}

void loop() {
  for (int i = 0; i < 3; i++) 
  {
    vTaskSuspend(t2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskResume(t2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  if (t1 != NULL) 
  {
    vTaskDelete(t1);
    t1 = NULL;
  }
}