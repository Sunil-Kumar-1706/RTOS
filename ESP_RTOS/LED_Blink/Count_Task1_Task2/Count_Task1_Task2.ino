int c1=0;
int c2=0;

void task1(void* parameter)
{
  while(1)
  {
    Serial.print("TASK1: ");
    Serial.println(c1++);
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void task2(void* parameter)
{
  while(1)
  {
    Serial.print("TASK2: ");
    Serial.println(c2++);
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(9600);
  xTaskCreatePinnedToCore(task1,"Task1",1024,NULL,1,NULL,1);
  xTaskCreatePinnedToCore(task2,"Task2",1024,NULL,1,NULL,1);
}

void loop()
{
}