const int LED=2;

void toggle_led(void* parameter)
{
  while(1)
  {
  digitalWrite(LED,HIGH);
  vTaskDelay(500/portTICK_PERIOD_MS);
  digitalWrite(LED,LOW);
  vTaskDelay(500/portTICK_PERIOD_MS);
  }
}

void setup()
{
  pinMode(LED,OUTPUT);
  xTaskCreatePinnedToCore(toggle_led,"Toggle_led",1024,NULL,1,NULL,1);
}

void loop()
{
}