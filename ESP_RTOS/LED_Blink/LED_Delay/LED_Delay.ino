#include <stdlib.h>
static const uint8_t buf_len = 20;
static const int LED = 2;
static int Delay = 500;   

void toggleLED(void *parameter) 
{
  while (1) 
  {
    digitalWrite(LED, HIGH);
    vTaskDelay(Delay / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(Delay / portTICK_PERIOD_MS);
  }
}

void readSerial(void *parameters) 
{

  char c;
  char buf[buf_len];
  uint8_t i = 0;
  memset(buf, 0, buf_len);

  while (1) 
  {

    if (Serial.available() > 0) 
    {
      c = Serial.read();

      if (c == '\n') 
      {
        Delay = atoi(buf);
        Serial.print("Updated Delay : ");
        Serial.println(Delay);
        memset(buf, 0, buf_len);
        i = 0;
      } 
      else 
      {
        if (i < buf_len - 1) 
        {
          buf[i] = c;
          i++ ;
        }
      }
    }
  }
}

void setup() 
{
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Enter the Delay to be updated in milliseconds: ");
  xTaskCreatePinnedToCore(toggleLED,"Toggle LED",1024,NULL,1,NULL,1);      
  xTaskCreatePinnedToCore(readSerial,"Read Serial",1024,NULL,1,NULL,1);       
  vTaskDelete(NULL);
}

void loop() {
}