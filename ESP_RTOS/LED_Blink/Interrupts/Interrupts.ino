#include "esp_timer.h"

const int led_pin = 2;
esp_timer_handle_t timer;
volatile bool toggle_flag = false;

void IRAM_ATTR onTimer(void* arg) {
  int state = digitalRead(led_pin);
  digitalWrite(led_pin, !state);
  toggle_flag = true;  // Set flag to indicate LED toggled
}

void setup() {
  Serial.begin(9600);
  pinMode(led_pin, OUTPUT);

  esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "toggle_led"
  };

  esp_timer_create(&timer_args, &timer);
  esp_timer_start_periodic(timer, 1000000); 
}

void loop() {
  if (toggle_flag) {
    toggle_flag = false;  
    Serial.println("LED toggled by timer interrupt.");
  }
}
