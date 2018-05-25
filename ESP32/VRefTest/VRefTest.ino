#include "driver/adc.h"

void setup() {
  pinMode(A12, INPUT);
  Serial.begin(115200);

  //IRM Routing internal VREF (1000 mV) to GPIO25 (A12)
  esp_err_t status = adc2_vref_to_gpio(GPIO_NUM_25);
  if (status == ESP_OK) {
      printf("v_ref routed to GPIO25\n");
  } else {
      printf("failed to route v_ref\n");
  }
  
}

void loop() {

    Serial.print("ADC Ref: ");

    //IRM Reading internal 1V reference via GPIO   
    Serial.println(analogRead(A12));
    delay(250);
}
