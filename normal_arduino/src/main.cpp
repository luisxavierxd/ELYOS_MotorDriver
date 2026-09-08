#include "ELYOS_DRIVER.h"

ELYOS_DRIVER driver;
// Blink aux 
uint32_t blink_aux = 0;

int main(){

  // Setup 
  pinMode(LED_BUILTIN, OUTPUT);
  driver.driver_Init();

  while(true){
    driver.runFOC();

    // Blink LED
    if(millis() - blink_aux >= 500){
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      blink_aux = millis();
    }
  }
  return 0;
}