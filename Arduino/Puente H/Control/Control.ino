#include <math.h>
#include "H_Bridge.h"

#define LED_PIN 13

HBRIDGE hb;

void setup() {
  hb.H_Bridge_Init(8, 7, 3);    //Motor Plus, Minus and PWM
  hb.H_Bridge_Set_Pwm(2*35);  //3, 5, 6, 9, 10, 11
  hb.H_Bridge_Set_Dir(H_BACKWARD);

  //Serial.begin(9600);
}

void loop() {
    
}
