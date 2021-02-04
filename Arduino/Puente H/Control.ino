#include <math.h>
#include "H_Bridge.h"

int16_t x, y, z;

HBRIDGE hb;

void setup() {
  hb.H_Bridge_Init(3,4,5);    //Motor Plus, Minus and PWM
  hb.H_Bridge_Set_Pwm(70);
  hb.H_Bridge_Set_Dir(H_FOWARD);
  Serial.begin(9600);
}

void loop() {
  hb.H_Bridge_Set_Dir(H_FOWARD);
  Serial.print("Foward\n");
  delay(2000);

  hb.H_Bridge_Set_Dir(H_OFF);
  Serial.print("Off\n");
  delay(2000);

  hb.H_Bridge_Set_Dir(H_BACKWARD);
  Serial.print("Backward\n");
  delay(2000);

}
