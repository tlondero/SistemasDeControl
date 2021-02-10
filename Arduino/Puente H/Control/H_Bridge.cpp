#include "H_Bridge.h"

#import <Arduino.h>
//#include <Wire.h>

#define MAX_PWM 140 //7V con 12V alimentacion
	
void HBRIDGE::H_Bridge_Init(int in_plus_, int in_minus_, int enb_pwm_)
{
	in_plus = in_plus_;  
	in_minus = in_minus_; 
	enb_pwm = enb_pwm_;
	
	pinMode(in_plus, OUTPUT); 
	pinMode(in_minus, OUTPUT);
	pinMode(enb_pwm, OUTPUT);
	
	digitalWrite(in_plus, LOW);
	digitalWrite(in_minus, LOW);
	analogWrite(enb_pwm,0);		
}

void HBRIDGE::H_Bridge_Set_Pwm(uint8_t pwm_value_)
{
  pwm_value = min(pwm_value_, MAX_PWM);
  analogWrite(enb_pwm, pwm_value);
}

void HBRIDGE::H_Bridge_Set_Dir(h_direction_t dir)
{	static h_direction_t old_dir=H_OFF;
	if(old_dir != dir){
		old_dir=dir;
	bool turnOff = false;
	analogWrite(enb_pwm, 0);
	delayMicroseconds(2);
	digitalWrite(in_plus, LOW);
	digitalWrite(in_minus, LOW);				
	
	switch(dir){
	case (H_FOWARD):
	digitalWrite(in_plus, HIGH);
	break;
	case (H_BACKWARD):
	digitalWrite(in_minus, HIGH);
	break;
	case (H_OFF):
		turnOff = true;
	break;
	default:
		turnOff = true;
	break;		
	}
	
	if(!turnOff){
		delayMicroseconds(2);
		analogWrite(enb_pwm, pwm_value);
	}
	}	
}	
