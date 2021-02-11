#include "angle.h"
#include "H_Bridge.h"
#define PC_DEBUG
#define VALUE 40
#define SETPOINT (-45)
float computePID(float inp);
//PID constants
//double kp = 0.3;//esto es bueno para -90 ¬ 0
double kp = 0.3;
double ki = 0.0015;
double kd = 0.7;
 HBRIDGE hb;
unsigned long currentTime, previousTime;
float elapsedTime;
float error;
float lastError;
float input, output, setPoint;
float cumError, rateError;
 bool first_time;
void setup(void)
{
  first_time=true;
    setPoint = SETPOINT;  
#ifdef PC_DEBUG
    Serial.begin(115200);  
#endif
    init_mpu();
    hb.H_Bridge_Init(8, 7, 3);    //Motor Plus, Minus and PWM
#ifdef PC_DEBUG
    bool rdy=false;
    while(!(rdy)){
      if(Serial.available()>0){
        rdy=true;
        delay(500);
while(Serial.available()) {Serial.read();}
      }
}
#endif
}

void loop(void)
{
  static int old_aux=0;
  static bool stop_bool=false;
  static float angle=0.0f;
if(!stop_bool){
    angle=get_angle()*180/PI;//read angle in degrees

        output = computePID(angle);
        output=output/4;
        if(output>VALUE){
          output=VALUE;}
          else if(output < -VALUE){
            output=-VALUE;//50 es un buen numero para cuadrante 4
            }     
      if(output > 0){//primer y cuarto cuadrante
        hb.H_Bridge_Set_Dir(H_FOWARD);//control the motor based on PID value
        
        int aux=(int)abs(output);
#ifdef PC_DEBUG
        if(!((int)aux > ((int)old_aux-2) &&((int)aux < ((int)old_aux+2)) )){
        old_aux=aux;
        Serial.print("Direction FORWARD\r\n");
        Serial.print("Output PID: ");
        Serial.println((int)aux);
        }
#endif
        if(aux>120)
        {
                 hb.H_Bridge_Set_Pwm(120);  //3, 5, 6, 9, 10, 11
          }
/*          else if(aux<20){
                 hb.H_Bridge_Set_Pwm(0);  //3, 5, 6, 9, 10, 11
            }*/
          else{
       hb.H_Bridge_Set_Pwm((int)aux);  //3, 5, 6, 9, 10, 11
          }
        
      }
      else{
          hb.H_Bridge_Set_Dir(H_BACKWARD);//control the motor based on PID value
        int aux=(int)abs(output);
#ifdef PC_DEBUG
                if(!((int)aux > ((int)old_aux-2) &&((int)aux < ((int)old_aux+2)) )){
        old_aux=aux;
        Serial.print("Direction BACKWARD\r\n");
        Serial.print("Output PID: ");
        Serial.println((int)aux);
        }
#endif
        if(aux>120)
        {
                 hb.H_Bridge_Set_Pwm(120);  //3, 5, 6, 9, 10, 11
          }
                   /* else if(aux<20){
                 hb.H_Bridge_Set_Pwm(0);  //3, 5, 6, 9, 10, 11
            }*/
          else{
            hb.H_Bridge_Set_Pwm((int)aux);  //3, 5, 6, 9, 10, 11
       }
      
}
#ifdef PC_DEBUG
      if(Serial.available()>0){
        stop_bool=true;
        hb.H_Bridge_Set_Dir(H_OFF);
        Serial.println("Apagado");
                delay(500);
while(Serial.available()) {Serial.read();}
      }
#endif
}
#ifdef PC_DEBUG
      if((stop_bool) && (Serial.available()>0)){
        first_time=true;
        stop_bool=false;
                Serial.println("Prendido");
                delay(500);
while(Serial.available()) {Serial.read();}
        }
#endif
}

float computePID(float inp){

        if(first_time){
          first_time=false;
          cumError=0;
          previousTime=millis();
          }     
        currentTime = millis();                //get current time
        elapsedTime = (float)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
} 
