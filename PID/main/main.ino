#include "angle.h"
#include "H_Bridge.h"
#define PC_DEBUG
#define VALUE 55
#define SETPOINT (45)

typedef float REAL;
#define NPOLE 4
#define NZERO 4
REAL acoeff[]={0.9634534510506384,-3.8896837377884457,5.888999744280504,-3.962769417093502,1};
REAL bcoeff[]={1,4,6,4,1};
REAL gain=395557945.0626531;
REAL xv[]={0,0,0,0,0};
REAL yv[]={0,0,0,0,0};

REAL applyfilter(REAL v)
{
  int i;
  REAL out=0;
  for (i=0; i<NZERO; i++) {xv[i]=xv[i+1];}
  xv[NZERO] = v/gain;
  for (i=0; i<NPOLE; i++) {yv[i]=yv[i+1];}
  for (i=0; i<=NZERO; i++) {out+=xv[i]*bcoeff[i];}
  for (i=0; i<NPOLE; i++) {out-=yv[i]*acoeff[i];}
  yv[NPOLE]=out;
  return out;
}

float computePID(float inp);
//PID constants
//double kp = 0.075;//esto es bueno para -90 ¬ 0
double kp = 0.7;
double ki = 0.00001;
double kd = 0.01;

//double kp = 0.05;
//double ki = 0.0002; andan piola para -45 grados
//double kd = 0.5;

HBRIDGE hb;
unsigned long currentTime, previousTime;
float elapsedTime;
float error;
float lastError;
float filteredError;
float input, output, setPoint;
float cumError, rateError, cumRateError;
bool first_time;
bool goingFoward;
bool clamped;
 
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
    #ifdef PC_DEBUG
      Serial.print("Angulo: ");
      Serial.println(angle);
    #endif
    if((int)output > VALUE){
      output=VALUE;
      clamped = true;
      #ifdef PC_DEBUG
      Serial.println("Acotado a VALUE");
      #endif
    }
    else if(output < -VALUE){
      output=-VALUE;//50 es un buen numero para cuadrante 4
      clamped = true;
      #ifdef PC_DEBUG
      Serial.println("Acotado a -VALUE");
      #endif
    }
    else{
      clamped = false;
    }
    int aux=(int)abs(output);
    #ifdef PC_DEBUG
      Serial.print("aux: ");
      Serial.println((int)aux);
    #endif
    if(output > 0){//primer y cuarto cuadrante
      if(!goingFoward){
        hb.H_Bridge_Set_Dir(H_FOWARD);//control the motor based on PID value
        goingFoward = true;
        #ifdef PC_DEBUG
        if(!((int)aux > ((int)old_aux-2) &&((int)aux < ((int)old_aux+2)) )){
          old_aux=aux;
          Serial.print("Direction FORWARD\r\n");
        }
        #endif
      }
    }
    else{
      if(goingFoward){
        hb.H_Bridge_Set_Dir(H_BACKWARD);//control the motor based on PID value
        goingFoward = false;
        #ifdef PC_DEBUG
        if(!((int)aux > ((int)old_aux-2) &&((int)aux < ((int)old_aux+2)) )){
          old_aux=aux;
          Serial.print("Direction BACKWARD\r\n");
        }
        #endif
      }
    }
    if(aux>VALUE){
      hb.H_Bridge_Set_Pwm(VALUE);  //3, 5, 6, 9, 10, 11
    }
  /*else if(aux<20){
    hb.H_Bridge_Set_Pwm(0);  //3, 5, 6, 9, 10, 11
  }*/
    else{
      hb.H_Bridge_Set_Pwm((int)aux);  //3, 5, 6, 9, 10, 11
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
    cumRateError=0;
    previousTime=millis();
  }     
  currentTime = millis();                //get current time
  elapsedTime = (float)(currentTime - previousTime);        //compute time elapsed from previous computation
  
  error = setPoint - inp;                        // determine error
  if(!(clamped && ((goingFoward && error>0)||(!goingFoward && error <0)))){
    cumError += error * elapsedTime;               // compute integral with anti windup
  }

  filteredError = applyfilter(error);
  rateError = (filteredError - lastError)/elapsedTime;
  
  double out = kp*error + ki*cumError + kd*rateError;                //PID output          
  #ifdef PC_DEBUG
    //Serial.print("Proportional: ");
    //Serial.println((kp*error));
    Serial.print("Integral: ");
    Serial.println((ki*cumError));
    Serial.print("Derivative: ");
    Serial.println((kd*rateError));
   #endif    

  lastError = filteredError;                                //remember current error
  previousTime = currentTime;                        //remember current time

  return out;                                        //have function return the PID output
} 
