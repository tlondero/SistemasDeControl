#include "angle.h"
#include "H_Bridge.h"
//#define PC_DEBUG
#define VALUE 40
#define NUM_SAMPLES 75
#define NPOLE 1
#define NZERO 1
#define SETPOINT 10

float computePID(float inp);

//double kp_1 = 0.090000;
//double ki_1 = 0.000100; //abajo de 30
//double kd_1 = 0.022500;
/*
double kp_1 = 0.060000;
double ki_1 = 0.000070; //abajo de 30
double kd_1 = 0.522500;

double kp_2 = 0.045000;
double ki_2 = 0.000050; //-30 a 30
double kd_2 = 1.640000;

double kp_3 = 0.020000;
double ki_3 = 0.000050; //arriba de 30
double kd_3 = 2.955500;
*/
double kp_1 = 0.00;
double ki_1 = 0.00001; //abajo de 30
double kd_1 = 0;

double kp_2 = 0.001;
double ki_2 = 0.00001; //-30 a 30
double kd_2 = 0;

double kp_3 = 0.5;
double ki_3 = 0.00001; //arriba de 30
double kd_3 = 0;
HBRIDGE hb;
unsigned long currentTime, previousTime;
float elapsedTime;
float error;
float lastError;
float filteredError;
float input, output, setPoint;
float cumError, rateError;
bool first_time;
bool goingFoward;
bool clamped;
double avg_buffer[NUM_SAMPLES];
int pointer;
bool changedSetpoint;
typedef double REAL;
REAL acoeff[]={-0.9964444320905282,1};
REAL bcoeff[]={1,1};
REAL gain=562.4980455786416;
REAL xv[]={-45.0,-45.0};
REAL yv[]={-45.0,-45.0};

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

void add_to_buffer(double angle)
{
  avg_buffer[pointer % NUM_SAMPLES] = angle;
  pointer++;
}

double get_filt_out(double angle)
{
  add_to_buffer(angle);
  double aux = 0;
  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    aux += avg_buffer[i];
  }
  return aux / (double)NUM_SAMPLES;
}
 
void setup(void)
{
  first_time=true;
  setPoint = SETPOINT;  

  Serial.begin(115200);  

  init_mpu();
  hb.H_Bridge_Init(8, 7, 3);    //Motor Plus, Minus and PWM
  #ifdef PC_DEBUG
  bool rdy=false;
    while(!(rdy)){
    if(Serial.available()>0){
      rdy=true;
      delay(500);
      while(Serial.available()) {Serial.read();}
      Serial.println("Prendido");
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
    setPoint = applyfilter(SETPOINT);
    #ifdef PC_DEBUG
    Serial.print("Setpoint:");
    Serial.print(setPoint);
    #endif
    angle = get_filt_out(get_angle() * 180 / PI); //read angle in degrees
    output = computePID(angle);

    #ifdef PC_DEBUG
    //Serial.print("Output:");
    //Serial.print(output);
    //Serial.print(",");
    //Serial.print("Angle:");
    //Serial.print(angle);
    //Serial.println();
    #endif
    
    if((int)output > VALUE){
      output=VALUE;
      clamped = true;
      #ifdef PC_DEBUG
      //Serial.println("Acotado a VALUE");
      #endif
    }
    else if(output < -VALUE){
      output=-VALUE;//50 es un buen numero para cuadrante 4
      clamped = true;
      #ifdef PC_DEBUG
      //Serial.println("Acotado a -VALUE");
      #endif
    }
    else{
      clamped = false;
    }
    int aux=(int)(abs(output));
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
    if(!goingFoward){
      aux *= 2;
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
  double out = 0;
  
  if(first_time){
    first_time=false;
    cumError=0;
    previousTime=millis();
  }     
  currentTime = millis();                //get current time
  elapsedTime = (float)(currentTime - previousTime);        //compute time elapsed from previous computation
  
  error = setPoint - inp;                        // determine error
  rateError = (error - lastError)/elapsedTime;
  
  if(!(clamped && ((goingFoward && error>0)||(!goingFoward && error <0)))){
    if(inp > -180 && inp <= -30){
      cumError += error * ki_1 * elapsedTime;               // compute integral with anti windup
    }
    else if (inp >-30 && inp <= 30){
      cumError += error * ki_2 * elapsedTime;
    }
    else if (inp > 30 && inp < 180){
      cumError += error * ki_3 * elapsedTime;
    }
  }

    if(inp > -180 && inp <= -30){
      out = kp_1*error + cumError + kd_1*rateError;
    }
    else if (inp >-30 && inp <= 30){
      out = kp_2*error + cumError*0.985 + kd_2*rateError;
    }
    else if (inp > 30 && inp < 180){
      out = kp_3*error + cumError*0.975 + kd_3*rateError;
    }

  lastError = error;                                //remember current error
  previousTime = currentTime;                        //remember current time

  return out;                                        //have function return the PID output
} 
