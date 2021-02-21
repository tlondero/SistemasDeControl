#include "angle.h"
#include "H_Bridge.h"
#include <PID_v1.h>
//#include <PID_AutoTune_v0.h>
#define PC_DEBUG
#define VALUE 80
#define SETPOINT 135
//#define SETPOINT 45 //No anda bien, habria que sacar peso y retocar valores.
#define NPOLE 1
#define NZERO 1
#define ANGLE_OFFSET 1
#define SAMPLE_TIME_IN_MS 5
#define INITIAL_ANGLE 90

typedef double REAL;
REAL acoeff[]={-0.9816475744248808,1};
REAL bcoeff[]={1,1};
REAL gain=108.97742054932769;
REAL xv[]={45.0,45.0};
REAL yv[]={45.0,45.0};

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


/* ultimos valores 15/2 */
/*double kp = 1.1;
double ki = 0.3/(SAMPLE_TIME_IN_MS/5);
double kd = 0.6*(SAMPLE_TIME_IN_MS/5);*/

/* ultimos valores 20/2
double kp = 1.42;
double ki = 3.305/(SAMPLE_TIME_IN_MS/5);
double kd = 0.575*(SAMPLE_TIME_IN_MS/5);*/

/*double kp = 1.2;
double ki = 0/(SAMPLE_TIME_IN_MS/5);
double kd = 0.425*(SAMPLE_TIME_IN_MS/5);*/
double kp = 0.55;
double ki = 3/(SAMPLE_TIME_IN_MS/5);
double kd = 0.45*(SAMPLE_TIME_IN_MS/5);


HBRIDGE hb;
float input, output, setPoint;
bool clamped;
bool stop_bool;
unsigned long int prevTime, curTime, elapsedTime, initTime;
double totalTime;

double Setpoint, Input, Output;
//                                    Kp, Ki, Kd
//PID myPID(&Input, &Output, &Setpoint, 1,0,0 , DIRECT);
//PID myPID(&Input, &Output, &Setpoint, 2,1.1,4, DIRECT);
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

//PID_ATune aTune(&Input, &Output);
#define NUM_SAMPLES 40
double avg_buffer[NUM_SAMPLES];
int pointer;

void init_buffer(void){
  unsigned int i;
  for(i=0;i<NUM_SAMPLES;i++){
    avg_buffer[i]=INITIAL_ANGLE;
  }
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

bool goingFoward;
void setup(void)
{

  Setpoint = SETPOINT;

  Serial.begin(115200);
  init_buffer();
  init_mpu();
  hb.H_Bridge_Init(8, 7, 3); //Motor Plus, Minus and PWM
  hb.H_Bridge_Set_Dir(H_FOWARD);
  digitalWrite(13, HIGH);
  stop_bool = false;
  myPID.SetMode(AUTOMATIC); //AUTOMATIC);
  myPID.SetOutputLimits(-VALUE, VALUE);
  goingFoward = true;
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  //myPID.SetSampleTime(SAMPLE_TIME_IN_MS);
  myPID.SetIntegralError(25);

    Serial.print("-0.25");
    Serial.print(",");
    Serial.print("90");
    Serial.print(",");
    Serial.print("90");
    Serial.print("\r\n");
    
    Serial.print("0.005");
    Serial.print(",");
    Serial.print("90");
    Serial.print(",");
    Serial.print("90");
    Serial.print("\r\n");
  
  prevTime = millis();
  initTime = millis();
}

void loop(void)
{ 
  static float angle = 0.0f;
  curTime = millis();
  elapsedTime = curTime - prevTime;
  totalTime = (double)((curTime - initTime)/1000.0);
  angle = (get_angle() * 180 / PI); //read angle in degrees
  if(angle < 0){
    angle += 360.0;  
  }
  angle = get_filt_out(angle) + ANGLE_OFFSET;
  Input = angle;
  if(elapsedTime >= SAMPLE_TIME_IN_MS){
    myPID.SetSampleTime(elapsedTime);
    Setpoint = applyfilter(SETPOINT);
    myPID.Compute();
    
    uint8_t aux = (uint8_t)abs(Output);
    if (Output > 0)
    {
      hb.H_Bridge_Set_Dir(H_FOWARD);
      digitalWrite(13, HIGH);
    }
    else
    {
      hb.H_Bridge_Set_Dir(H_BACKWARD);
      digitalWrite(13, LOW);
      aux=aux*2;
    }
    hb.H_Bridge_Set_Pwm(aux);
    prevTime = curTime;

    Serial.print(totalTime);
    Serial.print(",");
    Serial.print(Setpoint);
    Serial.print(",");
    Serial.print(angle);
    Serial.print("\r\n");
  }
}
