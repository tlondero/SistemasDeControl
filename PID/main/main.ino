#include "angle.h"
#include "H_Bridge.h"
#include <PID_v1.h>
//#include <PID_AutoTune_v0.h>
#define PC_DEBUG
#define VALUE 70
#define SETPOINT 160

//Esto estaba de antes
//double kp = 10;
//double ki = 0; //0.00001;
//double kd = 5; //0.01;

//PARA 160 y 145
//double kp = 0.5;
//double ki = 0.023;
//double kd = 0.09;

double kp = 0.35;
double ki = 0.035;
double kd = 0.09;

HBRIDGE hb;
float input, output, setPoint;
bool clamped;
bool stop_bool;

double Setpoint, Input, Output;
//                                    Kp, Ki, Kd
//PID myPID(&Input, &Output, &Setpoint, 1,0,0 , DIRECT);
//PID myPID(&Input, &Output, &Setpoint, 2,1.1,4, DIRECT);
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

//PID_ATune aTune(&Input, &Output);
#define NUM_SAMPLES 50
double avg_buffer[NUM_SAMPLES];
int pointer;

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
  init_mpu();
  hb.H_Bridge_Init(8, 7, 3); //Motor Plus, Minus and PWM
  stop_bool = false;
  myPID.SetMode(AUTOMATIC); //AUTOMATIC);
  myPID.SetOutputLimits(-VALUE, VALUE);
  goingFoward = true;
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  myPID.SetSampleTime(5);
}

void loop(void)
{ 
  static float angle = 0.0f;
  angle = get_filt_out(get_angle() * 180 / PI); //read angle in degrees
  if(angle < 0){
    angle += 360.0;  
  }
  Input = angle;

  myPID.Compute();
  Serial.print("Output:");
  Serial.print(Output);
  Serial.print(",");
  //output = computePID(angle);

  Serial.print("Angulo:");
  Serial.print(angle);
  Serial.print(",");
  Serial.print("direction:");
  int static dir = 1;
  uint8_t aux = (uint8_t)abs(Output);
  if (Output > 0)
  {
    hb.H_Bridge_Set_Dir(H_FOWARD);
    digitalWrite(13, HIGH);
    dir = 20;
  }
  else
  {
    hb.H_Bridge_Set_Dir(H_BACKWARD);
    digitalWrite(13, LOW);
    dir = -20;
  }
  hb.H_Bridge_Set_Pwm(aux);
  Serial.println(dir);
}
