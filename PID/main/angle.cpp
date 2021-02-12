#include "angle.h"
#define PC_DEBUG
#define PRINT_ANGLE_DEBUG
#define TIMES 200.0
#define TIME_CONSTANT_IN_MILIS (900.0)
static Adafruit_MPU6050 mpu;
float complementary_filter(float angle_, float omega_);

unsigned long currentTime_, previousTime_;
void init_mpu(void)
{
    
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    

#ifdef PC_DEBUG
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
#endif
    // Try to initialize!
    if (!mpu.begin())
    {
#ifdef PC_DEBUG
        Serial.println("Failed to find MPU6050 chip");
        #endif
        while (1)
        {
            delay(10);
        }
    }
        previousTime_= millis();
}
static float angles[(int)TIMES];
float get_angle(void)
{
  static int old_angle = 0;

    static float latest_angle=0;
    static bool first_time=true;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float angle = atan2(a.acceleration.y, a.acceleration.x);

    if(first_time){
      first_time=false;
      latest_angle=angle;
      }
   

    static unsigned long i=0,j=0;
    if((i++ == 100)){
      i=0;
        #ifdef PRINT_ANGLE_DEBUG
    Serial.println("");
    Serial.print("Angle medition of Acc:\t \t\t");
    Serial.println(latest_angle*180/PI);
        Serial.println("");
    #endif
    }

       float aux=0;      
       uint16_t k=0;
       if(j== (int)TIMES){
        j=0;
        }
       angles[j++]=angle;
      for(k=0;k<(int)TIMES;k++){
        aux+=angles[k];
        }
        latest_angle=aux/TIMES;
    return latest_angle;
}

float complementary_filter(float angle_, float omega_){
    static float old_theta =0;

  unsigned long deltaT=0;
    currentTime_ = millis();
    deltaT= currentTime_ - previousTime_;    
  float alpha= (TIME_CONSTANT_IN_MILIS/deltaT)/(1+(TIME_CONSTANT_IN_MILIS/deltaT));
  float beta=1-alpha;
  

    old_theta = alpha*(old_theta +deltaT*omega_ / 1000.0)+beta*(angle_);
  return old_theta;
}
