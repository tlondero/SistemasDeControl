#include "angle.h"
#define PC_DEBUG
#define PRINT_ANGLE_DEBUG
#define TIMES 100.0
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

float get_angle(void)
{

    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float angle = atan2(a.acceleration.y, a.acceleration.x);

    return angle;
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
