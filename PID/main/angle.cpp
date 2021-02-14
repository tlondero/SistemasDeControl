#include "angle.h"
#define PC_DEBUG
static Adafruit_MPU6050 mpu;

void init_mpu(void)
{
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

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
}

float get_angle(void)
{
  static int old_angle = 0;
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float angle = atan2(a.acceleration.y, a.acceleration.x);
//    if(!((old_angle > (int) (angle*180/PI)-2) && (old_angle <(int) (angle*180/PI)+2))){
//      old_angle=(int)  (angle*180/PI);
//#ifdef PC_DEBUG
//      Serial.print("Angulo: ");
//      Serial.println((int) (angle*180/PI));
//#endif
//    }
    return angle;
}
