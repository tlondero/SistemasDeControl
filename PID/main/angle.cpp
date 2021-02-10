#include "angle.h"

static Adafruit_MPU6050 mpu;

void init_mpu(void)
{
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.begin(115200);

    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    // Try to initialize!
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
}

float get_angle(void)
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float angle = atan2(a.acceleration.y, a.acceleration.x);
    Serial.println(angle);
    return angle;
}