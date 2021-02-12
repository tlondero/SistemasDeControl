// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float angle;
float wx,wy,wz;
void setup(void) {
  Serial.begin(115200);
  
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Angle: ");
  angle = atan2(a.acceleration.y, a.acceleration.x)*180/PI;
  Serial.println(angle);
  Serial.println("Gyroscope readings: ");
  Serial.print(" Wheading : ");
  wx=g.gyro.heading;
   Serial.println(wx);
     Serial.print(" Wpitch : ");
  wy=g.gyro.pitch;
   Serial.println(wy);
     Serial.print(" Wroll : ");
  wz=g.gyro.roll;
   Serial.println(wz);
  
  Serial.println("");
  delay(500);
}
