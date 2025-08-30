#include "Adafruit_MPU6050.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void start_imu(void)
{
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  // mpu.reset();
  // delay(100);


  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void get_imu_reading(sensors_event_t *accel, sensors_event_t *gyro)
{
  sensors_event_t temp;
  mpu.getEvent(accel, gyro, &temp);
}

void zero_imu(void)
{
  //calibration.gyro_offset
  int n = 50;
  float sum = 0;
  for(int i=0;i<n;i++) {
    sensors_event_t temp, accel, gyro;
    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.y;
  }

  float average = sum / n;
  Serial.print("Gyro calibration result: "); Serial.println(average, 4);
  calibration.gyro_offset = average;
  
	save_calibration();
}

void setup_motion_detection_wakeup_pin(void) 
{
  //mpu.setLowPowerAccel(MPU6050_CYCLE_5_HZ);

  mpu.setInterruptPinPolarity(false);
  mpu.setInterruptPinLatch(true);

  mpu.setMotionDetectionThreshold(1); // Set your desired motion detection threshold
  mpu.setMotionDetectionDuration(5); // Set the duration for motion detection
  mpu.setMotionInterrupt(true); // Enable the interrupt on motion detection


  mpu.setGyroStandby(true, true, true);
  mpu.setTemperatureStandby(true);
  // mpu.setCycleRate(MPU6050_CYCLE_5_HZ);
  // mpu.enableCycle(true);


  Serial.println("motion interrupt enabled");
}
