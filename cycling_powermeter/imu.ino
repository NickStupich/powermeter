#include "LSM6DS3.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A); // IMU

void start_imu(void)
{
  uint16_t errorsAndWarnings = 0; //TODO: remove
    uint8_t dataToWrite = 0;  //Temporary variable

  if (myIMU.beginCore() != 0) {
    Serial.println("IMU begin core error");
  } else {
    Serial.println("IMU OK!");
  }

    //Setup the accelerometer******************************
    dataToWrite = 0; //Start Fresh!
    dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_16g;
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

    //Now, write the patched together data
    errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

    
    //Setup the gyroscope******************************
    dataToWrite = 0; //Start Fresh!
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
    dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
    dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;

    //Now, write the patched together data
    errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

    
    // //Set the ODR bit
    // errorsAndWarnings += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
    // dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

}
float calcAccel( int16_t input )
{
  float output = (float)input * 0.061 * (16 >> 1) / 1000;
  return output;
}

float calcGyro( int16_t input )
{
	uint8_t gyroRangeDivisor = 1000 / 125;
	// if ( settings.gyroRange == 245 ) {
	// 	gyroRangeDivisor = 2;
	// }

	float output = (float)input * 4.375F * (gyroRangeDivisor) / 1000.0F * PI / 180.0F;
	return output;
}


//TODO: merge/simplify sensors_event_t* structs
void get_imu_reading(sensors_event_t *accel, sensors_event_t *gyro)
{
  int errors = 0;
  int16_t tempX, tempY, tempZ;
  //Acelerometer axis X
    if (myIMU.readRegisterInt16(&tempX, LSM6DS3_ACC_GYRO_OUTX_L_G) != 0) {
        errors++;
    }

    //Acelerometer axis Y
    if (myIMU.readRegisterInt16(&tempY, LSM6DS3_ACC_GYRO_OUTY_L_G) != 0) {
        errors++;
    }

    //Acelerometer axis Z
    if (myIMU.readRegisterInt16(&tempZ, LSM6DS3_ACC_GYRO_OUTZ_L_G) != 0) {
        errors++;
    }
    gyro->gyro.x = calcGyro(tempX);
    gyro->gyro.y = calcGyro(tempY);
    gyro->gyro.z = calcGyro(tempZ);


    // Serial.print(gyro->gyro.x); Serial.print("\t");
    // Serial.print(gyro->gyro.y); Serial.print("\t");
    // Serial.print(gyro->gyro.z); Serial.print("\t");    
    // Serial.println();



    if (myIMU.readRegisterInt16(&tempX, LSM6DS3_ACC_GYRO_OUTX_L_XL) != 0) {
        errors++;
    }

    //Acelerometer axis Y
    if (myIMU.readRegisterInt16(&tempY, LSM6DS3_ACC_GYRO_OUTY_L_XL) != 0) {
        errors++;
    }

    //Acelerometer axis Z
    if (myIMU.readRegisterInt16(&tempZ, LSM6DS3_ACC_GYRO_OUTZ_L_XL) != 0) {
        errors++;
    }
    accel->acceleration.x = calcAccel(tempX);
    accel->acceleration.y = calcAccel(tempY);
    accel->acceleration.z = calcAccel(tempZ);

    // Serial.print(accel->acceleration.x); Serial.print("\t");
    // Serial.print(accel->acceleration.y); Serial.print("\t");
    // Serial.print(accel->acceleration.z); Serial.print("\t");    
    // Serial.println();
}

void zero_imu(void)
{
  //calibration.gyro_offset
  int n = 50;
  float sum = 0;
  for(int i=0;i<n;i++) {
    sensors_event_t accel, gyro;
    get_imu_reading(&accel, &gyro);
    sum += gyro.gyro.x;
  }

  float average = sum / n;
  Serial.print("Gyro calibration result: "); Serial.println(average, 4);
  calibration.gyro_offset = average;
  
	save_calibration();
}

void setupMotionInterrupt(void) 
{
  
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, 0x05);

  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x20);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x90);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x02);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);
  
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);
}
