
#define ACCEL_SLEEP_LIMIT_MGs (5)
// #define SLEEP_DELAY_SECONDS (10)
#define SLEEP_DELAY_SECONDS (5*60)

const int sleep_check_samples = 100; //check accelerations every ~1s
float sumAccel = 0;
float sumAccel2 = 0;
int numSamples = 0;

time_t last_time_above_threshold = millis();
bool is_time_to_sleep(sensors_event_t *accel) {

  float total_accel2 = accel->acceleration.x * accel->acceleration.x + accel->acceleration.y * accel->acceleration.y + accel->acceleration.z * accel->acceleration.z;

  sumAccel2 += total_accel2;
  sumAccel += sqrt(total_accel2);
  numSamples++;

  if(numSamples >= sleep_check_samples) {
    float variance = sumAccel2 / numSamples - (sumAccel*sumAccel) / (numSamples * numSamples);
    if (variance < 0) {variance = 0;}
    float stddev_mg = sqrt(variance) * 1000;
    // Serial.print("accel stddev (mg): "); Serial.println(stddev_mg);

    if(stddev_mg >= ACCEL_SLEEP_LIMIT_MGs) {
      last_time_above_threshold = millis();
    }
    else
    {
      if(millis() - last_time_above_threshold > SLEEP_DELAY_SECONDS*1000) {
            Serial.println("Sleepytime");
          return true;
        }
    }

    numSamples = 0;
    sumAccel = 0;
    sumAccel2 = 0;
  }

  //todo: look at accelerometer/gyro
  
  return false;
}

void powerDownFlash(Adafruit_SPIFlash& flash, Adafruit_FlashTransport& transport) {
  uint32_t id_before = flash.getJEDECID();

  transport.begin();
  transport.runCommand(0xB9);  // SPI deep power-down command
  delay(10);

  uint32_t id_after = flash.getJEDECID();

  // return (id_after == 0xFFFFFF || id_after == 0xFFFFFFFF);
}

void go_to_sleep(void)
{
  Serial.println("Going to System OFF"); 
  digitalWrite(LED_BUILTIN, HIGH);

  ble_power_down();
  
  power_off_torque_sensor();

  pinMode(WAKE_PIN, INPUT);
  setupMotionInterrupt();
  
  Serial.flush();
  delay(500);
  Serial.end();

  delay(1000); // delay seems important to apply settings, before going to System OFF
  // //Ensure interrupt pin from IMU is set to wake up device
  
  // fatfs.end();
  powerDownFlash(flash, flashTransport);
  flash.end();
  Wire.end();
  delay(200);

  systemOff(WAKE_PIN, 1);

}