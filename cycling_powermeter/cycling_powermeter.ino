#include <bluefruit.h>
#include "Adafruit_TinyUSB.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "HX711.h"


const int LOADCELL_DOUT_PIN = 9;
const int LOADCELL_SCK_PIN = 10;
const int WAKE_PIN = 2;

const float CRANK_LENGTH_MM = 170;

struct sensor_state_t {
  float force_newtons;
  sensors_event_t accel, gyro;
};

struct power_state_t {
  float power_watts = 0;
  long revolutions = 0;
  long last_timestamp = 0;
};

void setup() {
  Serial.begin(115200);

  for(int i=0;i<100 && !Serial;i++)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens. but max 1 second if there's no serial link
  
  //start in active mode
  start_ble_advertising();
  
  reset_power_calculations();
  start_imu();
  start_torque_sensor();
  

  Serial.println("Done setup");
}

void loop() {

  sensor_state_t sensors;
  power_state_t power;

  //TODO: update imu/power/ble more often than crank force?
  if(get_crank_force(&sensors.force_newtons)) {
    get_imu_reading(&sensors.accel, &sensors.gyro);

    calculate_power(sensors, &power);

    if(is_time_to_sleep()) {
      go_to_sleep();
    }

    update_power_and_cadence(power.power_watts, power.revolutions, power.last_timestamp);

  }
  else {
    //Serial.println("No force");
  }

  delay(10);
  


}


