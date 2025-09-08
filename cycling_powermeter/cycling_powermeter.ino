#include <bluefruit.h>
#include "Adafruit_TinyUSB.h"
#include "Adafruit_MPU6050.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>


#include "HX711.h"


using namespace Adafruit_LittleFS_Namespace;

void data_storage_init(); //TODO: why is this needed??

const int LOADCELL_DOUT_PIN = 9;
const int LOADCELL_SCK_PIN = 10;
const int WAKE_PIN = 2;

const int SENSORS_SAMPLES_PER_SEC = 83; //found experimentally. datasheet says 80, weird
const int SENSORS_BUFFER_SIZE = 166;//two seconds of data
const int GATT_UPDATES_PER_SEC = 1;
const int SENSOR_UPDATES_PER_GATT = (SENSORS_SAMPLES_PER_SEC / GATT_UPDATES_PER_SEC);

const float CRANK_LENGTH_MM = 170;

struct sensor_state_t {
  float force_newtons;
  sensors_event_t accel, gyro;
};

struct power_state_t {
  float power_watts_raw = 0;
  float power_watts_buffer[SENSOR_UPDATES_PER_GATT] = {0};
  int power_buffer_index = 0;
  float power_watts_smoothed = 0;
  float revolutions_float = 0;
  long revolutions_long = 0;
  long last_timestamp = 0;
};

struct calibration_settings_t {
  float gyro_offset = 0;
  float strain_gauge_offset = 37000;
  float strain_gauge_counts_to_newtons = 0.000368144;
};


sensor_state_t sensors;
power_state_t power;
calibration_settings_t calibration;
time_t setup_complete_time;

void setup() {
  delay(500);

  Serial.begin(115200);

  for(int i=0;i<100 && !Serial;i++)
  // while(!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens. but max 1 second if there's no serial link
  
  Serial.println("Nick's Powermeter!");

  data_storage_init();
  data_recorder_init();
  
  load_calibration();

  //start in active mode
  start_ble_advertising();
  
  reset_power_calculations(&power);
  start_imu();
  start_torque_sensor();
  
  setup_complete_time = millis();
  Serial.println("Done setup");
  
}

long loop_count = 0;
time_t last_loop_time = millis();
void loop() {

  
  if(Serial.available()) {
    String command = Serial.readString();
    command = command.substring(0, command.length()-1);
    Serial.print("Read input: "); Serial.println(command);
    if(command.equals("read_raw")) {
      data_recorder_read_all_raw();
    }
    else if(command.equals("read_smooth")) {
      data_recorder_read_all_smooth();
    }
    else if(command.equals("read_accel")) {
      data_recorder_read_all_accel();
    }
    else {
      Serial.println("Unrecognized command");
    }
  }
  

  if(get_crank_force(&sensors.force_newtons)) {
    
    loop_count++;
    get_imu_reading(&sensors.accel, &sensors.gyro);


    if(is_time_to_sleep()) {
      go_to_sleep();
    }


    if(calculate_power(sensors, &power)) {
      update_power_and_cadence(power.power_watts_smoothed, power.revolutions_long, power.last_timestamp);
      
      data_recorder_add_smoothed_sample(power.power_watts_smoothed, power.revolutions_float);
    }

    data_recorder_add_raw_sample(sensors.force_newtons, -sensors.gyro.gyro.y, power.power_watts_raw);
    data_recorder_add_accel_sample(sensors.accel.acceleration.x,sensors.accel.acceleration.y,sensors.accel.acceleration.z,
        sensors.gyro.gyro.x, sensors.gyro.gyro.y, sensors.gyro.gyro.z);
    
    if(loop_count % 1000 == 0) {
      float sps = 1000.0 * (float)loop_count / (millis() - setup_complete_time);
      Serial.print("SPS:\t"); Serial.println(sps);
    }
  }
  else {
    //Serial.println("No force");
  }

  // delay(2);
  

}


