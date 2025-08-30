
unsigned long last_call_time_micros = 0;
void reset_power_calculations(power_state_t *power)
{
  last_call_time_micros = micros();
  power->power_watts_raw = 0;
  power->power_watts_smoothed = 0;
  power->revolutions_float = 0;
  power->revolutions_long = 0;
  power->last_timestamp = 0;
}

bool calculate_power(sensor_state_t sensors, power_state_t *power)
{
  bool has_new_smoothed_value = false;

  unsigned long current_call_time_micros = micros();

  unsigned long elapsed_micros = current_call_time_micros - last_call_time_micros;

  float gyro_rad_per_s = -(sensors.gyro.gyro.y - calibration.gyro_offset);

  power->power_watts_raw = 2.0 * CRANK_LENGTH_MM * gyro_rad_per_s * sensors.force_newtons / 1000.0;
  power->power_watts_buffer[power->power_buffer_index] = power->power_watts_raw;
  power->power_buffer_index++;

  if(power->power_buffer_index >= SENSOR_UPDATES_PER_GATT) {
    has_new_smoothed_value = true;
    float sum = 0;
    for(int i=0;i<SENSOR_UPDATES_PER_GATT;i++) {
      sum += power->power_watts_buffer[i];
    }
    power->power_watts_smoothed = sum / (float)SENSOR_UPDATES_PER_GATT;
    power->power_buffer_index = 0;
  }
  
  power->revolutions_float += gyro_rad_per_s * ((float)elapsed_micros) / (2*PI*1000000.0);
  // Serial.print("Rev: "); Serial.println(power->revolutions_float);

  long new_revolutions_long = (long)power->revolutions_float;
  if((new_revolutions_long+power->revolutions_long) % 2 == 1) {
    power->last_timestamp = millis(); //todo: interpolate this for better accuracy
  }
  power->revolutions_long = new_revolutions_long;

  last_call_time_micros = current_call_time_micros;

  if(false) {  
    // Serial.print(sensors.gyro.gyro.x); Serial.print("\t");
    // Serial.print(sensors.gyro.gyro.y);Serial.print("\t");
    // Serial.print(sensors.gyro.gyro.z);Serial.print("\t");
    // Serial.print(sensors.force_newtons);Serial.print("\t");
    Serial.print(power->revolutions_float);Serial.print("\t");
    Serial.print(power->revolutions_long);Serial.print("\t");
    Serial.print(power->last_timestamp);Serial.print("\t");
    Serial.print(power->power_watts_raw);Serial.print("\t");
    Serial.println();
  }
  
    // if(has_new_smoothed_value) {
    //   Serial.print("Smoothed:\t"); Serial.println(power->power_watts_smoothed);
    // }

  return has_new_smoothed_value;
}