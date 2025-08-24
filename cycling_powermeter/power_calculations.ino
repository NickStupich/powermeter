

void reset_power_calculations()
{

}

void calculate_power(sensor_state_t sensors, power_state_t *power)
{
  power->power_watts = 2.0 * CRANK_LENGTH_MM * sensors.gyro.gyro.y * sensors.force_newtons / 1000.0;
  power->revolutions = 0;
  power->last_timestamp = 0;


  if(true) {  
    Serial.print(sensors.gyro.gyro.x); Serial.print("\t");
    Serial.print(sensors.gyro.gyro.y);Serial.print("\t");
    Serial.print(sensors.gyro.gyro.z);Serial.print("\t");
    Serial.print(sensors.force_newtons);Serial.print("\t");
    Serial.print(power->power_watts);Serial.print("\t");
    Serial.println();
  }
}