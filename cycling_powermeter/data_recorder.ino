#undef ENABLE_DATA_RECORDER


#ifdef ENABLE_DATA_RECORDER

#define RAW_FILENAME "/raw.bin"
File raw_file(InternalFS);


struct raw_data_t {
  time_t ms;
  float torque;
  float ax; 
  float ay; 
  float az; 
  float gx; 
  float gy;
  float gz;
  float power;
};

struct ble_power_data_t {
  time_t ms;

};

void data_recorder_init() {
  InternalFS.begin();
  if(!raw_file.open(RAW_FILENAME, FILE_O_WRITE)) {
    Serial.println("Failed to open raw file");
  }
  raw_file.seek(0);

  Serial.println("Data recorder initialized");
}
size_t total_written = 0;
void data_recorder_add_raw_sample(float torque, float ax, float ay, float az, float gx, float gy, float gz, float power) {
  //record: milliseconds, torque, full accel, full gyro, instantaneous power calculation
  time_t ms = millis();

  raw_data_t sample = {ms, torque, ax, ay, az, gx, gy, gz, power};

//   size_t written = raw_file.write((const char*)&sample, sizeof(sample));
//   total_written+=written;
//   // Serial.print("Wrote "); Serial.print(written); Serial.println(" bytes");
//  Serial.print("Wrote "); Serial.print(total_written); Serial.println(" bytes");

  //also maybe: smoothed power, revolutions, timestamp
}

void data_recorder_add_ble_output(float power, float revolutions) {

}

void data_recorder_read_all_raw() {
  raw_file.close();
  // raw_file.openNextFile(FILE_O_READ);
  Serial.println("Closed raw_file");

  File raw_read(InternalFS);
  if(!raw_read.open(RAW_FILENAME, FILE_O_READ)) {
    Serial.println("Failed to open raw file for read");
    return;
  }
  raw_read.seek(0);
  Serial.println("Opened raw_file for read");

  Serial.println("Index,Time,Force,Ax,Ay,Az,Gx,Gy,Gz,Power");

  raw_data_t sample;
  int i=0;
  while(raw_read.available()) {
    raw_read.read((char*)&sample, sizeof(sample));
    
    Serial.print(i); Serial.print(",");
    Serial.print((long)sample.ms); Serial.print(",");
    Serial.print(sample.torque); Serial.print(",");
    Serial.print(sample.ax); Serial.print(",");
    Serial.print(sample.ay); Serial.print(",");
    Serial.print(sample.az); Serial.print(",");
    Serial.print(sample.gx); Serial.print(",");
    Serial.print(sample.gy); Serial.print(",");
    Serial.print(sample.gz); Serial.print(",");
    Serial.print(sample.power); Serial.print(",");

    Serial.println("");
    i++;
  } 
  
  Serial.println("Closed raw_file");
  raw_read.close();

  raw_file.open(RAW_FILENAME, FILE_O_WRITE);

}

void data_recorder_read_all_ble() {


}

void data_recorder_delete_all_raw() {
  raw_file.close();
  InternalFS.remove(RAW_FILENAME);
  raw_file.open(RAW_FILENAME, FILE_O_WRITE);
  raw_file.seek(0);
}

void data_recorder_delete_all_ble() {


}

#else

void data_recorder_init() {}
void data_recorder_add_raw_sample(float torque, float ax, float ay, float az, float gx, float gy, float gz, float power) {}
void data_recorder_add_ble_output(float power, float revolutions) {}
void data_recorder_read_all_raw() {}
void data_recorder_read_all_ble() {}
void data_recorder_delete_all_raw() {}
void data_recorder_delete_all_ble() {}

#endif