#include "SdFat_Adafruit_Fork.h"
#include <SPI.h>

#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;

#define RAW_FILENAME "/raw.bin"
#define SMOOTH_FILENAME "/smooth.bin"
#define ACCEL_FILENAME "/accel.bin"

#define RECORD_RAW 0x1
#define RECORD_SMOOTH 0x2
#define RECORD_ACCEL 0x4

uint8_t recording_state = 0;
File32 raw_file, smooth_file, accel_file;

struct raw_data_t {
  uint32_t ms;
  float torque;
  float gyro;
  float power;
};

struct smooth_data_t {
  uint32_t ms;
  float power;
  float revolutions_float;
};

struct accel_data_t {
  uint32_t ms;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

void data_recorder_init() {
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while (1) {
      delay(1);
    }
  }
  Serial.print("Flash chip JEDEC ID: 0x");
  Serial.println(flash.getJEDECID(), HEX);

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    Serial.println(
        "Was the flash chip formatted with the fatfs_format example?");
    while (1) {
      delay(1);
    }
  }
  Serial.println("Mounted filesystem!");


  Serial.println("Data recorder initialized");
}

time_t recording_start_time  = 0;
void data_recorder_start_recording(uint8_t recording_types) {
  if(recording_types == 0) {
    end_recording();
    return;
  }

  recording_state = recording_types;
  recording_start_time = millis();

  Serial.print("Start recording(): "); Serial.println(recording_types);

  fatfs.remove(RAW_FILENAME);
  fatfs.remove(SMOOTH_FILENAME);
  fatfs.remove(ACCEL_FILENAME);
  
   raw_file = fatfs.open(RAW_FILENAME, FILE_WRITE);
   raw_file.seek(0);

   smooth_file = fatfs.open(SMOOTH_FILENAME, FILE_WRITE);
   smooth_file.seek(0);
   
   accel_file = fatfs.open(ACCEL_FILENAME, FILE_WRITE);
   accel_file.seek(0);

   
  Serial.println("Opened files for recording");

}

size_t total_written = 0;
void data_recorder_add_raw_sample(float torque, float gyro, float power) {
  if(recording_state & RECORD_RAW) {
    uint32_t ms = (uint32_t)( millis() - recording_start_time);
    raw_data_t sample = {ms, torque, gyro, power};

    size_t written = raw_file.write((const char*)&sample, sizeof(sample));
    total_written+=written;

    if(written == 0) {
      Serial.print("Flash stopped writing after: ");
      Serial.print(total_written);
      Serial.println("Bytes");
      end_recording();
    }

  }
}

void end_recording() {

  Serial.println("Recording ended");
  if(raw_file){
    raw_file.flush();
    raw_file.close();
  }

  if(smooth_file) {
    smooth_file.flush();
    smooth_file.close();
  }

  if(accel_file) {
    accel_file.flush();
    accel_file.close();
  }

  recording_state = 0;

}

void data_recorder_add_smoothed_sample(float power, float revolutions) {
  if(recording_state & RECORD_SMOOTH) {
    uint32_t ms = (uint32_t)( millis() - recording_start_time);

    smooth_data_t sample = {ms, power, revolutions};

    size_t written = smooth_file.write((const char*)&sample, sizeof(sample));

    if(written == 0) {
      end_recording();
    }
  }
}


void data_recorder_add_accel_sample(float ax, float ay, float az, float gx, float gy, float gz) {
  if(recording_state & RECORD_ACCEL) {
    uint32_t ms = (uint32_t)( millis() - recording_start_time);

    accel_data_t sample = {ms, ax, ay, az, gx, gy, gz};

    size_t written = accel_file.write((const char*)&sample, sizeof(sample));

    if(written == 0) {
      end_recording();
    }
  }
}

void data_recorder_read_all_raw() {
  File32 raw_read = fatfs.open(RAW_FILENAME, FILE_READ);
  raw_read.seek(0);
  Serial.println("Opened raw_file for read");

  Serial.println("Index,Time,Force,Gyro,Power");

  raw_data_t sample;
  int i=0;
  while(raw_read.available()) {
    raw_read.read((char*)&sample, sizeof(sample));
    
    Serial.print(i); Serial.print(",");
    Serial.print((long)sample.ms); Serial.print(",");
    Serial.print(sample.torque); Serial.print(",");
    Serial.print(sample.gyro); Serial.print(",");
    Serial.print(sample.power); Serial.print(",");

    Serial.println("");
    i++;
  } 

}

void data_recorder_read_all_smooth() {
  File32 smooth_read = fatfs.open(SMOOTH_FILENAME, FILE_READ);
  smooth_read.seek(0);
  Serial.println("Opened smooth_file for read");

  Serial.println("Index,Time,Power,Revolutions");

  smooth_data_t sample;
  int i=0;
  while(smooth_read.available()) {
    smooth_read.read((char*)&sample, sizeof(sample));
    
    Serial.print(i); Serial.print(",");
    Serial.print((long)sample.ms); Serial.print(",");
    Serial.print(sample.power); Serial.print(",");
    Serial.print(sample.revolutions_float); Serial.print(",");


    Serial.println("");
    i++;
  } 
}


void data_recorder_read_all_accel() {
  File32 accel_read = fatfs.open(ACCEL_FILENAME, FILE_READ);
  accel_read.seek(0);
  Serial.println("Opened accel_file for read");

  Serial.println("Index,Time,Ax,Ay,Az,Gx,Gy,Gz");

  accel_data_t sample;
  int i=0;
  while(accel_read.available()) {
    accel_read.read((char*)&sample, sizeof(sample));
    
    Serial.print(i); Serial.print(",");
    Serial.print((long)sample.ms); Serial.print(",");
    Serial.print(sample.ax); Serial.print(",");
    Serial.print(sample.ay); Serial.print(",");
    Serial.print(sample.az); Serial.print(",");
    Serial.print(sample.gx); Serial.print(",");
    Serial.print(sample.gy); Serial.print(",");
    Serial.print(sample.gz); Serial.print(",");

    Serial.println("");
    i++;
  } 
}
