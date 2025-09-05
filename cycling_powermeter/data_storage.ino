
#include <Adafruit_TinyUSB.h> // for Serial

// using namespace Adafruit_LittleFS_Namespace;

#define CALIBRATION_FILENAME "/calibration.txt"
#define CAL_FILE_VERSION 1

void data_storage_init() {
  InternalFS.begin();
}

void load_calibration() {
  Adafruit_LittleFS_Namespace::File calibration_file(InternalFS);
  calibration_file.open(CALIBRATION_FILENAME, Adafruit_LittleFS_Namespace::FILE_O_READ);
  if(calibration_file) {
    Serial.println("Calibration file exists");

    // uint32_t readlen;
    // char buffer[256] = { 0 };
    // readlen = calibration_file.read(buffer, sizeof(buffer));

    uint8_t version = calibration_file.read();

    if(version != CAL_FILE_VERSION) {
      Serial.println("Wrong calibration file version");
      
    }
    else
    {
      calibration_file.read((uint8_t*)&calibration.gyro_offset, 4);
      Serial.print("Gyro offset: "); Serial.println(calibration.gyro_offset, 4);

      calibration_file.read((uint8_t*)&calibration.strain_gauge_offset, 4);
      Serial.print("Strain gauge offset: "); Serial.println(calibration.strain_gauge_offset);
      
      calibration_file.read((uint8_t*)&calibration.strain_gauge_counts_to_newtons, 4);
      Serial.print("Strain gauge scale: "); Serial.println(calibration.strain_gauge_counts_to_newtons, 8);
    }
      
  }
  else
  {
    Serial.println("Calibration file does NOT exist");
  }
  calibration_file.close();
}

void save_calibration() {
  Adafruit_LittleFS_Namespace::File calibration_file(InternalFS);
  calibration_file.open(CALIBRATION_FILENAME, Adafruit_LittleFS_Namespace::FILE_O_WRITE);
  
  if(!calibration_file) {
    Serial.println("Failed to open cal file for writing");
    return;
  }

  calibration_file.seek(0);
  calibration_file.write(CAL_FILE_VERSION);
  calibration_file.write((uint8_t*)&calibration.gyro_offset, 4);
  calibration_file.write((uint8_t*)&calibration.strain_gauge_offset, 4);
  calibration_file.write((uint8_t*)&calibration.strain_gauge_counts_to_newtons, 4);

  calibration_file.close();
  Serial.println("Done writing calibration file");

  
}