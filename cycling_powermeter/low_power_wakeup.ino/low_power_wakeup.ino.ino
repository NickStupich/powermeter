#include "LSM6DS3.h"
#include "Wire.h"
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include <bluefruit.h>

  SPIFlash_Device_t const p25q16h{
    .total_size = (1UL << 21),  // 2MiB
    .start_up_time_us = 10000,
    .manufacturer_id = 0x85,
    .memory_type = 0x60,
    .capacity = 0x15,
    .max_clock_speed_mhz = 55,
    .quad_enable_bit_mask = 0x02,
    .has_sector_protection = 1,
    .supports_fast_read = 1,
    .supports_qspi = 1,
    .supports_qspi_writes = 1,
    .write_status_register_split = 1,
    .single_status_byte = 0,
    .is_fram = 0,
  };



#if 0

  #define SS_SPI1 25  // Defaul SS or CS for the Onboard QSPI Flash Chip

  SPIClass SPI_2(NRF_SPIM0, PIN_QSPI_IO1, PIN_QSPI_SCK, PIN_QSPI_IO0);  // Onboard QSPI Flash chip
  Adafruit_FlashTransport_SPI flashTransport(PIN_QSPI_CS, SPI_2);      // CS for QSPI Flash
#else

      Adafruit_FlashTransport_QSPI flashTransport;

  //     SPIClass SPI_2(NRF_SPIM0, PIN_QSPI_IO1, PIN_QSPI_SCK, PIN_QSPI_IO0);  // Onboard QSPI Flash chip
  // Adafruit_FlashTransport_SPI flashTransport(PIN_QSPI_CS, SPI_2);      // CS for QSPI Flash


#endif

Adafruit_SPIFlash flash(&flashTransport);


bool powerDownFlash(Adafruit_SPIFlash& flash, Adafruit_FlashTransport& transport) {
  uint32_t id_before = flash.getJEDECID();

  transport.begin();
  transport.runCommand(0xB9);  // SPI deep power-down command
  delay(10);

  uint32_t id_after = flash.getJEDECID();

  return (id_after == 0xFFFFFF || id_after == 0xFFFFFFFF);
}


LSM6DS3 myIMU(I2C_MODE, 0x6A); // IMU
#define int1Pin PIN_LSM6DS3TR_C_INT1

const int ledPin = LED_BUILTIN; // set ledPin to on-board LED

time_t start_time;


void setup() {
  start_time = millis();
  Serial.begin(115200);
  
  for(int i=0;i<500 & !Serial; i++) delay(10);

  pinMode(ledPin, OUTPUT); // use the LED as an output
  Wire.setClock(1000000);  // Set I2C to 1 MHz (Fast Mode Plus)
  Serial.println("Hello, I am awake!");
  myIMU.settings.gyroEnabled = 0; // Gyro currently not used, disabled to save power

  bool began = flash.begin(&p25q16h, 1);
  // bool began = flash.begin();
  
  Serial.print("Flash.begin(): "); Serial.println(began);

  if (!began) {
    Serial.println("Error, failed to initialize flash chip!");
    flashTransport.runCommand(0xAB);
    if (!flash.begin(&p25q16h, 1))
    // if (!flash.begin())
    {
      Serial.println("Flash.begin() failed twice");
      // while (1) {
      //   delay(1);
      // }
    }
    else
    {
      
      Serial.println("Second Flash.begin() worked");
    }
  }



  if (myIMU.begin() != 0) {
    Serial.println("IMU error");
  } else {
    Serial.println("IMU OK!");
  }

  pinMode(int1Pin, INPUT);
}

void loop() {
  setLED(false);
  // Serial.print("Interrupt Counter: ");
  // Serial.println(interruptCount);

  // if (interruptCount > prevInterruptCount) {
  //   Serial.println("Interrupt received!");
  // }
  // prevInterruptCount = interruptCount;

  // if (interruptCount >= 3) {
  if(millis() - start_time > 10000) {
    // Trigger System OFF after 5 interrupts
    goToPowerOff();
  }

  delay(500);
}

void goToPowerOff() {
  Serial.println("Going to System OFF");
  setLED(true);
  setupMotionInterrupt();
  delay(1000); // delay seems important to apply settings, before going to System OFF
  //Ensure interrupt pin from IMU is set to wake up device

  Serial.end();
  powerDownFlash(flash, flashTransport);
  flash.end();
  Wire.end();
  delay(200);

  systemOff(int1Pin, 1);
}

void setupMotionInterrupt() {
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x20);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x90);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x02);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);
  
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);
}

void setLED(bool on)
{
  // data = 1 -> LED = On
  // data = 0 -> LED = Off
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}