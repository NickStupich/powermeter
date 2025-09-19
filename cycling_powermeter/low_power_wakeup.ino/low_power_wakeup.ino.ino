#include "LSM6DS3.h"
#include "Wire.h"
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include <bluefruit.h>

#if defined(CUSTOM_CS) && defined(CUSTOM_SPI)
  Adafruit_FlashTransport_SPI flashTransport(CUSTOM_CS, CUSTOM_SPI);

#elif defined(ARDUINO_ARCH_ESP32)
  // ESP32 use same flash device that store code.
  // Therefore there is no need to specify the SPI and SS
  Adafruit_FlashTransport_ESP32 flashTransport;

#else
  // On-board external flash (QSPI or SPI) macros should already
  // defined in your board variant if supported
  // - EXTERNAL_FLASH_USE_QSPI
  // - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
  #if defined(EXTERNAL_FLASH_USE_QSPI)
    Adafruit_FlashTransport_QSPI flashTransport;

  #elif defined(EXTERNAL_FLASH_USE_SPI)
    Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);

  #else
    #error No QSPI/SPI flash are defined on your board variant.h !
  #endif
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
  
  for(int i=0;i<100 & !Serial; i++) delay(10);

  pinMode(ledPin, OUTPUT); // use the LED as an output
  Wire.setClock(1000000);  // Set I2C to 1 MHz (Fast Mode Plus)
  Serial.println("Hello, I am awake!");
  myIMU.settings.gyroEnabled = 0; // Gyro currently not used, disabled to save power

  if (myIMU.begin() != 0) {
    Serial.println("IMU error");
  } else {
    Serial.println("IMU OK!");
  }

  pinMode(int1Pin, INPUT);
}

void loop() {
  setLED(false);
  Serial.print("Interrupt Counter: ");
  Serial.println(interruptCount);

  if (interruptCount > prevInterruptCount) {
    Serial.println("Interrupt received!");
  }
  prevInterruptCount = interruptCount;

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