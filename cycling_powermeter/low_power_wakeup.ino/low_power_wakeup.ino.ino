#include "LSM6DS3.h"
#include "Wire.h"
// #include <Adafruit_TinyUSB_API.h>
// #include <Arduino.h>

#include "SdFat.h"
#include "Adafruit_SPIFlash.h"

// Uncomment to run example with custom SPI and SS e.g with FRAM breakout
// #define CUSTOM_CS   A5
// #define CUSTOM_SPI  SPI

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

#include <bluefruit.h>

bool deepPowerDown(Adafruit_SPIFlash& flash, Adafruit_FlashTransport& transport) {
  uint32_t id_before = flash.getJEDECID();

  transport.begin();
  transport.runCommand(0xB9);  // SPI deep power-down command
  delay(10);

  uint32_t id_after = flash.getJEDECID();

  return (id_after == 0xFFFFFF || id_after == 0xFFFFFFFF);
}


LSM6DS3 myIMU(I2C_MODE, 0x6A); // IMU
#define int1Pin PIN_LSM6DS3TR_C_INT1
// #define int1Pin P0_11 // <--- GPIO pin number 

// #define int1Pin (NRF_GPIO_PIN_MAP(0, 11))

const int ledPin = LED_BUILTIN; // set ledPin to on-board LED

uint8_t interruptCount = 0; // Amount of received interrupts
uint8_t prevInterruptCount = 0; // Interrupt Counter from last loop

time_t start_time;


void setup() {
  start_time = millis();
  Serial.begin(115200);
  
  // while ( !Serial ) delay(10);   // for nrf52840 with native usb
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

  setupDoubleTapInterrupt();

  pinMode(int1Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(int1Pin), int1ISR, RISING);
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
  setupDoubleTapInterrupt();
  delay(1000); // delay seems important to apply settings, before going to System OFF
  //Ensure interrupt pin from IMU is set to wake up device

  Serial.end();

      if (!deepPowerDown(flash, flashTransport)) {
      // pinMode(LED_BUILTIN, OUTPUT);
      // digitalWrite(LED_BUILTIN, LOW);
      while (1) {
        yield();
      }
    }

    flash.end();
  Wire.end();
  delay(200);

    systemOff(int1Pin, 1);

  // nrf_gpio_cfg_sense_input(digitalPinToInterrupt(int1Pin), NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
  // delay(2000);// Trigger System OFF
  // NRF_POWER->SYSTEMOFF = 1;
}

void setupDoubleTapInterrupt() {
  uint8_t error = 0;
  uint8_t dataToWrite = 0;

  // Double Tap Config
  
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60); //* Acc = 416Hz (High-Performance mode)// Turn on the accelerometer
  // // ODR_XL = 416 Hz, FS_XL = 2g
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);// INTERRUPTS_ENABLE, SLOPE_FDS// Enable interrupts and tap detection on X, Y, Z-axis
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x85);// Set tap threshold 8C
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);// Set Duration, Quiet and Shock time windows 7F
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);// Single & double-tap enabled (SINGLE_DOUBLE_TAP = 1)
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x08);// Double-tap interrupt driven to INT1 pin
  
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60); //* Acc = 416Hz (High-Performance mode)// Turn on the accelerometer
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x81);
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x01);//enable significant motion interrupt
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x40);
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);// Double-tap interrupt driven to INT1 pin
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, 0x01);// motion threshold

  
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80);
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_SM_STEP_THS, 0x01);
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00);
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x20);
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x05);
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL, 0x40);
  
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x20);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x90);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x02);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20);

  
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL6_G, 0x10);


}

void int1ISR() {
  interruptCount++;
}

  void setLED(bool on)
  {
  // data = 1 -> LED = On
  // data = 0 -> LED = Off
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
  }