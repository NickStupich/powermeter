#include "Adafruit_TinyUSB.h"
// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


const int wake_pin = 2;
time_t setup_time;

const int LOADCELL_SCK_PIN = 13; //TODO: set high for power down to save power
//todo: configure mpu6050 for interrupt

void setup() {
  Serial.begin(115200);

  while (!Serial) delay(10); // will pause Zero, Leonardo, etc until serial console opens



  Serial.println("Power management tester");

  pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  
  for(int i=0;i<5;i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100); 
  }

  digitalWrite(LED_BUILTIN, HIGH);
  setup_time = millis();
  
  pinMode(wake_pin, INPUT_PULLDOWN);
  // LowPower.attachInterruptWakeup(wake_pin, callback, RISING);
  
  Serial.println("setup done");
}


void loop() {
  if(millis() - setup_time > 20000) {
    
      Serial.println("\ngoing to sleep...");
      
      mpu.setInterruptPinPolarity(false);
      mpu.setInterruptPinLatch(true);

      mpu.setMotionDetectionThreshold(2); // Set your desired motion detection threshold
      mpu.setMotionDetectionDuration(5); // Set the duration for motion detection
      mpu.setMotionInterrupt(true); // Enable the interrupt on motion detection
      Serial.println("motion interrupt enabled");

    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    systemOff(wake_pin, 1);

  }
  else
  {
    //Serial.print(".");
    sensors_event_t a, g, temp;
    //mpu.getEvent(&a, &g, &temp);
    mpu.getGyroSensor()->getEvent(&g);
    Serial.println(g.gyro.x);
    //Serial.println(mpu.getMotionInterruptStatus());
  }

  delay(100);
}
