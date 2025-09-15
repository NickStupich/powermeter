//----------------------------------------------------------------------------------------------
//Board Library : Seeed nRF52 Borads 1.1.1
//Board Select  : Seeed nRF52 Borads / Seeed XIAO nRF52840 Sense
//----------------------------------------------------------------------------------------------
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>   // for Serial.print()
#include <SoftwareSerial.h>

//Arduino15\packages\Seeeduino\hardware\nrf52\1.1.4\variants\Seeed_XIAO_nRF52840_Sense\variant.cpp
#define PIN_VBAT        (32)  // D32 battery voltage
#define PIN_VBAT_ENABLE (14)  // D14 LOW:read anable
#define PIN_HICHG       (22)  // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23)  // D23 charge indicatore LOW:charge HIGH:no charge

SoftwareSerial ser2(9,10);

void setup() 
{
  Serial.begin(115200);
  ser2.begin(115200);
  ser2.println("testtttt");
  // while(!Serial);

  //Serial.setPins(uint8_t pin_rx, uint8_t pin_tx) {
  
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);

  digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
  digitalWrite(PIN_HICHG, LOW);       // charge current 100mA
  
  // initialise ADC wireing_analog_nRF52.c:73
  analogReference(AR_DEFAULT);        // default 0.6V*6=3.6V  wireing_analog_nRF52.c:73
  analogReadResolution(12);           // wireing_analog_nRF52.c:39

  
  digitalWrite(LED_BUILTIN, LOW);
 
}

void loop() 
{
  int vbatt = analogRead(PIN_VBAT);
  Serial.print(vbatt, HEX);
  Serial.print("    ");
  Serial.print(2.961 * 3.6 * vbatt / 4096);   // Resistance ratio 2.961, Vref = 3.6V 

  ser2.println(2.961 * 3.6 * vbatt / 4096);
  //4.11-4.12 = full charge on 400mA?


  Serial.print("    ");
  Serial.println(digitalRead(PIN_CHG));       // 0:charge, 1:discharge 

  delay(10000);
}