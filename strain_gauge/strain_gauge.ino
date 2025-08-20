#include "Adafruit_TinyUSB.h"
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 12;
const int LOADCELL_SCK_PIN = 13;

HX711 scale;

time_t start_time;
void setup() {
  Serial.begin(115200);
  
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Starting hx711 setup");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  start_time = millis();
  Serial.println("Done setup");
}


//zero value: 70,000
//10 pounds = 191,000

long tare = 70000;
float countToKg = (10/2.2) / 121000.0; //10 pound weight

int count = 0;
void loop() {

  if (scale.is_ready()) {
    count++;
    long reading = scale.read();
    float kg = (reading - tare) * countToKg;
    //Serial.print("HX711 reading: ");
    if(count % 20 == 0) {
      // Serial.println(reading);
      Serial.println(kg);
    }
    if(count % 100 == 0) {
      Serial.print("SPS: ");
      Serial.println(((int)(count *1000 / (millis() - start_time))));
    }
    delay(10);
  } else {
    // Serial.println("HX711 not found.");
  }

  
}
