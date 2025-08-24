#include "Adafruit_TinyUSB.h"
#include "HX711.h"

HX711 scale;

void start_torque_sensor(void)
{
	
  //scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
	
	// pinMode(LOADCELL_DOUT_PIN, OUTPUT);
	// pinMode(LOADCELL_SCK_PIN, OUTPUT);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
	//delay(10);
	//Serial.println("Torque sensor setup complete");
	
}


//returns true if it has a reading, else false
bool get_crank_force(float *force_newtons)
{
	if(!scale.is_ready())
		return false;
  
	long reading = scale.read();
	long tare = 37000;
	Serial.print("torque raw: "); Serial.println(reading);
	float countToKg = (10/2.2) / 121000.0; //10 pound weight
  float kg = (reading - tare) * countToKg;
	*force_newtons = kg * 9.8;
	// Serial.print("Force: ");	Serial.println(*force_newtons);
	
	return true;
}