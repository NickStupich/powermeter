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

void tare_torque_sensor(void) {
	long reading = scale.read_average(50);
	calibration.strain_gauge_offset = reading;
	Serial.print("Strain gauge offset calibrated to: "); Serial.println(reading);
	save_calibration();
}

void calibrate_torque_sensor(uint16_t current_grams) {
	long reading = scale.read_average(50);

	float current_newtons = (float)current_grams * 9.8 / 1000.0;
	calibration.strain_gauge_counts_to_newtons = (reading - calibration.strain_gauge_offset) / current_newtons;

	save_calibration();
}


//returns true if it has a reading, else false
bool get_crank_force(float *force_newtons)
{
	if(!scale.is_ready())
		return false;
  
	long reading = scale.read();
	float newtons = (reading - calibration.strain_gauge_offset) * calibration.strain_gauge_counts_to_newtons;
	*force_newtons = newtons;
	// Serial.print("Force: ");	Serial.println(*force_newtons);
	
	return true;
}