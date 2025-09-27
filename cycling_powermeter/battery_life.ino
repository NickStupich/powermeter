// #define OUTPUT_BATTERY_RAW_ON_SERIAL

#ifdef OUTPUT_BATTERY_RAW_ON_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial ser2(9,10);
#endif



    
void battery_life_init() {
  battery_monitor_charge_setup();

#ifdef OUTPUT_BATTERY_RAW_ON_SERIAL
  ser2.begin(115200);
#endif

}



void battery_monitor_charge_setup() {
  
  
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_VBAT_ENABLE, OUTPUT);
  pinMode(PIN_HICHG, OUTPUT);
  pinMode(PIN_CHG, INPUT);

  digitalWrite(PIN_VBAT_ENABLE, LOW); // VBAT read enable
  digitalWrite(PIN_HICHG, LOW);       // charge current 100mA
  
  // // initialise ADC wireing_analog_nRF52.c:73
  analogReference(AR_DEFAULT);        // default 0.6V*6=3.6V  wireing_analog_nRF52.c:73
  analogReadResolution(12);           // wireing_analog_nRF52.c:39
}

void battery_monitor_sleep() {
  
  digitalWrite(PIN_VBAT_ENABLE, HIGH); 
  digitalWrite(PIN_HICHG, HIGH);       

  pinMode(PIN_HICHG, INPUT);
  pinMode(PIN_CHG, OUTPUT);
  // pinMode(PIN_VBAT_ENABLE, INPUT);
  // pinMode(PIN_HICHG, INPUT);
}


int get_battery_percentage(uint32_t vbatt) {
  uint32_t scale = 2;
  uint32_t min_count = 1082 / scale;
  uint32_t max_count = 1560 / scale;

  uint32_t lookup_table[239] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,6,6,6,6,6,7,7,8,8,8,9,9,9,10,11,12,12,13,14,15,15,16,16,17,17,18,19,20,21,22,24,25,26,26,27,28,30,31,32,33,35,36,38,39,40,42,43,45,47,48,49,50,51,53,54,55,56,56,57,58,59,59,60,60,61,62,62,63,63,64,64,65,66,66,67,67,68,68,69,70,70,71,71,72,72,73,73,74,75,75,76,76,76,77,77,78,79,79,79,80,81,81,82,82,82,83,83,84,84,85,85,86,86,87,87,88,88,88,89,90,90,90,91,91,91,92,92,93,93,94,94,94,95,95,96,96,97,97,97,98,98 };

  uint32_t lookup_value = vbatt / scale;
  if(lookup_value <= min_count) {
          return 1;
  } else if (lookup_value >= max_count) {
          return 100;
  } else {
          return lookup_table[lookup_value - min_count];
  }
}


void output_battery_loop() 
{
  static time_t last_call = 0;

  if(millis() - last_call >= 60*1000) {
    last_call = millis();

    uint32_t vbatt = analogRead(PIN_VBAT);
    // Serial.print(vbatt, HEX);
    // Serial.print("    ");
    // Serial.print(2.961 * 3.6 * vbatt / 4096);   // Resistance ratio 2.961, Vref = 3.6V 

#ifdef OUTPUT_BATTERY_RAW_ON_SERIAL
    ser2.print(millis() / 1000);
    ser2.print("\t");
    ser2.print(vbatt);
    ser2.print("\t");
    ser2.println(2.961 * 3.6 * vbatt / 4096);
    //4.11-4.12 = full charge on 400mA?
#endif
    
    uint32_t percentage = get_battery_percentage(vbatt);
    blebas.write(percentage);

    // Serial.print("    ");
    // Serial.println(digitalRead(PIN_CHG));       // 0:charge, 1:discharge 
  }
}
