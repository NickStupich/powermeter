#include "Adafruit_TinyUSB.h"
#include <bluefruit.h>
#include <SoftwareSerial.h>

BLEService        power_service = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic power_measure_char = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic power_feature_char = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic sensor_loc_char = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);

#define PIN_VBAT        (32)  // D32 battery voltage
#define PIN_VBAT_ENABLE (14)  // D14 LOW:read anable
#define PIN_HICHG       (22)  // D22 charge current setting LOW:100mA HIGH:50mA
#define PIN_CHG         (23)  // D23 charge indicatore LOW:charge HIGH:no charge

SoftwareSerial ser2(9,10);


BLEBas blebas;    // BAS (Battery Service) helper class instance


unsigned short flags = 0x20;
byte sensorlocation = 0x5; //left crank

double debug_revolutions = 0;
time_t debug_timestamp = 0;
time_t last_call_time = 0;
double debug_instantaneous_power;

const double cadence = 84;

time_t start_time;
void update_power() 
{  
  time_t current_time = millis();

  if(last_call_time == 0) {
    last_call_time = start_time;
  }

  time_t last_call_interval_ms = current_time - last_call_time;

  time_t elapsed_ms = (current_time - start_time);
  

  debug_instantaneous_power = (double)(100 + sin( 2 * PI * elapsed_ms / (1000*20)) * 10);
   double new_debug_revolutions = debug_revolutions + last_call_interval_ms / 1024.0 * cadence / 60.0;
  //Serial.println((unsigned long)last_call_interval_ms);

  if(((unsigned short)new_debug_revolutions) != (((unsigned short)debug_revolutions))) {
    debug_timestamp = current_time;
  }
  debug_revolutions = new_debug_revolutions;


  last_call_time = current_time;
}

void get_power(short *power, unsigned short *revolutions, unsigned short *timestamp) {



  *power = (short int) debug_instantaneous_power;
 
  
  *revolutions = (unsigned short)(debug_revolutions);
  *timestamp = (unsigned short)(((double)debug_timestamp) * 1024.0 / 1000.0);
  
  Serial.print(*revolutions);
  Serial.print("\t");
  Serial.println(*timestamp);

}

void setup()
{
  start_time = millis();

  Serial.begin(115200);
  // while ( !Serial ) delay(10);   // for nrf52840 with native usb
  for(int i=0;i<100 & !Serial; i++) delay(10);

  
  ser2.begin(115200);
  ser2.println("testtttt");

  
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

  Serial.println("Bluefruit52 HRM Example");
  Serial.println("-----------------------\n");

  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);


  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  Serial.println("Configuring the Power Meter Service");
  setupPM();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();

  Serial.println("\nAdvertising");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(power_service);

  Bluefruit.setName("NP_bletest");
  Bluefruit.Advertising.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void updatePowerMeasureChar(bool notify) {
  
  short power = 0;
  unsigned short revolutions = 0;
  unsigned short timestamp = 0;

  get_power(&power, &revolutions, &timestamp);
  
  unsigned char bleBuffer[8];
  bleBuffer[0] = flags & 0xff;
  bleBuffer[1] = (flags >> 8) & 0xff;
  bleBuffer[2] = power & 0xff;
  bleBuffer[3] = (power >> 8) & 0xff;
  bleBuffer[4] = revolutions & 0xff;
  bleBuffer[5] = (revolutions >> 8) & 0xff;
  bleBuffer[6] = timestamp & 0xff;
  bleBuffer[7] = (timestamp >> 8) & 0xff;

  if(notify) {  
    if ( power_measure_char.notify(bleBuffer, sizeof(bleBuffer)) ){
      //Serial.print("Power Measurement updated to: "); Serial.println(power); 
    }else{
      Serial.println("ERROR: Notify not set in the CCCD or not connected!");
    }
  }
  else
  {
    power_measure_char.write(bleBuffer, 8); //TODO
  }
}

void setupPM(void)
{
  power_service.begin();

  power_measure_char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  power_measure_char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  power_measure_char.setFixedLen(8);
  power_measure_char.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  power_measure_char.begin();

  
  updatePowerMeasureChar(false);


  power_feature_char.setProperties(CHR_PROPS_READ);
  power_feature_char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  power_feature_char.setFixedLen(4);
  power_feature_char.begin();
  unsigned char fBuffer[4] = {0x00, 0x00, 0x00, 0x08}; //Crank revolution data supported. nothing else
  power_feature_char.write(fBuffer, 4);


  
  sensor_loc_char.setProperties(CHR_PROPS_READ);
  sensor_loc_char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sensor_loc_char.setFixedLen(1);
  sensor_loc_char.begin();
  unsigned char slBuffer[1] = {sensorlocation};
  sensor_loc_char.write(slBuffer, 1);
}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value)
{
    Serial.print("CCCD Updated: ");
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == power_measure_char.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            Serial.println("Power Measurement 'Notify' enabled");
        } else {
            Serial.println("Power Measurement 'Notify' disabled");
        }
    }
}


void update_adv_loop() {
  static time_t last_call = 0;

  if(millis() - last_call >= 1000) {
    last_call = millis();
    // digitalToggle(LED_RED);
    
    if ( Bluefruit.connected() ) {
      updatePowerMeasureChar(true);
    }
  }
}

void update_power_calc_loop() {
  static time_t last_call = 0;

  if(millis() - last_call >= 2) {
    last_call = millis();

    update_power();
  }
}

void output_battery_loop() 
{
  static time_t last_call = 0;

  if(millis() - last_call >= 10000) {
    last_call = millis();

    int vbatt = analogRead(PIN_VBAT);
    Serial.print(vbatt, HEX);
    Serial.print("    ");
    Serial.print(2.961 * 3.6 * vbatt / 4096);   // Resistance ratio 2.961, Vref = 3.6V 

    ser2.println(2.961 * 3.6 * vbatt / 4096);
    //4.11-4.12 = full charge on 400mA?


    Serial.print("    ");
    Serial.println(digitalRead(PIN_CHG));       // 0:charge, 1:discharge 
  }
}

void loop()
{
  update_power_calc_loop();
  update_adv_loop();
  output_battery_loop();
  
}
