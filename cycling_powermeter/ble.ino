
BLEService        power_service = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic power_measure_char = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic power_feature_char = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic sensor_loc_char = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);

BLECharacteristic calibration_char = BLECharacteristic(0x1000);

//TODO: useful?
BLEBas blebas;    // BAS (Battery Service) helper class instance


unsigned short flags = 0x20;
byte sensorlocation = 0x5; //left crank

double debug_revolutions = 0;
time_t debug_timestamp = 0;
time_t last_call_time = 0;
double debug_instantaneous_power; 

const double cadence = 84;

void update_power_and_cadence(float _power, long _revolutions, long _timestamp)
{
	
    short power = (short) fabs(_power);
    unsigned short revolutions = (unsigned short) _revolutions;
    unsigned short timestamp = (unsigned short) _timestamp;

    // Serial.print("Rev:\t"); Serial.print(revolutions); Serial.print("\tTimestamp:\t"); Serial.println(timestamp);

    if ( Bluefruit.connected() ) {
      updatePowerMeasureChar(power, revolutions, timestamp, true);
    }
}


void start_ble_advertising()
{

  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  // Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();

  Bluefruit.setTxPower(8);

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


void stop_advertising()
{
  //TODO?
}


/* PRIVATE STUFF BELOW  */

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addService(power_service);

  Bluefruit.setName("Nickpower");
  Bluefruit.Advertising.addName();
  
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void updatePowerMeasureChar(
  short power,
  unsigned short revolutions,
  unsigned short timestamp,
bool notify) {
    
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
    long before = millis();
    if ( power_measure_char.notify(bleBuffer, sizeof(bleBuffer)) ){
      // Serial.println(millis() - before);
      // Serial.print("Power Measurement updated to: "); Serial.println(power); 
    }else{
      // Serial.println("ERROR: Notify not set in the CCCD or not connected!");
    }
  }
  else
  {
    power_measure_char.write(bleBuffer, sizeof(bleBuffer)); //TODO
  }
}

void setupPM(void)
{
  power_service.begin();
  
  Serial.println("power service began");

  power_measure_char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  power_measure_char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  power_measure_char.setFixedLen(8);
  power_measure_char.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates //TODO??
  power_measure_char.begin();
  Serial.println("power measure char began");

  
  updatePowerMeasureChar(0,0,0,false);


  power_feature_char.setProperties(CHR_PROPS_READ);
  power_feature_char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  power_feature_char.setFixedLen(4);
  power_feature_char.begin();
  unsigned char fBuffer[4] = {0x00, 0x00, 0x00, 0x08}; //Crank revolution data supported. nothing else
  power_feature_char.write(fBuffer, 4);
  Serial.println("power feature char began");


  
  sensor_loc_char.setProperties(CHR_PROPS_READ);
  sensor_loc_char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  sensor_loc_char.setFixedLen(1);
  sensor_loc_char.begin();
  unsigned char slBuffer[1] = {sensorlocation};
  sensor_loc_char.write(slBuffer, 1);
  Serial.println("sensor loc char began");

  calibration_char.setProperties(CHR_PROPS_WRITE);
  calibration_char.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  calibration_char.setWriteCallback(calibration_write_callback);
  calibration_char.setMaxLen(20);
  calibration_char.begin();
  Serial.println("calibration char began");
}

void calibration_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  uint16_t weight_grams;
  uint8_t recording_state;

  Serial.println("calibration_write_callback()");
  if(len == 0) {
    Serial.println("Length is 0");
    return;
  }

  switch(data[0]) {
    case 0:
      Serial.println("Zero command");
      tare_torque_sensor();
      zero_imu();
      break;

    case 1:
      Serial.println("Weight calibration command");
      if(len < 3) {
        Serial.println("Data too short");
        return;
      }
      weight_grams = ((uint16_t)data[1]) << 8 | data[2];
      Serial.print("Calibration weight: ");
      Serial.println(weight_grams);
      calibrate_torque_sensor(weight_grams);
      break;

    case 2:
      Serial.println("Start recording command");
      recording_state = data[1];
      data_recorder_start_recording(recording_state);
      break;
      

    default:
      Serial.print("Unrecognized command: ");
      Serial.println(data[0]);
  }

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

