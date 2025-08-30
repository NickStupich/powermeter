

time_t sleep_startup_time = millis();
bool is_time_to_sleep() {
  if(millis() - sleep_startup_time > 20*1000) {
    return true;
  }
  return false;
}

void go_to_sleep(void)
{
  Serial.println("go_to_sleep()");

  power_off_torque_sensor();
  setup_motion_detection_wakeup_pin();

  digitalWrite(LED_BUILTIN, LOW);
  Serial.end();
  delay(1000);
  
  Bluefruit.Advertising.stop();
  Bluefruit.disconnect(0);
  delay(100);
  
  sd_softdevice_disable();
  delay(50);


  __WFI();
  __SEV();
  __WFI();

  systemOff(WAKE_PIN, 1);

}