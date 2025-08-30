// #include "Adafruit_SPIFlash.h"


// Adafruit_FlashTransport_QSPI flashTransport;

// void QSPIF_sleep(void)
// {
//   flashTransport.begin();
//   flashTransport.runCommand(0xB9);
//   flashTransport.end();  
// }

const int wake_pin = 2;
time_t setup_time;

void setup() {
  setup_time = millis();

  pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

int count = 0;
void loop() {
  count++;
  if(millis() - setup_time > 10000) {
    digitalWrite(LED_BUILTIN, 0);
    // QSPIF_sleep();
    systemOff(wake_pin, 1);
  // put your main code here, to run repeatedly:
    }
    else
    {
    digitalWrite(LED_BUILTIN, count % 2);

    }
    delay(200);
}
