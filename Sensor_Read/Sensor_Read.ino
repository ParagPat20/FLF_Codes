#include <HardwareSerial.h>

// HardwareSerial instance on PA9 (TX) / PA10 (RX)
HardwareSerial Serial1(USART1);

// Sensor pins
const uint8_t sensorPins[8] = {PA4, PA5, PA6, PA0, PA1, PA2, PA3};

void setup() {
  // Set ADC resolution to 8 bits (0–255)
  analogReadResolution(12);  // <-- Resolution control

  // Sensor inputs
  for (auto p : sensorPins) pinMode(p, INPUT);


  // Start serial
  Serial1.begin(115200);
}

void loop() {
  // Read all sensors and print tab-separated
  for (uint8_t i = 0; i < 7; i++) {
    int val = analogRead(sensorPins[i]);
    Serial1.print(val);
    if (i < 6) Serial1.print('\t'); // tab separator
  }
  Serial1.println(); // new line after all 8

  delay(40); // ~20 readings per second
}
