#include "PWMReader.hpp"

// Define an instance of the PWMReader class
int PWMreader_pin = 25;
PWMReader pwmReader;

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);

  // Initialize the PWMReader for pin defined on PWMreader_pin
  if (pwmReader.begin(PWMreader_pin)) {
    Serial.println("PWMReader initialized successfully!");
  } else {
    Serial.println("Failed to initialize PWMReader. Check the pin number.");
    while(1);
  }
}

void loop() {
  // Read and print PWM signal information
  unsigned long dutyCycle = pwmReader.getDutyCicle_us();
  unsigned long period = pwmReader.getPeriod_us();
  unsigned long lastRisingEdge = pwmReader.getTimeLastRisingEdge_us();

  Serial.print("Duty Cycle (us): ");
  Serial.println(dutyCycle);

  Serial.print("Period (us): ");
  Serial.println(period);

  Serial.print("Time of Last Rising Edge (us): ");
  Serial.println(lastRisingEdge);

  delay(1000); // Wait for 1 second before reading again
}
