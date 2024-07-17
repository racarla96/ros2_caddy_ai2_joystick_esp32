#include "PWMReader.hpp"

// Define instances of the PWMReader class for three channels
int PWMreader_pin1 = 25;
int PWMreader_pin2 = 33;
int PWMreader_pin3 = 32;

PWMReader pwmReader1;
PWMReader pwmReader2;
PWMReader pwmReader3;

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);

  // Initialize the PWMReaders for the specified pins
  if (pwmReader1.begin(PWMreader_pin1)) {
    Serial.println("PWMReader 1 initialized successfully!");
  } else {
    Serial.println("Failed to initialize PWMReader 1. Check the pin number.");
    while(1);
  }

  if (pwmReader2.begin(PWMreader_pin2)) {
    Serial.println("PWMReader 2 initialized successfully!");
  } else {
    Serial.println("Failed to initialize PWMReader 2. Check the pin number.");
    while(1);
  }

  if (pwmReader3.begin(PWMreader_pin3)) {
    Serial.println("PWMReader 3 initialized successfully!");
  } else {
    Serial.println("Failed to initialize PWMReader 3. Check the pin number.");
    while(1);
  }
}

void loop() {
  // Read and print PWM signal information for channel 1
  Serial.println("Channel 1:");
  printPWMInfo(pwmReader1);

  // Read and print PWM signal information for channel 2
  Serial.println("Channel 2:");
  printPWMInfo(pwmReader2);

  // Read and print PWM signal information for channel 3
  Serial.println("Channel 3:");
  printPWMInfo(pwmReader3);

  delay(1000); // Wait for 1 second before reading again
}

void printPWMInfo(PWMReader& pwmReader) {
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
}
