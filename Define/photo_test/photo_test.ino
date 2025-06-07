#include <AFMotor.h>

// Create a DC motor object on motor shield slot 1.
// MOTOR12_64KHZ specifies a higher PWM frequency.
//AF_DCMotor motor1(1, MOTOR12_64KHZ);

// Define the analog pin for the photoresistor.
const int photoResistorPin = A0;

// Set a threshold value for light detection.
// Adjust this value based on your wiring and ambient light conditions.
const int lightThreshold = 500;

void setup() {
  Serial.begin(9600);
  Serial.println("Photoresistor motor test starting...");
  
  // Set the motor speed (0 to 255).
  //motor1.setSpeed(200);
}

void loop() {
  // Read the photoresistor value from the analog pin.
  int sensorValue = analogRead(photoResistorPin);
  Serial.print("Photoresistor reading: ");
  Serial.println(sensorValue);
  
  // If the reading is higher than our threshold, there is enough light.
  // Run the motor; otherwise, stop it.
  if (sensorValue > lightThreshold) {
    Serial.println("Light detected. Motor spinning...");
    //motor1.run(FORWARD);
  }
  else {
    Serial.println("No light. Motor stopped.");
    //motor1.run(RELEASE);
  }
  
  delay(200); // Short delay for stability.
}
