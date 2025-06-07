#include <AFMotor.h>   // Original Adafruit Motor Shield v1 library
#include <Servo.h>
#include <Arduino.h>

// ----------------- Sonar Sensor Setup -----------------
// Using pin 7 for trigger and pin 8 for echo.
const int sonarTrigPin = 7;
const int sonarEchoPin = 8;
const int sonarThreshold = 20; // in centimeters

int readSonarDistance() {
  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTrigPin, LOW);
  long duration = pulseIn(sonarEchoPin, HIGH, 30000); // timeout after 30ms
  int distance = duration * 0.034 / 2;
  return distance;
}

// ----------------- Motor and Servo Setup -----------------
// DC Motor for tube extension: connected to Motor Shield DC Slot 1.
AF_DCMotor tubeMotor(1, MOTOR12_64KHZ);
// DC Motor for receipt blocking: connected to Motor Shield DC Slot 2.
AF_DCMotor blockMotor(2, MOTOR12_64KHZ);
// Cutter Servo: located in the shield's servo slot 1, wired to digital pin 10.
Servo cutterServo;

// ----------------- State Machine Definition -----------------
enum State {
  WAITING,
  TEST
};
State currentState = WAITING;

void setup() {
  Serial.begin(9600);
  Serial.println("Test code starting (AFMotor + Servo + Sonar).");

  // Setup sonar sensor pins.
  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT);

  // Set motor speeds.
  tubeMotor.setSpeed(255);  // Maximum speed for tube extension
  blockMotor.setSpeed(150); // Adjust for proper blocking force
}

void loop() {
  if (currentState == WAITING) {
    int distance = readSonarDistance();
    Serial.print("Sonar Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    // If an object is detected within the threshold (and nonzero), start test.
    if (distance > 0 && distance <= sonarThreshold) {
      Serial.println("Detection triggered - starting test sequence.");
      currentState = TEST;
    }
    delay(100); // Short delay between readings
  }
  else if (currentState == TEST) {
    // --- Blocker DC Motor Test ---
    Serial.println("Activating block motor (DC Slot 2) for clamping...");
    blockMotor.run(FORWARD);
    delay(2000); // Run for 2 seconds to simulate clamping
    blockMotor.run(RELEASE);
    
    // --- Cutter Servo Test ---
    Serial.println("Activating cutter servo (digital pin 10)...");
    cutterServo.attach(10);  // Attach the servo when needed
    cutterServo.write(0);    // Start at 0°
    delay(1500);
    cutterServo.write(90);   // Move to 90° (cutting motion)
    delay(1500);
    cutterServo.detach();    // Detach to free Timer1, etc.
    delay(500);              // Allow a short settling time

    // --- Tube Motor Test ---
    Serial.println("Extending tube (DC Slot 1)...");
    tubeMotor.run(FORWARD);
    delay(4000); // Run forward for 4 seconds (extension)
    tubeMotor.run(RELEASE);
    delay(500);
    Serial.println("Retracting tube (DC Slot 1)...");
    tubeMotor.run(BACKWARD);
    delay(4000); // Run in reverse for 4 seconds (retraction)
    tubeMotor.run(RELEASE);
    delay(500);
    
    Serial.println("Test sequence complete. Returning to WAITING state.\n");
    currentState = WAITING;
  }
}
