/**************************************************************************
   Integrated Project with Thermal Printer, Sonar, and Motors
   Using AFMotor Library (Adafruit Motor Shield v1)

   Hardware Configuration:
     - Thermal Printer via SoftwareSerial on pins 5 & 6.
     - Sonar Sensor (HC-SR04) on pins 7 (Trig) and 8 (Echo).
     - DC Motor for Tube Extension in DC Slot 1.
     - DC Motor for Receipt Blocking in DC Slot 2.
     - Cutter Servo in the shield's servo slot 1 (wired to digital pin 10).
**************************************************************************/

#include <AFMotor.h>         // For Motor Shield v1
#include <Servo.h>
#include <SoftwareSerial.h>
#include "Adafruit_Thermal.h"
#include "digit_0.h"
#include "digit_1.h"
#include "digit_2.h"
#include "digit_3.h"
#include "digit_4.h"
#include "digit_5.h"
#include "digit_6.h"
#include "digit_7.h"
#include "digit_8.h"
#include "digit_9.h"

// ----------------- Printer Setup -----------------
#define TX_PIN 6   // Arduino TX -> Printer RX
#define RX_PIN 5   // Arduino RX <- Printer TX
SoftwareSerial mySerial(RX_PIN, TX_PIN);
Adafruit_Thermal printer(&mySerial);

// ----------------- Digit Bitmap Setup -----------------
#define DIGIT_WIDTH 32
#define DIGIT_HEIGHT 32
const uint8_t* digit_bitmaps[] = {
  digit_0_data, digit_1_data, digit_2_data,
  digit_3_data, digit_4_data, digit_5_data,
  digit_6_data, digit_7_data, digit_8_data, digit_9_data
};

void copyDigitToBuffer(uint8_t* dest, const uint8_t* src, int digitIndex, int digitCount) {
  int byteHeight = (DIGIT_HEIGHT + 7) / 8;
  for (int x = 0; x < DIGIT_WIDTH; x++) {
    for (int b = 0; b < byteHeight; b++) {
      int destIndex = (digitIndex * DIGIT_WIDTH + x) + (b * DIGIT_WIDTH * digitCount);
      int srcIndex = x + (b * DIGIT_WIDTH);
      dest[destIndex] = pgm_read_byte(&src[srcIndex]);
    }
  }
}

void printNumberHorizontal(int number) {
  String str = String(number);
  int digitCount = str.length();
  int byteHeight = (DIGIT_HEIGHT + 7) / 8;
  int bufferSize = DIGIT_WIDTH * digitCount * byteHeight;
  uint8_t buffer[bufferSize];
  memset(buffer, 0x00, bufferSize);
  for (int i = 0; i < digitCount; i++) {
    int digit = str[i] - '0';
    copyDigitToBuffer(buffer, digit_bitmaps[digit], i, digitCount);
  }
  printer.printBitmap(DIGIT_WIDTH * digitCount, DIGIT_HEIGHT, buffer);
  printer.feed(2);
}

// ----------------- Sonar Sensor Setup -----------------
// Use pins 7 (Trig) and 8 (Echo)
const int sonarTrigPin = 7;
const int sonarEchoPin = 8;
const int sonarThreshold = 20; // centimeters

int readSonarDistance() {
  
  pinMode(sonarTrigPin, OUTPUT);
  digitalWrite(sonarTrigPin, LOW);
  pinMode(sonarEchoPin, INPUT);
  
  digitalWrite(sonarTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sonarTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sonarTrigPin, LOW);
  long duration = pulseIn(sonarEchoPin, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  return distance;
}

// ----------------- Re-enable the Sonar -----------------
void enableSonar() {
  pinMode(sonarTrigPin, OUTPUT);
  digitalWrite(sonarTrigPin, LOW);
  pinMode(sonarEchoPin, INPUT);
}

// ----------------- Paper Feed Function -----------------
void feedPaperForLength(int targetFeedCount) {
  for (int i = 0; i < targetFeedCount; i++) {
    printer.feed(1);
    delay(100);
  }
}

// ----------------- Motor and Servo Setup -----------------
// DC Motor for tube extension: connected to Motor Shield DC Slot 1.
AF_DCMotor tubeMotor(1, MOTOR12_64KHZ);
// DC Motor for receipt blocking: connected to Motor Shield DC Slot 2.
AF_DCMotor blockMotor(2, MOTOR12_64KHZ);
// Cutter Servo in shield's servo slot 1 (wired to digital pin 10).
Servo cutterServo;

// ----------------- State Machine Definition -----------------
enum State {
  WAITING,
  TEST
};
State currentState = WAITING;
int currentPrintNumber = 1;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Integrated System...");

  // Initialize sonar sensor pins.
  pinMode(sonarTrigPin, OUTPUT);
  digitalWrite(sonarTrigPin, LOW);
  pinMode(sonarEchoPin, INPUT);
  
  // Initialize printer.
  mySerial.begin(19200);
  printer.begin();
  printer.setDefault();
  printer.justify('C');
  printer.wake();
  delay(1000);
  printer.setDefault();
  
  // Set speeds for the DC motors.
  tubeMotor.setSpeed(255);  // Full speed for tube extension.
  blockMotor.setSpeed(150); // Proper blocking force.
  
  // Ensure sonar is enabled at startup.
  enableSonar();
}

void loop() {
  if (currentState == WAITING) {
    int distance = readSonarDistance();
    Serial.print("Sonar Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    if (distance > 0 && distance <= sonarThreshold) {
      Serial.println("Object detected!");
      
      // Disable sensor interference by forcing pins low.
      pinMode(sonarTrigPin, OUTPUT);
      digitalWrite(sonarTrigPin, LOW);
      pinMode(sonarEchoPin, OUTPUT);
      digitalWrite(sonarEchoPin, LOW);
      
      currentState = TEST;
    }
    delay(100);
  }
  else if (currentState == TEST) {
    Serial.print("Printing number: ");
    Serial.println(currentPrintNumber);
    printNumberHorizontal(currentPrintNumber);
    
    Serial.println("Feeding extra paper...");
    feedPaperForLength(10);
    
    Serial.println("Activating block motor (DC Slot 2) for clamping...");
    blockMotor.run(FORWARD);
    delay(2000);
    blockMotor.run(RELEASE);
    
    Serial.println("Activating cutter servo (pin 10)...");
    cutterServo.attach(10);
    cutterServo.write(0);
    delay(1500);
    cutterServo.write(90);
    delay(1500);
    cutterServo.detach();
    delay(500);
    
    Serial.println("Extending tube (DC Slot 1)...");
    tubeMotor.run(FORWARD);
    delay(4000);
    tubeMotor.run(RELEASE);
    delay(500);
    
    Serial.println("Retracting tube (DC Slot 1)...");
    tubeMotor.run(BACKWARD);
    delay(4000);
    tubeMotor.run(RELEASE);
    delay(500);
    
    Serial.println("Test sequence complete. Incrementing counter and resetting sonar...\n");
    currentPrintNumber++;
    printer.sleep();
    
    // Re-enable the sonar sensor after test completion.
    //enableSonar();
    
    currentState = WAITING;
  }
}
