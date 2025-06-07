/**************************************************************************
  Integrated Project with ESC/POS Printer, L298N Motor, Photoresistor,
  Hall-Effect Endstops, Big-Digit Bitmap Printing—AUTOMATIC LOOPING

  Runs the same “TEST” cycle continuously, without waiting for an I²C message.
**************************************************************************/

#include <SoftwareSerial.h>
#include <ESC_POS_Printer.h>
#include <avr/pgmspace.h>

// —— Printer & Big-Digit Setup ——
#define PRINTER_RX    5   // ← Printer TX
#define PRINTER_TX    6   // → Printer RX
SoftwareSerial printerSerial(PRINTER_RX, PRINTER_TX);
ESC_POS_Printer printer(&printerSerial);

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
#define DIGIT_WIDTH   32
#define DIGIT_HEIGHT  32
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
      int srcIndex  = x + (b * DIGIT_WIDTH);
      dest[destIndex] = pgm_read_byte(&src[srcIndex]);
    }
  }
}

void printNumberHorizontal(int number) {
  String s = String(number);
  int digitCount = s.length();
  int byteHeight = (DIGIT_HEIGHT + 7) / 8;
  int bufferSize = DIGIT_WIDTH * digitCount * byteHeight;
  uint8_t buffer[bufferSize];
  memset(buffer, 0, bufferSize);
  for (int i = 0; i < digitCount; i++) {
    int d = s[i] - '0';
    copyDigitToBuffer(buffer, digit_bitmaps[d], i, digitCount);
  }
  printer.printBitmap(DIGIT_WIDTH * digitCount, DIGIT_HEIGHT, buffer, false);
  printer.feed(2);
}

// —— L298N Motor Setup ——
const int ENA        = 9;
const int IN1        = 8;
const int IN2        = 7;
const int motorSpeed = 255;

// —— Photoresistor ——
const int photoResistorPin = A0;
const int lightThreshold   = 500;

// —— Hall-Effect Endstops ——
const int hallExtendPin  = 2;
const int hallRetractPin = 3;
volatile bool isExtended  = false;
volatile bool isRetracted = true;

void ISR_extend()  { isExtended  = (digitalRead(hallExtendPin)  == LOW); }
void ISR_retract() { isRetracted = (digitalRead(hallRetractPin) == LOW); }

int currentPrintNumber = 1;

void setup() {
  Serial.begin(115200);
  Serial.println("Setup complete: starting automatic loop");

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Hall endstops w/ pull-ups & interrupts
  pinMode(hallExtendPin,  INPUT_PULLUP);
  pinMode(hallRetractPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallExtendPin),  ISR_extend,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallRetractPin), ISR_retract, CHANGE);

  // Printer initialization
  printerSerial.begin(115200);
  printer.begin();
  printer.setDefault();
  printer.justify('C');
  delay(100);
}

void loop() {
  // — 1) Big-digit print & cut —
  Serial.print("Printing #: "); Serial.println(currentPrintNumber);
  printNumberHorizontal(currentPrintNumber);
  delay(50);
  printer.feed(3);
  printer.write(0x1D);  // GS
  printer.write('V');   // V
  printer.write(0x00);  // full cut

  // — 2) Extend until endstop —
  Serial.println("Extending tube...");
  isExtended = false;
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, motorSpeed);
  while (!isExtended) { /* wait */ }
  analogWrite(ENA, 0);
  Serial.println("→ Fully extended");

  // — 3) Wait for photoresistor = black —
  Serial.println("Waiting for black...");
  while (analogRead(photoResistorPin) >= lightThreshold) {
    delay(100);
  }
  Serial.println("→ Black detected");

  // — 4) Retract until endstop —
  Serial.println("Retracting tube...");
  isRetracted = false;
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, motorSpeed);
  while (!isRetracted) { /* wait */ }
  analogWrite(ENA, 0);
  Serial.println("→ Fully retracted");

  // — 5) (Optional) Master notification removed —
  // If you still need to signal a host, re-insert your Wire.beginTransmission() here

  // — 6) Wrap up & loop again —
  currentPrintNumber++;
  Serial.println("<<< CYCLE COMPLETE, starting next\n");

  // Optional delay between cycles:
  delay(500);
}
