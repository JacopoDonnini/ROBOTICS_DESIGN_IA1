/**************************************************************************
  Integrated I²C-Slave Project with ESC/POS Printer, L298N Motor,
  Photoresistor, Hall-Effect Endstops, Big-Digit Bitmap Printing
**************************************************************************/

#include <Wire.h>
#include <SoftwareSerial.h>
#include <ESC_POS_Printer.h>
#include <avr/pgmspace.h>

// —— I²C Addresses & Commands ——
const uint8_t MY_ADDR      = 0b001;    // this node’s 3-bit address
const uint8_t MASTER_ADDR  = 0b000;    // master’s 3-bit address (for replies)
const uint8_t CMD_START    = 0b00001;  // payload to start a cycle
const uint8_t CMD_FINISHED = 0b11111;  // our reply when done

volatile bool    startFlag = false;    // set when we see a START
volatile uint8_t outByte   = 0;        // queued reply byte
volatile bool    haveOut   = false;    // reply ready?

// — I²C receive handler: look for “start” message — 
void receiveEvent(int) {
  if (Wire.available()) {
    uint8_t msg  = Wire.read();
    Serial.print(msg);
    uint8_t addr = (msg >> 5) & 0x07;
    uint8_t cmd  = msg & 0x1F;
    if (addr == MY_ADDR && cmd == CMD_START) {
      startFlag = true;
    }
  }
}

// — I²C request handler: master is polling for our reply — 
void requestEvent() {
  if (haveOut) {
    Wire.write(outByte);
    haveOut = false;
  } else {
    Wire.write((uint8_t)0);  // no-data fallback
  }
}

// —— Printer & Big-Digit Setup ——
#define PRINTER_RX    5   // ← Printer TX
#define PRINTER_TX    6   // → Printer RX
SoftwareSerial printerSerial(PRINTER_RX, PRINTER_TX);
ESC_POS_Printer printer(&printerSerial);

// Digit bitmaps (32×32 px) in PROGMEM
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

const uint8_t* const digitBitmaps[] = {
  digit_0_data, digit_1_data, digit_2_data, digit_3_data, digit_4_data,
  digit_5_data, digit_6_data, digit_7_data, digit_8_data, digit_9_data
};
const uint16_t digitWidths[] PROGMEM = {
  digit_0_width, digit_1_width, digit_2_width, digit_3_width, digit_4_width,
  digit_5_width, digit_6_width, digit_7_width, digit_8_width, digit_9_width
};
const uint16_t digitHeights[] PROGMEM = {
  digit_0_height, digit_1_height, digit_2_height, digit_3_height, digit_4_height,
  digit_5_height, digit_6_height, digit_7_height, digit_8_height, digit_9_height
};

// Streams one bitmap as a raster image
void customPrintBitmap(const uint8_t *bitmap, uint16_t width, uint16_t height) {
  uint16_t rowBytes = (width + 7) / 8;
  printer.write(0x1D); printer.write('v'); printer.write('0');
  printer.write((uint8_t)0x00);
  printer.write((uint8_t)(rowBytes & 0xFF));
  printer.write((uint8_t)(rowBytes >> 8));
  printer.write((uint8_t)(height & 0xFF));
  printer.write((uint8_t)(height >> 8));
  uint32_t total = (uint32_t)rowBytes * height;
  for (uint32_t i = 0; i < total; i++)
    printer.write(pgm_read_byte(bitmap + i));
}

// Prints all digits of ‘number’ side-by-side
void printBigNumber(long number) {
  bool neg = false;
  if (number < 0) { neg = true; number = -number; }

  char buf[12];
  ltoa(number, buf, 10);

  uint8_t digits[12], n = 0;
  if (neg) {
    printer.print("-");
  }
  for (char* p = buf; *p; ++p) {
    digits[n++] = *p - '0';
  }
  if (n == 0) digits[n++] = 0;

  uint16_t totalW = 0, maxH = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t d = digits[i];
    uint16_t w = pgm_read_word(&digitWidths[d]);
    uint16_t h = pgm_read_word(&digitHeights[d]);
    totalW += w; if (h > maxH) maxH = h;
  }
  uint16_t rowBytes = (totalW + 7) / 8;

  // send one big raster header
  printer.write(0x1D); printer.write('v'); printer.write('0');
  printer.write((uint8_t)0x00);
  printer.write((uint8_t)(rowBytes & 0xFF));
  printer.write((uint8_t)(rowBytes >> 8));
  printer.write((uint8_t)(maxH & 0xFF));
  printer.write((uint8_t)(maxH >> 8));

  // stream every row of each digit
  for (uint16_t row = 0; row < maxH; row++) {
    for (uint8_t i = 0; i < n; i++) {
      uint8_t d = digits[i];
      uint16_t w   = pgm_read_word(&digitWidths[d]);
      uint16_t h   = pgm_read_word(&digitHeights[d]);
      uint16_t bpr = (w + 7) / 8;
      if (row < h) {
        const uint8_t* base = digitBitmaps[d] + (uint32_t)row * bpr;
        for (uint16_t b = 0; b < bpr; b++)
          printer.write(pgm_read_byte(base + b));
      } else {
        for (uint16_t b = 0; b < bpr; b++)
          printer.write((uint8_t)0x00);
      }
    }
  }
}

// —— L298N Motor Setup ——
const int ENA        = 9;
const int IN1        = 8;
const int IN2        = 7;
const int motorSpeed = 255;

// —— Photoresistor ——
const int photoResistorPin = A0;
const int lightThreshold   = 650;

// —— Hall-Effect Endstops ——
const int hallExtendPin  = 2;
const int hallRetractPin = 3;
volatile bool isExtended  = false;
volatile bool isRetracted = true;
void ISR_extend()  { isExtended  = (digitalRead(hallExtendPin)  == LOW); }
void ISR_retract() { isRetracted = (digitalRead(hallRetractPin) == LOW); }

enum State { WAITING, TEST };
State currentState = WAITING;
int   currentPrintNumber = 1;

void setup() {
  Serial.begin(115200);
  Serial.println("Slave ready, awaiting I²C START...");

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Hall endstops
  pinMode(hallExtendPin,  INPUT_PULLUP);
  pinMode(hallRetractPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallExtendPin),  ISR_extend,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallRetractPin), ISR_retract, CHANGE);

  // Printer init
  printerSerial.begin(115200);
  printer.begin();
  printer.setDefault();
  printer.setLineHeight(64);
  printer.justify('C');
  printer.boldOn();
  printer.setSize('L');
  delay(100);

  // I²C slave init
  Wire.begin(MY_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop() {
  switch (currentState) {
    case WAITING:
      if (startFlag) {
        startFlag = false;
        Serial.println(">>> START received");
        currentState = TEST;
      }
      break;

    case TEST:
      Serial.print("Printing #: ");
      Serial.println(currentPrintNumber);
      printBigNumber(currentPrintNumber);
      printer.feed(3);
      printer.write(0x1B);  // partial cut
      printer.write(0x6D);

      // (motor / sensor sequence commented out here…)

      // queue our “finished” reply
      outByte = (MASTER_ADDR << 5) | CMD_FINISHED;
      haveOut = true;
      Serial.println("→ FINISHED queued for master");

      currentPrintNumber++;
      Serial.println("<<< Cycle complete, back to WAITING\n");
      currentState = WAITING;
      break;
  }
}
