#include <Wire.h>
#include <SoftwareSerial.h>
#include <ESC_POS_Printer.h>
#include <avr/pgmspace.h>

// —— I²C master configuration ——
const uint8_t SLAVES[]        = {1,2,3,4,5};
const uint8_t N_SLAVES         = sizeof(SLAVES)/sizeof(SLAVES[0]);
const uint8_t CMD_START        = 0b00001;
const uint8_t CMD_FINISHED     = 0b11111;
// External trigger message: any slave sending 0b00100001
const uint8_t EXTERNAL_START   = (1 << 5) | CMD_START;  // 0b00100001 == 33

unsigned long lastPoll = 0;
volatile bool masterStartFlag = false;

// —— Printer & Big-Digit Setup ——
#define PRINTER_RX    5   // ← Printer TX
#define PRINTER_TX    6   // → Printer RX
SoftwareSerial printerSerial(PRINTER_RX, PRINTER_TX);
ESC_POS_Printer  printer(&printerSerial);

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
const uint16_t digitWidths[]  PROGMEM = {
  digit_0_width, digit_1_width, digit_2_width, digit_3_width, digit_4_width,
  digit_5_width, digit_6_width, digit_7_width, digit_8_width, digit_9_width
};
const uint16_t digitHeights[] PROGMEM = {
  digit_0_height, digit_1_height, digit_2_height, digit_3_height, digit_4_height,
  digit_5_height, digit_6_height, digit_7_height, digit_8_height, digit_9_height
};

void customPrintBitmap(const uint8_t *bitmap, uint16_t width, uint16_t height) {
  uint16_t rowBytes = (width + 7) / 8;
  printer.write(0x1D); printer.write('v'); printer.write('0');
  printer.write((uint8_t)0x00);
  printer.write((uint8_t)(rowBytes & 0xFF));
  printer.write((uint8_t)(rowBytes >> 8));
  printer.write((uint8_t)(height & 0xFF));
  printer.write((uint8_t)(height >> 8));
  uint32_t total = (uint32_t)rowBytes * height;
  for (uint32_t i = 0; i < total; i++) {
    printer.write(pgm_read_byte(bitmap + i));
  }
}

void printBigNumber(long number) {
  bool neg = false;
  if (number < 0) { neg = true; number = -number; }
  char buf[12]; ltoa(number, buf, 10);

  uint8_t digits[12]; uint8_t n = 0;
  if (neg) printer.print("-");
  for (char* p = buf; *p; ++p) digits[n++] = *p - '0';
  if (n == 0) digits[n++] = 0;

  uint16_t totalW = 0, maxH = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint16_t w = pgm_read_word(&digitWidths[digits[i]]);
    uint16_t h = pgm_read_word(&digitHeights[digits[i]]);
    totalW += w; if (h > maxH) maxH = h;
  }
  uint16_t rowBytes = (totalW + 7) / 8;

  // send combined raster header
  printer.write(0x1D); printer.write('v'); printer.write('0');
  printer.write((uint8_t)0x00);
  printer.write((uint8_t)(rowBytes & 0xFF));
  printer.write((uint8_t)(rowBytes >> 8));
  printer.write((uint8_t)(maxH & 0xFF));
  printer.write((uint8_t)(maxH >> 8));

  // stream rows
  for (uint16_t row = 0; row < maxH; row++) {
    for (uint8_t i = 0; i < n; i++) {
      uint8_t d = digits[i];
      uint16_t w   = pgm_read_word(&digitWidths[d]);
      uint16_t h   = pgm_read_word(&digitHeights[d]);
      uint16_t bpr = (w + 7) / 8;
      if (row < h) {
        const uint8_t* base = digitBitmaps[d] + (uint32_t)row * bpr;
        for (uint16_t b = 0; b < bpr; b++) {
          printer.write(pgm_read_byte(base + b));
        }
      } else {
        for (uint16_t b = 0; b < bpr; b++) {
          printer.write((uint8_t)0);
        }
      }
    }
  }
}

// —— Motor & Sensors ——
const int ENA            = 9;
const int IN1            = 8;
const int IN2            = 7;
const int photoResistorPin = A0;
const int lightThreshold = 650;
const int hallExtendPin  = 2;
const int hallRetractPin = 3;
volatile bool isExtended  = true;
volatile bool isRetracted = true;
void ISR_extend()  { isExtended  = (digitalRead(hallExtendPin) == LOW); }
void ISR_retract() { isRetracted = (digitalRead(hallRetractPin) == LOW); }

enum State { IDLE, RUNNING };
State state = IDLE;
int currentPrintNum = 1;

// — Helpers for bit-string I/O —
uint8_t bitStringToByte(const String& s) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 8; i++) v = (v << 1) | (s.charAt(i) - '0');
  return v;
}
String byteToBitString(uint8_t v) {
  String r;
  for (int8_t i = 7; i >= 0; i--) r += char('0' + ((v >> i) & 1));
  return r;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Master ready. Will start loop on external 00100001 message.");
  Serial.println("Enter 8-bit [addr3][cmd5] to send to slaves anytime.");

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Hall endstops
  pinMode(hallExtendPin, INPUT_PULLUP);
  pinMode(hallRetractPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallExtendPin),  ISR_extend,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallRetractPin), ISR_retract, CHANGE);

  // Printer
  printerSerial.begin(115200);
  printer.begin();
  printer.setDefault();
  printer.setLineHeight(64);
  printer.justify('C');
  printer.boldOn();
  printer.setSize('L');
  delay(100);

  // I²C as master
  Wire.begin();
}

void loop() {
  // —— 1) Serial → send to slaves ——
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 8) {
      uint8_t b    = bitStringToByte(line);
      uint8_t dest = b >> 5;
      if (dest == 0) {
        for (uint8_t s : SLAVES) {
          Wire.beginTransmission(s);
          Wire.write(b);
          Wire.endTransmission();
        }
        Serial.print("→ Broadcast "); Serial.println(line);
      } else {
        bool sent = false;
        for (uint8_t s : SLAVES) {
          if (s == dest) {
            Wire.beginTransmission(s);
            Wire.write(b);
            Wire.endTransmission();
            Serial.print("→ To "); Serial.print(s);
            Serial.print(": "); Serial.println(line);
            sent = true;
            break;
          }
        }
        if (!sent) {
          Serial.print("! No slave at address "); Serial.println(dest);
        }
      }
    } else {
      Serial.println("! Error: enter exactly 8 bits");
    }
  }

  // —— 2) Poll each slave for replies ——
  if (millis() - lastPoll > 200) {
    lastPoll = millis();
    for (uint8_t s : SLAVES) {
      Wire.requestFrom(s, (uint8_t)1);
      if (Wire.available()) {
        uint8_t v = Wire.read();
        String bits = byteToBitString(v);
        Serial.print("← From "); Serial.print(s);
        Serial.print(": "); Serial.println(bits);

        // detect external start message
        if (v == EXTERNAL_START) {
          masterStartFlag = true;
          Serial.println(">>> External START trigger received");
        }

        // detect slave-finished
        if ((v & 0x1F) == CMD_FINISHED) {
          Serial.println("<<< Slave reports FINISHED, but handling independently");
        }
      }
    }
  }

  // —— 3) If external start received, run local cycle ——
  if (masterStartFlag) {
    masterStartFlag = false;
    Serial.println(">>> Running local cycle");
    printer.feed(3);
    printer.write(0x1B); printer.write(0x6D);
    // … motor / sensor code can go here …
    Serial.print("Cycle printed #"); Serial.println(currentPrintNum);
    printBigNumber(currentPrintNum);
    currentPrintNum++;
    Serial.println("<<< Local cycle complete\n");
  }
}
