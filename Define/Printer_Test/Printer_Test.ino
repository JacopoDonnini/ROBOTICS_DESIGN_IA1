#include <SoftwareSerial.h>
#include <ESC_POS_Printer.h>
#include <avr/pgmspace.h>

#include "digit_0.h"   // defines digit_0_data[], digit_0_width, digit_0_height
#include "digit_1.h"
#include "digit_2.h"
#include "digit_3.h"
#include "digit_4.h"
#include "digit_5.h"
#include "digit_6.h"
#include "digit_7.h"
#include "digit_8.h"
#include "digit_9.h"
#include "adaqrcode.h" // defines adaqrcode_data[], adaqrcode_width, adaqrcode_height
#include "BW_Skippy_Logo.h"

#define PRX 5  // ← Printer TX via MAX3232 R1OUT
#define PTX 6  // → Printer RX via MAX3232 T1IN

SoftwareSerial ss(PRX, PTX);
ESC_POS_Printer printer(&ss);

// ——— arrays to index your bitmaps ———
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

  uint8_t digits[12], n = 0;
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

  printer.write(0x1D); printer.write('v'); printer.write('0');
  printer.write((uint8_t)0x00);
  printer.write((uint8_t)(rowBytes & 0xFF));
  printer.write((uint8_t)(rowBytes >> 8));
  printer.write((uint8_t)(maxH & 0xFF));
  printer.write((uint8_t)(maxH >> 8));

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
          printer.write((uint8_t)0x00);
        }
      }
    }
  }
}

void setup() {
  // 1) Initialize serial link & printer
  ss.begin(115200);
  delay(100);
  printer.begin();
  printer.setDefault();
  printer.setCharset(CHARSET_USA);
  printer.setCodePage(CODEPAGE_CP437);  

  // 4) Print your QR code at its native size
  printer.justify('C');
  customPrintBitmap(BW_logo_data, 128, 128);
  printer.feed(5);
  printer.println("Here is your queue number, I'll call you when it's your turn!");
  printer.feed(1);
  printBigNumber(69);
  printer.feed(6);
  printer.println("Come back to tell me");
  printer.println("when you are finished!");
  printer.feed(3);

  // 5) Cut paper (full cut via ESC i)
  printer.write(0x1B);
  printer.write(0x69);

}

void loop() {
  // nothing to do here
}
