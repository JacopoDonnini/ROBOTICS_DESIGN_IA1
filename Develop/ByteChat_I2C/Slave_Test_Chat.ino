#include <Wire.h>
#define MY_ADDR  1    // change this to 1,2,3,4 or 5 for each node

volatile uint8_t inByte    = 0;
volatile bool    msgReady  = false;

volatile uint8_t outByte   = 0;
volatile bool    haveOut   = false;

// Helpers same as master
uint8_t bitStringToByte(const String& s) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 8; i++)
    v = (v << 1) | (s.charAt(i) - '0');
  return v;
}

String byteToBitString(uint8_t v) {
  String r;
  for (int8_t i = 7; i >= 0; i--)
    r += char('0' + ((v >> i) & 1));
  return r;
}

void setup() {
  Wire.begin(MY_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Serial.begin(9600);
  Serial.print(F("Slave ")); Serial.print(MY_ADDR);
  Serial.println(F(" ready. Enter 8-bit msg [addr3][data5]."));
}

void loop() {
  // — Print incoming when flagged —
  if (msgReady) {
    noInterrupts();
      uint8_t v = inByte;
      msgReady = false;
    interrupts();

    uint8_t dst     = v >> 5;
    uint8_t payload = v & 0x1F;      // low 5 bits
    if (dst == 0 || dst == MY_ADDR) {
      // only show if it's a broadcast or for me
      // display data bits and decimal
      String bin5;
      for (int8_t i = 4; i >= 0; i--)
        bin5 += char('0' + ((payload >> i) & 1));
      Serial.print(F("Chat ← payload ")); 
      Serial.print(bin5);
      Serial.print(F(" (")); Serial.print(payload); Serial.println(F(")"));
    }
  }

  // — Queue up local typing —
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() != 8) {
      Serial.println(F("↯ Error: please enter exactly 8 bits"));
    } else {
      outByte = bitStringToByte(line);
      haveOut = true;
      Serial.print(F("Queued ")); Serial.println(byteToBitString(outByte));
    }
  }
}

// Master → Slave write
void onReceive(int howMany) {
  if (Wire.available()) {
    inByte   = Wire.read();
    msgReady = true;
  }
}

// Master polling for our queued byte
void onRequest() {
  if (haveOut) {
    Wire.write(outByte);
    haveOut = false;
  }
}
