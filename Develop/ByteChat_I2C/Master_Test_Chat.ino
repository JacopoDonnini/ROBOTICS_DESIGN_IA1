#include <Wire.h>
const uint8_t SLAVES[] = {1,2,3,4,5};     // hardware I²C addresses for your up to-5 slaves
const uint8_t N_SLAVES = sizeof(SLAVES)/sizeof(SLAVES[0]);

unsigned long lastPoll = 0;

// Convert an 8-char "0"/"1" string → byte
uint8_t bitStringToByte(const String& s) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 8; i++)
    v = (v << 1) | (s.charAt(i) - '0');
  return v;
}
// Turn a byte back into "01010101"
String byteToBitString(uint8_t v) {
  String r;
  for (int8_t i = 7; i >= 0; i--)
    r += char('0' + ((v >> i) & 1));
  return r;
}

void setup() {
  Wire.begin();           
  Serial.begin(115200);
  Serial.println(F("Master ready. Enter 8-bit messages: [addr3][data5], e.g. 00100000"));
  Serial.println(F(" addr=000 → broadcast, addr=001–101 → slaves 1–5"));
}

void loop() {
  // —— 1) Read from PC as exactly 8 bits ——
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() != 8) {
      Serial.println(F("↯ Error: please enter exactly 8 bits"));
    } else {
      uint8_t b     = bitStringToByte(line);
      uint8_t dest  = b >> 5;           // top 3 bits
      String bits8 = byteToBitString(b);

      // → broadcast?
      if (dest == 0) {
        for (uint8_t i = 0; i < N_SLAVES; i++) {
          Wire.beginTransmission(SLAVES[i]);
          Wire.write(b);
          Wire.endTransmission();
        }
        Serial.print(F("→ Broadcast ")); Serial.println(bits8);
      } 
      // → direct to one slave?
      else {
        bool sent = false;
        for (uint8_t i = 0; i < N_SLAVES; i++) {
          if (SLAVES[i] == dest) {
            Wire.beginTransmission(dest);
            Wire.write(b);
            Wire.endTransmission();
            Serial.print(F("→ To   ")); Serial.print(dest);
            Serial.print(F(": ")); Serial.println(bits8);
            sent = true;
            break;
          }
        }
        if (!sent) {
          Serial.print(F("↯ Error: no slave at address "));
          Serial.println(dest);
        }
      }
    }
  }

  // —— 2) Poll each slave for its outgoing byte ——  
  if (millis() - lastPoll > 200) {
    lastPoll = millis();
    for (uint8_t i = 0; i < N_SLAVES; i++) {
      uint8_t src = SLAVES[i];
      Wire.requestFrom(src, (uint8_t)1);
      if (Wire.available()) {
        uint8_t v    = Wire.read();
        uint8_t dst  = v >> 5;
        String bits8 = byteToBitString(v);

        // Print what came in
        if (dst == 0) {
          Serial.print(F("← From ")); Serial.print(src);
          Serial.print(F(" (bcast): ")); Serial.println(bits8);
        } else {
          Serial.print(F("← From ")); Serial.print(src);
          Serial.print(F(" → ")); Serial.print(dst);
          Serial.print(F(": ")); Serial.println(bits8);
        }

        // Forward on-bus:
        if (dst == 0) {
          // rebroadcast to everyone except origin
          for (uint8_t j = 0; j < N_SLAVES; j++) {
            if (SLAVES[j] == src) continue;
            Wire.beginTransmission(SLAVES[j]);
            Wire.write(v);
            Wire.endTransmission();
          }
        } else if (dst != src) {
          // unicast to "dst"
          Wire.beginTransmission(dst);
          Wire.write(v);
          Wire.endTransmission();
        }
      }
    }
  }
}
