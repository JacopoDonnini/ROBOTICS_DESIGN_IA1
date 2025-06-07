#include <Wire.h>
#include <SoftwareSerial.h>
#include <ESC_POS_Printer.h>
#include <avr/pgmspace.h>
#include <TimeLib.h>

// —— I²C master configuration ——
const uint8_t SLAVES[]       = {1,2,3,4};
const uint8_t N_SLAVES       = 4;
// —— Addresses ——
const uint8_t MY_ADDR        = 0b001;       // this master's 3-bit address
const uint8_t ACT2_ADDR      = 0b010;       // actuation2's 3-bit address
const uint8_t COMM_ADDR      = 0b011;       // communication's 3-bit address
const uint8_t LOC_ADDR       = 0b100;       // localization's 3-bit address
// —— Commands ——
// -GENERAL-
// Not implemented as of now...
const uint8_t START_SYS_CMD  = 0b00001;           //M → L,C,A
const uint8_t STOP_SYS_CMD   = 0b11111;           //M → L,C,A

// First messages to send: in booting sequence all slaves must answer the master's ASK with a TELL
const uint8_t ASK_READY_CMD  = 0b00011;           //M → L,C,A !!!
const uint8_t TELL_READY_CMD = 0b11100;           //M ← L,C,A !!!

// Messages to indicate the robot is moving -> go into low consumption mode; stopped moving -> go into active mode
const uint8_t GO_IDLE_CMD  = 0b10111;             //M → L,C,A !!!
const uint8_t GO_ACTIVE_CMD = 0b11000;            //M → L,C,A !!!

//NOTIFY ERROR?

// -LOCALIZATION-
// Message from master to localization to allow movement
const uint8_t START_MOVEMENT_CMD = 0b00010;       //M → L 
// Messages from communication to master to tell when movement has begun and ended
const uint8_t STARTED_MOVEMENT_CMD = 0b00100;     //M ← L !!!
const uint8_t ENDED_MOVEMENT_CMD = 0b00111;       //M ← L !!!

// -COMMUNICATION-
// Unknown commands... TODO: ask communication
const uint8_t START_INTERACTION_CMD = 0b01000;    //M → C
const uint8_t STARTED_INTERACTION_CMD = 0b00100;  //M ← C
const uint8_t ENDED_INTERACTION_CMD = 0b01111;    //M ← C
// Message from communication to master to signal someone has touched skipy's head
const uint8_t NOTIFY_HEADTOUCH_CMD = 0b01001;     //M ← C !!!
// Messages from master to communication to signal which ball has been extracted by actuation2
const uint8_t NOTIFY_LUCKYBALL_CMD = 0b01010;     //M → C !!!
const uint8_t NOTIFY_NORMALBALL_CMD = 0b01011;    //M → C !!!

//ACTUATION2
// Message from master to actuation2 to start the airflow
const uint8_t START_FLOW_CMD = 0b10001;           //M → A !!!
// Messages from actuation2 to master to signal the beginning and end of airflow
const uint8_t STARTED_FLOW_CMD = 0b10010;         //M ← A !!!
const uint8_t ENDED_FLOW_CMD = 0b10011;           //M ← A !!!
// Messages from actuation2 to master to signal which ball has been extracted
const uint8_t DETECTED_LUCKYBALL_CMD = 0b11010;   //M ← A !!!
const uint8_t DETECTED_NORMALBALL_CMD = 0b11011;  //M ← A !!!

// booleans to keep track which slaves are ready
bool act2_ready = false;
bool comm_ready = false;
bool loc_ready = false;

bool lucky = false;

unsigned long lastPoll       = 0;

// —— Printer & Big‐Digit Setup ——
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
#include "BW_Skippy_Logo.h"
#include "Skipy_Star.h"

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

void printBigNumber(uint8_t number) {
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

void print_ticket(int num, bool luck){
   Serial.print("Printing #: "); 
    Serial.println(num);
    printer.println("");
    customPrintBitmap(BW_logo_data, BW_logo_width, BW_logo_height);
    //printer.feed(5);
    //lucky=true;
  if (!lucky) {
    printer.println("Here is your queue number, I'll call you when it's your turn!");
    //printer.feed(1);
    printBigNumber(num);
    //printer.feed(6);
    printer.println("Come back to tell me");
    printer.println("when you are finished!");
    printer.feed(3);
    printer.write(0x1B); printer.write(0x69); //Partial cut command
  } else {
    printer.println("You got the lucky ball! you will be called ASAP!");
    //printer.feed(1);
    customPrintBitmap(Skipy_Star_data, Skipy_Star_width, Skipy_Star_height);
    printer.feed(1);
    printer.println("Come back to tell me");
    printer.println("when you are finished!");
    printer.feed(3);
    printer.write(0x1B); printer.write(0x69); //Partial cut command
  }
  //currentPrintnum++;
}

// —— Motor / Sensors ——
const int ENA             = 9;
const int IN1             = 8;
const int IN2             = 7;
const int photoResistorPin= A0;
const int lightThreshold  = 650;
const int hallExtendPin   = 2;
const int hallRetractPin  = 3;
volatile bool isExtended  = false;
volatile bool isRetracted = true;
void ISR_extend()  { isExtended  = (digitalRead(hallExtendPin) == LOW); }
void ISR_retract() { isRetracted = (digitalRead(hallRetractPin) == LOW); }

// —— Non-blocking State Machine ——
enum MasterState { S_BOOTING, S_IDLE, S_PRINT, S_EXTEND, S_WAIT_RETRACT };
MasterState mState;
int currentPrintNum = 1;
unsigned long stateStartMillis = 0;

// — Helpers for bit-string I/O —
uint8_t bitStringToByte(const String& s) {
  uint8_t v = 0;
  for (uint8_t i = 0; i < 8; i++) v = (v<<1)|(s.charAt(i)-'0');
  return v;
}

String byteToBitString(uint8_t v) {
  String r;
  for (int8_t i = 7; i >= 0; i--) r += char('0'+((v>>i)&1));
  return r;
}

// — handle incoming i2c messages: split, match, act — 
void handle_message(uint8_t sender, uint8_t msg) {
  uint8_t dest = msg >> 5;
  uint8_t cmd  = msg & 0x1F;
  // handle special messages (none for now)
  if (dest != MY_ADDR){
    return;
  }

  uint8_t fwd;

  switch (cmd) {
    case TELL_READY_CMD:
      // A slave has answered and is ready
      Serial.println(">>> TELL_READY_CMD received");
      switch (sender) {
        case 2:
          Serial.println("Actuation 2 slave is ready");
          act2_ready = true;
      
        case 3:
          Serial.println("Communication slave is ready");
          comm_ready = true;

        case 4:
          Serial.println("Localization slave is ready");
          loc_ready = true;

        default:
          Serial.println("Unknown ready received from slave");

      }

    case STARTED_MOVEMENT_CMD:
      // Skippy is moving from the charinging station to the microwave area.
      // Tell all slaves to go in idle mode to preserve power.
      Serial.println(">>> STARTED_MOVEMENT_CMD received");
      for (uint8_t j: SLAVES) if (j != MY_ADDR) {
        fwd = (j << 5) | GO_IDLE_CMD;
        Wire.beginTransmission(j);
        Wire.write(fwd);
        Wire.endTransmission();
      }
      break;
    
    case ENDED_MOVEMENT_CMD:
      // Skippy has reached the microwave area.
      // Tell all slaves to go into active mode to begin function.
      Serial.println(">>> STARTED_MOVEMENT_CMD received");
      for (uint8_t j: SLAVES) if (j != MY_ADDR) {
        fwd = (j << 5) | GO_ACTIVE_CMD;
        Wire.beginTransmission(j);
        Wire.write(fwd);
        Wire.endTransmission();
      }
      break;

    case NOTIFY_HEADTOUCH_CMD:
      // Skippy has been touched on the head from communication:
      // We send the message to Actuation 2 to start the flow.
      Serial.println(">>> NOTIFY_HEADTOUCH_CMD received");
      fwd = (ACT2_ADDR << 5) | START_FLOW_CMD;
      Wire.beginTransmission(ACT2_ADDR);
      Wire.write(fwd);
      Wire.endTransmission();
      break;

    case STARTED_FLOW_CMD:
      // The flow has started. We wait for it to end.
      Serial.println(">>> STARTED_FLOW_CMD received");
      break;
    
    case ENDED_FLOW_CMD:
      // The flow has ended. We wait for ball detection.
      Serial.println(">>> ENDED_FLOW_CMD received");
      break;
    
    case DETECTED_LUCKYBALL_CMD:
      // Lucky ball has been extracted by actuation2
      // Forward the message to communication
      // Then switch state to lucky print
      Serial.println(">>> DETECTED_LUCKYBALL_CMD received; forwarding to COMM");
      // Rebuild packet: [COMM_ADDR:3][cmd:5]
      bool lucky = true;
      mState = S_PRINT;
      fwd = (COMM_ADDR << 5) | NOTIFY_LUCKYBALL_CMD;
      Wire.beginTransmission(COMM_ADDR);
      Wire.write(fwd);
      Wire.endTransmission();
      break;

    case DETECTED_NORMALBALL_CMD:
      // Normal ball has been extracted by actuation2
      // Forward the message to communication
      // Then switch state to print
      Serial.println(">>> DETECTED_NORMALBALL_CMD received; forwarding to COMM");
      // rebuild packet: [COMM_ADDR:3][cmd:5]
      mState = S_PRINT;
      fwd = (COMM_ADDR << 5) | NOTIFY_NORMALBALL_CMD;
      Wire.beginTransmission(COMM_ADDR);
      Wire.write(fwd);
      Wire.endTransmission();
      break;
    

    default:
      Serial.print(">>> Unknown CMD: ");
      Serial.println(msg);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  mState = S_PRINT;
  Serial.println("Master ready. Enter [addr3][cmd5], or await start command.");

  // Motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Hall-effect endstops
  pinMode(hallExtendPin, INPUT_PULLUP);
  pinMode(hallRetractPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallExtendPin), ISR_extend, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallRetractPin), ISR_retract, CHANGE);

  // Printer init
  printerSerial.begin(115200);
  printer.begin();
  printer.setDefault();
  printer.setLineHeight(64);
  printer.justify('C');
  printer.boldOn();
  printer.setSize('M');
  delay(100);

  // I²C as master
  Wire.begin();

  //Start the clock at 11:58 of 1/1/2025
  //Date/time messages will be supplied by localization's raspberry py
  setTime(11, 58, 0, 1, 1, 2025);
}

void loop() {
  // — 1) Handle user serial commands TO BE REMOVED—
  //Serial.println("loop");
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    Serial.println(line);
    line.trim();
    if (line.length() == 8) {
      uint8_t b    = bitStringToByte(line);
      uint8_t dest = b >> 5;
      if (dest == 0) {
        for (uint8_t s: SLAVES) {
          Wire.beginTransmission(s);
          Wire.write(b);
          Wire.endTransmission();
        }
        Serial.print("→ Broadcast ");
        Serial.println(line);
      } else if (dest == 1){
        Serial.print("Manual command received from user. Executing...");
        handle_message(1,b);
      } else {
        bool sent = false;
        for (uint8_t s: SLAVES) if (s == dest) {
          Wire.beginTransmission(s);
          Wire.write(b);
          Wire.endTransmission();
          Serial.print("→ To "); Serial.print(s);
          Serial.print(": "); Serial.println(line);
          sent = true;
        }
        if (!sent) Serial.println("! No such slave");
      }
    }
  }

  // — 2) Poll slaves for messages & forwarding —
  if (millis() - lastPoll > 200) {
    //Serial.println("pinging...");
    lastPoll = millis();
    for (uint8_t s: SLAVES) {
      Wire.requestFrom(s, (uint8_t)1);
      if (Wire.available()) {
        uint8_t v    = Wire.read();
        uint8_t dest = v >> 5;
        uint8_t cmd  = v & 0x1F;
        String bits  = byteToBitString(v);

        handle_message(s,v);
        
        // forward
        if (dest == 0) {
          for (uint8_t j: SLAVES) if (j != s) {
            Wire.beginTransmission(j);
            Wire.write(v);
            Wire.endTransmission();
          }
        } else if (dest != s) {
          Wire.beginTransmission(dest);
          Wire.write(v);
          Wire.endTransmission();
        }
      }
    }
  }

  // — 3) Run non-blocking master cycle —
  switch (mState) {
    case S_BOOTING:
      // Keep pinging slaves asking if they are ready, store which ones are.
      if (!act2_ready) {
        int8_t msg = (ACT2_ADDR << 5) | ASK_READY_CMD;
            Wire.beginTransmission(ACT2_ADDR);
            Wire.write(msg);
            Wire.endTransmission();
      }
      if (!comm_ready) {
        int8_t msg = (COMM_ADDR << 5) | ASK_READY_CMD;
            Wire.beginTransmission(COMM_ADDR);
            Wire.write(msg);
            Wire.endTransmission();
      }
      if (!loc_ready) {
        int8_t msg = (LOC_ADDR << 5) | ASK_READY_CMD;
            Wire.beginTransmission(LOC_ADDR);
            Wire.write(msg);
            Wire.endTransmission();
      }
      if (act2_ready && comm_ready && loc_ready){
        Serial.println("All slaves ready, boot-up complete.");
        Serial.println("Switching to idle state");
        mState = S_IDLE;
      }
      break;

    case S_IDLE:
      Serial.println("IDLE");
      // TODO: replace with an incoming message from localization as they have the RasPy and better time reference
      if (hour() == 12) {
        Serial.println("Lunchtime! Telling Localization to start movement.");
        uint8_t msg = (LOC_ADDR << 5) | START_MOVEMENT_CMD;
        Wire.beginTransmission(LOC_ADDR);
        Wire.write(msg);
        Wire.endTransmission();
      }
      break;

    case S_PRINT:
      //Printing ticket code
      print_ticket(currentPrintNum, lucky);
      currentPrintNum++;
      lucky = false;
      stateStartMillis = millis();
      delay(5000);
      mState = S_EXTEND;
      break;

    case S_EXTEND:
      // start extending
      Serial.println("Extending...");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 250);
      if (isExtended) {
        analogWrite(ENA, 0);
        Serial.println("→ Extended");
        stateStartMillis = millis();
        mState = S_WAIT_RETRACT;
      }
      isRetracted = false;
      break;

    case S_WAIT_RETRACT:
      int sensorValue = analogRead(photoResistorPin);
      Serial.print("Photoresistor reading: ");
      Serial.println(sensorValue);
      if (analogRead(photoResistorPin) > lightThreshold) {
        Serial.println("→ Black detected");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, 250);
        Serial.println(mState);
      }
      if (isRetracted) {
        analogWrite(ENA, 0);
        Serial.println("→ Retracted");
        Serial.println("<<< Master cycle complete\n");
        mState = S_IDLE;
      }
      isExtended = false;
      break;      

    default:
      Serial.println("Default");
  }
}
