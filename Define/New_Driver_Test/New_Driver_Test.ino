// L298N single‑motor test on Arduino Uno
// Motor on OUT1/OUT2, ENA on D9, IN1 on D8, IN2 on D7.

const int ENA = 9;   // PWM pin for speed
const int IN1 = 8;   // direction pin 1
const int IN2 = 7;   // direction pin 2

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.begin(9600);
  Serial.println("L298N 5V motor test starting...");
}

void loop() {
  // --- Forward ---
  Serial.println("Forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);  // 0–255 → speed
  delay(2000);

  // // --- Stop ---
  // Serial.println("Stop");
  // analogWrite(ENA, 0);
  // delay(1000);

  // // --- Backward ---
  // Serial.println("Backward");
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // analogWrite(ENA, 255);
  // delay(2000);

  // // --- Stop ---
  // Serial.println("Stop");
  // analogWrite(ENA, 0);
  // delay(1000);
}
