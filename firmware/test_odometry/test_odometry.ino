/**
 * ODOMETRY TEST - DOĞRU KALİBRASYON
 */

// Motor pinleri
const int LEFT_ENA = 5;
const int LEFT_IN1 = 4;
const int LEFT_IN2 = 12;
const int LEFT_IN3 = 3;
const int LEFT_IN4 = 2;
const int LEFT_ENB = 6;

const int RIGHT_ENA = 10;
const int RIGHT_IN1 = 7;
const int RIGHT_IN2 = 9;
const int RIGHT_IN3 = 8;
const int RIGHT_IN4 = 13;
const int RIGHT_ENB = 11;

// Enkoder pinleri
const int LEFT_ENCODER_A = A0;
const int LEFT_ENCODER_B = A1;
const int RIGHT_ENCODER_A = A2;
const int RIGHT_ENCODER_B = A3;

// KALİBRE EDİLMİŞ PARAMETRELER
const float COUNTS_PER_WHEEL_REV = 378.0;  // Ölçülen gerçek değer

// BAŞLANGIÇ - KALİBRASYON YOK
const float NOMINAL_DIAMETER = 0.080;  // 80mm (ölçülen)
const float EFFECTIVE_DIAMETER = 0.080;  // Başlangıç için aynı

const float WHEEL_CIRCUMFERENCE = PI * EFFECTIVE_DIAMETER;
const float METERS_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV;
const float WHEEL_BASE = 0.21;  // ÖLÇÜP GÜNCELLE!

// HIZ AYARI (%65)
const int FORWARD_SPEED = 65;
const int TURN_SPEED = 52;

// Enkoder sayaçları
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile uint8_t prevLeftA = 0;
volatile uint8_t prevRightA = 0;

// Yön düzeltme (sadece bu satırları değiştir)
const int LEFT_ENCODER_DIR = -1;   // 1'den -1'e değişti
const int RIGHT_ENCODER_DIR = 1;   // -1'den 1'e değişti
const int LEFT_MOTOR_DIR = -1;     // aynı
const int RIGHT_MOTOR_DIR = -1;    // aynı

// Odometri
float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Motor pinleri
  pinMode(LEFT_ENA, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);
  pinMode(LEFT_IN4, OUTPUT);
  pinMode(LEFT_ENB, OUTPUT);
  pinMode(RIGHT_ENA, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);
  
  // Enkoder pinleri
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  // Interrupt
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8);
  PCMSK1 |= (1 << PCINT10);
  
  prevLeftA = digitalRead(LEFT_ENCODER_A);
  prevRightA = digitalRead(RIGHT_ENCODER_A);
  
  stopMotors();
  
  Serial.println(F("\n=== ATLAS ODO TEST - DOĞRU KALIBRASYON ==="));
  Serial.print(F("Counts/tur: ")); Serial.println(COUNTS_PER_WHEEL_REV);
  Serial.print(F("Nominal cap: ")); Serial.print(NOMINAL_DIAMETER*1000); Serial.println(F("mm"));
  Serial.print(F("Efektif cap: ")); Serial.print(EFFECTIVE_DIAMETER*1000, 1); Serial.println(F("mm"));
  Serial.print(F("m/count: ")); Serial.println(METERS_PER_COUNT, 6);
  Serial.print(F("Hiz: ")); Serial.print(FORWARD_SPEED); Serial.println(F("/255"));
  delay(2000);
}

void loop() {
  Serial.println(F("\n--- MENU ---"));
  Serial.println(F("1: 50cm ileri"));
  Serial.println(F("2: 1m ileri"));
  Serial.println(F("3: 90deg don"));
  Serial.println(F("4: Kare (50cm)"));
  Serial.println(F("5: Kare (1m)"));
  Serial.println(F("6: Pozisyon"));
  Serial.println(F("7: Enkoder test"));
  Serial.println(F("0: Reset"));
  
  while (!Serial.available()) delay(100);
  
  char c = Serial.read();
  while (Serial.available()) Serial.read();
  
  switch (c) {
    case '0': resetOdom(); Serial.println(F("Reset!")); break;
    case '1': testDistance(0.5); break;
    case '2': testDistance(1.0); break;
    case '3': testRotate(90.0); break;
    case '4': testSquare(0.5); break;
    case '5': testSquare(1.0); break;
    case '6': printPos(); break;
    case '7': testEncoderDir(); break;
    default: Serial.println(F("?")); break;
  }
  delay(1000);
}

void testEncoderDir() {
  Serial.println(F("\n>>> ENKODER TEST <<<"));
  Serial.println(F("Elle ileri cevir (10sn)"));
  
  resetOdom();
  unsigned long start = millis();
  
  while (millis() - start < 10000) {
    Serial.print(F("L: ")); Serial.print(leftEncoderCount);
    Serial.print(F(" R: ")); Serial.println(rightEncoderCount);
    delay(500);
  }
  
  if (leftEncoderCount > 0 && rightEncoderCount > 0) {
    Serial.println(F("OK!"));
  } else {
    Serial.println(F("HATA!"));
  }
}

void testDistance(float meters) {
  Serial.println(F("\n>>> MESAFE TEST <<<"));
  Serial.print(F("Hedef: ")); Serial.print(meters*100); Serial.println(F("cm"));
  delay(3000);
  
  resetOdom();
  long startL = leftEncoderCount;
  long startR = rightEncoderCount;
  long targetCounts = abs(meters / METERS_PER_COUNT);
  
  Serial.print(F("Target counts: ")); Serial.println(targetCounts);
  Serial.println(F("GO!"));
  
  setLeftMotors(FORWARD_SPEED);
  setRightMotors(FORWARD_SPEED);
  
  unsigned long lastPrint = 0;
  while (true) {
    updateOdom();
    long avgCnt = (abs(leftEncoderCount - startL) + abs(rightEncoderCount - startR)) / 2;
    
    if (millis() - lastPrint > 500) {
      Serial.print(avgCnt); Serial.print(F("/")); Serial.print(targetCounts);
      Serial.print(F(" | ")); Serial.print(robotX*100, 1); Serial.println(F("cm"));
      lastPrint = millis();
    }
    
    if (avgCnt >= targetCounts) break;
    delay(10);
  }
  
  stopMotors();
  delay(1000);
  updateOdom();
  
  Serial.println(F("\n=== SONUC ==="));
  Serial.print(F("Hedef: ")); Serial.print(meters*100, 1); Serial.println(F("cm"));
  Serial.print(F("Hesap: ")); Serial.print(robotX*100, 1); Serial.println(F("cm"));
  Serial.print(F("EncL: ")); Serial.print(abs(leftEncoderCount-startL));
  Serial.print(F(" EncR: ")); Serial.println(abs(rightEncoderCount-startR));
  Serial.println(F("\nMEZURA ile GERCEK mesafeyi olc!"));
}

void testRotate(float deg) {
  Serial.println(F("\n>>> DONUS TEST <<<"));
  Serial.print(F("Hedef: ")); Serial.print(deg); Serial.println(F("deg"));
  delay(3000);
  
  resetOdom();
  float target = deg * PI / 180.0;
  
  setLeftMotors(TURN_SPEED);
  setRightMotors(-TURN_SPEED);
  
  while (abs(robotTheta) < abs(target) * 0.95) {
    updateOdom();
    delay(10);
  }
  
  stopMotors();
  delay(500);
  updateOdom();
  
  Serial.print(F("Olculen: ")); Serial.print(robotTheta*180/PI, 1); Serial.println(F("deg"));
}

void testSquare(float side) {
  Serial.println(F("\n>>> KARE TEST <<<"));
  Serial.print(F("Kenar: ")); Serial.print(side*100); Serial.println(F("cm"));
  delay(5000);
  
  resetOdom();
  
  for (int i = 0; i < 4; i++) {
    Serial.print(F("Kenar ")); Serial.println(i+1);
    moveDistance(side);
    delay(500);
    rotateDegrees(90.0);
    delay(500);
  }
  
  Serial.println(F("\n=== TAMAMLANDI ==="));
  printPos();
  
  float err = sqrt(robotX*robotX + robotY*robotY);
  Serial.print(F("Hata: ")); Serial.print(err*100, 1); Serial.println(F("cm"));
}

void moveDistance(float m) {
  long start = (abs(leftEncoderCount) + abs(rightEncoderCount)) / 2;
  long target = abs(m / METERS_PER_COUNT);
  
  setLeftMotors(FORWARD_SPEED);
  setRightMotors(FORWARD_SPEED);
  
  while (true) {
    updateOdom();
    long curr = (abs(leftEncoderCount) + abs(rightEncoderCount)) / 2;
    if (abs(curr - start) >= target) break;
    delay(10);
  }
  
  stopMotors();
  delay(300);
}

void rotateDegrees(float deg) {
  float start = robotTheta;
  float target = deg * PI / 180.0;
  
  setLeftMotors(TURN_SPEED);
  setRightMotors(-TURN_SPEED);
  
  while (abs(robotTheta - start) < abs(target) * 0.92) {
    updateOdom();
    delay(10);
  }
  
  stopMotors();
  delay(300);
}

void resetOdom() {
  noInterrupts();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  interrupts();
  robotX = 0.0;
  robotY = 0.0;
  robotTheta = 0.0;
}

void printPos() {
  Serial.println(F("\n--- POS ---"));
  Serial.print(F("X: ")); Serial.print(robotX*100, 1); Serial.println(F("cm"));
  Serial.print(F("Y: ")); Serial.print(robotY*100, 1); Serial.println(F("cm"));
  Serial.print(F("Theta: ")); Serial.print(robotTheta*180/PI, 1); Serial.println(F("deg"));
}

void updateOdom() {
  static long prevL = 0;
  static long prevR = 0;
  
  noInterrupts();
  long cntL = leftEncoderCount;
  long cntR = rightEncoderCount;
  interrupts();
  
  long deltaL = cntL - prevL;
  long deltaR = cntR - prevR;
  
  float distL = deltaL * METERS_PER_COUNT;
  float distR = deltaR * METERS_PER_COUNT;
  
  float center = (distL + distR) / 2.0;
  float dTheta = (distR - distL) / WHEEL_BASE;
  
  robotX += center * cos(robotTheta + dTheta / 2.0);
  robotY += center * sin(robotTheta + dTheta / 2.0);
  robotTheta += dTheta;
  
  while (robotTheta > PI) robotTheta -= 2*PI;
  while (robotTheta < -PI) robotTheta += 2*PI;
  
  prevL = cntL;
  prevR = cntR;
}

void setLeftMotors(int spd) {
  spd = constrain(spd, -255, 255) * LEFT_MOTOR_DIR;
  
  if (spd > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(LEFT_IN3, HIGH);
    digitalWrite(LEFT_IN4, LOW);
    analogWrite(LEFT_ENA, abs(spd));
    analogWrite(LEFT_ENB, abs(spd));
  } else if (spd < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    digitalWrite(LEFT_IN3, LOW);
    digitalWrite(LEFT_IN4, HIGH);
    analogWrite(LEFT_ENA, abs(spd));
    analogWrite(LEFT_ENB, abs(spd));
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(LEFT_IN3, LOW);
    digitalWrite(LEFT_IN4, LOW);
    analogWrite(LEFT_ENA, 0);
    analogWrite(LEFT_ENB, 0);
  }
}

void setRightMotors(int spd) {
  spd = constrain(spd, -255, 255) * RIGHT_MOTOR_DIR;
  
  if (spd > 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    digitalWrite(RIGHT_IN3, HIGH);
    digitalWrite(RIGHT_IN4, LOW);
    analogWrite(RIGHT_ENA, abs(spd));
    analogWrite(RIGHT_ENB, abs(spd));
  } else if (spd < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, HIGH);
    analogWrite(RIGHT_ENA, abs(spd));
    analogWrite(RIGHT_ENB, abs(spd));
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, LOW);
    analogWrite(RIGHT_ENA, 0);
    analogWrite(RIGHT_ENB, 0);
  }
}

void stopMotors() {
  setLeftMotors(0);
  setRightMotors(0);
}

ISR(PCINT1_vect) {
  uint8_t currLA = digitalRead(LEFT_ENCODER_A);
  if (currLA != prevLeftA) {
    uint8_t leftB = digitalRead(LEFT_ENCODER_B);
    leftEncoderCount += ((currLA == leftB) ? 1 : -1) * LEFT_ENCODER_DIR;
    prevLeftA = currLA;
  }
  
  uint8_t currRA = digitalRead(RIGHT_ENCODER_A);
  if (currRA != prevRightA) {
    uint8_t rightB = digitalRead(RIGHT_ENCODER_B);
    rightEncoderCount += ((currRA == rightB) ? 1 : -1) * RIGHT_ENCODER_DIR;
    prevRightA = currRA;
  }
}