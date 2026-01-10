/**
 * ATLAS AGV - TEST ODOMETRY (2 Motor + 1 Caster Wheel)
 * 
 * Hardware: 2x DC Motor + Encoders + 1 Caster Wheel
 * Yeni Pin Konfigürasyonu
 */

// ============================================
// PIN TANIMLARI
// ============================================
#define LEFT_PWM 10
#define LEFT_IN1 8
#define LEFT_IN2 9
#define RIGHT_PWM 11
#define RIGHT_IN1 12
#define RIGHT_IN2 13
#define LEFT_ENC_A A1
#define LEFT_ENC_B A0
#define RIGHT_ENC_A A2
#define RIGHT_ENC_B A3

// ============================================
// KALİBRE EDİLMİŞ PARAMETRELER
// ============================================
const float COUNTS_PER_WHEEL_REV = 435.0;  // Ölçülen gerçek değer

// Tekerlek parametreleri
const float WHEEL_DIAMETER = 0.080;  // 80mm (ölçülen)
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float METERS_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV;
const float WHEEL_BASE = 0.21*1;  // 210mm (motorlar arası mesafe)

// Hız ayarları
const int FORWARD_SPEED = 90;   // İleri hız (%25 PWM)
const int TURN_SPEED = 90;     // Dönüş hızı

// MOTOR KALİBRASYON FAKTÖRLERI (Düz gidiş için)
// Sağa kayıyorsa: LEFT_SPEED_FACTOR'u artır
// Sola kayıyorsa: RIGHT_SPEED_FACTOR'u artır
const float LEFT_SPEED_FACTOR = 1.0;   // Sol motor çarpanı (0.8 - 1.2)
const float RIGHT_SPEED_FACTOR = 1.07;  // Sağ motor çarpanı (0.8 - 1.2)

// Enkoder ve motor yön düzeltmeleri
const int LEFT_ENCODER_DIR = 1;
const int RIGHT_ENCODER_DIR = 1;
const int LEFT_MOTOR_DIR = -1;
const int RIGHT_MOTOR_DIR = -1;

// ============================================
// GLOBAL DEĞİŞKENLER
// ============================================
// Enkoder sayaçları
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile uint8_t prevLeftA = 0;
volatile uint8_t prevRightA = 0;

// Odometri değişkenleri
float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  
  // Motor pinleri (OUTPUT)
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  
  // Enkoder pinleri (INPUT_PULLUP)
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  
  // Pin Change Interrupt ayarları
  // A1 (PCINT9) ve A2 (PCINT10) için PCINT1 vektörü
  PCICR |= (1 << PCIE1);      // PCINT1 grubunu etkinleştir
  PCMSK1 |= (1 << PCINT9);    // A1 (LEFT_ENC_A)
  PCMSK1 |= (1 << PCINT10);   // A2 (RIGHT_ENC_A)
  
  prevLeftA = digitalRead(LEFT_ENC_A);
  prevRightA = digitalRead(RIGHT_ENC_A);
  
  stopMotors();
  
  Serial.println(F("\n=== ATLAS AGV - TEST ODOMETRY (2 MOTOR) ==="));
  Serial.print(F("Counts/rev: ")); Serial.println(COUNTS_PER_WHEEL_REV);
  Serial.print(F("Wheel diameter: ")); Serial.print(WHEEL_DIAMETER*1000); Serial.println(F("mm"));
  Serial.print(F("Wheel base: ")); Serial.print(WHEEL_BASE*1000); Serial.println(F("mm"));
  Serial.print(F("m/count: ")); Serial.println(METERS_PER_COUNT, 6);
  Serial.print(F("Forward speed: ")); Serial.print(FORWARD_SPEED); Serial.println(F("/255"));
  Serial.print(F("Turn speed: ")); Serial.print(TURN_SPEED); Serial.println(F("/255"));
  delay(2000);
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  Serial.println(F("\n--- MENU ---"));
  Serial.println(F("1: 50cm ileri"));
  Serial.println(F("2: 1m ileri"));
  Serial.println(F("3: 90deg don"));
  Serial.println(F("4: Kare (50cm)"));
  Serial.println(F("5: Kare (1m)"));
  Serial.println(F("6: Pozisyon"));
  Serial.println(F("7: Enkoder test"));
  Serial.println(F("8: Motor test"));
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
    case '8': testMotors(); break;
    default: Serial.println(F("?")); break;
  }
  delay(1000);
}

// ============================================
// TEST FONKSİYONLARI
// ============================================

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
    Serial.println(F("OK! Her iki enkoder pozitif"));
  } else {
    Serial.println(F("HATA! Enkoder yonlerini kontrol et"));
    Serial.print(F("LEFT_ENCODER_DIR = ")); Serial.println(leftEncoderCount > 0 ? "1 (OK)" : "-1 (Ters)");
    Serial.print(F("RIGHT_ENCODER_DIR = ")); Serial.println(rightEncoderCount > 0 ? "1 (OK)" : "-1 (Ters)");
  }
}

void testMotors() {
  Serial.println(F("\n>>> MOTOR TEST <<<"));
  
  Serial.println(F("Sol motor ileri - 3sn"));
  setLeftMotor(150);
  delay(3000);
  setLeftMotor(0);
  delay(1000);
  
  Serial.println(F("Sol motor geri - 3sn"));
  setLeftMotor(-150);
  delay(3000);
  setLeftMotor(0);
  delay(1000);
  
  Serial.println(F("Sag motor ileri - 3sn"));
  setRightMotor(150);
  delay(3000);
  setRightMotor(0);
  delay(1000);
  
  Serial.println(F("Sag motor geri - 3sn"));
  setRightMotor(-150);
  delay(3000);
  setRightMotor(0);
  delay(1000);
  
  Serial.println(F("Her iki motor ileri - 3sn"));
  setLeftMotor(150);
  setRightMotor(150);
  delay(3000);
  stopMotors();
  
  Serial.println(F("Test tamamlandi!"));
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
  
  setLeftMotor(FORWARD_SPEED);
  setRightMotor(FORWARD_SPEED);
  
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
  Serial.print(F("Hesaplanan: ")); Serial.print(robotX*100, 1); Serial.println(F("cm"));
  Serial.print(F("EncL: ")); Serial.print(abs(leftEncoderCount-startL));
  Serial.print(F(" EncR: ")); Serial.println(abs(rightEncoderCount-startR));
  Serial.println(F("\nMEZURA ile GERCEK mesafeyi olc!"));
}

void testRotate(float deg) {
  Serial.println(F("\n>>> DONUS TEST <<<"));
  Serial.print(F("Hedef: ")); Serial.print(deg); Serial.println(F("deg"));
  Serial.print(F("WHEEL_BASE: ")); Serial.print(WHEEL_BASE*1000, 1); Serial.println(F("mm"));
  delay(3000);
  
  resetOdom();  // ✅ Zaten var
  
  // ✅ ENKODER SAYAÇLARINI DA KAYDET
  long startL = leftEncoderCount;
  long startR = rightEncoderCount;
  
  float target = deg * PI / 180.0;
  
  setLeftMotor(TURN_SPEED);
  setRightMotor(-TURN_SPEED);
  
  unsigned long lastPrint = 0;
  unsigned long startTime = millis();
  
  // ✅ TIMEOUT EKLE (güvenlik için)
  while (abs(robotTheta) < abs(target) * 0.98 && millis() - startTime < 10000) {
    updateOdom();
    
    if (millis() - lastPrint > 300) {
      Serial.print(F("Theta: ")); Serial.print(robotTheta*180/PI, 1);
      Serial.print(F(" / ")); Serial.print(deg);
      Serial.print(F(" | EncL: ")); Serial.print(abs(leftEncoderCount-startL));
      Serial.print(F(" EncR: ")); Serial.println(abs(rightEncoderCount-startR));
      lastPrint = millis();
    }
    
    delay(10);
  }
  
  stopMotors();
  delay(500);
  updateOdom();
  
  Serial.println(F("\n=== SONUC ==="));
  Serial.print(F("Hedef: ")); Serial.print(deg, 1); Serial.println(F(" deg"));
  Serial.print(F("Hesaplanan: ")); Serial.print(robotTheta*180/PI, 1); Serial.println(F(" deg"));
  Serial.print(F("EncL: ")); Serial.print(abs(leftEncoderCount-startL));
  Serial.print(F(" EncR: ")); Serial.println(abs(rightEncoderCount-startR));
  Serial.println(F("\nACIOLCER ile GERCEK aciyi olc!"));
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
  Serial.print(F("Baslangic hata: ")); Serial.print(err*100, 1); Serial.println(F("cm"));
}

// ============================================
// YARDIMCI FONKSİYONLAR
// ============================================

void moveDistance(float m) {
  long start = (abs(leftEncoderCount) + abs(rightEncoderCount)) / 2;
  long target = abs(m / METERS_PER_COUNT);
  
  setLeftMotor(FORWARD_SPEED);
  setRightMotor(FORWARD_SPEED);
  
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
  
  setLeftMotor(TURN_SPEED);
  setRightMotor(-TURN_SPEED);
  
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
  Serial.println(F("\n--- POZISYON ---"));
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
  
  // Differential drive kinematics
  float center = (distL + distR) / 2.0;
  float dTheta = (distR - distL) / WHEEL_BASE;
  
  robotX += center * cos(robotTheta + dTheta / 2.0);
  robotY += center * sin(robotTheta + dTheta / 2.0);
  robotTheta += dTheta;
  
  // Normalize angle to -PI to +PI
  while (robotTheta > PI) robotTheta -= 2*PI;
  while (robotTheta < -PI) robotTheta += 2*PI;
  
  prevL = cntL;
  prevR = cntR;
}

// ============================================
// MOTOR KONTROL FONKSİYONLARI
// ============================================

void setLeftMotor(int speed) {
  // Kalibrasyon faktörü uygula
  speed = (int)(speed * LEFT_SPEED_FACTOR);
  speed = constrain(speed, -255, 255) * LEFT_MOTOR_DIR;
  
  if (speed > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_PWM, abs(speed));
  } else if (speed < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    analogWrite(LEFT_PWM, abs(speed));
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_PWM, 0);
  }
}

void setRightMotor(int speed) {
  // Kalibrasyon faktörü uygula
  speed = (int)(speed * RIGHT_SPEED_FACTOR);
  speed = constrain(speed, -255, 255) * RIGHT_MOTOR_DIR;
  
  if (speed > 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_PWM, abs(speed));
  } else if (speed < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(RIGHT_PWM, abs(speed));
  } else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_PWM, 0);
  }
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

// ============================================
// ENKODER INTERRUPT HANDLER
// ============================================
ISR(PCINT1_vect) {
  // Sol enkoder (A1)
  uint8_t currLA = digitalRead(LEFT_ENC_A);
  if (currLA != prevLeftA) {
    uint8_t leftB = digitalRead(LEFT_ENC_B);
    leftEncoderCount += ((currLA == leftB) ? 1 : -1) * LEFT_ENCODER_DIR;
    prevLeftA = currLA;
  }
  
  // Sağ enkoder (A2)
  uint8_t currRA = digitalRead(RIGHT_ENC_A);
  if (currRA != prevRightA) {
    uint8_t rightB = digitalRead(RIGHT_ENC_B);
    rightEncoderCount += ((currRA == rightB) ? 1 : -1) * RIGHT_ENCODER_DIR;
    prevRightA = currRA;
  }
}