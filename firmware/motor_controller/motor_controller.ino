/**
 * ATLAS AGV - MOTOR CONTROLLER (2 Motor + 1 Caster Wheel)
 * 
 * ROS 2 Compatible Serial Protocol
 * Pin Configuration: Updated for 2-motor differential drive
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
const float WHEEL_DIAMETER = 0.080;  // 80mm
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float METERS_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV;  // ~0.000578 m/count
const float WHEEL_BASE = 0.21;  // 210mm (kalibre edilmiş)

// Motor kalibrasyon faktörleri (düz gidiş için)
const float LEFT_SPEED_FACTOR = 1.0;
const float RIGHT_SPEED_FACTOR = 1.07;  // Sağ motor hafif hızlandırıldı

// Hız limitleri
const int MAX_PWM = 100;         // Güç kaynağı limiti
const int MIN_PWM = 70;          // Minimum hareket eşiği
const float MAX_LINEAR_SPEED = 0.5;   // m/s
const float MAX_ANGULAR_SPEED = 2.0;  // rad/s

// Enkoder ve motor yön düzeltmeleri (kalibre edilmiş)
const int LEFT_ENCODER_DIR = 1;
const int RIGHT_ENCODER_DIR = 1;
const int LEFT_MOTOR_DIR = -1;
const int RIGHT_MOTOR_DIR = -1;

// Serial protokol
const unsigned long UPDATE_INTERVAL = 50;  // 20Hz odometry
const unsigned long CMD_TIMEOUT = 500;     // 200ms timeout

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
long prevLeftCount = 0;
long prevRightCount = 0;
float leftWheelVel = 0.0;
float rightWheelVel = 0.0;

// Motor durumu
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

// Zamanlama
unsigned long prevTime = 0;
unsigned long lastCmdTime = 0;

// Serial buffer
String inputString = "";
bool stringComplete = false;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // Motor pinleri
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  
  // Enkoder pinleri
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  
  // Pin Change Interrupt setup
  PCICR |= (1 << PCIE1);      // PCINT1 grubunu etkinleştir
  PCMSK1 |= (1 << PCINT9);    // A1 (LEFT_ENC_A)
  PCMSK1 |= (1 << PCINT10);   // A2 (RIGHT_ENC_A)
  
  prevLeftA = digitalRead(LEFT_ENC_A);
  prevRightA = digitalRead(RIGHT_ENC_A);
  
  stopMotors();
  
  inputString.reserve(64);
  prevTime = millis();
  lastCmdTime = millis();
  
  delay(1000);
  Serial.println(F("READY"));
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  unsigned long currentTime = millis();
  
  // 20Hz odometry update
  if (currentTime - prevTime >= UPDATE_INTERVAL) {
    float deltaTime = (currentTime - prevTime) / 1000.0;
    updateOdometry(deltaTime);
    publishOdometry();
    prevTime = currentTime;
  }
  
  // Command timeout (safety)
  if (currentTime - lastCmdTime > CMD_TIMEOUT) {
    if (currentLeftSpeed != 0 || currentRightSpeed != 0) {
      stopMotors();
    }
  }
  
  // Serial komut işleme
  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

// ============================================
// SERIAL EVENT
// ============================================
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// ============================================
// KOMUT İŞLEME
// ============================================
void parseCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith(F("CMD,"))) {
    // CMD,left_pwm,right_pwm
    int comma = cmd.indexOf(',', 4);
    if (comma > 0) {
      int leftPWM = cmd.substring(4, comma).toInt();
      int rightPWM = cmd.substring(comma + 1).toInt();
      setMotors(leftPWM, rightPWM);
      lastCmdTime = millis();
    }
  }
  else if (cmd == F("STOP")) {
    stopMotors();
    Serial.println(F("ACK"));
  }
  else if (cmd == F("RESET")) {
    resetOdom();
    Serial.println(F("ACK"));
  }
  else if (cmd == F("STATUS")) {
    Serial.print(F("STATUS,"));
    Serial.print(currentLeftSpeed);
    Serial.print(F(","));
    Serial.println(currentRightSpeed);
  }
}

// ============================================
// ODOMETRY FONKSIYONLARI
// ============================================
void updateOdometry(float deltaTime) {
  // Atomic enkoder okuma
  noInterrupts();
  long leftCount = leftEncoderCount;
  long rightCount = rightEncoderCount;
  interrupts();
  
  // Delta hesapla
  long leftDelta = leftCount - prevLeftCount;
  long rightDelta = rightCount - prevRightCount;
  
  // Mesafeye çevir
  float leftDistance = leftDelta * METERS_PER_COUNT;
  float rightDistance = rightDelta * METERS_PER_COUNT;
  
  // Hız hesapla
  if (deltaTime > 0) {
    leftWheelVel = leftDistance / deltaTime;
    rightWheelVel = rightDistance / deltaTime;
  }
  
  // Differential drive kinematics
  float centerDistance = (leftDistance + rightDistance) / 2.0;
  float deltaTheta = (rightDistance - leftDistance) / WHEEL_BASE;
  
  // Pozisyon güncelle
  robotX += centerDistance * cos(robotTheta + deltaTheta / 2.0);
  robotY += centerDistance * sin(robotTheta + deltaTheta / 2.0);
  robotTheta += deltaTheta;
  
  // Açıyı normalize et (-π ile +π arası)
  while (robotTheta > PI) robotTheta -= 2.0 * PI;
  while (robotTheta < -PI) robotTheta += 2.0 * PI;
  
  prevLeftCount = leftCount;
  prevRightCount = rightCount;
}

void publishOdometry() {
  // Format: ODO,x,y,theta,vl,vr,encL,encR
  Serial.print(F("ODO,"));
  Serial.print(robotX, 4);
  Serial.print(F(","));
  Serial.print(robotY, 4);
  Serial.print(F(","));
  Serial.print(robotTheta, 4);
  Serial.print(F(","));
  Serial.print(leftWheelVel, 4);
  Serial.print(F(","));
  Serial.print(rightWheelVel, 4);
  Serial.print(F(","));
  
  noInterrupts();
  Serial.print(leftEncoderCount);
  Serial.print(F(","));
  Serial.println(rightEncoderCount);
  interrupts();
}

void resetOdom() {
  noInterrupts();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  interrupts();
  
  prevLeftCount = 0;
  prevRightCount = 0;
  robotX = 0.0;
  robotY = 0.0;
  robotTheta = 0.0;
  leftWheelVel = 0.0;
  rightWheelVel = 0.0;
}

// ============================================
// MOTOR KONTROL FONKSİYONLARI
// ============================================
void setMotors(int leftPWM, int rightPWM) {
  // Kalibrasyon faktörü uygula
  leftPWM = (int)(leftPWM * LEFT_SPEED_FACTOR);
  rightPWM = (int)(rightPWM * RIGHT_SPEED_FACTOR);
  
  // Limit uygula
  leftPWM = constrain(leftPWM, -MAX_PWM, MAX_PWM);
  rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);
  
  // Minimum eşik kontrolü
  if (abs(leftPWM) > 0 && abs(leftPWM) < MIN_PWM) {
    leftPWM = (leftPWM > 0) ? MIN_PWM : -MIN_PWM;
  }
  if (abs(rightPWM) > 0 && abs(rightPWM) < MIN_PWM) {
    rightPWM = (rightPWM > 0) ? MIN_PWM : -MIN_PWM;
  }
  
  setLeftMotor(leftPWM);
  setRightMotor(rightPWM);
}

void setLeftMotor(int speed) {
  currentLeftSpeed = speed;
  speed *= LEFT_MOTOR_DIR;  // Yön düzeltmesi
  
  if (speed > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_PWM, abs(speed));
  } 
  else if (speed < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    analogWrite(LEFT_PWM, abs(speed));
  } 
  else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    analogWrite(LEFT_PWM, 0);
  }
}

void setRightMotor(int speed) {
  currentRightSpeed = speed;
  speed *= RIGHT_MOTOR_DIR;  // Yön düzeltmesi
  
  if (speed > 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_PWM, abs(speed));
  } 
  else if (speed < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(RIGHT_PWM, abs(speed));
  } 
  else {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_PWM, 0);
  }
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
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