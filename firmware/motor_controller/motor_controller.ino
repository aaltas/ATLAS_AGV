/**
 * Atlas AGV - Motor Controller
 * ROS 2 Serial Communication
 * 
 * Serial Protocol:
 * Arduino -> Jetson: ODO,x,y,theta,vl,vr,encL,encR
 * Jetson -> Arduino: CMD,left_speed,right_speed
 */

// ============================================
// PIN DEFINITIONS
// ============================================
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

const int LEFT_ENCODER_A = A0;
const int LEFT_ENCODER_B = A1;
const int RIGHT_ENCODER_A = A2;
const int RIGHT_ENCODER_B = A3;

// ============================================
// CALIBRATED PARAMETERS
// ============================================
const float COUNTS_PER_WHEEL_REV = 378.0;
const float WHEEL_DIAMETER = 0.080;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float METERS_PER_COUNT = WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV;
const float WHEEL_BASE = 0.21;

// Direction corrections
const int LEFT_ENCODER_DIR = -1;
const int RIGHT_ENCODER_DIR = 1;
const int LEFT_MOTOR_DIR = -1;
const int RIGHT_MOTOR_DIR = -1;

// ============================================
// GLOBAL VARIABLES
// ============================================
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile uint8_t prevLeftA = 0;
volatile uint8_t prevRightA = 0;

long prevLeftCount = 0;
long prevRightCount = 0;

unsigned long prevTime = 0;
const unsigned long UPDATE_INTERVAL = 50; // 20Hz

float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;
float leftWheelVelocity = 0.0;
float rightWheelVelocity = 0.0;

int currentLeftSpeed = 0;
int currentRightSpeed = 0;

String inputString = "";
boolean stringComplete = false;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // Motor pins
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
  
  // Encoder pins
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  
  // Pin Change Interrupt
  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8);
  PCMSK1 |= (1 << PCINT10);
  
  prevLeftA = digitalRead(LEFT_ENCODER_A);
  prevRightA = digitalRead(RIGHT_ENCODER_A);
  
  stopMotors();
  
  Serial.println("READY");
  prevTime = millis();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  unsigned long currentTime = millis();
  
  // Odometry update and publish at 20Hz
  if (currentTime - prevTime >= UPDATE_INTERVAL) {
    float deltaTime = (currentTime - prevTime) / 1000.0;
    updateOdometry(deltaTime);
    publishOdometry();
    prevTime = currentTime;
  }
  
  // Handle serial commands
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
// COMMAND PARSER
// ============================================
void parseCommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("CMD,")) {
    // CMD,left_speed,right_speed
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    
    if (secondComma > 0) {
      int leftSpeed = cmd.substring(firstComma + 1, secondComma).toInt();
      int rightSpeed = cmd.substring(secondComma + 1).toInt();
      
      setLeftMotors(leftSpeed);
      setRightMotors(rightSpeed);
    }
  }
  else if (cmd == "STOP") {
    stopMotors();
  }
  else if (cmd == "RESET") {
    resetOdometry();
  }
  else if (cmd == "STATUS") {
    Serial.print("STATUS,");
    Serial.print(currentLeftSpeed);
    Serial.print(",");
    Serial.println(currentRightSpeed);
  }
}

// ============================================
// ODOMETRY
// ============================================
void updateOdometry(float deltaTime) {
  noInterrupts();
  long leftCount = leftEncoderCount;
  long rightCount = rightEncoderCount;
  interrupts();
  
  long leftDelta = leftCount - prevLeftCount;
  long rightDelta = rightCount - prevRightCount;
  
  float leftDistance = leftDelta * METERS_PER_COUNT;
  float rightDistance = rightDelta * METERS_PER_COUNT;
  
  float centerDistance = (leftDistance + rightDistance) / 2.0;
  float deltaTheta = (rightDistance - leftDistance) / WHEEL_BASE;
  
  robotX += centerDistance * cos(robotTheta + deltaTheta / 2.0);
  robotY += centerDistance * sin(robotTheta + deltaTheta / 2.0);
  robotTheta += deltaTheta;
  
  // Normalize angle
  while (robotTheta > PI) robotTheta -= 2 * PI;
  while (robotTheta < -PI) robotTheta += 2 * PI;
  
  // Calculate velocities
  if (deltaTime > 0) {
    leftWheelVelocity = leftDistance / deltaTime;
    rightWheelVelocity = rightDistance / deltaTime;
  }
  
  prevLeftCount = leftCount;
  prevRightCount = rightCount;
}

void publishOdometry() {
  Serial.print("ODO,");
  Serial.print(robotX, 4);
  Serial.print(",");
  Serial.print(robotY, 4);
  Serial.print(",");
  Serial.print(robotTheta, 4);
  Serial.print(",");
  Serial.print(leftWheelVelocity, 4);
  Serial.print(",");
  Serial.print(rightWheelVelocity, 4);
  Serial.print(",");
  Serial.print(leftEncoderCount);
  Serial.print(",");
  Serial.println(rightEncoderCount);
}

void resetOdometry() {
  noInterrupts();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  interrupts();
  
  prevLeftCount = 0;
  prevRightCount = 0;
  robotX = 0.0;
  robotY = 0.0;
  robotTheta = 0.0;
  
  Serial.println("ACK");
}

// ============================================
// MOTOR CONTROL
// ============================================
void setLeftMotors(int speed) {
  speed = constrain(speed, -255, 255);
  currentLeftSpeed = speed;
  speed *= LEFT_MOTOR_DIR;
  
  if (speed > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(LEFT_IN3, HIGH);
    digitalWrite(LEFT_IN4, LOW);
    analogWrite(LEFT_ENA, abs(speed));
    analogWrite(LEFT_ENB, abs(speed));
  } else if (speed < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    digitalWrite(LEFT_IN3, LOW);
    digitalWrite(LEFT_IN4, HIGH);
    analogWrite(LEFT_ENA, abs(speed));
    analogWrite(LEFT_ENB, abs(speed));
  } else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(LEFT_IN3, LOW);
    digitalWrite(LEFT_IN4, LOW);
    analogWrite(LEFT_ENA, 0);
    analogWrite(LEFT_ENB, 0);
  }
}

void setRightMotors(int speed) {
  speed = constrain(speed, -255, 255);
  currentRightSpeed = speed;
  speed *= RIGHT_MOTOR_DIR;
  
  if (speed > 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    digitalWrite(RIGHT_IN3, HIGH);
    digitalWrite(RIGHT_IN4, LOW);
    analogWrite(RIGHT_ENA, abs(speed));
    analogWrite(RIGHT_ENB, abs(speed));
  } else if (speed < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    digitalWrite(RIGHT_IN3, LOW);
    digitalWrite(RIGHT_IN4, HIGH);
    analogWrite(RIGHT_ENA, abs(speed));
    analogWrite(RIGHT_ENB, abs(speed));
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

// ============================================
// ENCODER INTERRUPT
// ============================================
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
