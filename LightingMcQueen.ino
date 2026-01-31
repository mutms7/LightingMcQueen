// ============================================
// WINTER OLYMPICS HACKATHON - FULL COURSE
// Path: Green (Target) FIRST → Red (Obstacle) SECOND
// Max Points: 63
// ============================================

// ---- PIN DEFINITIONS ----
// Motor Driver (L298N)
#define ENA         5   // PWM
#define IN1         6
#define IN2         7
#define IN3         8
#define IN4         9
#define ENB         10  // PWM

// Ultrasonic
#define TRIG        11
#define ECHO        12

// IR Sensors (line/edge detection)
#define IR_LEFT     A0
#define IR_RIGHT    A1

// Color Sensor (TCS3200)
#define S0          2
#define S1          3
#define S2          4
#define S3          A2
#define COLOR_OUT   A3

// Servos
#define CLAW_SERVO_PIN      A4
#define LAUNCHER_SERVO_PIN  A5

#include <Servo.h>

Servo clawServo;
Servo launcherServo;

// ---- STATE MACHINE ----
enum RobotState {
  // Phase 1: Green Path (Target Shooting)
  START,
  GREEN_TO_TARGET_SHOOT,
  CLIMB_CURVED_RAMP,
  NAVIGATE_TARGET_COLORS,
  REACH_CENTER,
  SHOOT_BALL,
  DESCEND_RAMP,
  
  // Phase 2: Red Path (Obstacle Course)
  PICKUP_BOX_2,
  NAVIGATE_TO_RED_SPLIT,
  DROP_BOX_RED,
  OBSTACLE_COURSE,
  PICKUP_BOX_3,              // Pick up box on return
  
  // Phase 3: Finish
  RETURN_TO_START,
  DONE
};

RobotState currentState = START;

// ---- TIMING ----
unsigned long stateStartTime = 0;
unsigned long totalStartTime = 0;
const unsigned long MAX_TIME = 300000;  // 5 minutes

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(9600);
  
  initMotors();
  initUltrasonic();
  initIRSensors();
  initColorSensor();
  initServos();
  
  clawServo.write(0);       // Claw open
  launcherServo.write(0);   // Launcher ready
  
  Serial.println("Ready - Waiting for start...");
  delay(3000);
  
  totalStartTime = millis();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Safety timeout
  if (millis() - totalStartTime > MAX_TIME) {
    currentState = DONE;
  }
  
  switch (currentState) {
    // ======== PHASE 1: GREEN PATH ========
    case START:
      Serial.println("STATE: START");
      if(readDistance() <= 10){
        pickupBox();
      }
      if(followLineToSplit('R')){ //First junction has red overlay
        turnLeft(100);
        moveForward(100);
        currentState = GREEN_TO_TARGET_SHOOT;
      };
      break;
    case GREEN_TO_TARGET_SHOOT:
      if(followLineToSplit('B')){
        dropBoxAtBlueCircle();
        moveForward(200);
      }
      if(followLineToSplit('P')){ //Either Pause or wait for reupload.
        turnLeft(50);
        moveForward(50);
        currentState = CLIMB_CURVED_RAMP;
      }

      break;
      
    case CLIMB_CURVED_RAMP:
      if (followLineToSplit('B')) {
        Serial.println("Reached top of ramp");
        currentState = NAVIGATE_TARGET_COLORS;
      }
      break;
      
    case WAIT_FOR_REUPLOAD_1:
      stopMotors();
      // Manual intervention point - remove if not using
      // Press button or serial command to continue
      currentState = NAVIGATE_TARGET_COLORS;
      break;
      
    case NAVIGATE_TARGET_COLORS:
      if (navigateTargetByColor()) {
        Serial.println("Reached center (black)");
        currentState = SHOOT_BALL;
        // Change to WAIT_FOR_REUPLOAD_2 if using reupload
      }
      break;
      
    case WAIT_FOR_REUPLOAD_2:
      stopMotors();
      // Manual intervention for shooting code
      currentState = SHOOT_BALL;
      break;
      
    case SHOOT_BALL:
      if (launchBall()) {
        Serial.println("Ball launched!");
        currentState = DESCEND_RAMP;
      }
      break;
      
    case DESCEND_RAMP:
      if (descendRamp()) {
        Serial.println("Back at bottom");
        currentState = PICKUP_BOX_2;
      }
      break;
      
    // ======== PHASE 2: RED PATH ========
    case PICKUP_BOX_2:
      if (pickupBox()) {
        Serial.println("Box 2 picked up");
        currentState = NAVIGATE_TO_RED_SPLIT;
      }
      break;
      
    case NAVIGATE_TO_RED_SPLIT:
      if (followLineToSplit('R')) {  // 'R' for red
        currentState = DROP_BOX_RED;
      }
      break;
      
    case DROP_BOX_RED:
      if (dropBoxAtBlueCircle()) {
        Serial.println("Box dropped - Red unlocked");
        currentState = OBSTACLE_COURSE;
      }
      break;
      
    case OBSTACLE_COURSE:
      if (navigateObstacleCourse()) {
        Serial.println("Obstacle course complete");
        currentState = PICKUP_BOX_3;
      }
      break;
      
    case PICKUP_BOX_3:
      if (pickupBox()) {
        Serial.println("Box 3 picked up");
        currentState = RETURN_TO_START;
      }
      break;
      
    // ======== PHASE 3: FINISH ========
    case RETURN_TO_START:
      if (returnHome()) {
        Serial.println("FINISHED!");
        currentState = DONE;
      }
      break;
      
    case DONE:
      stopMotors();
      // Celebrate
      break;
  }
}

// ============================================
// INITIALIZATION
// ============================================
void initMotors() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void initUltrasonic() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void initIRSensors() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
}

void initColorSensor() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  
  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void initServos() {
  clawServo.attach(CLAW_SERVO_PIN);
  launcherServo.attach(LAUNCHER_SERVO_PIN);
}

// ============================================
// MOTOR CONTROL
// ============================================
void setMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(ENA, constrain(leftSpeed, 0, 255));
  
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

void moveForward(int speed)  { setMotors(speed, speed); }
void moveBackward(int speed) { setMotors(-speed, -speed); }
void turnLeft(int speed)     { setMotors(-speed, speed); }
void turnRight(int speed)    { setMotors(speed, -speed); }
void stopMotors()            { setMotors(0, 0); }

// ============================================
// SENSOR READING
// ============================================
long readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH, 30000);
  return duration * 0.034 / 2;  // cm
}

int readColorFrequency(char color) {
  switch (color) {
    case 'R':
      digitalWrite(S2, LOW);
      digitalWrite(S3, LOW);
      break;
    case 'G':
      digitalWrite(S2, HIGH);
      digitalWrite(S3, HIGH);
      break;
    case 'B':
      digitalWrite(S2, LOW);
      digitalWrite(S3, HIGH);
      break;
  }
  return pulseIn(COLOR_OUT, LOW);
}

char detectColor() {
  int r = readColorFrequency('R');
  int g = readColorFrequency('G');
  int b = readColorFrequency('B');
  
  // TODO: Calibrate these thresholds at hackathon
  // Lower frequency = more of that color detected
  
  if (r < 30 && g < 30 && b < 30) return 'K';  // Black
  if (r < g && r < b) return 'R';               // Red
  if (g < r && g < b) return 'G';               // Green
  if (b < r && b < g) return 'B';               // Blue
  
  return 'X';  // Unknown
}

bool irLeft()  { return digitalRead(IR_LEFT) == LOW; }   // Adjust for your sensor
bool irRight() { return digitalRead(IR_RIGHT) == LOW; }

// ============================================
// CLAW CONTROL
// ============================================
void openClaw()  { clawServo.write(0);   delay(500); }
void closeClaw() { clawServo.write(90);  delay(500); }

// ============================================
// ACTION FUNCTIONS - PHASE 1
// ============================================
bool pickupBox() {
  // Approach box using ultrasonic
  long dist = readDistance();
  
  if (dist > 10) {
    moveForward(150);
    return false;
  }
  
  stopMotors();
  closeClaw();
  return true;
}

bool followLineToSplit(char targetPath) {
  // Line following until we detect the split color
  char color = detectColor();
  
  // Check if we've reached the target split
  if (targetPath == color {
    stopMotors();
    return true;
  }
  
  // Basic line following with IR
  bool left = irLeft();
  bool right = irRight();
  
  if (left && right) {
    moveForward(150);
  } else if (left && !right) {
    setMotors(100, 180);  // Veer right
  } else if (!left && right) {
    setMotors(180, 100);  // Veer left
  } else {
    moveForward(120);     // Lost line - go slow
  }
  
  return false;
}

bool dropBoxAtBlueCircle() {
  // Look for blue circle
  char color = detectColor();
  
  if (color == 'B') {
    stopMotors();
    openClaw();
    moveBackward(150);
    delay(300);
    stopMotors();
    return true;
  }
  
  moveForward(100);
  return false;
}

bool climbRamp(bool curved) {
  // TODO: Implement ramp climbing
  // Use ultrasonic for edge detection
  // If curved: follow the curve using IR/color
  
  static unsigned long climbStart = 0;
  if (climbStart == 0) climbStart = millis();
  
  // Placeholder: drive forward for set time
  // Curved ramp needs steering
  if (curved) {
    setMotors(200, 180);  // Slight curve
  } else {
    moveForward(200);
  }
  
  // TODO: Detect when at top (level surface, color change, etc.)
  if (millis() - climbStart > 3000) {
    stopMotors();
    climbStart = 0;
    return true;
  }
  
  return false;
}

bool navigateTargetByColor() {
  // Navigate: Blue(outer) → Red → Green → Black(center)
  // Goal: Get all wheels off previous ring
  
  char color = detectColor();
  
  if (color == 'K') {  // Black = center
    stopMotors();
    return true;
  }
  
  // Strategy: Always move toward darker/inner colors
  // Simple approach: spiral inward
  
  switch (color) {
    case 'B':  // On blue - move inward
      setMotors(150, 120);
      break;
    case 'R':  // On red - continue inward
      setMotors(140, 110);
      break;
    case 'G':  // On green - almost there
      setMotors(130, 100);
      break;
    default:
      moveForward(100);
  }
  
  return false;
}

bool launchBall() {
  // Fire the launcher servo
  stopMotors();
  delay(200);
  
  launcherServo.write(180);  // Fire!
  delay(500);
  launcherServo.write(0);    // Reset
  delay(300);
  
  return true;
}

bool descendRamp() {
  // Go back down - careful not to fall off
  static unsigned long descendStart = 0;
  if (descendStart == 0) descendStart = millis();
  
  moveBackward(150);  // Slow descent
  
  // TODO: Detect bottom (color change, level surface)
  if (millis() - descendStart > 3000) {
    stopMotors();
    descendStart = 0;
    return true;
  }
  
  return false;
}

// ============================================
// ACTION FUNCTIONS - PHASE 2
// ============================================
bool navigateObstacleCourse() {
  // Follow red path, avoid black obstacles
  // Use ultrasonic for obstacle detection
  
  long dist = readDistance();
  char color = detectColor();
  
  // Check if course complete (back at main area)
  // TODO: Define completion condition
  static int turnCount = 0;
  
  if (dist < 15) {
    // Obstacle ahead - decide direction
    stopMotors();
    delay(100);
    
    // Simple avoidance: alternate turns
    if (turnCount % 2 == 0) {
      turnLeft(150);
    } else {
      turnRight(150);
    }
    delay(400);
    turnCount++;
    return false;
  }
  
  // Follow the red path
  if (color == 'R') {
    moveForward(180);  // Fast on clear path
  } else {
    moveForward(120);  // Slower when unsure
  }
  
  // TODO: Detect course completion
  // Placeholder: time-based
  static unsigned long obstacleStart = 0;
  if (obstacleStart == 0) obstacleStart = millis();
  
  if (millis() - obstacleStart > 10000) {
    stopMotors();
    obstacleStart = 0;
    turnCount = 0;
    return true;
  }
  
  return false;
}

// ============================================
// ACTION FUNCTIONS - PHASE 3
// ============================================
bool returnHome() {
  // Navigate back to START position
  // Follow black path
  
  char color = detectColor();
  
  // Use line following to get back
  bool left = irLeft();
  bool right = irRight();
  
  if (left && right) {
    moveForward(150);
  } else if (left) {
    setMotors(100, 160);
  } else if (right) {
    setMotors(160, 100);
  } else {
    moveForward(100);
  }
  
  // TODO: Detect START position
  // Could use color, ultrasonic, or time
  static unsigned long returnStart = 0;
  if (returnStart == 0) returnStart = millis();
  
  if (millis() - returnStart > 5000) {
    stopMotors();
    returnStart = 0;
    return true;
  }
  
  return false;
}