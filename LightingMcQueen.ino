// ============================================
// WINTER OLYMPICS HACKATHON - FULL COURSE
// ============================================
// CONFIGURATION SECTION - EDIT THESE VALUES!
// ============================================

// ================== PIN DEFINITIONS ==================
// Motor Driver (L298N)
#define PIN_ENA             5       // PWM Left Motor
#define PIN_IN1             6       // Left Motor Direction A
#define PIN_IN2             7       // Left Motor Direction B
#define PIN_IN3             8       // Right Motor Direction A
#define PIN_IN4             9       // Right Motor Direction B
#define PIN_ENB             10      // PWM Right Motor

// Ultrasonic Sensor (HC-SR04)
#define PIN_TRIG            11
#define PIN_ECHO            12

// IR Line Sensors
#define PIN_IR_LEFT         A0
#define PIN_IR_RIGHT        A1

// Color Sensor (TCS3200)
#define PIN_COLOR_S0        2
#define PIN_COLOR_S1        3
#define PIN_COLOR_S2        4
#define PIN_COLOR_S3        A2
#define PIN_COLOR_OUT       A3

// Servos
#define PIN_CLAW_SERVO      A4
#define PIN_LAUNCHER_SERVO  A5

// ================== PATH CONFIGURATION ==================
// CRITICAL: Set these based on track layout at hackathon!
// 1 = turn LEFT to enter path, -1 = turn RIGHT to enter path
const int GREEN_PATH_DIRECTION  = 1;    // 1=left, -1=right
const int RED_PATH_DIRECTION    = -1;   // 1=left, -1=right

// Turn angles for entering paths (degrees)
const int PATH_ENTRY_ANGLE      = 45;   // Angle to turn onto path
const int PATH_ENTRY_FORWARD    = 300;  // Distance to move after turn (ms)

// Which ramp to take on green path
const bool USE_CURVED_RAMP      = true; // true=curved (4pts), false=straight (2pts)

// ================== MOTOR SPEEDS ==================
const int SPEED_STOP        = 0;
const int SPEED_CRAWL       = 80;     // Very slow, precise movements
const int SPEED_SLOW        = 120;    // Careful navigation
const int SPEED_BASE        = 150;    // Normal operation
const int SPEED_FAST        = 200;    // Straight runs
const int SPEED_MAX         = 255;    // Full power (ramps)
const int SPEED_TURN        = 130;    // In-place rotation

// Arc movement differential (for circling target)
const int ARC_DIFFERENTIAL  = 40;     // Speed difference between wheels

// Motor correction (if robot drifts, adjust these)
const float LEFT_MOTOR_MULT   = 1.0;
const float RIGHT_MOTOR_MULT  = 1.0;

// ================== TIMING CONSTANTS ==================
// Turn calibration - MEASURE CAREFULLY!
const int MS_PER_90_DEG     = 500;    // Milliseconds for 90° turn

// Step movement durations (ms)
const int STEP_FORWARD_MS   = 150;
const int STEP_BACKWARD_MS  = 150;
const int STEP_INWARD_MS    = 200;

// State timeouts (ms)
const unsigned long TIMEOUT_PICKUP      = 5000;
const unsigned long TIMEOUT_SPLIT       = 10000;
const unsigned long TIMEOUT_DROP        = 5000;
const unsigned long TIMEOUT_CLIMB       = 6000;
const unsigned long TIMEOUT_NAV_ARC     = 5000;
const unsigned long TIMEOUT_NAV_ENTRY   = 5000;
const unsigned long TIMEOUT_OBSTACLE    = 30000;
const unsigned long TIMEOUT_RETURN      = 15000;
const unsigned long TIMEOUT_LINE_LOST   = 2000;
const unsigned long TOTAL_TIME_LIMIT    = 300000;  // 5 minutes

// Delays (ms)
const int DELAY_AFTER_STOP      = 50;
const int DELAY_AFTER_TURN      = 50;
const int DELAY_SERVO_MOVE      = 15;
const int DELAY_COLOR_READ      = 5;
const int DELAY_STABILIZE       = 200;

// ================== SERVO POSITIONS ==================
const int CLAW_OPEN         = 10;
const int CLAW_CLOSED       = 80;
const int LAUNCHER_READY    = 0;
const int LAUNCHER_FIRE     = 150;

// ================== DISTANCE THRESHOLDS (cm) ==================
const int DIST_BOX_PICKUP   = 8;
const int DIST_BOX_DETECT   = 25;
const int DIST_BOX_SEARCH   = 40;
const int DIST_OBSTACLE     = 15;
const int DIST_WALL_CLOSE   = 30;
const int DIST_NO_READING   = 999;

// ================== COLOR SENSOR CALIBRATION ==================
const bool COLOR_S0_STATE   = HIGH;
const bool COLOR_S1_STATE   = LOW;   // = 20% scaling
const int COLOR_SAMPLES     = 5;

// Raw frequency thresholds - CALIBRATE AT HACKATHON!
// Lower value = more of that color detected
const int THRESH_BLACK_R    = 50;
const int THRESH_BLACK_G    = 50;
const int THRESH_BLACK_B    = 50;
const int COLOR_DIFF_MIN    = 20;
const int PURPLE_RB_DIFF    = 30;
const unsigned long COLOR_PULSE_TIMEOUT = 10000;

// ================== IR SENSOR CALIBRATION ==================
// true = LOW means line detected, false = HIGH means line detected
const bool IR_ACTIVE_LOW    = true;

// ================== NAVIGATION PARAMETERS ==================
const int NAV_SCAN_STEP_DEG = 30;
const int NAV_FULL_ROTATION = 360;
const int AVOID_TURN_DEG    = 45;
const int AVOID_FORWARD_MS  = 400;
const int SHARP_TURN_SCAN   = 45;

// ================== RAMP PARAMETERS ==================
const int RAMP_CURVE_DIFF   = 30;
const int RAMP_ASCEND_TIME  = 2500;
const int RAMP_DESCEND_TIME = 2000;
const int RAMP_TOP_READINGS = 3;

// ================== DEBUG OPTIONS ==================
const bool DEBUG_MOTORS     = false;
const bool DEBUG_SENSORS    = false;
const bool DEBUG_COLOR      = true;
const bool DEBUG_STATE      = true;
const bool DEBUG_NAV        = true;
const bool DEBUG_LINE       = true;

// ============================================
// END OF CONFIGURATION SECTION
// ============================================

#include <Servo.h>

Servo clawServo;
Servo launcherServo;

// ---- MAIN STATE MACHINE ----
enum RobotState {
  // Phase 1: Start and Green Path
  STATE_START,
  STATE_FOLLOW_TO_FIRST_JUNCTION,
  STATE_PICKUP_BOX_1,
  STATE_NAVIGATE_TO_GREEN,
  STATE_DROP_BOX_GREEN,
  STATE_CLIMB_RAMP,
  STATE_NAVIGATE_TARGET,
  STATE_SHOOT_BALL,
  STATE_DESCEND_RAMP,
  
  // Phase 2: Red Path
  STATE_RETURN_TO_JUNCTION,
  STATE_PICKUP_BOX_2,
  STATE_NAVIGATE_TO_RED,
  STATE_DROP_BOX_RED,
  STATE_OBSTACLE_COURSE,
  STATE_PICKUP_BOX_3,
  
  // Phase 3: Finish
  STATE_RETURN_HOME,
  STATE_DONE
};

// ---- SUB-STATE MACHINES ----
enum LineFollowState {
  LINE_FOLLOWING,
  LINE_LOST_SEARCHING,
  LINE_FOUND_TARGET
};

enum TargetNavState {
  NAV_INIT,
  NAV_FIND_WALL,
  NAV_IDENTIFY_ZONE,
  NAV_ARC_AROUND,
  NAV_STEP_INWARD,
  NAV_CORRECTION,
  NAV_PREPARE_ENTRY,
  NAV_ALIGN_TO_WALL,
  NAV_ENTER_BLACK,
  NAV_COMPLETE
};

enum PickupState {
  PICKUP_INIT,
  PICKUP_SCAN_FOR_BOX,
  PICKUP_TURN_TO_BOX,
  PICKUP_APPROACH,
  PICKUP_FINAL_APPROACH,
  PICKUP_GRAB,
  PICKUP_VERIFY,
  PICKUP_DONE
};

enum DropState {
  DROP_SEARCH_BLUE,
  DROP_CENTER_ON_BLUE,
  DROP_RELEASE,
  DROP_BACKUP,
  DROP_DONE
};

enum ClimbState {
  CLIMB_START,
  CLIMB_ASCENDING,
  CLIMB_DETECT_TOP,
  CLIMB_DONE
};

enum DescendState {
  DESCEND_TURN_AROUND,
  DESCEND_GO_DOWN,
  DESCEND_DETECT_BOTTOM,
  DESCEND_DONE
};

enum ObstacleState {
  OBS_START,
  OBS_FOLLOW_PATH,
  OBS_AVOID_OBSTACLE,
  OBS_SHARP_TURN,
  OBS_CHECK_COMPLETE,
  OBS_DONE
};

enum LaunchState {
  LAUNCH_PREPARE,
  LAUNCH_FIRE,
  LAUNCH_RESET,
  LAUNCH_DONE
};

enum SplitNavState {
  SPLIT_FOLLOW_LINE,
  SPLIT_DETECT_COLOR,
  SPLIT_TURN_TO_PATH,
  SPLIT_ENTER_PATH,
  SPLIT_DONE
};

// ---- GLOBAL STATE VARIABLES ----
RobotState currentState = STATE_START;
TargetNavState navState = NAV_INIT;
PickupState pickupState = PICKUP_INIT;
DropState dropState = DROP_SEARCH_BLUE;
ClimbState climbState = CLIMB_START;
DescendState descendState = DESCEND_TURN_AROUND;
ObstacleState obsState = OBS_START;
LaunchState launchState = LAUNCH_PREPARE;
SplitNavState splitState = SPLIT_FOLLOW_LINE;
LineFollowState lineState = LINE_FOLLOWING;

// ---- TIMING VARIABLES ----
unsigned long stateStartTime = 0;
unsigned long totalStartTime = 0;
unsigned long actionTimer = 0;
unsigned long lineLostTime = 0;

// ---- TARGET NAV VARIABLES ----
int wallDirection = -1;
bool wallFound = false;
char currentZone = 'X';
unsigned long arcStartTime = 0;
int arcDirection = 1;

// ---- TRACKING VARIABLES ----
bool hasBox = false;
int obstacleCount = 0;
int avoidDirection = 1;
char targetPathColor = 'X';
int targetPathDirection = 0;

// ---- BOX DETECTION VARIABLES ----
int boxDirection = 0;  // -1 = left, 1 = right, 0 = unknown

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
  
  clawServo.write(CLAW_OPEN);
  launcherServo.write(LAUNCHER_READY);
  
  if (DEBUG_STATE) {
    Serial.println(F("================================="));
    Serial.println(F("  WINTER OLYMPICS ROBOT v2.0"));
    Serial.println(F("================================="));
    Serial.println(F("Configuration:"));
    Serial.print(F("  Green path: "));
    Serial.println(GREEN_PATH_DIRECTION > 0 ? "LEFT" : "RIGHT");
    Serial.print(F("  Red path: "));
    Serial.println(RED_PATH_DIRECTION > 0 ? "LEFT" : "RIGHT");
    Serial.print(F("  Ramp: "));
    Serial.println(USE_CURVED_RAMP ? "CURVED" : "STRAIGHT");
    Serial.println(F("---------------------------------"));
    Serial.println(F("Starting in 3 seconds..."));
  }
  
  delay(3000);
  
  totalStartTime = millis();
  stateStartTime = millis();
}

// ============================================
// INITIALIZATION FUNCTIONS
// ============================================
void initMotors() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  stopMotors();
  if (DEBUG_SENSORS) Serial.println(F("Motors OK"));
}

void initUltrasonic() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  if (DEBUG_SENSORS) Serial.println(F("Ultrasonic OK"));
}

void initIRSensors() {
  pinMode(PIN_IR_LEFT, INPUT);
  pinMode(PIN_IR_RIGHT, INPUT);
  if (DEBUG_SENSORS) Serial.println(F("IR Sensors OK"));
}

void initColorSensor() {
  pinMode(PIN_COLOR_S0, OUTPUT);
  pinMode(PIN_COLOR_S1, OUTPUT);
  pinMode(PIN_COLOR_S2, OUTPUT);
  pinMode(PIN_COLOR_S3, OUTPUT);
  pinMode(PIN_COLOR_OUT, INPUT);
  
  digitalWrite(PIN_COLOR_S0, COLOR_S0_STATE);
  digitalWrite(PIN_COLOR_S1, COLOR_S1_STATE);
  if (DEBUG_SENSORS) Serial.println(F("Color Sensor OK"));
}

void initServos() {
  clawServo.attach(PIN_CLAW_SERVO);
  launcherServo.attach(PIN_LAUNCHER_SERVO);
  if (DEBUG_SENSORS) Serial.println(F("Servos OK"));
}

// ============================================
// MOTOR CONTROL FUNCTIONS
// ============================================
void setMotors(int leftSpeed, int rightSpeed) {
  // Apply motor correction multipliers
  leftSpeed = (int)(leftSpeed * LEFT_MOTOR_MULT);
  rightSpeed = (int)(rightSpeed * RIGHT_MOTOR_MULT);
  
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PIN_ENA, constrain(leftSpeed, 0, 255));
  
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PIN_ENB, constrain(rightSpeed, 0, 255));
  
  if (DEBUG_MOTORS) {
    Serial.print(F("M: L="));
    Serial.print(leftSpeed);
    Serial.print(F(" R="));
    Serial.println(rightSpeed);
  }
}

void moveForward(int speed) {
  setMotors(speed, speed);
}

void moveBackward(int speed) {
  setMotors(-speed, -speed);
}

void turnLeft(int speed) {
  setMotors(-speed, speed);
}

void turnRight(int speed) {
  setMotors(speed, -speed);
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

// Turn by direction: positive = right, negative = left
void turnByDirection(int direction, int speed) {
  if (direction > 0) {
    turnRight(speed);
  } else {
    turnLeft(speed);
  }
}

void turnDegrees(int degrees, int direction) {
  // direction: 1 = right, -1 = left
  int duration = (abs(degrees) * MS_PER_90_DEG) / 90;
  turnByDirection(direction, SPEED_TURN);
  delay(duration);
  stopMotors();
  delay(DELAY_AFTER_TURN);
}

void turnLeftDegrees(int degrees) {
  turnDegrees(degrees, -1);
}

void turnRightDegrees(int degrees) {
  turnDegrees(degrees, 1);
}

void moveArcClockwise(int speed) {
  setMotors(speed, speed - ARC_DIFFERENTIAL);
}

void moveArcCounterClockwise(int speed) {
  setMotors(speed - ARC_DIFFERENTIAL, speed);
}

void stepForward(int duration) {
  moveForward(SPEED_SLOW);
  delay(duration);
  stopMotors();
  delay(DELAY_AFTER_STOP);
}

void stepBackward(int duration) {
  moveBackward(SPEED_SLOW);
  delay(duration);
  stopMotors();
  delay(DELAY_AFTER_STOP);
}

// ============================================
// SENSOR READING FUNCTIONS
// ============================================
long readDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  
  long duration = pulseIn(PIN_ECHO, HIGH, 30000);
  if (duration == 0) return DIST_NO_READING;
  
  long distance = duration * 0.034 / 2;
  
  if (DEBUG_SENSORS) {
    Serial.print(F("Dist: "));
    Serial.print(distance);
    Serial.println(F("cm"));
  }
  
  return distance;
}

int readColorFrequency(char color) {
  switch (color) {
    case 'R':
      digitalWrite(PIN_COLOR_S2, LOW);
      digitalWrite(PIN_COLOR_S3, LOW);
      break;
    case 'G':
      digitalWrite(PIN_COLOR_S2, HIGH);
      digitalWrite(PIN_COLOR_S3, HIGH);
      break;
    case 'B':
      digitalWrite(PIN_COLOR_S2, LOW);
      digitalWrite(PIN_COLOR_S3, HIGH);
      break;
  }
  delayMicroseconds(100);
  return pulseIn(PIN_COLOR_OUT, LOW, COLOR_PULSE_TIMEOUT);
}

char detectColor() {
  int r = readColorFrequency('R');
  int g = readColorFrequency('G');
  int b = readColorFrequency('B');
  
  if (r == 0 && g == 0 && b == 0) return 'X';
  
  // Black detection
  if (r < THRESH_BLACK_R && g < THRESH_BLACK_G && b < THRESH_BLACK_B) {
    return 'K';
  }
  
  // Purple detection (reupload points)
  if (r < g && b < g && abs(r - b) < PURPLE_RB_DIFF) {
    return 'P';
  }
  
  // Dominant color detection
  if (r < g - COLOR_DIFF_MIN && r < b - COLOR_DIFF_MIN) return 'R';
  if (g < r - COLOR_DIFF_MIN && g < b - COLOR_DIFF_MIN) return 'G';
  if (b < r - COLOR_DIFF_MIN && b < g - COLOR_DIFF_MIN) return 'B';
  
  // Fallback
  if (r <= g && r <= b) return 'R';
  if (g <= r && g <= b) return 'G';
  return 'B';
}

char detectColorStable() {
  int rSum = 0, gSum = 0, bSum = 0;
  
  for (int i = 0; i < COLOR_SAMPLES; i++) {
    rSum += readColorFrequency('R');
    gSum += readColorFrequency('G');
    bSum += readColorFrequency('B');
    delay(DELAY_COLOR_READ);
  }
  
  int r = rSum / COLOR_SAMPLES;
  int g = gSum / COLOR_SAMPLES;
  int b = bSum / COLOR_SAMPLES;
  
  if (DEBUG_COLOR) {
    Serial.print(F("RGB: "));
    Serial.print(r); Serial.print(F(","));
    Serial.print(g); Serial.print(F(","));
    Serial.println(b);
  }
  
  if (r == 0 && g == 0 && b == 0) return 'X';
  
  if (r < THRESH_BLACK_R && g < THRESH_BLACK_G && b < THRESH_BLACK_B) return 'K';
  
  if (r < g && b < g && abs(r - b) < PURPLE_RB_DIFF) return 'P';
  
  if (r < g - COLOR_DIFF_MIN && r < b - COLOR_DIFF_MIN) return 'R';
  if (g < r - COLOR_DIFF_MIN && g < b - COLOR_DIFF_MIN) return 'G';
  if (b < r - COLOR_DIFF_MIN && b < g - COLOR_DIFF_MIN) return 'B';
  
  if (r <= g && r <= b) return 'R';
  if (g <= r && g <= b) return 'G';
  return 'B';
}

bool irLeft() {
  bool reading = digitalRead(PIN_IR_LEFT);
  return IR_ACTIVE_LOW ? (reading == LOW) : (reading == HIGH);
}

bool irRight() {
  bool reading = digitalRead(PIN_IR_RIGHT);
  return IR_ACTIVE_LOW ? (reading == LOW) : (reading == HIGH);
}

// ============================================
// CLAW CONTROL
// ============================================
void openClaw() {
  int current = clawServo.read();
  if (current > CLAW_OPEN) {
    for (int pos = current; pos > CLAW_OPEN; pos -= 2) {
      clawServo.write(pos);
      delay(DELAY_SERVO_MOVE);
    }
  }
  clawServo.write(CLAW_OPEN);
  delay(DELAY_STABILIZE);
}

void closeClaw() {
  int current = clawServo.read();
  if (current < CLAW_CLOSED) {
    for (int pos = current; pos < CLAW_CLOSED; pos += 2) {
      clawServo.write(pos);
      delay(DELAY_SERVO_MOVE);
    }
  }
  clawServo.write(CLAW_CLOSED);
  delay(DELAY_STABILIZE);
}

// ============================================
// LINE FOLLOWING FUNCTION
// Returns: 'K' = on black line
//          'G','R','B','P' = detected color marker
//          'L' = lost line
//          'F' = following normally
// ============================================
char followBlackLine() {
  bool left = irLeft();
  bool right = irRight();
  char color = detectColor();
  
  // Check for color markers (not black)
  if (color == 'G' || color == 'R' || color == 'B' || color == 'P') {
    stopMotors();
    if (DEBUG_LINE) {
      Serial.print(F("Line: Color marker: "));
      Serial.println(color);
    }
    return color;
  }
  
  // Line following logic
  if (left && right) {
    // Both sensors on line - go straight
    moveForward(SPEED_BASE);
    lineState = LINE_FOLLOWING;
    return 'F';
  } else if (left && !right) {
    // Drifting right - veer left
    setMotors(SPEED_SLOW, SPEED_FAST);
    lineState = LINE_FOLLOWING;
    return 'F';
  } else if (!left && right) {
    // Drifting left - veer right
    setMotors(SPEED_FAST, SPEED_SLOW);
    lineState = LINE_FOLLOWING;
    return 'F';
  } else {
    // Lost line
    if (lineState != LINE_LOST_SEARCHING) {
      lineLostTime = millis();
      lineState = LINE_LOST_SEARCHING;
      if (DEBUG_LINE) Serial.println(F("Line: LOST"));
    }
    
    // Try to recover
    if (millis() - lineLostTime < TIMEOUT_LINE_LOST) {
      // Slow forward while searching
      moveForward(SPEED_CRAWL);
      return 'F';
    } else {
      stopMotors();
      return 'L';  // Lost for too long
    }
  }
}

// ============================================
// SCAN FOR BOX BESIDE PATH
// Scans left and right to find box
// Returns: -1 = box on left, 1 = box on right, 0 = not found
// ============================================
int scanForBox() {
  stopMotors();
  delay(100);
  
  long distFront = readDistance();
  
  // Check left
  turnLeftDegrees(45);
  delay(100);
  long distLeft = readDistance();
  turnRightDegrees(45);  // Back to center
  
  // Check right
  turnRightDegrees(45);
  delay(100);
  long distRight = readDistance();
  turnLeftDegrees(45);  // Back to center
  
  if (DEBUG_NAV) {
    Serial.print(F("Box scan - L:"));
    Serial.print(distLeft);
    Serial.print(F(" F:"));
    Serial.print(distFront);
    Serial.print(F(" R:"));
    Serial.println(distRight);
  }
  
  // Find closest box
  if (distLeft < DIST_BOX_SEARCH && distLeft < distRight && distLeft < distFront) {
    return -1;  // Box on left
  } else if (distRight < DIST_BOX_SEARCH && distRight < distLeft && distRight < distFront) {
    return 1;   // Box on right
  } else if (distFront < DIST_BOX_SEARCH) {
    return 0;   // Box in front (edge case)
  }
  
  return 0;  // Not found
}

// ============================================
// BOX PICKUP FUNCTION
// Handles finding and picking up box beside the path
// ============================================
bool pickupBox() {
  static unsigned long pickupTimer = 0;
  static int scanAttempts = 0;
  
  long dist;
  
  switch (pickupState) {
    case PICKUP_INIT:
      if (DEBUG_STATE) Serial.println(F("Pickup: Starting"));
      openClaw();
      scanAttempts = 0;
      boxDirection = 0;
      pickupState = PICKUP_SCAN_FOR_BOX;
      pickupTimer = millis();
      break;
      
    case PICKUP_SCAN_FOR_BOX:
      boxDirection = scanForBox();
      
      if (boxDirection != 0 || scanAttempts > 2) {
        if (boxDirection != 0) {
          if (DEBUG_STATE) {
            Serial.print(F("Pickup: Box found "));
            Serial.println(boxDirection > 0 ? "RIGHT" : "LEFT");
          }
          pickupState = PICKUP_TURN_TO_BOX;
        } else {
          // No box found, check if something is in front
          dist = readDistance();
          if (dist < DIST_BOX_DETECT) {
            if (DEBUG_STATE) Serial.println(F("Pickup: Box in front"));
            pickupState = PICKUP_APPROACH;
          } else {
            // Move forward and try again
            stepForward(200);
            scanAttempts++;
          }
        }
      }
      
      // Timeout
      if (millis() - pickupTimer > TIMEOUT_PICKUP) {
        if (DEBUG_STATE) Serial.println(F("Pickup: Timeout"));
        pickupState = PICKUP_DONE;
      }
      break;
      
    case PICKUP_TURN_TO_BOX:
      // Turn toward the box
      turnDegrees(60, boxDirection);
      pickupState = PICKUP_APPROACH;
      break;
      
    case PICKUP_APPROACH:
      dist = readDistance();
      
      if (dist <= DIST_BOX_DETECT && dist > DIST_BOX_PICKUP) {
        moveForward(SPEED_SLOW);
      } else if (dist <= DIST_BOX_PICKUP) {
        stopMotors();
        pickupState = PICKUP_FINAL_APPROACH;
        pickupTimer = millis();
      } else {
        // Box too far or lost - search
        moveForward(SPEED_CRAWL);
        if (millis() - pickupTimer > 3000) {
          pickupState = PICKUP_SCAN_FOR_BOX;
          scanAttempts++;
        }
      }
      break;
      
    case PICKUP_FINAL_APPROACH:
      // Move forward slightly to get box in claw
      stepForward(100);
      pickupState = PICKUP_GRAB;
      break;
      
    case PICKUP_GRAB:
      stopMotors();
      closeClaw();
      pickupTimer = millis();
      pickupState = PICKUP_VERIFY;
      break;
      
    case PICKUP_VERIFY:
      if (millis() - pickupTimer > 300) {
        // Back up slightly
        stepBackward(100);
        
        // Turn back to original direction if we turned to get box
        if (boxDirection != 0) {
          turnDegrees(60, -boxDirection);  // Opposite direction
        }
        
        hasBox = true;
        pickupState = PICKUP_DONE;
      }
      break;
      
    case PICKUP_DONE:
      if (DEBUG_STATE) Serial.println(F("Pickup: Complete"));
      pickupState = PICKUP_INIT;  // Reset
      return true;
  }
  
  return false;
}

// ============================================
// NAVIGATE TO PATH SPLIT
// Follows line until target color is found
// ============================================
bool navigateToSplit(char targetColor, int pathDirection) {
  static unsigned long splitTimer = 0;
  char detected;
  
  switch (splitState) {
    case SPLIT_FOLLOW_LINE:
      detected = followBlackLine();
      
      // Check for target color or blue (drop zone)
      if (detected == targetColor || detected == 'B') {
        stopMotors();
        if (DEBUG_STATE) {
          Serial.print(F("Split: Found "));
          Serial.println(detected);
        }
        
        if (detected == 'B') {
          // At blue drop zone
          splitState = SPLIT_DONE;
        } else {
          // At path split
          splitState = SPLIT_DETECT_COLOR;
        }
      }
      
      // Handle lost line
      if (detected == 'L') {
        // Try to recover by searching
        stopMotors();
        searchForLine();
      }
      break;
      
    case SPLIT_DETECT_COLOR:
      // Confirm with stable reading
      detected = detectColorStable();
      if (detected == targetColor) {
        splitState = SPLIT_TURN_TO_PATH;
      } else {
        // False positive, continue following
        splitState = SPLIT_FOLLOW_LINE;
      }
      break;
      
    case SPLIT_TURN_TO_PATH:
      if (DEBUG_STATE) {
        Serial.print(F("Split: Turning "));
        Serial.println(pathDirection > 0 ? "RIGHT" : "LEFT");
      }
      
      turnDegrees(PATH_ENTRY_ANGLE, pathDirection);
      splitTimer = millis();
      splitState = SPLIT_ENTER_PATH;
      break;
      
    case SPLIT_ENTER_PATH:
      // Move forward onto the path
      moveForward(SPEED_BASE);
      if (millis() - splitTimer > PATH_ENTRY_FORWARD) {
        stopMotors();
        splitState = SPLIT_DONE;
      }
      break;
      
    case SPLIT_DONE:
      splitState = SPLIT_FOLLOW_LINE;  // Reset
      return true;
  }
  
  return false;
}

// ============================================
// SEARCH FOR LINE WHEN LOST
// ============================================
void searchForLine() {
  if (DEBUG_LINE) Serial.println(F("Searching for line..."));
  
  // Try turning left
  for (int i = 0; i < 3; i++) {
    turnLeftDegrees(15);
    if (irLeft() || irRight()) {
      if (DEBUG_LINE) Serial.println(F("Line found LEFT"));
      return;
    }
  }
  
  // Return to center and try right
  turnRightDegrees(45);
  
  for (int i = 0; i < 3; i++) {
    turnRightDegrees(15);
    if (irLeft() || irRight()) {
      if (DEBUG_LINE) Serial.println(F("Line found RIGHT"));
      return;
    }
  }
  
  // Return to center
  turnLeftDegrees(45);
  
  // Try moving forward
  stepForward(200);
  
  if (DEBUG_LINE) Serial.println(F("Line search complete"));
}

// ============================================
// DROP BOX AT BLUE CIRCLE
// ============================================
bool dropBoxAtBlue() {
  static unsigned long dropTimer = 0;
  char color;
  
  switch (dropState) {
    case DROP_SEARCH_BLUE:
      color = detectColor();
      
      if (color == 'B') {
        stopMotors();
        if (DEBUG_STATE) Serial.println(F("Drop: Blue found"));
        dropState = DROP_CENTER_ON_BLUE;
        dropTimer = millis();
      } else {
        // Continue forward slowly looking for blue
        moveForward(SPEED_SLOW);
        
        if (millis() - stateStartTime > TIMEOUT_DROP) {
          if (DEBUG_STATE) Serial.println(F("Drop: Timeout, dropping anyway"));
          stopMotors();
          dropState = DROP_RELEASE;
        }
      }
      break;
      
    case DROP_CENTER_ON_BLUE:
      // Move forward a bit to center on blue
      moveForward(SPEED_CRAWL);
      if (millis() - dropTimer > 150) {
        stopMotors();
        dropState = DROP_RELEASE;
      }
      break;
      
    case DROP_RELEASE:
      if (DEBUG_STATE) Serial.println(F("Drop: Releasing"));
      openClaw();
      hasBox = false;
      dropTimer = millis();
      dropState = DROP_BACKUP;
      break;
      
    case DROP_BACKUP:
      moveBackward(SPEED_BASE);
      if (millis() - dropTimer > 400) {
        stopMotors();
        dropState = DROP_DONE;
      }
      break;
      
    case DROP_DONE:
      if (DEBUG_STATE) Serial.println(F("Drop: Complete"));
      dropState = DROP_SEARCH_BLUE;  // Reset
      return true;
  }
  
  return false;
}

// ============================================
// RAMP CLIMBING
// ============================================
bool climbRamp(bool curved) {
  static unsigned long climbTimer = 0;
  static int topReadings = 0;
  char color;
  
  switch (climbState) {
    case CLIMB_START:
      if (DEBUG_STATE) {
        Serial.print(F("Climb: Starting "));
        Serial.println(curved ? "CURVED" : "STRAIGHT");
      }
      climbTimer = millis();
      topReadings = 0;
      climbState = CLIMB_ASCENDING;
      break;
      
    case CLIMB_ASCENDING:
      if (curved) {
        setMotors(SPEED_FAST, SPEED_FAST - RAMP_CURVE_DIFF);
      } else {
        moveForward(SPEED_FAST);
      }
      
      if (millis() - climbTimer > RAMP_ASCEND_TIME) {
        climbState = CLIMB_DETECT_TOP;
      }
      break;
      
    case CLIMB_DETECT_TOP:
      moveForward(SPEED_SLOW);
      
      color = detectColor();
      
      // Top has target colors (blue outer ring)
      if (color == 'B' || color == 'R' || color == 'G') {
        topReadings++;
        if (topReadings >= RAMP_TOP_READINGS) {
          stopMotors();
          if (DEBUG_STATE) Serial.println(F("Climb: Top reached!"));
          climbState = CLIMB_DONE;
        }
      } else {
        topReadings = 0;
      }
      
      // Timeout
      if (millis() - climbTimer > TIMEOUT_CLIMB) {
        stopMotors();
        if (DEBUG_STATE) Serial.println(F("Climb: Timeout"));
        climbState = CLIMB_DONE;
      }
      break;
      
    case CLIMB_DONE:
      climbState = CLIMB_START;  // Reset
      return true;
  }
  
  return false;
}

// ============================================
// TARGET NAVIGATION HELPERS
// ============================================
bool isMoreInward(char detected, char current) {
  if (current == 'B' && detected == 'R') return true;
  if (current == 'R' && detected == 'G') return true;
  if (current == 'G' && detected == 'K') return true;
  return false;
}

bool isMoreOutward(char detected, char current) {
  if (current == 'R' && detected == 'B') return true;
  if (current == 'G' && detected == 'R') return true;
  if (current == 'K' && detected == 'G') return true;
  return false;
}

bool isBlackZone(char color) {
  return color == 'K';
}

void scanForWall() {
  int minDistance = DIST_NO_READING;
  int minAngle = 0;
  
  if (DEBUG_NAV) Serial.println(F("Nav: Scanning for wall..."));
  
  for (int angle = 0; angle < NAV_FULL_ROTATION; angle += NAV_SCAN_STEP_DEG) {
    long dist = readDistance();
    
    if (dist > 0 && dist < minDistance) {
      minDistance = dist;
      minAngle = angle;
    }
    
    turnRightDegrees(NAV_SCAN_STEP_DEG);
  }
  
  wallDirection = minAngle;
  wallFound = true;
  
  if (DEBUG_NAV) {
    Serial.print(F("Nav: Wall at "));
    Serial.print(wallDirection);
    Serial.print(F("° dist:"));
    Serial.println(minDistance);
  }
}

void faceWall() {
  if (DEBUG_NAV) Serial.println(F("Nav: Facing wall..."));
  
  int minDist = DIST_NO_READING;
  int currentAngle = 0;
  int bestAngle = 0;
  
  for (int i = 0; i < (NAV_FULL_ROTATION / NAV_SCAN_STEP_DEG); i++) {
    long dist = readDistance();
    
    if (dist > 0 && dist < minDist) {
      minDist = dist;
      bestAngle = currentAngle;
    }
    
    turnRightDegrees(NAV_SCAN_STEP_DEG);
    currentAngle += NAV_SCAN_STEP_DEG;
  }
  
  if (bestAngle > 0) {
    turnRightDegrees(bestAngle);
  }
  
  if (DEBUG_NAV) {
    Serial.print(F("Nav: Now facing wall at "));
    Serial.println(minDist);
  }
}

// ============================================
// MAIN TARGET NAVIGATION (Safe ring traversal)
// ============================================
bool navigateTargetByColor() {
  static unsigned long navTimer = 0;
  char detected;
  
  switch (navState) {
    case NAV_INIT:
      if (DEBUG_NAV) Serial.println(F("=== TARGET NAV START ==="));
      stopMotors();
      delay(300);
      wallFound = false;
      arcDirection = 1;
      navState = NAV_FIND_WALL;
      break;
      
    case NAV_FIND_WALL:
      if (!wallFound) {
        scanForWall();
      }
      navState = NAV_IDENTIFY_ZONE;
      break;
      
    case NAV_IDENTIFY_ZONE:
      detected = detectColorStable();
      currentZone = detected;
      
      if (DEBUG_NAV) {
        Serial.print(F("Nav: Zone = "));
        Serial.println(currentZone);
      }
      
      if (currentZone == 'K') {
        // Already on black - back up!
        if (DEBUG_NAV) Serial.println(F("Nav: WARNING on black!"));
        navState = NAV_CORRECTION;
      } else if (currentZone == 'G') {
        navState = NAV_PREPARE_ENTRY;
      } else if (currentZone == 'B' || currentZone == 'R') {
        arcStartTime = millis();
        navState = NAV_ARC_AROUND;
      } else {
        // Unknown - move and re-identify
        stepForward(200);
      }
      break;
      
    case NAV_ARC_AROUND:
      detected = detectColorStable();
      
      // SAFETY: Never enter black accidentally!
      if (isBlackZone(detected)) {
        if (DEBUG_NAV) Serial.println(F("Nav: DANGER black detected!"));
        stopMotors();
        navState = NAV_CORRECTION;
        break;
      }
      
      // Found next inward ring
      if (isMoreInward(detected, currentZone)) {
        stopMotors();
        if (DEBUG_NAV) {
          Serial.print(F("Nav: Transition "));
          Serial.print(currentZone);
          Serial.print(F(" -> "));
          Serial.println(detected);
        }
        currentZone = detected;
        navState = NAV_STEP_INWARD;
        break;
      }
      
      // Drifted outward - correct
      if (isMoreOutward(detected, currentZone)) {
        stopMotors();
        if (DEBUG_NAV) Serial.println(F("Nav: Drifted out, correcting"));
        stepForward(100);
        detected = detectColorStable();
        currentZone = detected;
        break;
      }
      
      // Continue arcing
      if (arcDirection > 0) {
        moveArcClockwise(SPEED_BASE);
      } else {
        moveArcCounterClockwise(SPEED_BASE);
      }
      
      // Timeout - reverse direction
      if (millis() - arcStartTime > TIMEOUT_NAV_ARC) {
        stopMotors();
        arcDirection *= -1;
        arcStartTime = millis();
        if (DEBUG_NAV) Serial.println(F("Nav: Arc timeout, reversing"));
      }
      break;
      
    case NAV_STEP_INWARD:
      if (DEBUG_NAV) {
        Serial.print(F("Nav: Stepping to "));
        Serial.println(currentZone);
      }
      
      stepForward(STEP_INWARD_MS);
      
      detected = detectColorStable();
      
      if (isBlackZone(detected)) {
        if (DEBUG_NAV) Serial.println(F("Nav: Overshot to black!"));
        navState = NAV_CORRECTION;
        break;
      }
      
      currentZone = detected;
      
      if (currentZone == 'G') {
        navState = NAV_PREPARE_ENTRY;
      } else {
        arcStartTime = millis();
        navState = NAV_ARC_AROUND;
      }
      break;
      
    case NAV_CORRECTION:
      if (DEBUG_NAV) Serial.println(F("Nav: CORRECTING"));
      
      stepBackward(STEP_BACKWARD_MS);
      
      detected = detectColorStable();
      
      if (!isBlackZone(detected)) {
        currentZone = detected;
        if (DEBUG_NAV) {
          Serial.print(F("Nav: Corrected to "));
          Serial.println(currentZone);
        }
        
        if (currentZone == 'G') {
          navState = NAV_PREPARE_ENTRY;
        } else {
          arcStartTime = millis();
          navState = NAV_ARC_AROUND;
        }
      }
      break;
      
    case NAV_PREPARE_ENTRY:
      if (DEBUG_NAV) Serial.println(F("Nav: On GREEN, preparing entry"));
      stopMotors();
      delay(200);
      navState = NAV_ALIGN_TO_WALL;
      navTimer = millis();
      break;
      
    case NAV_ALIGN_TO_WALL:
      if (DEBUG_NAV) Serial.println(F("Nav: Aligning to wall"));
      
      faceWall();
      delay(200);
      
      detected = detectColorStable();
      if (isBlackZone(detected)) {
        navState = NAV_CORRECTION;
        break;
      }
      
      navState = NAV_ENTER_BLACK;
      navTimer = millis();
      break;
      
    case NAV_ENTER_BLACK:
      detected = detectColorStable();
      
      if (isBlackZone(detected)) {
        stopMotors();
        stepForward(150);  // Center in black zone
        if (DEBUG_NAV) Serial.println(F("=== NAV: CENTERED! ==="));
        navState = NAV_COMPLETE;
        break;
      }
      
      // Slow approach
      moveForward(SPEED_CRAWL);
      delay(80);
      stopMotors();
      delay(30);
      
      if (millis() - navTimer > TIMEOUT_NAV_ENTRY) {
        if (DEBUG_NAV) Serial.println(F("Nav: Entry timeout"));
        navState = NAV_COMPLETE;
      }
      break;
      
    case NAV_COMPLETE:
      stopMotors();
      navState = NAV_INIT;  // Reset
      return true;
  }
  
  return false;
}

// ============================================
// BALL LAUNCHER
// ============================================
bool launchBall() {
  static unsigned long launchTimer = 0;
  
  switch (launchState) {
    case LAUNCH_PREPARE:
      stopMotors();
      if (DEBUG_STATE) Serial.println(F("Launch: Aiming"));
      delay(200);
      launchTimer = millis();
      launchState = LAUNCH_FIRE;
      break;
      
    case LAUNCH_FIRE:
      if (DEBUG_STATE) Serial.println(F("Launch: FIRE!"));
      launcherServo.write(LAUNCHER_FIRE);
      
      if (millis() - launchTimer > 500) {
        launchState = LAUNCH_RESET;
        launchTimer = millis();
      }
      break;
      
    case LAUNCH_RESET:
      launcherServo.write(LAUNCHER_READY);
      
      if (millis() - launchTimer > 300) {
        launchState = LAUNCH_DONE;
      }
      break;
      
    case LAUNCH_DONE:
      if (DEBUG_STATE) Serial.println(F("Launch: Complete!"));
      launchState = LAUNCH_PREPARE;  // Reset
      return true;
  }
  
  return false;
}

// ============================================
// RAMP DESCENT
// ============================================
bool descendRamp() {
  static unsigned long descendTimer = 0;
  char color;
  
  switch (descendState) {
    case DESCEND_TURN_AROUND:
      if (DEBUG_STATE) Serial.println(F("Descend: Turning around"));
      turnRightDegrees(180);
      descendTimer = millis();
      descendState = DESCEND_GO_DOWN;
      break;
      
    case DESCEND_GO_DOWN:
      moveForward(SPEED_SLOW);
      
      if (millis() - descendTimer > RAMP_DESCEND_TIME) {
        descendState = DESCEND_DETECT_BOTTOM;
      }
      break;
      
    case DESCEND_DETECT_BOTTOM:
      moveForward(SPEED_SLOW);
      
      color = detectColor();
      
      // Bottom has green path or black main
      if (color == 'G' || color == 'K') {
        stopMotors();
        if (DEBUG_STATE) Serial.println(F("Descend: Bottom reached"));
        descendState = DESCEND_DONE;
      }
      
      if (millis() - descendTimer > 5000) {
        stopMotors();
        descendState = DESCEND_DONE;
      }
      break;
      
    case DESCEND_DONE:
      descendState = DESCEND_TURN_AROUND;  // Reset
      return true;
  }
  
  return false;
}

// ============================================
// OBSTACLE COURSE
// ============================================
bool navigateObstacleCourse() {
  static unsigned long obsTimer = 0;
  long dist;
  char color;
  bool left, right;
  
  switch (obsState) {
    case OBS_START:
      if (DEBUG_STATE) Serial.println(F("Obstacle: Starting"));
      obsTimer = millis();
      obstacleCount = 0;
      avoidDirection = 1;
      obsState = OBS_FOLLOW_PATH;
      break;
      
    case OBS_FOLLOW_PATH:
      dist = readDistance();
      
      // Check for obstacles
      if (dist > 0 && dist < DIST_OBSTACLE) {
        stopMotors();
        if (DEBUG_STATE) {
          Serial.print(F("Obstacle: Detected at "));
          Serial.println(dist);
        }
        obsState = OBS_AVOID_OBSTACLE;
        break;
      }
      
      // Line following
      left = irLeft();
      right = irRight();
      
      if (!left && !right) {
        stopMotors();
        obsState = OBS_SHARP_TURN;
        break;
      }
      
      if (left && right) {
        moveForward(SPEED_FAST);
      } else if (left && !right) {
        setMotors(SPEED_SLOW, SPEED_FAST);
      } else {
        setMotors(SPEED_FAST, SPEED_SLOW);
      }
      
      // Check for completion (back at black main path)
      color = detectColor();
      if (color == 'K' && millis() - obsTimer > 5000) {
        obsState = OBS_CHECK_COMPLETE;
      }
      
      // Timeout
      if (millis() - obsTimer > TIMEOUT_OBSTACLE) {
        obsState = OBS_DONE;
      }
      break;
      
    case OBS_AVOID_OBSTACLE:
      obstacleCount++;
      
      stepBackward(150);
      turnDegrees(AVOID_TURN_DEG, avoidDirection);
      stepForward(AVOID_FORWARD_MS);
      turnDegrees(AVOID_TURN_DEG, -avoidDirection);
      
      avoidDirection *= -1;  // Alternate
      obsState = OBS_FOLLOW_PATH;
      break;
      
    case OBS_SHARP_TURN:
      if (DEBUG_STATE) Serial.println(F("Obstacle: Sharp turn"));
      
      // Scan for line
      turnLeftDegrees(SHARP_TURN_SCAN);
      if (irLeft() || irRight()) {
        obsState = OBS_FOLLOW_PATH;
        break;
      }
      
      turnRightDegrees(SHARP_TURN_SCAN * 2);
      if (irLeft() || irRight()) {
        obsState = OBS_FOLLOW_PATH;
        break;
      }
      
      turnLeftDegrees(SHARP_TURN_SCAN);
      stepForward(200);
      obsState = OBS_FOLLOW_PATH;
      break;
      
    case OBS_CHECK_COMPLETE:
      stopMotors();
      color = detectColorStable();
      
      if (color == 'K') {
        if (DEBUG_STATE) {
          Serial.println(F("Obstacle: Complete!"));
          Serial.print(F("Avoided: "));
          Serial.println(obstacleCount);
        }
        obsState = OBS_DONE;
      } else {
        obsState = OBS_FOLLOW_PATH;
      }
      break;
      
    case OBS_DONE:
      obsState = OBS_START;  // Reset
      return true;
  }
  
  return false;
}

// ============================================
// RETURN HOME
// ============================================
bool returnHome() {
  static enum {
    HOME_FIND_LINE,
    HOME_FOLLOW,
    HOME_DETECT_START,
    HOME_DONE
  } homeState = HOME_FIND_LINE;
  
  static unsigned long homeTimer = 0;
  char color;
  bool left, right;
  long dist;
  
  switch (homeState) {
    case HOME_FIND_LINE:
      if (DEBUG_STATE) Serial.println(F("Home: Finding line"));
      
      color = detectColor();
      if (color == 'K') {
        homeState = HOME_FOLLOW;
        homeTimer = millis();
      } else {
        // Rotate to find black line
        turnRight(SPEED_SLOW);
        delay(100);
        stopMotors();
      }
      break;
      
    case HOME_FOLLOW:
      left = irLeft();
      right = irRight();
      
      if (left && right) {
        moveForward(SPEED_BASE);
      } else if (left && !right) {
        setMotors(SPEED_SLOW, SPEED_BASE);
      } else if (!left && right) {
        setMotors(SPEED_BASE, SPEED_SLOW);
      } else {
        moveForward(SPEED_SLOW);
      }
      
      if (millis() - homeTimer > 8000) {
        homeState = HOME_DETECT_START;
      }
      break;
      
    case HOME_DETECT_START:
      stopMotors();
      
      dist = readDistance();
      if (dist < 20) {
        if (DEBUG_STATE) Serial.println(F("Home: START reached!"));
        homeState = HOME_DONE;
      } else {
        stepForward(300);
        if (millis() - homeTimer > TIMEOUT_RETURN) {
          homeState = HOME_DONE;
        }
      }
      break;
      
    case HOME_DONE:
      stopMotors();
      homeState = HOME_FIND_LINE;  // Reset
      return true;
  }
  
  return false;
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Safety timeout
  if (millis() - totalStartTime > TOTAL_TIME_LIMIT) {
    if (DEBUG_STATE) Serial.println(F("!!! TIME LIMIT !!!"));
    currentState = STATE_DONE;
  }
  
  switch (currentState) {
    // ======== PHASE 1: GREEN PATH ========
    case STATE_START:
      if (DEBUG_STATE) Serial.println(F("=== STATE: START ==="));
      stateStartTime = millis();
      currentState = STATE_FOLLOW_TO_FIRST_JUNCTION;
      break;
      
    case STATE_FOLLOW_TO_FIRST_JUNCTION:
      {
        char result = followBlackLine();
        
        // Look for blue circle (box pickup area) or color marker
        if (result == 'B') {
          stopMotors();
          if (DEBUG_STATE) Serial.println(F("=== STATE: At first junction ==="));
          currentState = STATE_PICKUP_BOX_1;
          stateStartTime = millis();
        }
      }
      break;
      
    case STATE_PICKUP_BOX_1:
      if (pickupBox()) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: Box 1 acquired ==="));
        currentState = STATE_NAVIGATE_TO_GREEN;
        stateStartTime = millis();
        targetPathColor = 'G';
        targetPathDirection = GREEN_PATH_DIRECTION;
      }
      break;
      
    case STATE_NAVIGATE_TO_GREEN:
      if (navigateToSplit(targetPathColor, targetPathDirection)) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: At green path ==="));
        currentState = STATE_DROP_BOX_GREEN;
        stateStartTime = millis();
      }
      break;
      
    case STATE_DROP_BOX_GREEN:
      if (dropBoxAtBlue()) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: Green unlocked ==="));
        currentState = STATE_CLIMB_RAMP;
        stateStartTime = millis();
      }
      break;
      
    case STATE_CLIMB_RAMP:
      if (climbRamp(USE_CURVED_RAMP)) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: At top of ramp ==="));
        currentState = STATE_NAVIGATE_TARGET;
        stateStartTime = millis();
        navState = NAV_INIT;
      }
      break;
      
    case STATE_NAVIGATE_TARGET:
      if (navigateTargetByColor()) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: At center ==="));
        currentState = STATE_SHOOT_BALL;
        stateStartTime = millis();
      }
      break;
      
    case STATE_SHOOT_BALL:
      if (launchBall()) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: Ball launched ==="));
        currentState = STATE_DESCEND_RAMP;
        stateStartTime = millis();
      }
      break;
      
    case STATE_DESCEND_RAMP:
      if (descendRamp()) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: Back at bottom ==="));
        currentState = STATE_RETURN_TO_JUNCTION;
        stateStartTime = millis();
      }
      break;
      
    // ======== PHASE 2: RED PATH ========
    case STATE_RETURN_TO_JUNCTION:
      {
        char result = followBlackLine();
        if (result == 'B') {
          stopMotors();
          if (DEBUG_STATE) Serial.println(F("=== STATE: Back at junction ==="));
          currentState = STATE_PICKUP_BOX_2;
          stateStartTime = millis();
        }
      }
      break;
      
    case STATE_PICKUP_BOX_2:
      if (pickupBox()) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: Box 2 acquired ==="));
        currentState = STATE_NAVIGATE_TO_RED;
        stateStartTime = millis();
        targetPathColor = 'R';
        targetPathDirection = RED_PATH_DIRECTION;
      }
      break;
      
    case STATE_NAVIGATE_TO_RED:
      if (navigateToSplit(targetPathColor, targetPathDirection)) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: At red path ==="));
        currentState = STATE_DROP_BOX_RED;
        stateStartTime = millis();
      }
      break;
      
    case STATE_DROP_BOX_RED:
      if (dropBoxAtBlue()) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: Red unlocked ==="));
        currentState = STATE_OBSTACLE_COURSE;
        stateStartTime = millis();
      }
      break;
      
    case STATE_OBSTACLE_COURSE:
      if (navigateObstacleCourse()) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: Obstacle complete ==="));
        currentState = STATE_PICKUP_BOX_3;
        stateStartTime = millis();
      }
      break;
      
    case STATE_PICKUP_BOX_3:
      if (pickupBox()) {
        if (DEBUG_STATE) Serial.println(F("=== STATE: Box 3 acquired ==="));
        currentState = STATE_RETURN_HOME;
        stateStartTime = millis();
      }
      break;
      
    // ======== PHASE 3: FINISH ========
    case STATE_RETURN_HOME:
      if (returnHome()) {
        if (DEBUG_STATE) Serial.println(F("=== COURSE COMPLETE! ==="));
        currentState = STATE_DONE;
      }
      break;
      
    case STATE_DONE:
      stopMotors();
      if (hasBox) {
        openClaw();
        hasBox = false;
      }
      // Could add celebration (beep/blink)
      break;
  }
}