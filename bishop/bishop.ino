#include <Servo.h>    // servo-control library (out of the box)
#include <NewPing.h>  // sonar-control library, external

// motors controller L298N connection pins
#define LEFT_MOTOR_FORWARD 7
#define LEFT_MOTOR_BACKWARD 6
#define RIGHT_MOTOR_FORWARD 5
#define RIGHT_MOTOR_BACKWARD 4

// servo connection pin
#define SERVO_MOTOR_PIN 12

// sonar HC-SR04 connection pins
#define SONAR_TRIG_PIN A0  // analog input 1
#define SONAR_ECHO_PIN A1  // analog input 2

#define MAXIMUM_DISTANCE 300
#define CRITICAL_DISTANCE 20

#define DEFAULT_SERVO_POSITION 110
#define EDGE_SERVO_RIGHT_POSITION 50
#define EDGE_SERVO_LEFT_POSITION 170

#define DEFAULT_SERVO_ANGLE 0
#define EDGE_SERVO_RIGHT_ANGLE 90
#define EDGE_SERVO_LEFT_ANGLE -90

#define RUN_LOOP true

#define SERIAL_PORT_SPEED 9600
#define DEBUG true
#define Serial \
  if (DEBUG) Serial

#define MAIN_LOOP_DELAY 50UL
#define FULL_TURNOVER_TIME 1000UL
#define SONAR_DELAY 200UL

const char *ROBOT_NAME = { "B1sh0p" };
const float ROBOT_SOFTWARE_VERSION = 0.02;

boolean isMoveForward = false;
unsigned int distance = 100;

struct LedColor {
  byte rLevel;
  byte gLevel;
  byte bLevel;
};

LedColor red = { 255, 0, 0 }; 
LedColor green = { 0, 255, 0 }; 
LedColor blue = { 0, 0, 255 }; 

struct LedLight {
  byte r;
  byte g;
  byte b;
};

LedLight leftLight = { 9, 10, 11 };
LedLight rightLight = { 0, 1, 2 };

NewPing sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN, MAXIMUM_DISTANCE);  //sensor function
Servo servo_motor;                                                //our servo name

void initSerialConsole() {
  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println("--- this is " + (String)ROBOT_NAME + " version " + (String)ROBOT_SOFTWARE_VERSION);
  Serial.println("--- init serial console... speed: " + (String)SERIAL_PORT_SPEED);
  Serial.println();
}

void initLED(LedLight light) {
  pinMode(light.r, OUTPUT);
  pinMode(light.g, OUTPUT);
  pinMode(light.b, OUTPUT);
}

void lightOn(LedLight light, LedColor color) {
  analogWrite(light.r, color.rLevel);
  analogWrite(light.g, color.gLevel);
  analogWrite(light.b, color.bLevel);
}

void lightOff(LedLight light) {
  digitalWrite(light.r, LOW);
  digitalWrite(light.g, LOW);
  digitalWrite(light.b, LOW);
}

void checkLight() {
  lightOn(leftLight, red);
  lightOn(rightLight, green);
}

void setup() {
  initLED(leftLight);
  initLED(rightLight);
  checkLight();

  initSerialConsole();
  randomSeed(analogRead(0));  // init random numbers generator

  // init Arduino built-in LED
  pinMode(LED_BUILTIN, OUTPUT);

  // init engines control
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  servo_motor.attach(SERVO_MOTOR_PIN);  // servo pin
  resetServoShaft();
  delay(1000);

  // check is servo ok
  readDistanceByAngle(EDGE_SERVO_LEFT_POSITION);
  resetServoShaft();

  readDistanceByAngle(EDGE_SERVO_RIGHT_POSITION);
  resetServoShaft();
  Serial.println("--- servo ok...");
  delay(2000);
}

void loop() {
  if (RUN_LOOP) {
    delay(MAIN_LOOP_DELAY);
    distance = readDistanceAhead();

    if (distance <= CRITICAL_DISTANCE) {
      stop();
      moveBackward(400);

      int distanceRight = readDistanceByAngle(EDGE_SERVO_RIGHT_POSITION);
      resetServoShaft();
      int distanceLeft = readDistanceByAngle(EDGE_SERVO_LEFT_POSITION);
      resetServoShaft();

      Serial.println("***SONAR: distance ahead: " + (String)distance + " distance <<left: " + (String)distanceLeft + " distance right>>: " + (String)distanceRight);

      if (distanceRight <= CRITICAL_DISTANCE && distanceLeft <= CRITICAL_DISTANCE) {
        switch (random(0, 2)) {
          case 0:
            turnOverLeft();
            break;
          case 1:
            turnOverRight();
            break;
          default:
            break;
        }
      } else if (distanceRight >= distanceLeft) {
        switch (random(0, 4)) {
          case 0:
            turnRightToAngle(45);
            break;
          case 1:
            turnRightToAngle(60);
            break;
          case 2:
            turnRightToAngle(90);
            break;
          case 3:
            turnOverRight();
            break;
          default:
            break;
        }
      } else if (distanceLeft >= distanceRight) {
        switch (random(0, 4)) {
          case 0:
            turnLeftToAngle(45);
            break;
          case 1:
            turnLeftToAngle(60);
            break;
          case 2:
            turnLeftToAngle(90);
            break;
          case 3:
            turnOverLeft();
            break;
          default:
            break;
        }
      }


    } else {
      moveForward(0);
    }
  }
}

void resetServoShaft() {
  servo_motor.write(DEFAULT_SERVO_POSITION);
}

unsigned int readDistanceByAngle(unsigned int angle) {
  servo_motor.write(angle);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(SONAR_DELAY);
  digitalWrite(LED_BUILTIN, LOW);

  return readDistanceAhead();
}

unsigned int readDistanceAhead() {
  unsigned int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = MAXIMUM_DISTANCE;
  }
  return cm;
}

void stop() {
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
}

void moveForward(unsigned int time) {

  if (!isMoveForward) {

    isMoveForward = true;

    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);

    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);

    execute(time);
  }
}

void moveBackward(unsigned int time) {

  isMoveForward = false;

  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);

  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);

  execute(time);
}

void turnLeft(unsigned int time) {
  digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);

  digitalWrite(LEFT_MOTOR_FORWARD, LOW);
  digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);

  execute(time);
}

void turnRight(unsigned int time) {

  digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
  digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);

  digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
  digitalWrite(RIGHT_MOTOR_FORWARD, LOW);

  execute(time);
}

unsigned long getAngleTurnTime(unsigned int angle) {
  return ((float)angle * float(FULL_TURNOVER_TIME)) / 180.0;
}

void turnLeftToAngle(unsigned int angle) {
  turnLeft(getAngleTurnTime(angle));
}

void turnRightToAngle(unsigned int angle) {
  turnRight(getAngleTurnTime(angle));
}

void turnOverLeft() {
  turnLeft(FULL_TURNOVER_TIME);
}

void turnOverRight() {
  turnRight(FULL_TURNOVER_TIME);
}

void execute(unsigned int time) {
  if (time > 0) {
    delay(time);
    stop();
  }
}
