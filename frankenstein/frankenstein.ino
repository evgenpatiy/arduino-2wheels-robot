
#include <Servo.h>          // servo-control library (out of the box)
#include <NewPing.h>        // sonar-control library, external

// motors controller L298N connection pins
const byte LeftMotorForward = 7;
const byte LeftMotorBackward = 6;
const byte RightMotorForward = 5;
const byte RightMotorBackward = 4;

// servo connection pin
const byte servo_pin = 8;

// sonar HC-SR04 connection pins
#define trig_pin A0 // analog input 1
#define echo_pin A1 // analog input 2

#define maximum_distance 1000
#define serial_port_speed 9600

#define default_servo_shaft_angle 110
#define edge_servo_right_angle 50
#define edge_servo_left_angle 170

#define DEBUG true
#define Serial if(DEBUG)Serial

#define RUN_LOOP true

const char *robot_name = { "B1sh0p"};
const float robot_software_version = 0.01;

boolean goesForward = false;
int distance = 100;
const unsigned int critical_distance = 20;

const unsigned long full_turnover_time = 1000UL;
const unsigned long sonar_delay = 200UL;

#define main_loop_delay 50UL

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name

void initSerialConsole() {
  Serial.begin(serial_port_speed);
  Serial.println("--- this is " + (String)robot_name + " version " + (String)robot_software_version);
  Serial.println("--- init serial console... speed: " + (String)serial_port_speed);
  Serial.println();
}

void setup() {
  initSerialConsole();

  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  servo_motor.attach(servo_pin); //Пин подключения сервомотора
  resetServoShaft();
  delay(1000);

  // check is servo ok
  readDistanceByAngle(edge_servo_left_angle);
  resetServoShaft();

  readDistanceByAngle(edge_servo_right_angle);
  resetServoShaft();
  Serial.println("--- servo ok...");
}

void loop() {
  if (RUN_LOOP) {
    delay(main_loop_delay);
    distance = readDistanceAhead();

    if (distance <= critical_distance) {
      stop();
      moveBackward(400);

      int distanceRight = readDistanceByAngle(edge_servo_right_angle);
      resetServoShaft();
      int distanceLeft = readDistanceByAngle(edge_servo_left_angle);
      resetServoShaft();

      Serial.println("***SONAR: distance ahead: " + (String)distance + " distance <<left: " + (String)distanceLeft + " distance right>>: " + (String)distanceRight);

      if (distanceRight <= critical_distance && distanceLeft <= critical_distance) {
          switch (random(1)) {
              case 0: turnOverLeft();
                break;
              case 1: turnOverLeft();
                break;
              default:
                break;
          }
      } else if (distanceRight >= distanceLeft) {
          switch (random(3)) {
              case 0: turnRightToAngle(45);
                break;
              case 1: turnRightToAngle(60);
                break;
              case 2: turnRightToAngle(90);
                break;
              case 3: turnOverRight();
                break;
              default:
                break;
          }
      } else if (distanceLeft >= distanceRight) {
          switch (random(3)) {
              case 0: turnLeftToAngle(45);
                break;
              case 1: turnLeftToAngle(60);
                break;
              case 2: turnLeftToAngle(90);
                break;
              case 3: turnOverLeft();
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
  servo_motor.write(default_servo_shaft_angle);
}

unsigned int readDistanceByAngle(unsigned int angle) {
  servo_motor.write(angle);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(sonar_delay);
  digitalWrite(LED_BUILTIN, LOW);
  
  return readDistanceAhead();
}

unsigned int readDistanceAhead() {
  delay(50);

  unsigned int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = maximum_distance;
  }
  return cm;
}

void stop() {
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward(unsigned int time) {

  if (!goesForward) {

    goesForward = true;

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);

    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);

    execute(time);
  }
}

void moveBackward(unsigned int time) {

  goesForward = false;

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);

  execute(time);
}

void turnRight(unsigned int time) {

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);

  execute(time);
}

void turnLeftToAngle(unsigned int angle) {
  turnLeft(getAngleTurnTime(angle));
}

void turnRightToAngle(unsigned int angle) {
  turnRight(getAngleTurnTime(angle));
}

void turnLeft(unsigned int time) {
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  execute(time);
}

unsigned long getAngleTurnTime(unsigned int angle) {
  return ((float)angle * float(full_turnover_time)) / 180.0;
}

void turnOverLeft() {
  turnLeft(full_turnover_time);
}

void turnOverRight() {
  turnRight(full_turnover_time);
}

void execute(unsigned int time) {
  if (time > 0) {
    delay(time);
    stop();
  }
}
