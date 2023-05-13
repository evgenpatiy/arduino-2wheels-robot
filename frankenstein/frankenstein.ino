#include <Servo.h>          // servo-control library (out of the box)
#include <NewPing.h>        // sonar-control library, external

// motors controller L298N connection pins
const int LeftMotorForward = 7;
const int LeftMotorBackward = 6;
const int RightMotorForward = 5;
const int RightMotorBackward = 4;

// servo connection pin
const int servo_pin = 8;

// sonar HC-SR04 connection pins
#define trig_pin A0 // analog input 1
#define echo_pin A1 // analog input 2

#define maximum_distance 200
#define serial_port_speed 9600
#define is_debug_enabled false

const char *robot_name = { "Frankenstein"};
const float robot_software_version = 0.01;

boolean goesForward = false;
int distance = 100;
const int critical_distance = 20;

#define main_loop_delay 50

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name

void initSerialConsole() {

  if (is_debug_enabled) {
    Serial.begin(serial_port_speed);

    Serial.print("--- this is ");
    Serial.print(robot_name);
    Serial.print(" version ");
    Serial.println(robot_software_version);

    Serial.println("--- init serial console...");
    Serial.print("--- port speed: ");
    Serial.println(serial_port_speed);
    Serial.println();

    delay(200);
  }
}

void setup() {
  initSerialConsole();

  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  servo_motor.attach(servo_pin); //Пин подключения сервомотора
  servo_motor.write(115);
  delay(2000);

  // check is servo ok
  lookLeft();
  lookRight();

  if (is_debug_enabled) {
    Serial.println("--- servo ok...");
  }

  delay(1000);

  // check is motors ok
  moveForward();
  delay(200);
  moveStop();
  delay(200);
  moveBackward();
  delay(200);
  moveStop();

  if (is_debug_enabled) {
    Serial.println("--- motors ok...");
  }

  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop() {

  int distanceRight = 0;
  int distanceLeft = 0;
  delay(main_loop_delay);

  // read distance from sonar
  distance = readPing();

  if (is_debug_enabled) {
    Serial.print("--->>> sonar reported distance:  ");
    Serial.println(distance);
  }

  if (distance <= critical_distance) {
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);

    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distance >= distanceLeft) {
      turnRight();
      moveStop();
    }
    else if (distance >= distanceRight) {
      turnLeft();
      moveStop();
    }
  }
  else {
    moveForward();
  }

}

int lookRight() {
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft() {
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}

void moveStop() {

  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward() {

  if (!goesForward) {

    goesForward = true;

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);

    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
  }
}

void moveBackward() {

  goesForward = false;

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);

}

void turnRight() {

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);

  delay(500);

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);



}

void turnLeft() {

  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(500);

  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}
