#include <CytronMotorDriver.h>
#include <Pixy2.h>  // Allows communication with the Pixy2 camera
#include <Servo.h>  // Controls the servo motor for the claw


CytronMD leftMotor(PWM_DIR, 3, 4);  // pins for left motor
CytronMD rightMotor(PWM_DIR, 6, 7); // pins for right motor


int IRSensor1 = 2;   // left sensor
int IRSensor2 = A1;  // middle sensor
int IRSensor3 = A3;  // right sensor


// Create objects for Pixy2 camera and claw servo
Pixy2 pixy;
Servo claw;
Servo lift;


// Constants (DO NOT CHANGE)
const int CENTER_X = 150;      // The x-position that represents the center of the camera's view
const int TARGET_WIDTH = 200;  // The object's width when it is close enough to grab
const float Kp = 1.5;          // Proportional gain for turning control (used for feedback)
const int MAX_SPEED = 255;     // Maximum motor speed
const int MIN_SPEED = 100;     // Minimum motor speed (ensures the robot moves instead of stalling)
const int FORWARD_SPEED = 250; // Speed for moving forward when approaching the object
const int TURN_THRESHOLD = 5;  // How close the object must be to the center before moving forward


int instruction = 0; // Declares the state of the rover so it goes through the line following code first


void setup() {
  Serial.begin(9600);
  pinMode(IRSensor1, INPUT);
  pinMode(IRSensor2, INPUT);
  pinMode(IRSensor3, INPUT);

  claw.attach(A2); // Attach the claw servo motor to pin A2
  lift.attach(5);  // Attach the lift servo motor to pin 5
  claw.write(0);   // Start with the claw open
  lift.write(0);   // Start with the lift lowered
  pixy.init();     // Initialize Pixy2 camera
}


void loop() {
  if (instruction == 0) {  // will start with line following code
    followLine();
  } else {
    trackObject();          // then goes into object tracking code
  }
}


void followLine() {
  int sensorStatus1 = digitalRead(IRSensor1);
  int sensorStatus2 = digitalRead(IRSensor2);
  int sensorStatus3 = digitalRead(IRSensor3);

  // prints which sensor detects line for testing purposes
  Serial.print(sensorStatus1);
  Serial.print(sensorStatus2);
  Serial.println(sensorStatus3);

  if (sensorStatus1 == 1 && sensorStatus2 == 1 && sensorStatus3 == 1) {
    // All sensors detect black — stop and switch to object tracking
    moveForward(225);
    delay(250);
    stopMotors();
    instruction = 1;
    Serial.println("Instruction changed to 1");
    Serial.println("Stop");

  } else if (sensorStatus1 == 0 && sensorStatus2 == 1 && sensorStatus3 == 0) {
    // Only middle sensor detects black — move forward
    moveForward(190);
    Serial.println("Move forward");

  } else if (sensorStatus1 == 1 && sensorStatus2 == 1 && sensorStatus3 == 0) {
    // Left and middle sensors detect black — sharp left
    turnLeft(180);
    Serial.println("Turn Left");

  } else if (sensorStatus1 == 0 && sensorStatus2 == 1 && sensorStatus3 == 1) {
    // Middle and right sensors detect black — sharp right
    turnRight(180);
    Serial.println("Turn Right");

  } else if (sensorStatus1 == 1 && sensorStatus2 == 0 && sensorStatus3 == 0) {
    // Only left sensor detects black — turn left
    turnLeft(195);
    Serial.println("Turn Left");

  } else if (sensorStatus1 == 0 && sensorStatus2 == 0 && sensorStatus3 == 1) {
    // Only right sensor detects black — turn right
    turnRight(195);
    Serial.println("Turn Right");
  }
}


// Function to move forward
void moveForward(int speed) {
  leftMotor.setSpeed(-speed);
  rightMotor.setSpeed(speed);
}


// Function to turn right
void turnRight(int speed2) {
  leftMotor.setSpeed(speed2);
  rightMotor.setSpeed(speed2);
}


// Function to turn left
void turnLeft(int speed3) {
  leftMotor.setSpeed(-speed3);
  rightMotor.setSpeed(-speed3);
}


// Function to stop motors
void stopMotors() {
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
}


void trackObject() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks > 0) {
    int object_x = pixy.ccc.blocks[0].m_x;
    int object_width = pixy.ccc.blocks[0].m_y;
    int error = object_x - CENTER_X;
    int turn_speed = abs(error) * Kp;

    turn_speed = constrain(turn_speed, MIN_SPEED, MAX_SPEED);

    if (object_width >= TARGET_WIDTH) {
      // Object is close enough — grab it
      stopMotors();
      grabObject();
      return;
    }

    if (abs(error) <= TURN_THRESHOLD) {
      // Rover is centered — move forward
      moveForward(FORWARD_SPEED);
    } else if (error < -TURN_THRESHOLD) {
      // Object is left of center — turn left
      turnLeft(turn_speed);
    } else if (error > TURN_THRESHOLD) {
      // Object is right of center — turn right
      turnRight(turn_speed);
    }
  }
}


// Function to grab object
void grabObject() {
  stopMotors();
  delay(500);
  claw.write(90); // Close claw
  delay(500);
  lift.write(90); // Lift claw
}
