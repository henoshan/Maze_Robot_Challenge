#include <Arduino.h>
#include <math.h>
#include <NewPing.h>

// Constants for robot dimensions
#define WHEEL_BASE 12.0 // Distance between wheels in cm
#define WHEEL_DIAMETER 6.5 //6.5 cm diameter
#define CM_PER_REV (PI * WHEEL_DIAMETER) // Circumference of the wheel

// // Encoder
// const byte encoderLeft = 2;  // on;y 2,3 are interrupt pins
// const byte encoderRight = 3;  // 

// Other constants
#define TIME_STEP 30 // Time interval in ms for odometry updates
// Define pins for ultrasonic sensors
#define triggerFront 2
#define echoFront 3
#define triggerLeft A2
#define echoLeft A3
#define triggerRight A0
#define echoRight A1
// #define irPin 4 

#define MAX_DISTANCE 200 // max distance to measure 150cm

NewPing left(triggerLeft, echoLeft, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing right(triggerRight, echoRight, MAX_DISTANCE);
NewPing front(triggerFront, echoFront, MAX_DISTANCE);

// #define irPin A0 

// Motor control pins
#define rightForward 8
#define rightBackward 9
#define enableRight 10
#define enableLeft 11
#define leftForward 12
#define leftBackward 13

// Robot speed
int forwardLeftSpeed = 70; // for moving forward--> adjusted
int forwardRightSpeed = 75;
int turnLeftSpeed = 55;  // for turning right,Left and around --> constant
int turnRightSpeed = 57;
int offset = 8;
int fMin = 4;
int offsetMin = 15;  // not using
int offsetTurn = 18; //18
// front offset 8 cm 
float oldLeftDist, oldRightDist, oldFrontDist, leftDist, rightDist, frontDist;  //, leftSensor, rightSensor, frontSensor

int L, R, F, T = 0;
int path[100]; //Maze[10][9];  // Array to store moves
int pathIndex = 0;  // Current index of the path array
float robotAngle=0; // turned angle of robot
float targetAngle=0;

//PID
float P = 0.5 ;
float D = 0.65 ;
float I = 0.02 ;
float oldErrorP ;
float totalError ;
// pid left right offset 5cm tolerance 3cm
// int offset = 5 ;  // check and change

// Function Prototypes
void moveForward(int l=0,int r=0);
void startRobot(int Sl = 0,int Sr = 0);
void stopRobot(int d= 0);
void turnLeft();
void turnRight();
void turnAround();
void moveBackward(int d = 0);
void adjustLeft(int &lSpeed, int &rSpeed);
void adjustRight(int &lSpeed, int &rSpeed);

void selectMovement();
void pid();
void ReadSensors();
long readUltrasonic(int trigPin, int echoPin);
// void followPath();
// void backtrack();
void addToPath(char movement);


void setup() {
  // Initialize sensor pins
  pinMode(triggerFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(triggerLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(triggerRight, OUTPUT);
  pinMode(echoRight, INPUT);
  // pinMode(irPin, INPUT);

  // Initialize motor control pins
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(enableLeft, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(enableRight, OUTPUT);

  Serial.begin(9600); // for testing ultrasonic
  // stopRobot();
}
// Ultrasonic sensor function
void ReadSensors() {

  leftDist = left.ping_cm(); //ping in cm
  rightDist = right.ping_cm();
  frontDist = front.ping_cm();

  // Serial.print(rightDist);
  // Serial.print(" ");
  // Serial.print(frontDist);
  // Serial.print(" ");
  // Serial.println(leftDist);
}

void loop() {
      ReadSensors();
      //  delay(1000);
       if (leftDist > offsetTurn){
        turnLeft();
       }else if (rightDist >offsetTurn){
        turnRight();
       }
      stopRobot();
  
  // ReadSensors();
  // selectMovement();
  // pid();
  // delay(30);
  // oldLeftDist = leftDist;
  // oldRightDist = rightDist;
  // oldFrontDist = frontDist;
}

// Functions //
void selectMovement(){
   if(frontDist < fMin ){      // move backward if too clode to wall
      moveBackward(150);
   }
   if(leftDist > offsetTurn || rightDist > offsetTurn){
    if((rightDist > offsetTurn) && (leftDist > offsetTurn)){
      if(R%4==0){   // check and change
        turnLeft();
      }else{
        turnRight();
      }
    }else if(rightDist > offsetTurn && leftDist < offsetTurn){
      turnRight();
    }else{
      turnLeft();   
    }
  }else{
    if(frontDist < offset || frontDist > 300){   //&&(rightDist < offset)&&(leftDist < offset)
      turnAround();
    }else moveForward();
  }

}

void pid() {
  float errorP;
  float errorD;
  static float prevErrorI = 0;  // Store previous errorI value
  float errorI;

  errorP = leftDist - rightDist ;
  errorD = errorP - oldErrorP;
  errorI = (2.0 / 3.0) * prevErrorI + errorP ;

  totalError = P * errorP + D * errorD + I * errorI ;
  
  oldErrorP = errorP ;

  forwardRightSpeed = forwardRightSpeed + totalError ;
  forwardLeftSpeed = forwardLeftSpeed - totalError ;
  // Constrain the values to be between 60 and 95
  forwardRightSpeed = constrain(forwardRightSpeed, 60, 95);
  forwardLeftSpeed = constrain(forwardLeftSpeed, 60, 95);

  analogWrite(enableRight , forwardRightSpeed);
  analogWrite(enableLeft , forwardLeftSpeed);
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4); // check and increase for better iterative reading
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH); //limit max range
  return duration * 0.034 / 2; // Distance in cm ( increased precision may require more time--> more delay in getting the readings)
}

void startRobot(int Sl = 0,int Sr = 0) {
  analogWrite(enableLeft, forwardLeftSpeed + Sl);
  analogWrite(enableRight, forwardRightSpeed + Sr);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
}

void moveBackward(int d = 0 ) {
  analogWrite(enableLeft, forwardLeftSpeed);
  analogWrite(enableRight, forwardRightSpeed);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
  delay(d);
  stopRobot();
}

void stopRobot(int d = 0) {
  // delay(d);
  analogWrite(enableLeft, 0);
  analogWrite(enableRight, 0);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
  delay(3000); // check and adjust
}

void moveForward(int l=0,int r=0) {
  int lSpeed = forwardLeftSpeed;
  int rSpeed = forwardRightSpeed;
  lSpeed = constrain(lSpeed, 65, 100);
  rSpeed = constrain(rSpeed, 65, 100);

  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  analogWrite(enableLeft, lSpeed);
  analogWrite(enableRight, rSpeed);

  long frontDist = readUltrasonic(triggerFront, echoFront);
  long leftDist = readUltrasonic(triggerLeft, echoLeft);
  long rightDist = readUltrasonic(triggerRight, echoRight);
  int initialDiffLeft = 0; // for adjusting and turning time
  int initialDiffRight = 0;
  while (!((frontDist < offset || frontDist > 300) || leftDist > offsetTurn || rightDist > offsetTurn)) {
    if(initialDiffLeft > (leftDist-readUltrasonic(triggerLeft, echoLeft))){
      adjustRight(lSpeed, rSpeed); // Too close to the left wall: adjust to the right
    }else if(initialDiffRight>(rightDist-readUltrasonic(triggerRight, echoRight))){
      adjustLeft(lSpeed, rSpeed);  // Too close to the right wall: adjust to the left
    }
    //   if (leftDist > rightDist) {
    //     adjustRight(lSpeed, rSpeed); // Too close to the left wall: adjust to the right
    //   } else {
    //     adjustLeft(lSpeed, rSpeed);  // Too close to the right wall: adjust to the left
    //   }
    // }
    delay(10);
    // Update sensor readings inside the loop
    frontDist = readUltrasonic(triggerFront, echoFront);
    leftDist = readUltrasonic(triggerLeft, echoLeft);
    rightDist = readUltrasonic(triggerRight, echoRight);
    robotAngle += calculateDeviationAngle(forwardLeftSpeed, forwardRightSpeed);
  }
  stopRobot(10);
  startRobot(l,r);
  long startf =millis();
  while((readUltrasonic(triggerFront, echoFront) > offsetMin) && ((millis()-startf) < 300)){ // move forward a bit more // check front >300
    delay(10);
  }
  startf=0;
  stopRobot(80);
  addToPath(1); //front =1
  F++;
}

void turnLeft() {

  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  analogWrite(enableLeft, turnLeftSpeed);
  analogWrite(enableRight, turnRightSpeed);

  float l1 = left.ping_cm();
  float f1, f = front.ping_cm();
  float r = right.ping_cm();
  delay(250);
  while (!(((f > (l1-3)) && (f < (l1+3))) || ((r > (f1-3)) && (r < (f1+3))))) {    // front sensor becomes left sensor and right sensor becomes  front sensor
    delay(10);
    f = front.ping_cm();
    r = right.ping_cm();
  }
  delay(75);
  stopRobot();
  // addToPath(2); // left =2
  // L++;
}

void turnRight() {

  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
  analogWrite(enableLeft, turnLeftSpeed);
  analogWrite(enableRight, turnRightSpeed);

  float r1 = right.ping_cm();
  float f1,f = front.ping_cm();
  float l = left.ping_cm();
  delay(250);
  while (!(((f > (r1-3)) && (f < (r1+3))) || ((l > (f1-3)) && (l < (f1+3)))))  {
    delay(15);
    f = front.ping_cm();
    l = left.ping_cm();
  }
  delay(75);
  // addToPath(3); // right = 3
  // R++;
}

void turnAround() {
  analogWrite(enableLeft, forwardLeftSpeed);
  analogWrite(enableRight, forwardRightSpeed);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);

  long fD = front.ping_cm();
  long lD = left.ping_cm();
  long rD = right.ping_cm();
  int startT= millis();
  while (!((fD > offset*3) && (lD < offset) && (rD < offset))) {
    delay(10);
    fD = front.ping_cm();
    lD = left.ping_cm();
    rD = right.ping_cm();
  }

  stopRobot();
  addToPath(4); // turnaround = 4 
  T++;

}

void adjustLeft(int &lSpeed, int &rSpeed) {
  lSpeed -= 1;
  rSpeed += 1;
  analogWrite(enableLeft, lSpeed);
  analogWrite(enableRight, rSpeed);
}

void adjustRight(int &lSpeed, int &rSpeed) {
  lSpeed += 1;
  rSpeed -= 1;
  analogWrite(enableLeft, lSpeed);
  analogWrite(enableRight, rSpeed);
}


// Function to add movement to the path
void addToPath(int movement) {
  if (pathIndex < 200) {
    path[pathIndex++] = movement;
  }
}
void backtrack() {
  // Remove the last movement from the path
  if (pathIndex > 0) {
    pathIndex--;  // Go back one step
  }
}

void followPath() {
  for (int i = 0; i < pathIndex; i++) {
    switch (path[i]) {
      case 'F':
        moveForward();
        break;
      case 'L':
        turnLeft();
        break;
      case 'R':
        turnRight();
        break;
      case 'B':
        turnAround();
        break;
    }
  }
}
void turnByAngle(float targetAngle) {
  // Convert target angle to required wheel difference
  float turnTime = abs(targetAngle) / 90.0 * 260; // Approximate turning time
  int turnSpeed = 80; // Adjust turn speed

  if (targetAngle > 0) {
    // Turn right
    analogWrite(enableLeft, turnSpeed);
    analogWrite(enableRight, 0);
    digitalWrite(leftForward, HIGH);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, LOW);
    digitalWrite(rightBackward, LOW);
  } else {
    // Turn left
    analogWrite(enableLeft, 0);
    analogWrite(enableRight, turnSpeed);
    digitalWrite(leftForward, LOW);
    digitalWrite(leftBackward, LOW);
    digitalWrite(rightForward, HIGH);
    digitalWrite(rightBackward, LOW);
  }

  delay(turnTime);
  stopRobot();
}
float calculateDeviationAngle(int leftPWM, int rightPWM) {
  // Estimate wheel speeds in cm/s based on PWM
  float leftSpeed = (leftPWM / 255.0) * CM_PER_REV;
  float rightSpeed = (rightPWM / 255.0) * CM_PER_REV;

  // Calculate distances covered in TIME_STEP
  float leftDistance = leftSpeed * (TIME_STEP / 1000.0);
  float rightDistance = rightSpeed * (TIME_STEP / 1000.0);

  // Calculate angular deviation
  float angleChange = (rightDistance - leftDistance) / WHEEL_BASE * (180.0 / PI); // Convert radians to degrees
  return angleChange;
}
