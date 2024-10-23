#include <Arduino.h>

// Define pins for ultrasonic sensors
#define trigFront 5
#define echoFront 4
#define trigLeft 2
#define echoLeft 3
#define trigRight 7
#define echoRight 6
// #define irPin A0 // IR Sensor

// Motor control pins
#define enableLeft 11
#define leftForward 12
#define leftBackward 13
#define rightForward 8
#define rightBackward 9
#define enableRight 10

// Robot speed
int forwardLeftSpeed = 80; // for moving forward--> adjusted
int forwardRightSpeed = 86;
int turnLeftSpeed = 80;  // for turning right,Left and around --> constant
int turnRightSpeed = 86;
int offset = 12;
int offsetMin = 8;
int offsetTurn = 18; //18
int L, R, F, T = 0;
int path[200];  // Array to store moves
int pathIndex = 0;  // Current index of the path array

// Function Prototypes
long readUltrasonic(int trigPin, int echoPin);
void moveForward();
void startRobot(int Sl,int Sr);
void stopRobot(int d=1);
void turnLeft();
void turnRight();
void turnAround();
void adjustLeft(int &lSpeed, int &rSpeed);
void adjustRight(int &lSpeed, int &rSpeed);
void moveBackward(int d);
// void followPath();
// void backtrack();
void addToPath(char movement);
long readUltrasonicFiltered(int trigPin, int echoPin); 

void setup() {
  // Initialize sensor pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);
  // pinMode(irPin, INPUT);

  // Initialize motor control pins
  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(enableLeft, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(enableRight, OUTPUT);

  // Serial.begin(9600); // for testing ultrasonic
  stopRobot(0);
}

void loop() {
  delay(50);
  // Read distances from the ultrasonic sensors and store them in variables
  long frontDist = readUltrasonic(trigFront, echoFront);
  long leftDist = readUltrasonic(trigLeft, echoLeft);
  long rightDist = readUltrasonic(trigRight, echoRight);
  // int irValue = analogRead(irPin); // Read IR sensor value
  // // Check for the endpoint using the IR sensor
  // if (irValue < 200) { // Adjust threshold based on your IR sensor
  //   stopRobot();
  //   Serial.println("End Point Reached!");
  //   delay(5000); // Stop at the end point
  // }

  if(leftDist > offsetTurn || rightDist > offsetTurn){
    if((rightDist > offsetTurn) && (leftDist > offsetTurn)){
      if(readUltrasonic(trigFront, echoFront) < offsetMin ){
          moveBackward(150);
      }
      if(R%4==0){
        turnLeft();
      }else{
        turnRight();
      }
    }else if(rightDist > offsetTurn && leftDist < offsetTurn){
      if(readUltrasonic(trigFront, echoFront) < offsetMin ){
        moveBackward(150);
      }
      turnRight();
    }else{
      if(readUltrasonic(trigFront, echoFront) < offsetMin ){
        moveBackward(150);
      }
      turnLeft();   
    }
  }else{
    frontDist = readUltrasonic(trigFront, echoFront);
    if(frontDist < offset || frontDist > 300){   //&&(rightDist < offset)&&(leftDist < offset)
      if(readUltrasonic(trigFront, echoFront) < offsetMin ){
        moveBackward(150);
      }
      turnAround();
    }else moveForward();
  }
  // move backward if too clode to wall
  frontDist = readUltrasonic(trigFront, echoFront);
  if(frontDist < offsetMin || frontDist > 300){
    moveBackward(100);
  }
}

// Functions //
// Ultrasonic sensor function
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10); // check and increase for better iterative reading
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(40);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH,20000); //limit max range
  if(duration == 0){
    duration=20000;
  }
  delay(10);
  return duration * 0.034 / 2; // Distance in cm ( increased precision may require more time--> more delay in getting the readings)
}

void startRobot(int Sl,int Sr) {
  analogWrite(enableLeft, forwardLeftSpeed + Sl);
  analogWrite(enableRight, forwardRightSpeed + Sr);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
}

void moveBackward(int d) {
  analogWrite(enableLeft, forwardLeftSpeed);
  analogWrite(enableRight, forwardRightSpeed);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
  delay(d);
  stopRobot();
}

void stopRobot(int d) {
  delay(d);
  analogWrite(enableLeft, 0);
  analogWrite(enableRight, 0);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
  delay(300); // check and adjust
}

void moveForward() {
  int lSpeed = forwardLeftSpeed;
  int rSpeed = forwardRightSpeed;
  lSpeed = constrain(lSpeed, 70, 100);
  rSpeed = constrain(rSpeed, 70, 100);

  analogWrite(enableLeft, lSpeed);
  analogWrite(enableRight, rSpeed);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);

  long frontDist = readUltrasonic(trigFront, echoFront);
  long leftDist = readUltrasonic(trigLeft, echoLeft);
  long rightDist = readUltrasonic(trigRight, echoRight);
  int initialDiff = abs(leftDist-rightDist);
  while (!((frontDist < offset || frontDist > 300) || leftDist > offsetTurn || rightDist > offsetTurn)) {
    if(initialDiff != abs(leftDist-rightDist)){
      if (leftDist > rightDist) {
        adjustRight(lSpeed, rSpeed); // Too close to the left wall: adjust to the right
      } else {
        adjustLeft(lSpeed, rSpeed);  // Too close to the right wall: adjust to the left
      }
    }
    delay(10);
    // Update sensor readings inside the loop
    frontDist = readUltrasonic(trigFront, echoFront);
    leftDist = readUltrasonic(trigLeft, echoLeft);
    rightDist = readUltrasonic(trigRight, echoRight);
  }
  stopRobot(10);
  startRobot(0,0);
  long startf =millis();
  while((readUltrasonic(trigFront, echoFront) > offsetMin-3) && ((millis()-startf) < 300)){ // move forward a bit more // check front >300
    delay(10);
  }
  startf=0;
  stopRobot(80);
  addToPath(1); //front =1
  F++;
}

void turnLeft() {
  // if(readUltrasonic(trigFront, echoFront) < offset ){
  //   moveBackward(150);
  // }
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  // analogWrite(enableLeft, turnLeftSpeed);
  analogWrite(enableRight, turnRightSpeed);
  long startl =millis();
  while ((readUltrasonic(trigLeft, echoLeft) > offsetMin) && ((millis()-startl) < 530)) {
    delay(20);
  }
  stopRobot();
  addToPath(2); // left =2
  L++;

  startRobot(0,4);
  startl =millis();
  while((readUltrasonic(trigFront, echoFront) > offset) && ((millis()-startl) < 260)){ // move forward a bit more
    delay(10);
  }
  startl=0;
  stopRobot(40);
}

void turnRight() {
  // if(readUltrasonic(trigFront, echoFront) < offset ){
  //   moveBackward(150);
  // }
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
  analogWrite(enableLeft, turnLeftSpeed);
  // analogWrite(enableRight, turnRightSpeed);
  long startr =millis();
  while ((readUltrasonic(trigRight, echoRight) > offsetMin) && ((millis()-startr) < 530)) {
    delay(20);
  }
  stopRobot();
  addToPath(3); // right = 3
  R++;
  startRobot(4,0);
  startr =millis();
  while((readUltrasonic(trigFront, echoFront) > offset) && ((millis()-startr) < 260)){ // move forward a bit more
    delay(10);
  }
  startr=0;
  stopRobot(40);
}

void turnAround() {
  analogWrite(enableLeft, forwardLeftSpeed-10);
  analogWrite(enableRight, forwardRightSpeed-9);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);

  long fD = readUltrasonic(trigFront, echoFront);
  long lD = readUltrasonic(trigLeft, echoLeft);
  long rD = readUltrasonic(trigRight, echoRight);
  int startT= millis();
  while (!((fD > offset*2) && (lD < offset) && (rD < offset))) {
    delay(10);
    fD = readUltrasonic(trigFront, echoFront);
    lD = readUltrasonic(trigLeft, echoLeft);
    rD = readUltrasonic(trigRight, echoRight);
    if ((millis()-startT) >=1000){
      break;
    }
  }

  stopRobot();
  addToPath(4); // turnaround = 4 
  // backtrack(); // remove the last movement
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
// void backtrack() {
//   // Remove the last movement from the path
//   if (pathIndex > 0) {
//     pathIndex--;  // Go back one step
//   }
// }

// void followPath() {
//   for (int i = 0; i < pathIndex; i++) {
//     switch (path[i]) {
//       case 'F':
//         moveForward();
//         break;
//       case 'L':
//         turnLeft();
//         break;
//       case 'R':
//         turnRight();
//         break;
//       case 'B':
//         turnAround();
//         break;
//     }
//   }
// }
