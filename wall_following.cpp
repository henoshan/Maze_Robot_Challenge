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
int offset = 25;
int offsetMin = 15;
// const int moveForwardDelay = 10;
int L, R, F, T = 0;
// char path[100];  // Array to store moves
// int pathIndex = 0;  // Current index of the path array

// Function Prototypes
long readUltrasonic(int trigPin, int echoPin);
void moveForward();
void startRobot();
void stopRobot();
void turnLeft();
void turnRight();
void turnAround();
void adjustLeft(int &lSpeed, int &rSpeed);
void adjustRight(int &lSpeed, int &rSpeed);
void moveBackward();
// void followPath();
// void backtrack();
// void addToPath(char movement);
// long readUltrasonicFiltered(int trigPin, int echoPin); 

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

  Serial.begin(9600); // for testing ultrasonic
  stopRobot();
}

void loop() {
  // Read distances from the ultrasonic sensors and store them in variables
  long frontDist = readUltrasonic(trigFront, echoFront);
  long leftDist = readUltrasonic(trigLeft, echoLeft);
  long rightDist = readUltrasonic(trigRight, echoRight);
  // int irValue = analogRead(irPin); // Read IR sensor value
  Serial.print(" iam loop ");

  // // Check for the endpoint using the IR sensor
  // if (irValue < 200) { // Adjust threshold based on your IR sensor
  //   stopRobot();
  //   Serial.println("End Point Reached!");
  //   delay(5000); // Stop at the end point
  // }

  if ((frontDist < offsetMin || frontDist > 250) && leftDist < offsetMin && rightDist < offsetMin) {
    stopRobot();
    turnAround();
  } else if (leftDist > offset || rightDist > offset) {
    // Obstacle ahead: choose left or right
    if ((leftDist > offset && rightDist > offset) || (leftDist < offset && rightDist > offset)) {
      stopRobot();
      turnRight();
    } else if (leftDist > offset && rightDist < offset) {
      stopRobot();
      turnLeft();
    }
  } else if ((frontDist > offset) && (leftDist < offsetMin) && (rightDist < offsetMin)) {
    moveForward();// Move forward
  } else if (frontDist < 5 || frontDist > 250) {
    moveBackward();
  }
}

// Functions //

// Ultrasonic sensor function
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4); // check and increase for better iterative reading
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH,15000); //max range 255cm
  return duration * 0.034 / 2; // Distance in cm ( increased precision may require more time--> more delay in getting the readings)
}
// long readUltrasonicFiltered(int trigPin, int echoPin) {  // filtered for echo
//   long totalDistance = 0;
//   int numReadings = 5; // Take 5 readings and average them

//   for (int i = 0; i < numReadings; i++) {
//     totalDistance += readUltrasonic(trigPin, echoPin);
//     delay(10); // Delay between each reading to avoid echo overlap
//   }
  
//   return totalDistance / numReadings; // Return the average distance
// }

void startRobot() {
  analogWrite(enableLeft, forwardLeftSpeed);
  analogWrite(enableRight, forwardRightSpeed);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
}

void moveBackward() {
  analogWrite(enableLeft, forwardLeftSpeed);
  analogWrite(enableRight, forwardRightSpeed);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
  delay(200);
  stopRobot();
}

void stopRobot() {
  analogWrite(enableLeft, 0);
  analogWrite(enableRight, 0);
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
  delay(100); // check and adjust
}

void moveForward() {
  int lSpeed = forwardLeftSpeed;
  int rSpeed = forwardRightSpeed;
  lSpeed = constrain(lSpeed, 0, 255);
  rSpeed = constrain(rSpeed, 0, 255);

  analogWrite(enableLeft, lSpeed);
  analogWrite(enableRight, rSpeed);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);

  Serial.println("Moving forward");
  long frontDist = readUltrasonic(trigFront, echoFront);
  long leftDist = readUltrasonic(trigLeft, echoLeft);
  long rightDist = readUltrasonic(trigRight, echoRight);

  while (!(frontDist < offset || frontDist > 300 || leftDist > offset || rightDist > offset)) {
    if (leftDist > rightDist) {
      adjustRight(lSpeed, rSpeed); // Too close to the left wall: adjust to the right
    } else {
      adjustLeft(lSpeed, rSpeed);  // Too close to the right wall: adjust to the left
    }
    delay(100);

    // Update sensor readings inside the loop
    frontDist = readUltrasonic(trigFront, echoFront);
    leftDist = readUltrasonic(trigLeft, echoLeft);
    rightDist = readUltrasonic(trigRight, echoRight);
  }

  stopRobot();
  // addToPath('F');
  F++;
}

void turnLeft() {
  if(L>=1){  // dont move forward if there is a opening in the start
  //  if(!(path[pathIndex]=='R' || path[pathIndex]=='L')){
  //   startRobot();
  //   delay(300);
  //   stopRobot();
  //  }
    startRobot();
    delay(300);
    stopRobot();
  }
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);

  analogWrite(enableLeft, turnLeftSpeed);
  analogWrite(enableRight, turnRightSpeed);

  long leftDist = readUltrasonic(trigLeft, echoLeft);

  while (!(leftDist < offset)) {
    leftDist = readUltrasonic(trigLeft, echoLeft);
  }
  delay(50); // turn to 90 degrees
  stopRobot();
  // addToPath('L');
  L++;
  startRobot();
  delay(250);
  stopRobot();
}

void turnRight() {
  if(R>=1){  // dont move forward if there is a opening in the start
  //  if(!(path[pathIndex]=='R' || path[pathIndex]=='L')){ // for continuous turns
  //   startRobot();
  //   delay(300);
  //   stopRobot();
  //  }
    startRobot();
    delay(300);
    stopRobot();
  }
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);

  analogWrite(enableLeft, turnLeftSpeed);
  analogWrite(enableRight, turnRightSpeed);

  long rightDist = readUltrasonic(trigRight, echoRight);

  while (!(rightDist < offset)) {
    rightDist = readUltrasonic(trigRight, echoRight);
  }
  delay(50);
  stopRobot();
  // addToPath('R');
  R++;
  startRobot();
  delay(250);   // for contiuous turns it will fail
  stopRobot();
}

void turnAround() {
  analogWrite(enableLeft, forwardLeftSpeed);
  analogWrite(enableRight, forwardRightSpeed);
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);

  long frontDist = readUltrasonic(trigFront, echoFront);
  long leftDist = readUltrasonic(trigLeft, echoLeft);
  long rightDist = readUltrasonic(trigRight, echoRight);

  while (!((frontDist > offset) && (leftDist < offsetMin) && (rightDist < offsetMin))) {
    frontDist = readUltrasonic(trigFront, echoFront);
    leftDist = readUltrasonic(trigLeft, echoLeft);
    rightDist = readUltrasonic(trigRight, echoRight);
  }

  stopRobot();
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
// void addToPath(char movement) {
//   if (pathIndex < 100) {
//     path[pathIndex++] = movement;
//   }
// }
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

