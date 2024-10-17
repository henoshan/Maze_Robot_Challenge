#Lets start Guys
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
int offset = 30;
int offsetMin = 15;
const int moveForwardDelay = 200;
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
  // analogWrite(enableLeft, forwardLeftSpeed);  // Use forwardLeftSpeed for enableLeft
  // analogWrite(enableRight, forwardRightSpeed);  // Use forwardRightSpeed for enableRight
  stopRobot();
}

void loop() {
  // Read distances from the ultrasonic sensors
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


  if (frontDist < offsetMin && leftDist < offsetMin && rightDist < offsetMin) {
    stopRobot();
    turnAround();
  } else if (leftDist > offset || rightDist > offset) {
    // Obstacle ahead: choose left or right
    if ((leftDist > offset && rightDist > offset) ||(leftDist < offset && rightDist > offset)) {
      stopRobot();
      turnRight();
    } else if (leftDist > offset && rightDist < offset) {
      stopRobot();
      turnLeft();
    }
  } else{
      // Move forward 
      moveForward();
  }
  // moveForward();
  // delay(4000); // Small delay for testing
}

                            // Functions //
// Ultrasonic sensor function
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(8); // check and increase for better iterative reading
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(40);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Distance in cm ( increased precision may require more time--> more delay in getting the readings)
}
void startRobot(){
  analogWrite(enableLeft, forwardLeftSpeed); 
  analogWrite(enableRight, forwardRightSpeed); 
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
}
void stopRobot() {
  analogWrite(enableLeft, 0);  
  analogWrite(enableRight, 0);  
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
  delay(200); // check and adjust
}
void moveForward() {
  int lSpeed = forwardLeftSpeed;
  int rSpeed = forwardRightSpeed;
  // Constrain speed values
  lSpeed = constrain(lSpeed, 0, 255);
  rSpeed = constrain(rSpeed, 0, 255);
  analogWrite(enableLeft, lSpeed); 
  analogWrite(enableRight, rSpeed); 
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
  
  // Keep moving forward until the front ultrasonic sensor detects an obstacle
  while (!((readUltrasonic(trigFront, echoFront) < offset) || (readUltrasonic(trigLeft, echoLeft) > offset) || (readUltrasonic(trigRight, echoRight) > offset))) {
    if (readUltrasonic(trigLeft, echoLeft) > readUltrasonic(trigRight, echoRight)) {
      adjustRight(lSpeed,rSpeed);       // Too close to the left wall: adjust to the right
    } else {
      adjustLeft(lSpeed,rSpeed);        // Too close to the right wall: adjust to the left
    }
    // if (leftDist < 7) {
    //   adjustRight(lSpeed,rSpeed);       // Too close to the left wall: adjust to the right
    // } else if (rightDist < 7) {
    //   adjustLeft(lSpeed,rSpeed);
    // }       // Too close to the right wall: adjust to the left           
    // delay(5);
  }
  //check where the robot stops when moving forward
  // delay(50);
  delay(moveForwardDelay); // move forward a bit more to reach the center
  stopRobot();
}

void turnLeft() {
  startRobot();
  delay(400); 
  // Rotate the robot left
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);

  analogWrite(enableLeft, turnLeftSpeed);  
  analogWrite(enableRight, turnRightSpeed); 

  // Keep turning left until there's no obstacle ahead
  while (!(readUltrasonic(trigLeft, echoLeft) < offset)) { // will not work if no block infront
    // delay(10);
  }
  delay(200); //turn to 90 degree// check and change

  stopRobot(); // stop for a few ms
  startRobot();
  delay(moveForwardDelay); // move forward a bit after turning
  stopRobot();
}

void turnRight() {  
  startRobot();
  delay(400);   
  // Rotate the robot right
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);

  analogWrite(enableLeft, turnLeftSpeed);  
  analogWrite(enableRight, turnRightSpeed); 
  
  //Keep turning right until there's no obstacle ahead
  while (!(readUltrasonic(trigRight, echoRight) < offset)) {
    // delay(10);
  }
  delay(100);// check and change //turn to 90 degree
  
  stopRobot();
  startRobot();
  delay(moveForwardDelay); // move forward a bit after turning

  stopRobot();
}

void turnAround() {
  analogWrite(enableLeft, forwardLeftSpeed);  
  analogWrite(enableRight, forwardRightSpeed);  
  // Turn the robot 180 degrees
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);

  // Keep turning around until the front sensor detects a clear path
  while (!((readUltrasonic(trigFront, echoFront) > offset) && (readUltrasonic(trigLeft, echoLeft) < offsetMin) && (readUltrasonic(trigRight, echoRight) < offsetMin))) {
    // delay(10);
  }

  stopRobot();
}
void adjustLeft(int &lSpeed, int &rSpeed) {
  lSpeed -=1; // check and remove
  rSpeed +=1;
  analogWrite(enableLeft, lSpeed);  
  analogWrite(enableRight, rSpeed);  
}
void adjustRight(int &lSpeed, int &rSpeed) {
  lSpeed +=1;
  rSpeed -=1; // check and remove
  analogWrite(enableLeft, lSpeed);  
  analogWrite(enableRight, rSpeed); 
}
