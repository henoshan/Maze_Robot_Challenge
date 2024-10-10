// Motor Control Pins
#define enableLeft 10      // Enable pin for the left motor (ENA)
#define enableRight 11   // Enable pin for the right motor (ENB)

#define motorLeftForward 9   // IN1 pin for left motor forward
#define motorLeftBackward 8  // IN2 pin for left motor backwa#define motorRightForward 13  // IN3 pin for right motor forward
#define motorRightBackward 12 // IN4 pin for right motor backward
#define motorRightForward 13 // IN4 pin for right motor forward
// Speed Variables (0 - 255)
int leftMotorSpeed = 150;  // Speed for the left motor
int rightMotorSpeed = 150; // Speed for the right motor

void stopMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();

void setup() {
  // Set motor pins as outputs
  pinMode(enableLeft, OUTPUT);
  pinMode(enableRight, OUTPUT);
  
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);

  // Initial stop
  stopMotors();
}

void loop() {
  // Example sequence of movements
  moveForward();  // Move forward
  delay(2000);    // Move for 2 seconds
  stopMotors();
  delay(1000);
  // stopMotors();   // Stop motors
  // delay(1000);    // Wait for 1 second

  // turnLeft();     // Turn left
  // delay(1000);    // Turn for 1 second
  // stopMotors();   // Stop motors
  // delay(1000);    // Wait for 1 second

  // moveBackward(); // Move backward
  // delay(2000);    // Move for 2 seconds
  // stopMotors();   // Stop motors
  // delay(1000);    // Wait for 1 second

  // turnRight();    // Turn right
  // delay(1000);    // Turn for 1 second
  // stopMotors();   // Stop motors
  // delay(1000);    // Wait for 1 second
}


// Function to move forward
void moveForward() {
  // Set motor direction: Forward
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightBackward, LOW);
  
  // Control motor speed via enable pins
  // analogWrite(enableLeft, leftMotorSpeed);
  // analogWrite(enableRight, rightMotorSpeed);
}

// Function to move backward
void moveBackward() {
  // Set motor direction: Backward
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, HIGH);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, HIGH);
  
  // Control motor speed via enable pins
  analogWrite(enableLeft, leftMotorSpeed);
  analogWrite(enableRight, rightMotorSpeed);
}

// Function to turn left
void turnLeft() {
  // Set left motor to move backward and right motor to move forward
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, HIGH);
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightBackward, LOW);
  
  // Control motor speed via enable pins
  analogWrite(enableLeft, leftMotorSpeed);
  analogWrite(enableRight, rightMotorSpeed);
}

// Function to turn right
void turnRight() {
  // Set left motor to move forward and right motor to move backward
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, HIGH);
  
  // Control motor speed via enable pins
  analogWrite(enableLeft, leftMotorSpeed);
  analogWrite(enableRight, rightMotorSpeed);
}

// Function to stop the motors
void stopMotors() {
  // Set enable pins to 0 to stop motors
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftBackward, LOW);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightBackward, LOW);
}