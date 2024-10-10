// Pin definitions
int ENA = 9; // PWM pin to control speed
int IN1 = 7; // Direction pin 1
int IN2 = 8; // Direction pin 2

void setup() {
  // Set pins as output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Start with low speed
  analogWrite(ENA, 100);  // Set PWM duty cycle (0-255)
  digitalWrite(IN1, HIGH);  // Set direction
  digitalWrite(IN2, LOW);
}

void loop() {
  // You can change PWM value and direction to test further
}
