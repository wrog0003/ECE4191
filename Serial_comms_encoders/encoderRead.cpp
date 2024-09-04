// Pin definitions
const int motor1Pin1 = 6;   // Motor 1 direction pin 1
const int motor1Pin2 = 7;   // Motor 1 direction pin 2
const int motor2Pin1 = 8;   // Motor 2 direction pin 1
const int motor2Pin2 = 9;   // Motor 2 direction pin 2

const int encoderA1 = 2;  // Encoder 1 Channel A (must be an interrupt pin)
const int encoderB1 = 3;  // Encoder 1 Channel B (must be an interrupt pin)
const int encoderA2 = 4;  // Encoder 2 Channel A (must be an interrupt pin)
const int encoderB2 = 5;  // Encoder 2 Channel B (must be an interrupt pin)

const long targetCounts = 3590; // Target counts for one revolution

// Variables to store the counts
volatile long count1 = 0;
volatile long count2 = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set motor control pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Set up interrupts for encoder 1
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoder1B, CHANGE);

  // Set up interrupts for encoder 2
  attachInterrupt(digitalPinToInterrupt(encoderA2), handleEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB2), handleEncoder2B, CHANGE);

  // Run motors forward
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);

  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}

void loop() {
  // Print encoder values
  Serial.print("Encoder 1 Count: ");
  Serial.print(count1);
  Serial.print(" | Encoder 2 Count: ");
  Serial.println(count2);

  // Check if the motors have completed one revolution
  if (count1 >= targetCounts || count2 >= targetCounts) {
    stopMotors();
  }

  // Add a small delay for readability
  delay(100);
}

void stopMotors() {
  // Stop motors
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void handleEncoder1A() {
  if (digitalRead(encoderA1) == digitalRead(encoderB1)) {
    count1++;
  } else {
    count1--;
  }
}

void handleEncoder1B() {
  if (digitalRead(encoderA1) == digitalRead(encoderB1)) {
    count1--;
  } else {
    count1++;
  }
}

void handleEncoder2A() {
  if (digitalRead(encoderA2) == digitalRead(encoderB2)) {
    count2++;
  } else {
    count2--;
  }
}

void handleEncoder2B() {
  if (digitalRead(encoderA2) == digitalRead(encoderB2)) {
    count2--;
  } else {
    count2++;
  }
}