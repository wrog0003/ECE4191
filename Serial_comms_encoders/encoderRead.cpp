// Pin definitions
const int encoderA1 = 2;  // Encoder 1 Channel A (must be an interrupt pin)
const int encoderB1 = 3;  // Encoder 1 Channel B (must be an interrupt pin)
const int encoderA2 = 4;  // Encoder 2 Channel A (must be an interrupt pin)
const int encoderB2 = 5;  // Encoder 2 Channel B (must be an interrupt pin)

// Variables to store the counts
volatile long count1 = 0;
volatile long count2 = 0;

unsigned long lastMillis = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set up interrupts for encoder 1
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoder1B, CHANGE);

  // Set up interrupts for encoder 2
  attachInterrupt(digitalPinToInterrupt(encoderA2), handleEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB2), handleEncoder2B, CHANGE);
}

void loop() {
  // Send data to Raspberry Pi every 1 ms
  if (millis() - lastMillis >= 1) {
    lastMillis = millis();
    Serial.print(count1);
    Serial.print(",");
    Serial.println(count2);
  }
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
