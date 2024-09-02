// Define pins for encoder channels
#define encoder1_ChA 7  // Channel A of the encoder 1 connected to Digital Pin 7
#define encoder1_ChB 8  // Channel B of the encoder 1 connected to Digital Pin 8
#define encoder2_ChA 3  // Channel A of the encoder 2 connected to Digital Pin 3
#define encoder2_ChB 4  // Channel B of the encoder 2 connected to Digital Pin 4

// Variables to store the current state of encoder channels
int encoder1_ChA_value;
int encoder1_ChB_value;
int encoder2_ChA_value;
int encoder2_ChB_value;

void setup() {
   Serial.begin(9600); // Initialize serial communication at 9600 baud rate

   // Set encoder pins as inputs with internal pull-up resistors 
   // to avoid floating states and provide stable readings
   pinMode(encoder1_ChA, INPUT_PULLUP); 
   pinMode(encoder1_ChB, INPUT_PULLUP);
   pinMode(encoder2_ChA, INPUT_PULLUP); 
   pinMode(encoder2_ChB, INPUT_PULLUP);
}

void loop() {
    // Read the current state of encoder channels
   encoder1_ChA_value = digitalRead(encoder1_ChA); 
   encoder1_ChB_value = digitalRead(encoder1_ChB); 
   encoder2_ChA_value = digitalRead(encoder2_ChA); 
   encoder2_ChB_value = digitalRead(encoder2_ChB); 
 
    // Print the encoder values to the serial monitor
   Serial.print(encoder1_ChA_value); 
   Serial.print(" ");  // Separate the values with a space
   Serial.print(encoder1_ChB_value);  // Print the second value
   Serial.print(" ");  // Separate the values with a space
   Serial.print(encoder2_ChA_value); 
   Serial.print(" ");  // Separate the values with a space
   Serial.println(encoder2_ChB_value);  // Print the second value and move to a new line

   // Print result example:
   // 1 0 0 1
   // 1ChA 1ChB 2ChA 2ChB
}