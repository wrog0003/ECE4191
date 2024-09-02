// Define pins for encoder channels
#define encoder_ChA 7  // Channel A of the encoder connected to Digital Pin 7
#define encoder_ChB 8  // Channel B of the encoder connected to Digital Pin 8

// Variables to store the current state of encoder channels
int encoder_ChA_value;
int encoder_ChB_value;

void setup() {
   Serial.begin(9600); // Initialize serial communication at 9600 baud rate

   // Set encoder pins as inputs with internal pull-up resistors 
   // to avoid floating states and provide stable readings
   pinMode(encoder_ChA, INPUT_PULLUP); 
   pinMode(encoder_ChB, INPUT_PULLUP);
}

void loop() {
    // Read the current state of encoder channels
   encoder_ChA_value = digitalRead(encoder_ChA); 
   encoder_ChB_value = digitalRead(encoder_ChB); 
 
    // Print the encoder values to the serial monitor
   Serial.print(encoder_ChA_value); 
   Serial.print(" ");  // Separate the values with a space
   Serial.println(encoder_ChB_value);  // Print the second value and move to a new line
}