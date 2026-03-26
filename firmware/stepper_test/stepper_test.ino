// Simple NEMA 17 Stepper Motor Diagnosis Test 
// Tests wiring between the Arduino and Stepper Driver

// According to your latest message:
const int PIN_DIR = 8; 
const int PIN_PUL = 9; 

void setup() {
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PUL, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("--- Stepper Motor Test Started ---");
  Serial.println("If wired correctly, motor will alternate directions every second.");
}

void loop() {
  // 1. Move Forward (Right)
  Serial.println("Moving Direction 1...");
  digitalWrite(PIN_DIR, HIGH); // Set forward direction
  
  for (int x = 0; x < 2000; x++) {
    // Generate a pulse
    digitalWrite(PIN_PUL, HIGH);
    delayMicroseconds(500); // Wait 500 microseconds
    digitalWrite(PIN_PUL, LOW);
    delayMicroseconds(500); // Total period 1000us = 1000 steps per second
  }

  // Rest for 1 second
  delay(1000);

  // 2. Move Backward (Left)
  Serial.println("Moving Direction 2...");
  digitalWrite(PIN_DIR, LOW); // Set reverse direction
  
  for (int x = 0; x < 2000; x++) {
    // Generate a pulse
    digitalWrite(PIN_PUL, HIGH);
    delayMicroseconds(500); 
    digitalWrite(PIN_PUL, LOW);
    delayMicroseconds(500); 
  }

  // Rest for 1 second
  delay(1000);
}
