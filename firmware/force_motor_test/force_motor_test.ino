// --- BRUTE FORCE MOTOR TEST ---
// This code ignores all sensors and just tries to spin the motor.
// If this doesn't work, check your power, wiring, and Enable pin!

const int PIN_DIR = 8; 
const int PIN_PUL = 9; 

// If your driver has an EN (Enable) pin, define it here:
// const int PIN_EN = 10; 

void setup() {
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PUL, OUTPUT);
  
  // If you use an EN pin, uncomment these:
  // pinMode(PIN_EN, OUTPUT);
  // digitalWrite(PIN_EN, LOW); // Usually LOW is enabled
  
  digitalWrite(PIN_DIR, HIGH);
}

void loop() {
  // Move 200 steps (one full revolution on most motors)
  for(int i = 0; i < 200; i++) {
    digitalWrite(PIN_PUL, HIGH);
    delayMicroseconds(500); // 1ms pulse period = 1000 steps/sec
    digitalWrite(PIN_PUL, LOW);
    delayMicroseconds(500);
  }
  
  delay(1000); // Wait 1 second
  
  // Reverse direction
  digitalWrite(PIN_DIR, !digitalRead(PIN_DIR));
}
