// LIBRARY-FREE MANUAL PULSE TEST
// Pins: D10 (DIR), D11 (STEP)
// LPD3806 Encoder: D2 (Green), D3 (White)

const int stepPin = 11;
const int dirPin = 10;
const int encoderA = 2;
const int encoderB = 3;

volatile long encoderCount = 0;

void setup() {
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), isrB, CHANGE);

  Serial.println("--- MANUAL NO-LIBRARY TEST STARTING ---");
  Serial.println("Direction: Forward to 28500...");
}

void loop() {
  // --- Forward Sequence ---
  digitalWrite(dirPin, HIGH); 
  for(long i = 0; i < 28500; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(200); // 1 / (200us * 2) = 2500 Hz (SAFE SPEED)
    digitalWrite(stepPin, LOW);
    delayMicroseconds(200);
    
    if(i % 500 == 0) printStatus(i);
  }
  delay(1000);

  // --- Backward Sequence ---
  digitalWrite(dirPin, LOW);
  for(long i = 28500; i > 0; i--) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(200);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(200);
    
    if(i % 500 == 0) printStatus(i);
  }
  delay(1000);
}

void printStatus(long stepperVal) {
  Serial.print("Stepper: "); Serial.print(stepperVal);
  Serial.print(" | Encoder: "); Serial.println(encoderCount);
}

void isrA() {
  encoderCount += digitalRead(encoderA) == digitalRead(encoderB) ? -1 : 1;
}

void isrB() {
  encoderCount += digitalRead(encoderA) != digitalRead(encoderB) ? -1 : 1;
}
