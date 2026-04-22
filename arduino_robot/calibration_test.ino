// Auto Torque-Limit Finder
// Tries speeds from 8000 down to 1000 in steps of 500
// Records the highest speed where encoder actually moves

#include <AccelStepper.h>
#include <Encoder.h>

#define STEP_PIN 11
#define DIR_PIN 10
#define ENC_A 2
#define ENC_B 3

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder myEncoder(ENC_A, ENC_B);

const long LIMIT_RIGHT = -27000;
const long LIMIT_LEFT  = -500;

float testSpeed = 8000;       // Start from max
const float SPEED_STEP = 500; // Reduce by this each test
const float MIN_SPEED = 1000; // Lowest we'll go

float lastWorkingSpeed = 0;
bool testRunning = false;
unsigned long testStart = 0;
long encAtStart = 0;

void setup() {
  Serial.begin(115200);
  stepper.setMaxSpeed(testSpeed);
  myEncoder.write(0);
  stepper.setCurrentPosition(0);
  delay(1000);
  runTest();
}

void runTest() {
  Serial.print("Testing speed: "); Serial.println(testSpeed);
  stepper.setMaxSpeed(testSpeed);
  stepper.setSpeed(testSpeed); // go right
  encAtStart = myEncoder.read();
  testStart = millis();
  testRunning = true;
}

void loop() {
  if (!testRunning) return;

  long enc = myEncoder.read();

  // Safety: stop at rails
  if (enc <= LIMIT_RIGHT) {
    stepper.setSpeed(-testSpeed);
  } else if (enc >= LIMIT_LEFT) {
    stepper.setSpeed(testSpeed);
  }

  stepper.runSpeed();

  // After 1.5s, check if encoder moved
  if (millis() - testStart > 1500) {
    testRunning = false;
    long movement = abs(myEncoder.read() - encAtStart);

    if (movement > 100) {
      lastWorkingSpeed = testSpeed;
      Serial.print("  OK — encoder moved "); Serial.print(movement); Serial.print(" counts @ speed "); Serial.println(testSpeed);
      testSpeed -= SPEED_STEP;
      if (testSpeed >= MIN_SPEED) {
        runTest();
      } else {
        stepper.setSpeed(0);
        Serial.println("=============================");
        Serial.print("MAX SAFE SPEED: "); Serial.println(lastWorkingSpeed);
        Serial.println("=============================");
        // Set bounce at the safe speed
        stepper.setSpeed(lastWorkingSpeed);
        testRunning = true; // resume bouncing at safe speed
      }
    } else {
      stepper.setSpeed(0);
      Serial.print("  STALL @ speed "); Serial.println(testSpeed);
      Serial.println("=============================");
      Serial.print("MAX SAFE SPEED: "); Serial.println(lastWorkingSpeed);
      Serial.println("=============================");
      // Continue bounce at safe speed
      testSpeed = lastWorkingSpeed;
      stepper.setMaxSpeed(testSpeed);
      stepper.setSpeed(testSpeed);
      testRunning = true;
    }
  }
}
