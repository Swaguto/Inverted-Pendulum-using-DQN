#include <AccelStepper.h>

// PIN SETTINGS (Matches original arduino_robot setup)
#define STEP_PIN 11
#define DIR_PIN 10
#define ENCODER_A 2
#define ENCODER_B 3

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
volatile long encoder_count = 0;

void setup() {
  Serial.begin(115200);
  
  // ENCODER SETUP (LPD3806 Wiring: Green=2, White=3)
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR_B, CHANGE);

  // STEPPER PERFORMANCE (EXTREME SPEED)
  stepper.setMaxSpeed(20000); // 20kHz pulse rate
  stepper.setAcceleration(50000); // Snap to speed
  
  Serial.println("--- HIGH SPEED SERVO TEST STARTING ---");
  Serial.println("Moving between 0 and 28500...");
  delay(1000);
}

void loop() {
  // Move to 28500
  stepper.moveTo(28500);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    printStatus();
  }
  delay(500);

  // Move back to 0
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    printStatus();
  }
  delay(500);
}

void printStatus() {
  static unsigned long last_print = 0;
  if (millis() - last_print > 50) {
    last_print = millis();
    Serial.println(encoder_count);
  }
}

// LPD3806 ISR Logic (From user snippet)
void encoderISR_A() {
  encoder_count += digitalRead(ENCODER_A) == digitalRead(ENCODER_B) ? -1 : 1;
}

void encoderISR_B() {
  encoder_count += digitalRead(ENCODER_A) != digitalRead(ENCODER_B) ? -1 : 1;
}
