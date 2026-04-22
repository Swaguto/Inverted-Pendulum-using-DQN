#include <AccelStepper.h>
#include <Encoder.h>

// USER VERIFIED PINS
#define STEP_PIN 10
#define DIR_PIN 11
#define ENC_A 2
#define ENC_B 3

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Encoder myEncoder(ENC_A, ENC_B);

void setup() {
  Serial.begin(115200);
  
  // High-performance stepper settings
  stepper.setMaxSpeed(8000); 
  stepper.setAcceleration(50000);
  
  Serial.println("--- ENCODER FOLLOWER MODE ACTIVE ---");
  Serial.println("Rotate the encoder to move the cart!");
}

void loop() {
  // Read the physical encoder position
  long currentEncoderPos = myEncoder.read();
  
  // Command the stepper to match this position exactly
  stepper.moveTo(currentEncoderPos);
  
  // Execute moves
  stepper.run();
  
  // Periodic Debug Output
  static unsigned long last_print = 0;
  if (millis() - last_print > 100) {
    last_print = millis();
    Serial.print("Target (Enc): "); Serial.print(currentEncoderPos);
    Serial.print(" | Actual (Step): "); Serial.println(stepper.currentPosition());
  }
}
