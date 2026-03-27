/*
 * Inverted Pendulum Hardware Firmware
 * Hardware: Stepper Motor (NEMA 17) + Rotary Encoders
 * Comm: Serial @ 115200 Baud
 */

#include <AccelStepper.h>

// --- PIN DEFINITIONS ---
#define STEP_PIN 2
#define DIR_PIN 3
#define ENCODER_PEND_A 18  // Must support interrupts
#define ENCODER_PEND_B 19
#define ENCODER_CART_A 20
#define ENCODER_CART_B 21

// --- CONSTANTS ---
const float STEPS_PER_REV = 200.0;
const float MM_PER_REV = 40.0; // Example for GT2 pulley

// --- GLOBALS ---
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
volatile long pend_count = 0;
volatile long cart_count = 0;

// Interrupt Service Routines for Encoders
void doEncoderPend() {
  if (digitalRead(ENCODER_PEND_A) == digitalRead(ENCODER_PEND_B)) pend_count++;
  else pend_count--;
}

void doEncoderCart() {
  if (digitalRead(ENCODER_CART_A) == digitalRead(ENCODER_CART_B)) cart_count++;
  else cart_count--;
}

void setup() {
  Serial.begin(115200);
  
  // Setup Encoders
  pinMode(ENCODER_PEND_A, INPUT_PULLUP);
  pinMode(ENCODER_PEND_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PEND_A), doEncoderPend, CHANGE);
  
  pinMode(ENCODER_CART_A, INPUT_PULLUP);
  pinMode(ENCODER_CART_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CART_A), doEncoderCart, CHANGE);

  // Setup Stepper
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(10000);
}

void loop() {
  // 1. Check for incoming commands from PC
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("V")) { // Velocity Command: V<speed>
      float speed = cmd.substring(1).toFloat();
      stepper.setSpeed(speed);
    } 
    else if (cmd.startsWith("R")) { // Reset/Home Command
      pend_count = 0;
      cart_count = 0;
      stepper.setCurrentPosition(0);
    }
  }

  // 2. Run Stepper (Constant speed mode)
  stepper.runSpeed();

  // 3. Optional: Send state back to PC every 20ms (50Hz)
  static unsigned long last_send = 0;
  if (millis() - last_send > 20) {
    last_send = millis();
    // Format: S,<cart_pos>,<cart_vel>,<pend_angle>,<pend_vel>
    Serial.print("S,");
    Serial.print(cart_count);
    Serial.print(",");
    Serial.print(stepper.speed());
    Serial.print(",");
    Serial.println(pend_count);
  }
}
