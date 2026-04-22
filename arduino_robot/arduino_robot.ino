/*
 * Inverted Pendulum - Curiosity-Creates Style Hardware Integration
 * Hardware: Stepper Motor (NEMA 17) + Rotary Encoders (A0/A1, A4/A5)
 */

#include <AccelStepper.h>

#define STEP_PIN 11
#define DIR_PIN 10
#define ENCODER_PEND_A A2
#define ENCODER_PEND_B A3
#define ENCODER_CART_A A4
#define ENCODER_CART_B A5

volatile long pend_count = 0;
volatile long cart_count = 0;
long target_speed = 0; // Steps per second
unsigned long last_step_time = 0;
unsigned long step_interval = 0; // Microseconds between steps
bool step_dir = true;

// Hard limits based on encoder (0 = left rail, 28800 = right rail)
const long CART_MIN = 200;   // Safety buffer from left end
const long CART_MAX = 28600; // Safety buffer from right end

// PCINT1 for Port C (A0-A5)
ISR(PCINT1_vect) {
  static bool last_pend_a = false;
  static bool last_cart_a = false;

  // A2/A3 for Pendulum (bits 2/3)
  bool pend_a = (PINC & (1 << PINC2));
  bool pend_b = (PINC & (1 << PINC3));
  // A4/A5 for Cart (bits 4/5)
  bool cart_a = (PINC & (1 << PINC4));
  bool cart_b = (PINC & (1 << PINC5));

  if (pend_a != last_pend_a) {
    if (pend_a == pend_b)
      pend_count++;
    else
      pend_count--;
  }
  last_pend_a = pend_a;

  if (cart_a != last_cart_a) {
    if (cart_a == cart_b)
      cart_count++;
    else
      cart_count--;
  }
  last_cart_a = cart_a;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_PEND_A, INPUT_PULLUP);
  pinMode(ENCODER_PEND_B, INPUT_PULLUP);
  pinMode(ENCODER_CART_A, INPUT_PULLUP);
  pinMode(ENCODER_CART_B, INPUT_PULLUP);

  PCICR |= (1 << PCIE1);
  // Unmask A2, A3, A4, A5 (PCINT 10, 11, 12, 13)
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13);

  Serial.setTimeout(2); // Fix for slow parseInt behavior
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'V') {
      target_speed = Serial.parseInt();
      if (target_speed == 0) {
        step_interval = 0;
      } else {
        step_interval = 1000000UL / abs(target_speed);
        step_dir = (target_speed > 0);
        digitalWrite(DIR_PIN, step_dir ? HIGH : LOW);
      }
    } else if (cmd == 'R') {
      target_speed = 0;
      step_interval = 0;
      pend_count = 0;
      cart_count = 0;
    } else if (cmd == 'X') {
      target_speed = 0;
      step_interval = 0;
    }
  }

  // --- Manual High-Speed Pulsing with Hard Encoder Limits ---
  if (step_interval > 0) {
    // Check bounds BEFORE pulsing
    bool at_min = (cart_count <= CART_MIN && !step_dir); // Moving left past min
    bool at_max = (cart_count >= CART_MAX && step_dir);  // Moving right past max

    if (at_min || at_max) {
      // Hard stop — kill the motor
      step_interval = 0;
      target_speed = 0;
    } else if (micros() - last_step_time >= step_interval) {
      last_step_time = micros();
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
      if (step_dir) cart_count++;
      else          cart_count--;
    }
  }

  static unsigned long last_send = 0;
  if (millis() - last_send > 20) {
    last_send = millis();
    Serial.print("S,");
    Serial.print(pend_count); // Transmitting what Python expects as parts[1]
    Serial.print(",");
    Serial.print(target_speed);
    Serial.print(",");
    Serial.println(cart_count); // Transmitting what Python expects as parts[3]
  }
}
