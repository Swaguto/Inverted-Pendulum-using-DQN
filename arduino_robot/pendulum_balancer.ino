/*
 * ============================================================
 *  INVERTED PENDULUM — BALANCER (v7.0)
 * ============================================================
 *  Pins:
 *    PEND encoder → D2, D3  (hardware INT0/INT1)
 *    CART encoder → D7, D8  (PCINT)
 *    Stepper STEP → D11,  DIR → D10
 *
 *  Startup sequence (ABSOLUTE centering — no flags needed):
 *    1. Drive toward rail A (+stepper direction) until stall
 *    2. Drive toward rail B (-stepper direction) until stall
 *    3. Center = midpoint of railA and railB encoder counts
 *    4. Drive back to center
 *    5. Calibrate LIMIT_LEFT / LIMIT_RIGHT from measured rails
 *    6. Wait for pendulum to be lifted into balance window
 * ============================================================
 */

#include <AccelStepper.h>

// ── Pin Definitions ─────────────────────────────────────────
#define STEP_PIN 11
#define DIR_PIN 10
#define PEND_ENC_A 2 // INT0
#define PEND_ENC_B 3 // INT1
#define CART_ENC_A 7 // PCINT23 (PD7)
#define CART_ENC_B 8 // PCINT0  (PB0)

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ── Encoder Counts ──────────────────────────────────────────
volatile long pend_counts = 0;
volatile long cart_counts = 0;

// ── Pendulum ISRs — hardware interrupts ─────────────────────
void pend_isr_A() {
  pend_counts += (digitalRead(PEND_ENC_A) == digitalRead(PEND_ENC_B)) ? -1 : 1;
}
void pend_isr_B() {
  pend_counts += (digitalRead(PEND_ENC_A) != digitalRead(PEND_ENC_B)) ? -1 : 1;
}

// ── Cart ISRs — PCINT ────────────────────────────────────────
ISR(PCINT2_vect) {
  static bool last_a = false;
  bool a = !!(PIND & (1 << PD7));
  bool b = !!(PINB & (1 << PB0));
  if (a != last_a) {
    cart_counts += (a == b) ? -1 : 1;
    last_a = a;
  }
}
ISR(PCINT0_vect) {
  static bool last_b = false;
  bool a = !!(PIND & (1 << PD7));
  bool b = !!(PINB & (1 << PB0));
  if (b != last_b) {
    cart_counts += (a != b) ? -1 : 1;
    last_b = b;
  }
}

// ── Track limits — set dynamically by homeAndCenter() ───────
long LIMIT_LEFT = 0;
long LIMIT_RIGHT = 0;
long CENTER_VAL = 0;
const long SOFT_MARGIN = 2500;

// ── Speed / Accel ────────────────────────────────────────────
const float MAX_SPEED = 8000.0f;
const float HOME_SPEED = 1200.0f; // slow enough to be quiet
const float HOME_ACCEL = 2000.0f;

// ── Pendulum Config ──────────────────────────────────────────
const double PEND_CPR = 2400.0;
const float BALANCE_WINDOW = 20.0;

// ── Motor Direction ──────────────────────────────────────────
const bool MOTOR_REVERSED = true;

// ── PID Gains ────────────────────────────────────────────────
double KP = 700.0;
double KI = 0.0;
double KD = 50.0;
double KC = 0.10; // cart centering term

double pidIntegral = 0.0;
double pidLastError = 0.0;
unsigned long pidLastTime = 0;

// ── Atomic helpers ───────────────────────────────────────────
long readCart() {
  noInterrupts();
  long v = cart_counts;
  interrupts();
  return v;
}
long readPend() {
  noInterrupts();
  long v = pend_counts;
  interrupts();
  return v;
}

// ── Find one rail ────────────────────────────────────────────
// Motor runs at constant speed. We wait until the cart encoder
// has moved at least once (confirming the cart is actually moving),
// then declare a limit when the encoder stops changing for 500 ms.
// Prints every encoder change so you can watch it in Serial Monitor.
long findRail(float speed) {
  stepper.setSpeed(speed);

  long last100     = readCart() / 100L;  // track hundreds place — ignores ones/tens jitter
  bool moving      = false;
  unsigned long lastChange = 0;

  while (true) {
    stepper.runSpeed();
    long curr100 = readCart() / 100L;

    if (curr100 != last100) {          // hundreds place changed → cart genuinely moving
      Serial.print(F("enc: ")); Serial.println(curr100 * 100L);
      last100    = curr100;
      lastChange = millis();
      moving     = true;
    }

    // Hundreds place frozen for 500 ms → cart is at the rail
    if (moving && (millis() - lastChange > 500)) break;
  }

  stepper.setSpeed(0);
  stepper.runSpeed();
  return readCart();
}


// ── Absolute Homing ──────────────────────────────────────────
void homeAndCenter() {
  stepper.setMaxSpeed(HOME_SPEED);

  Serial.println(
      F("Finding rail A (encoder values will print while moving)..."));
  long railA = findRail(HOME_SPEED);
  Serial.print(F("  >> Rail A = "));
  Serial.println(railA);
  delay(300);

  Serial.println(F("Finding rail B..."));
  long railB = findRail(-HOME_SPEED);
  Serial.print(F("  >> Rail B = "));
  Serial.println(railB);
  delay(300);

  LIMIT_LEFT = max(railA, railB);
  LIMIT_RIGHT = min(railA, railB);
  CENTER_VAL = (LIMIT_LEFT + LIMIT_RIGHT) / 2L;

  Serial.print(F("  Left="));
  Serial.print(LIMIT_LEFT);
  Serial.print(F("  Right="));
  Serial.print(LIMIT_RIGHT);
  Serial.print(F("  Center="));
  Serial.println(CENTER_VAL);

  // Drive to center
  Serial.println(F("Moving to center..."));
  float centerSpd = (readCart() < CENTER_VAL) ? HOME_SPEED : -HOME_SPEED;
  stepper.setSpeed(centerSpd);
  while (abs(readCart() - CENTER_VAL) > 50)
    stepper.runSpeed();
  stepper.setSpeed(0);
  stepper.runSpeed();

  noInterrupts();
  cart_counts = CENTER_VAL;
  interrupts();
  stepper.setMaxSpeed(MAX_SPEED);

  Serial.println(F("Centered. Lift pendulum to begin balancing."));
}

// ── setup() ──────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Pendulum encoder — hardware external interrupts
  pinMode(PEND_ENC_A, INPUT_PULLUP);
  pinMode(PEND_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PEND_ENC_A), pend_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PEND_ENC_B), pend_isr_B, CHANGE);

  // Cart encoder — pin-change interrupts
  pinMode(CART_ENC_A, INPUT_PULLUP);
  pinMode(CART_ENC_B, INPUT_PULLUP);
  PCICR |= (1 << PCIE2) | (1 << PCIE0);
  PCMSK2 |= (1 << PCINT23);
  PCMSK0 |= (1 << PCINT0);

  stepper.setMinPulseWidth(5);
  stepper.setMaxSpeed(MAX_SPEED);
  pend_counts = 0;

  Serial.println(F("\n=== BALANCER v7.0 — Absolute Homing ==="));
  homeAndCenter();
}

// ── loop() ────────────────────────────────────────────────────
void loop() {
  long cartSnap = readCart();
  long pendSnap = readPend();

  // 1. Hard safety stop
  if (cartSnap > LIMIT_LEFT || cartSnap < LIMIT_RIGHT) {
    stepper.setSpeed(0);
    stepper.runSpeed();
    Serial.print(F("!! LIMIT HIT cart="));
    Serial.println(cartSnap);
    while (1)
      ;
  }

  // 2. Angle (0° = hanging, 180° = upright)
  double angle = (double)pendSnap * (360.0 / PEND_CPR);

  // 3. Error from upright, wrapped ±180°
  double error = angle - 180.0;
  while (error > 180.0)
    error -= 360.0;
  while (error < -180.0)
    error += 360.0;

  // 4. Control
  if (abs(error) > BALANCE_WINDOW) {
    stepper.setSpeed(0);
    stepper.runSpeed();
    pidIntegral = 0.0;
    pidLastError = error;
    pidLastTime = millis();

    static unsigned long lp = 0;
    if (millis() - lp > 500) {
      lp = millis();
      Serial.print(F("Waiting... ang="));
      Serial.println(angle, 1);
    }

  } else {
    unsigned long now = millis();
    double dt = (now - pidLastTime) / 1000.0;
    if (dt <= 0.0 || dt > 0.5)
      dt = 0.010;
    pidLastTime = now;

    pidIntegral += error * dt;
    double dError = (error - pidLastError) / dt;
    pidLastError = error;

    double output = (KP * error) + (KI * pidIntegral) + (KD * dError);

    // Cart centering nudge
    output += KC * (double)(cartSnap - CENTER_VAL);

    // Soft speed limit near either rail
    long distMin = min(abs(cartSnap - LIMIT_LEFT), abs(cartSnap - LIMIT_RIGHT));
    float softMax =
        (distMin < SOFT_MARGIN)
            ? max(800.0f, MAX_SPEED * (float)distMin / (float)SOFT_MARGIN)
            : MAX_SPEED;

    output = constrain(output, -softMax, softMax);
    stepper.setSpeed(MOTOR_REVERSED ? -output : output);
    stepper.runSpeed();
  }

  // 5. Debug @ 10 Hz
  static unsigned long rp = 0;
  if (millis() - rp > 100) {
    rp = millis();
    Serial.print(F("cnt:"));
    Serial.print(pendSnap);
    Serial.print(F(" ang:"));
    Serial.print(angle, 1);
    Serial.print(F(" err:"));
    Serial.print(error, 1);
    Serial.print(F(" cart:"));
    Serial.print(cartSnap);
    Serial.print(F(" ctr:"));
    Serial.println(CENTER_VAL);
  }
}
