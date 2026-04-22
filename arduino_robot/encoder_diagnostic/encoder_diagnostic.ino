/*
 * ============================================================
 *  ENCODER DIAGNOSTIC
 * ============================================================
 *  Opens Serial at 115200. Shows live encoder counts + raw pin
 *  states so you can verify which physical wires go to which pins.
 *
 *  Serial commands:
 *    r  → reset both encoder counts to 0
 *    p  → print current pin assignments
 *    1  → swap encoder 1 pins (A↔B) to fix reversed direction
 *    2  → swap encoder 2 pins (A↔B) to fix reversed direction
 *
 *  Edit ENC1_A/B and ENC2_A/B below to match your wiring,
 *  then re-flash and watch the counts update.
 * ============================================================
 */

// ── Edit these to match your physical wiring ───────────────
#define ENC1_A  2    // Encoder 1 — channel A pin
#define ENC1_B  3    // Encoder 1 — channel B pin

#define ENC2_A  7    // Encoder 2 — channel A pin
#define ENC2_B  8    // Encoder 2 — channel B pin
// ───────────────────────────────────────────────────────────

volatile long enc1 = 0;
volatile long enc2 = 0;

// ── Encoder 1 ISRs (hardware interrupts on pins 2 & 3) ─────
void enc1_isr_A() {
  enc1 += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? -1 : 1;
}
void enc1_isr_B() {
  enc1 += (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ? -1 : 1;
}

// ── Encoder 2 ISRs (PCINT on pins 7 & 8) ──────────────────
ISR(PCINT2_vect) {
  static bool last_a = false;
  bool a = !!(PIND & (1 << PD7));
  bool b = !!(PINB & (1 << PB0));
  if (a != last_a) { enc2 += (a == b) ? -1 : 1; last_a = a; }
}
ISR(PCINT0_vect) {
  static bool last_b = false;
  bool a = !!(PIND & (1 << PD7));
  bool b = !!(PINB & (1 << PB0));
  if (b != last_b) { enc2 += (a != b) ? -1 : 1; last_b = b; }
}

void setup() {
  Serial.begin(115200);

  // Encoder 1 — hardware interrupts
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1_isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), enc1_isr_B, CHANGE);

  // Encoder 2 — PCINT
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  PCICR  |= (1 << PCIE2) | (1 << PCIE0);
  PCMSK2 |= (1 << PCINT23);  // PD7 = pin 7
  PCMSK0 |= (1 << PCINT0);   // PB0 = pin 8

  Serial.println(F("=== ENCODER DIAGNOSTIC ==="));
  Serial.println(F("Commands: r=reset  p=pins  1=swap enc1  2=swap enc2"));
  Serial.println(F(""));
  printPins();
}

void printPins() {
  Serial.println(F("--- Current pin assignments ---"));
  Serial.print(F("  Encoder 1: A=D")); Serial.print(ENC1_A);
  Serial.print(F("  B=D")); Serial.println(ENC1_B);
  Serial.print(F("  Encoder 2: A=D")); Serial.print(ENC2_A);
  Serial.print(F("  B=D")); Serial.println(ENC2_B);
  Serial.println(F("-------------------------------"));
}

void loop() {
  // Handle Serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r' || cmd == 'R') {
      noInterrupts(); enc1 = 0; enc2 = 0; interrupts();
      Serial.println(F(">> Encoders reset to 0"));
    } else if (cmd == 'p' || cmd == 'P') {
      printPins();
    } else if (cmd == '1') {
      Serial.println(F(">> Direction fix: negate enc1 (swap A<->B effect)"));
      noInterrupts(); enc1 = -enc1; interrupts();
    } else if (cmd == '2') {
      Serial.println(F(">> Direction fix: negate enc2 (swap A<->B effect)"));
      noInterrupts(); enc2 = -enc2; interrupts();
    }
  }

  // Print live values at 10 Hz
  static unsigned long last = 0;
  if (millis() - last > 100) {
    last = millis();

    noInterrupts();
    long e1 = enc1;
    long e2 = enc2;
    interrupts();

    // Raw pin states
    bool a1 = digitalRead(ENC1_A);
    bool b1 = digitalRead(ENC1_B);
    bool a2 = digitalRead(ENC2_A);
    bool b2 = digitalRead(ENC2_B);

    Serial.print(F("ENC1[D")); Serial.print(ENC1_A);
    Serial.print(F(",")); Serial.print(ENC1_B); Serial.print(F("]: "));
    Serial.print(e1);
    Serial.print(F("  (A=")); Serial.print(a1);
    Serial.print(F(" B=")); Serial.print(b1); Serial.print(F(")"));

    Serial.print(F("    ENC2[D")); Serial.print(ENC2_A);
    Serial.print(F(",")); Serial.print(ENC2_B); Serial.print(F("]: "));
    Serial.print(e2);
    Serial.print(F("  (A=")); Serial.print(a2);
    Serial.print(F(" B=")); Serial.print(b2); Serial.println(F(")"));
  }
}
