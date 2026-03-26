// Inverted Pendulum Hardware Controller
// Reads two rotary encoders via Pin Change Interrupts (PCINT)
// Commands a NEMA 17 stepper motor via STEP/DIR pins

// Pins
const int PIN_STEP = 9; // PUL
const int PIN_DIR = 8;  // DIR

// --- State Variables ---
volatile long pend_count = 0;
volatile long cart_count = 0;

volatile long target_velocity = 0;   // steps per second
volatile unsigned long step_interval_us = 0; 
unsigned long last_step_time = 0;

// Quadrature decoder lookup table
static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
static uint8_t pend_state = 0;
static uint8_t cart_state = 0;

void setup() {
  Serial.begin(115200);   // Fast serial for real-time control
  
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  // Pendulum Encoder on A4, A5
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  
  // Cart Encoder on A2, A3
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  
  // --- Enable PCINT for PORTC (Arduino Uno/Nano) ---
  // A2 = PCINT10, A3 = PCINT11, A4 = PCINT12, A5 = PCINT13
  PCICR |= (1 << PCIE1); 
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13);
}

// Software Interrupt Routine for the Encoders
ISR(PCINT1_vect) {
  uint8_t p = PINC; // Read the whole memory mapped port

  // Decode Cart (Bits 2 and 3 -> A2 and A3)
  uint8_t new_cart = (p >> 2) & 0x03;
  cart_state = ((cart_state << 2) | new_cart) & 0x0F;
  cart_count -= lookup_table[cart_state]; // subtracted or added depending on wiring direction

  // Decode Pendulum (Bits 4 and 5 -> A4 and A5)
  uint8_t new_pend = (p >> 4) & 0x03;
  pend_state = ((pend_state << 2) | new_pend) & 0x0F;
  pend_count += lookup_table[pend_state];
}

void loop() {
  // 1. Process Motor Stepping
  unsigned long now = micros();
  noInterrupts();
  unsigned long interval = step_interval_us;
  long current_vel = target_velocity;
  interrupts();

  if (interval > 0) {
    if (now - last_step_time >= interval) {
      last_step_time = now;
      digitalWrite(PIN_DIR, current_vel > 0 ? HIGH : LOW);
      digitalWrite(PIN_STEP, HIGH);
      delayMicroseconds(2);
      digitalWrite(PIN_STEP, LOW);
    }
  }

  // 2. Process High-Speed Serial Commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'S') {
      // Respond to state request
      noInterrupts();
      long p_count = pend_count;
      long c_count = cart_count;
      interrupts();
      
      Serial.print(c_count);
      Serial.print(",");
      Serial.println(p_count);
    } 
    else if (c == 'A') {
      // Respond to action command (e.g. "A1200\n" for 1200 steps/sec)
      long vel = Serial.parseInt();
      
      // Consume any trailing newline characters
      while(Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
        Serial.read();
      }
      
      noInterrupts();
      target_velocity = vel;
      if (vel == 0) {
        step_interval_us = 0;
      } else {
        step_interval_us = 1000000UL / abs(vel);
      }
      interrupts();
    }
  }
}
