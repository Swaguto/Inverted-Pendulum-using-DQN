// Inverted Pendulum Hardware Controller
// Reads two rotary encoders via Pin Change Interrupts (PCINT)
// Commands a NEMA 17 stepper motor via STEP/DIR pins

// Pins
const int PIN_STEP = 8; // PUL
const int PIN_DIR = 9;  // DIR

// --- State Variables ---
volatile long pend_count = 0;
volatile long cart_count = 0;
volatile long int_count = 0;

volatile long target_velocity = 0;   // steps per second (goal)
float current_velocity = 0;          // actual current steps per second
const float acceleration = 250000.0; // steps per second^2 (hyper-speed ramp)
unsigned long last_ramp_time = 0;

volatile unsigned long step_interval_us = 0; 
unsigned long last_step_time = 0;

// Quadrature decoder lookup table
static const int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
volatile uint8_t pend_state = 0;
volatile uint8_t cart_state = 0;

void setup() {
  Serial.begin(250000);   
  
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  // Pendulum (A4, A5), Cart (A2, A3)
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  
  // Initialize states
  uint8_t p = PINC;
  cart_state = (p >> 2) & 0x03;
  pend_state = (p >> 4) & 0x03;

  // --- Enable PCINT for PORTC ---
  PCICR |= (1 << PCIE1); 
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13);
  
  last_ramp_time = micros();
  Serial.println("READY");
}

// Software Interrupt Routine for the Encoders
ISR(PCINT1_vect) {
  uint8_t p = PINC; 
  int_count++;

  // Decode Cart (Bits 2 and 3 -> A2 and A3)
  uint8_t new_cart = (p >> 2) & 0x03;
  cart_state = ((cart_state << 2) | new_cart) & 0x0F;
  cart_count += lookup_table[cart_state]; 

  // Decode Pendulum (Bits 4 and 5 -> A4 and A5)
  uint8_t new_pend = (p >> 4) & 0x03;
  pend_state = ((pend_state << 2) | new_pend) & 0x0F;
  pend_count += lookup_table[pend_state];
}

void loop() {
  unsigned long now = micros();

  // 1. Velocity Ramping (Linear Acceleration)
  float dt = (now - last_ramp_time) / 1000000.0;
  if(dt > 0.05) dt = 0.05; // Cap dt for stability
  last_ramp_time = now;

  noInterrupts();
  long goal_vel = target_velocity;
  interrupts();

  if (current_velocity != goal_vel) {
    float diff = goal_vel - current_velocity;
    float step_change = acceleration * dt;
    
    if (abs(diff) <= step_change) {
      current_velocity = goal_vel;
    } else {
      current_velocity += (diff > 0 ? step_change : -step_change);
    }
    
    // Update interval based on current ramped velocity
    if (abs(current_velocity) < 10) {
      step_interval_us = 0;
    } else {
      step_interval_us = 1000000UL / abs(current_velocity);
    }
  }

  // 2. Process Motor Stepping
  if (step_interval_us > 0) {
    if (now - last_step_time >= step_interval_us) {
      last_step_time = now;
      digitalWrite(PIN_DIR, current_velocity > 0 ? HIGH : LOW);
      digitalWrite(PIN_STEP, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_STEP, LOW);
    }
  }

  // 3. Process High-Speed Serial Commands
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'S') {
      noInterrupts();
      long p_count = pend_count;
      long c_count = cart_count;
      interrupts();
      
      Serial.print(c_count);
      Serial.print(",");
      Serial.println(p_count);
    } 
    else if (c == 'A') {
      long vel = Serial.parseInt();
      
      while(Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
        Serial.read();
      }
      
      noInterrupts();
      target_velocity = vel;
      interrupts();
    }
  }
}
