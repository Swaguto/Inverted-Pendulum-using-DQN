/*
 * ============================================================
 *  INVERTED PENDULUM — HARDWARE DIAGNOSTIC
 * ============================================================
 *  Use this to verify your encoders are working correctly.
 *  
 *  1. Open Serial Monitor at 115200 baud.
 *  2. Move the cart by hand — 'CartPos' should change.
 *  3. Rotate the pendulum — 'PendDeg' should change.
 *  
 *  USER MAPPING:
 *  - 0° = START / HANGING DOWN
 *  - 180° = UPRIGHT / BALANCED
 *  
 *  PIN ASSIGNMENTS:
 *  Cart Enc: D2/D3 (Int) | Pend Enc: D7/D8 (Poll)
 * ============================================================
 */

#include <Encoder.h>

#define CART_ENC_A  2
#define CART_ENC_B  3
#define PEND_ENC_A  7
#define PEND_ENC_B  8

Encoder cartEnc(CART_ENC_A, CART_ENC_B);
Encoder pendEnc(PEND_ENC_A, PEND_ENC_B);

const double PEND_CPR = 2400.0; // 600 PPR * 4 for quadrature

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n--- HARDWARE DIAGNOSTIC START ---"));
  Serial.println(F("Rotate sensors manually to verify readings."));
  Serial.println(F("Press 'Z' in serial monitor to zero both encoders.\n"));
}

void loop() {
  long cartPos = cartEnc.read();
  long pendRaw = pendEnc.read();
  
  // Angle calculated from the starting point (0 = HANGING)
  double angleDeg = (double)pendRaw * (360.0 / PEND_CPR);
  
  // Standardize so it's always +/- 180 or +/- 360
  // (We'll keep it raw here to help identify drift)

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) { 
    lastPrint = millis();
    
    Serial.print(F("Cart Raw: "));     Serial.print(cartPos);
    Serial.print(F("\tPend Raw: "));    Serial.print(pendRaw);
    Serial.print(F("\tPend Deg: "));    Serial.print(angleDeg, 1);
    
    // Help visualizer based on user request (180 = UP)
    double absDeg = abs(angleDeg);
    if (abs(absDeg - 180.0) < 15.0) {
      Serial.print(F(" [UPRIGHT]"));
    } else if (absDeg < 15.0) {
      Serial.print(F(" [DOWN]"));
    }
    
    Serial.println();
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'Z' || c == 'z') {
      cartEnc.write(0);
      pendEnc.write(0);
      Serial.println(F("\n>>> ENCODERS ZEROED <<<\n"));
    }
  }
}
