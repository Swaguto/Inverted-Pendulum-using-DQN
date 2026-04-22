#include <AccelStepper.h>
#include <Encoder.h>

// Original Hardware Pins
#define STEP_PIN 11
#define DIR_PIN 10
#define ENC_A A2
#define ENC_B A3

const bool inverted = false;
const int stepMode = 3;
const int stepModes[6][5] = {
  {5, 1, 0, 0, 0},
  {10, 2, 1, 0, 0},
  {20, 4, 0, 1, 0},
  {40, 8, 1, 1, 0},
  {80, 16, 0, 0, 1},
  {160, 32, 1, 1, 1}
};
double stepsPerMM;
int outputDir;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// PID controller variables
double p_error, i_error, d_error, last_error;
unsigned long last_pid_time;
double kp, ki, kd, input, output, setPoint, scaleFactor;

Encoder encoder(ENC_A, ENC_B);

void setup() {
  Serial.begin(115200);

  if (inverted) {
    outputDir = 1;
    kp = 100.00;
    ki = 10500.00;
    kd = 0.00;
    scaleFactor = -1/37.5;
  } else {
    outputDir = -1;
    kp = 250.00;
    ki = 0.00;
    kd = 5.00;
    scaleFactor = -1/100;
  }
  
  stepsPerMM = stepModes[stepMode][0];
  stepper.setMaxSpeed(5000); 
  stepper.setMinPulseWidth(5);
  
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(6, stepModes[stepMode][2]);
  digitalWrite(7, stepModes[stepMode][3]);
  digitalWrite(8, stepModes[stepMode][4]);

  setPoint = 0;
  last_pid_time = millis();

  if (inverted) {
    encoder.write(-1006);
  } else {
    encoder.write(0);
  }

  while(encoder.read() < 0) {
    // NOOP
  }
}

void loop() {
  unsigned long now = millis();
  double dt = (now - last_pid_time) / 1000.0;
  if (dt <= 0) dt = 0.01; // Avoid division by zero

  int count = encoder.read();
  double ang = count * (360.00 / 2000.00);
  
  // State calculation
  input = scaleFactor * stepper.currentPosition() / stepsPerMM + 200 * sin(ang * (PI / 180));
  double error = setPoint - input;
  
  // Manual PID calculation
  p_error = error;
  i_error += error * dt;
  d_error = (error - last_error) / dt;
  
  output = (kp * p_error) + (ki * i_error) + (kd * d_error);
  
  // Clamp output to stepper limits
  if (output > 5000) output = 5000;
  if (output < -5000) output = -5000;

  stepper.setSpeed(outputDir * output);
  stepper.runSpeed();

  last_error = error;
  last_pid_time = now;
  
  // Periodic status for debugging
  static unsigned long last_print = 0;
  if (millis() - last_print > 100) {
    last_print = millis();
    Serial.print("Ang: "); Serial.print(ang);
    Serial.print(" Pos: "); Serial.print(stepper.currentPosition());
    Serial.print(" Out: "); Serial.println(output);
  }
}
