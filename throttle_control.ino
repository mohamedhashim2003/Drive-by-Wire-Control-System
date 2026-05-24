#include <math.h> // For log & sqrt used in ECO/SPORT curves

// ── Pin Definitions (Not final!!!, Validate them with waly) ────────────────────────────────────────────────
const int PIN_PEDAL = A2;
const int PIN_TPS   = A0;
const int PIN_ENA   = 9;
const int PIN_IN1   = 7;
const int PIN_IN2   = 8;

// ── Drive Modes ────────────────────────────────────────────────────
enum DriveMode { ECO = 0, NORMAL = 1, SPORT = 2 }; //so that it appears Current Mode = Mode not 0,1,2 on python
DriveMode currentMode = NORMAL;

// ── PID Tuning (Not final, essam will tune them on MATLAB) ──────────────────────────────────────────────────────
float Kp = 2.0;  //float is better than define so that I can send tuning commands over Serial directly in python
float Ki = 0.05;
float Kd = 0.8;
float integral      = 0.0; //accumulates error × dt every cycle
float previousError = 0.0; //calculate the derivative: (currentError - previousError) / dt
unsigned long lastTime = 0; // stores the timestamp of the last loop execution in milliseconds

// ── Config ─────────────────────────────────────────────────────────
const int   PWM_MIN        = 60;    //below this generates a magnetic field that isn't strong enough to move the shaft
const int   PWM_MAX        = 255;
const float ERROR_DEADBAND = 2.0;   //So that sensor error is not accounted for
const int   LOOP_MS        = 10;    //100Hz
const float INTEGRAL_LIMIT = 100.0; //integral term cap
const float IDLE_CRACK_DEG = 5.0;   //Running engines throttle doesnt' drop to zero to protect the throttle body mechanically

// ──────────────────────────────────────────────────────────────────
void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  motorStop();

  Serial.begin(115200);
  Serial.setTimeout(10); // FIX: prevent readStringUntil from blocking the PID loop for up to 1000ms
  Serial.println("pedal,tps,error,pwm,mode");
  lastTime = millis();
}


void loop() {
// ── Read incoming mode and tuning commands from GUI ────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // Process Drive Mode Changes
    if      (cmd == "M0") currentMode = ECO;
    else if (cmd == "M1") currentMode = NORMAL;
    else if (cmd == "M2") currentMode = SPORT;
    
    // Process Live Tuning Packets (Expected Format: "T2.5,0.05,0.8")
    else if (cmd.startsWith("T")) {
      String values = cmd.substring(1); // Strip the 'T' header
      
      int firstComma  = values.indexOf(',');
      int secondComma = values.indexOf(',', firstComma + 1);
      
      if (firstComma != -1 && secondComma != -1) {
        float newKp = values.substring(0, firstComma).toFloat();
        float newKi = values.substring(firstComma + 1, secondComma).toFloat();
        float newKd = values.substring(secondComma + 1).toFloat();

        // FIX: reject zero or negative gains — they invert or kill the control law
        if (newKp > 0.0 && newKi >= 0.0 && newKd >= 0.0) {
          Kp = newKp;
          Ki = newKi;
          Kd = newKd;
          // Flush historical error memory to prevent sudden integral spikes
          integral = 0.0;
        }
      }
    }
  }

  unsigned long now = millis();
  // FIX: use unsigned subtraction so millis() rollover (~49.7 days) is handled correctly
  float dt = (float)(now - lastTime) / 1000.0;
  if (dt < LOOP_MS / 1000.0) return;
  lastTime = now;

  float pedalDeg = analogRead(PIN_PEDAL) * (90.0 / 1023.0); //Multiplying by 90.0/1023.0 maps that to 0–90 degrees
  float tpsDeg   = analogRead(PIN_TPS)   * (90.0 / 1023.0);

  float targetDeg = computeTarget(pedalDeg, currentMode);
  if (targetDeg < IDLE_CRACK_DEG) targetDeg = IDLE_CRACK_DEG;

  float error = targetDeg - tpsDeg;

  if (abs(error) < ERROR_DEADBAND) {
    motorStop();
    integral = 0;
    previousError = error;
    logSerial(pedalDeg, tpsDeg, 0, 0);
    return;
  }

  integral += error * dt;
  integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = (error - previousError) / dt;
  previousError = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int pwm = constrain((int)abs(output), 0, PWM_MAX);
  if (pwm > 0 && pwm < PWM_MIN) pwm = PWM_MIN;

  if      (output > 0) motorForward(pwm);
  else if (output < 0) motorReverse(pwm);
  else                 motorStop();

  logSerial(pedalDeg, tpsDeg, error, (output > 0 ? pwm : -pwm));
}

// ──────────────────────────────────────────────────────────────────
float computeTarget(float pedalDeg, DriveMode mode) {
  float p = pedalDeg / 90.0;

  switch (mode) {
    case ECO:    return 90.0 * (log(1.0 + p * (M_E - 1.0))); //natural log curve, small pedal inputs give even smaller throttle responses
    case NORMAL: return pedalDeg; //1:1 linear
    case SPORT:  return 90.0 * sqrt(p); // Square root grows very fast near zero then slows so the first 30% of pedal travel opens 55% of the valve.
  }
  return pedalDeg;
}

// ── Motor Control ──────────────────────────────────────────────────
void motorForward(int pwm) {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, pwm);
}

void motorReverse(int pwm) {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  analogWrite(PIN_ENA, pwm);
}

void motorStop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_ENA, 0);
}

// ── Serial Output ──────────────────────────────────────────────────
void logSerial(float pedal, float tps, float error, int pwm) {
  Serial.print(pedal, 1);   Serial.print(",");
  Serial.print(tps, 1);     Serial.print(",");
  Serial.print(error, 1);   Serial.print(",");
  Serial.print(pwm);        Serial.print(",");
  Serial.println(currentMode);
}
