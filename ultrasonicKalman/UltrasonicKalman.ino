// Ultrasonic + 2-state Kalman Filter (distance + velocity)
// For HC-SR04 style module (trigger/echo).
// Author: Dmytro Maslov, Porto, Portugal
// Hardware: Arduino Uno R3, HC-SR04 


// TODO: add the temperature/humidity update for sound speed velocity
// TODO: add green/blue/red as indicator of front/still/reverse movement

// Pin definitions
const int TRIG_PIN = 12;   // MISO output
const int ECHO_PIN = 11;   // PWM/MOSI output

// Speed of sound (m/s)
const float SPEED_OF_SOUND = 343.0f;

// Kalman filter state
// x = [distance; velocity]
float x00 = 0.0; // distance estimate (m)
float x10 = 0.0; // velocity estimate (m/s)

// Covariance matrix P (2x2)
// 2x2 matrix is on the limit of what UNO can do
float P00 = 1.0;
float P01 = 0.0;
float P10 = 0.0;
float P11 = 1.0;

// Process noise covariance Q 
// TODO: test this variable
float q_pos = 0.001f; // process noise for position
float q_vel = 0.01f;  // process noise for velocity

// Measurement noise R  (no data on sensor noise, assuming fololowing numbers)
float R_meas = 0.05f * 0.05f; // variance in meters^2 (e.g., 5 cm std -> 0.05^2)

// Timing
unsigned long lastMicros = 0;
unsigned long lastMeasurementMicros = 0;
float dt = 0.06f; // sensor reading max is 60 ms

// Useful constants
const unsigned long MIN_MEAS_INTERVAL_US = 20000UL; // 20 ms lower bound between pings

// Setup the pinout roles
void setup() {
  Serial.begin(9600);  // adjust this to the serial output baud rate
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  delay(100);

  // Serial.println(F("Ultrasonic readings + Kalman prediction (distance + velocity)"));
}

float measureDistanceMeters() {
  // Trigger a 30 microsecond pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(30);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo pulse width (microseconds)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30 ms timeout
  if (duration == 0) {
    // no echo (timeout) -> return NaN to indicate invalid measurement
    return NAN;
  }

  // distance = (time_us / 1e6) * speed_of_sound / 2
  float dist = (duration * 1e-6f) * SPEED_OF_SOUND * 0.5f;
  return dist;
}

void kalmanPredict(float dt_local) {
  // State prediction definition: x = F * x + omega
  // x_pos' = x_pos + dt * x_vel
  // x_vel' = x_vel
  float x_pos_pred = x00 + dt_local * x10;
  float x_vel_pred = x10;

  // Covariance prediction: P = F P F^T + Q
  // F = [1 dt; 0 1]
  // Compute P' entries
  // P00' = P00 + dt*(P10 + P01) + dt*dt*P11 + Q00
  // P01' = P01 + dt*P11 + Q01
  // P10' = P10 + dt*P11 + Q10
  // P11' = P11 + Q11

  float P00p = P00 + dt_local * (P10 + P01) + dt_local * dt_local * P11 + q_pos;
  float P01p = P01 + dt_local * P11;
  float P10p = P10 + dt_local * P11;
  float P11p = P11 + q_vel;

  // rewrite predictions
  x00 = x_pos_pred;
  x10 = x_vel_pred;

  P00 = P00p;
  P01 = P01p;
  P10 = P10p;
  P11 = P11p;
}

void kalmanUpdate(float z) {
  // Measurement residual: y = z - H x (H = [1 0])
  float y = z - x00; // scalar

  // S = H P H^T + R = P00 + R
  float S = P00 + R_meas;

  // Kalman gain K = P * H^T * S^-1 = [P00; P10] / S
  float K0 = P00 / S;
  float K1 = P10 / S;

  // State update x = x + K * y
  x00 = x00 + K0 * y;
  x10 = x10 + K1 * y;

  // Covariance matrix update P = (I - K H) P
  // (I - K H) = [[1-K0, -K0*0], [-K1, 1-K1*0]], but with H=[1 0], makes simplier version :
  // P = P - K * [P00 P01]
  // in plain cpp:
  float P00n = (1.0f - K0) * P00;
  float P01n = (1.0f - K0) * P01;
  float P10n = P10 - K1 * P00;
  float P11n = P11 - K1 * P01;

  P00 = P00n;
  P01 = P01n;
  P10 = P10n;
  P11 = P11n;
}

void loop() {
  unsigned long now = micros();
  // enforce minimum interval between measurements
  if (now - lastMeasurementMicros < MIN_MEAS_INTERVAL_US) {
    delayMicroseconds(100);
    return;
  }

  // measure dt from previous successful measurement time (in seconds)
  if (lastMeasurementMicros != 0) {
    dt = (now - lastMeasurementMicros) * 1e-6f;
    if (dt <= 0) dt = 0.05f;
  }
  lastMeasurementMicros = now;

  float z = measureDistanceMeters(); // measured distance (m) or NAN
  // Predict step (always run)
  kalmanPredict(dt);

  if (!isnan(z)) {
    // Update with measurement for non-NaNs
    kalmanUpdate(z);
  } else {
    // No valid measurement(we calculated NaN): skip update (only predicted state)
  }

  // Print raw and filtered (convert to cm for readability)
  Serial.print("raw_cm: ");
  if (!isnan(z)) {
    Serial.print(z * 100.0f, 1);
  } else {
    Serial.print("NaN");
  }
  Serial.print("  | filt_cm: ");
  Serial.print(x00 * 100.0f, 1);
  Serial.print("  | vel_cm_s: ");
  Serial.println(x10 * 100.0f, 2);

  // small delay to avoid overwhelming serial
  // careful not to make it too much more than the master-slave pinging cycle
  delay(20);
}
