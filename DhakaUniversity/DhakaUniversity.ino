// --- Motor Pins (PWM on IN1 & IN3) ---
#define IN1 9   // Left forward (PWM)
#define IN2 6   // Left backward
#define IN3 10  // Right forward (PWM)
#define IN4 11  // Right backward

// --- Sensor Pins ---
int sensors[6] = {A0, A1, A2, A3, A4, A5};
int sensorValue[6];

// --- Sensor Threshold ---
int threshold = 500; // Adjust based on testing

// --- PID Control ---
float Kp = 20;   // Proportional constant
float Kd = 8;    // Derivative constant
int baseSpeed = 150; // Base motor speed (0-255)
int lastError = 0;

// --- Weights for sensors ---
int weights[6] = {-5, -3, -1, 1, 3, 5};

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  for (int i = 0; i < 6; i++) {
    pinMode(sensors[i], INPUT);
  }

  Serial.begin(9600);
}

void loop() {
  int error = 0, activeCount = 0;

  // --- Read sensors & calculate position error ---
  for (int i = 0; i < 6; i++) {
    sensorValue[i] = (analogRead(sensors[i]) > threshold) ? 0 : 1;
    if (sensorValue[i] == 1) {
      error += weights[i];
      activeCount++;
    }
    Serial.print(sensorValue[i]);
    Serial.print(" ");
  }
  Serial.println();

  if (activeCount > 0) {
    error /= activeCount;
  }

  // --- PID Calculation ---
  int P = error * Kp;
  int D = (error - lastError) * Kd;
  int correction = P + D;

  lastError = error;

  // --- Motor speed adjustment ---
  int leftSpeed = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  // Constrain PWM values
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  moveMotors(leftSpeed, rightSpeed);
}

// --- Motor Control Function ---
void moveMotors(int leftPWM, int rightPWM) {
  // Left motor forward
  digitalWrite(IN2, LOW); // backward pin off
  analogWrite(IN1, leftPWM);

  // Right motor forward
  digitalWrite(IN4, LOW); // backward pin off
  analogWrite(IN3, rightPWM);
}
