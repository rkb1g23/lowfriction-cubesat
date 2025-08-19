#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_VEML7700.h>
#include <Adafruit_TCS34725.h>

#define PCA9546A_ADDR 0x70

const int pwmModePin = 3;  // Mode switch channel
const int pwmYawPin = 6;   // Yaw control stick input
#define IN1 8
#define IN2 9
#define PWM 10
#define LED_PIN 13

// Light Sensors and IMU
ICM_20948_I2C imu;
Adafruit_VEML7700 veml1;
Adafruit_VEML7700 veml2;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);

// Modes
enum Mode { MODE_UNKNOWN, MODE_LIGHT_TRACKING, MODE_STANDBY, MODE_YAW_CONTROL };
Mode currentMode = MODE_STANDBY;

// PID for Light Tracking
float light_Kp = 1.5, light_Ki = 0.0, light_Kd = 0.3;
float light_error = 0, light_previousError = 0, light_integral = 0, light_derivative = 0, light_output = 0;
const float AMBIENT_MARGIN = 12000.0;
const float LOCK_TOLERANCE = 0.25;
const int SEARCH_PWM = 255;
bool lightDetected = false;
uint16_t ambientC = 0;

// RC yaw control
const int pwmMin = 982;
const int pwmMax = 1998;
const int deadZoneCenter = 1490;
const int deadZoneRange = 100;
const int motorMin = -255;
const int motorMax = 255;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(pwmModePin, INPUT);
  pinMode(pwmYawPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  enableChannel(0);
  if (imu.begin() != ICM_20948_Stat_Ok) while (1);

  enableChannel(1);
  if (!veml1.begin()) while (1);

  enableChannel(2);
  if (!veml2.begin()) while (1);

  enableChannel(3);
  if (!tcs.begin()) while (1);

  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  ambientC = c;

  stopMotor();
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  unsigned long pulseWidth = pulseIn(pwmModePin, HIGH, 100000);
  Mode newMode = currentMode;
  if (pulseWidth > 800 && pulseWidth < 1100) newMode = MODE_LIGHT_TRACKING;
  else if (pulseWidth >= 1100 && pulseWidth < 1600) newMode = MODE_STANDBY;
  else if (pulseWidth >= 1900) newMode = MODE_YAW_CONTROL;

  if (newMode != currentMode) {
    currentMode = newMode;
    lightDetected = false;
    light_integral = 0;
    stopMotor();
    digitalWrite(LED_PIN, LOW);
  }

  switch (currentMode) {
    case MODE_LIGHT_TRACKING: lightTrackingLoop(); break;
    case MODE_YAW_CONTROL: yawControlLoop(); break;
    case MODE_STANDBY: stopMotor(); break;
    default: break;
  }
}

// ------------------ Light Tracking ------------------
void lightTrackingLoop() {
  enableChannel(3);
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  Serial.print("Clear channel: "); Serial.println(c);

  // Detect if flashlight is present
  if (!lightDetected && c > ambientC + AMBIENT_MARGIN) {
    lightDetected = true;
    Serial.println("Flashlight detected! Switching to tracking mode.");
    delay(100);  // Small delay before transitioning to active tracking
  }

  // If no light detected yet, stay in search spin
  if (!lightDetected) {
    Serial.println("Searching for light...");
    searchSpin();
    return;
  }

  // Light detected, perform tracking using VEML sensors
  enableChannel(1);
  float lux1 = veml1.readLux();
  enableChannel(2);
  float lux2 = veml2.readLux();
  float luxSum = lux1 + lux2 + 1e-6;  // avoid division by zero
  float normalizedError = (lux2 - lux1) / luxSum;

  // PID calculation
  light_error = normalizedError;
  light_integral += light_error;
  light_derivative = light_error - light_previousError;
  light_output = (light_Kp * light_error) + (light_Ki * light_integral) + (light_Kd * light_derivative);
  light_previousError = light_error;

  // Debug output
  Serial.print("Lux1: "); Serial.print(lux1);
  Serial.print(" | Lux2: "); Serial.print(lux2);
  Serial.print(" | Error: "); Serial.print(light_error, 4);
  Serial.print(" | PID Output: "); Serial.println(light_output, 4);

  // Actuate motor to correct
  if (abs(light_error) < LOCK_TOLERANCE) {
    Serial.println("Light centered (within tolerance). Minor corrections.");
    digitalWrite(LED_PIN, HIGH);
    stopMotor();  // You can choose to keep motor idle or make tiny corrections
  } else {
    digitalWrite(LED_PIN, LOW);
    int pwmVal = constrain(abs(light_output) * 255, 80, 255);
    Serial.print("Tracking light | Direction: ");
    Serial.print(light_output > 0 ? "Left" : "Right");
    Serial.print(" | Speed: "); Serial.println(pwmVal);
    moveMotor(pwmVal, light_output > 0 ? -1 : 1);
  }
}



// ------------------ Yaw Manual Control ------------------
void yawControlLoop() {
  // Read the PWM signal from the input pin
  int pwmValue = pulseIn(pwmYawPin, HIGH);
  Serial.println(pwmValue);
  // Check if the PWM signal is valid
  if (pwmValue == 0) {
    Serial.println("No valid PWM signal detected");
    return;
  }

  // Check for dead zone
  if (pwmValue > (deadZoneCenter - deadZoneRange) && pwmValue < (deadZoneCenter + deadZoneRange)) {
    analogWrite(PWM, 0);  // Stop motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    Serial.println("Motor stopped");
  } else {
    // Map PWM value to motor speed
    int motorSpeed;
    if (pwmValue <= deadZoneCenter - deadZoneRange) {
      motorSpeed = map(pwmValue, pwmMin, deadZoneCenter - deadZoneRange, motorMax, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);  // Reverse direction
    } else {
      motorSpeed = map(pwmValue, deadZoneCenter + deadZoneRange, pwmMax, 0, motorMin);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);   // Forward direction
    }
    motorSpeed = constrain(abs(motorSpeed), 0, 255);

    // Set motor speed
    analogWrite(PWM, motorSpeed);
    Serial.print("Motor speed: ");
    Serial.println(motorSpeed);
  }

  delay(20);  // Small delay for stability
}

// ------------------ Helpers ------------------
void moveMotor(int speed, int direction) {
  speed = constrain(speed, 0, 255);
  analogWrite(PWM, speed);
  digitalWrite(IN1, direction > 0 ? LOW : HIGH);
  digitalWrite(IN2, direction > 0 ? HIGH : LOW);
}

void stopMotor() {
  analogWrite(PWM, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void searchSpin() {
  analogWrite(PWM, SEARCH_PWM);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(LED_PIN, LOW);
}

void enableChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9546A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}
