// Encoders
#include "Encoder.h"

// Serial baud rate
#define BAUD_RATE 115200

// Encoder pin definitions
#define ENCODER_GPIO_1 A0
#define ENCODER_GPIO_2 A1

// Angle offsets for encoders
#define ANGLE_OFFSET_1 180
#define ANGLE_OFFSET_2 360

// Encoder instances
Encoder encoder1(ENCODER_GPIO_1);
Encoder encoder2(ENCODER_GPIO_2);

// Motor 1 pin definitions
#define IN1 4
#define IN2 5
#define ENB1 6

// Motor 2 pin definitions
#define IN3 7
#define IN4 8
#define ENB2 9

// Motor direction variables
bool motorDirection1 = true; // true = right, false = left
bool motorDirection2 = true; // true = right, false = left

// Array to store configurations
int configurations[][2] = {
  {85, -83}, // 1
  {85, -81}, // 2
  {84, -79}, // 3
  {83, -75}, // 4
  {82, -72}, // 5
  {82, -70}, // 6
  {83, -68}, // 7
  {85, -69}, // 8
  {87, -71}, // 9
  {90, -74}, // 10
  {93, -77}, // 11
  {96, -80}, // 12
  {98, -83}, // 13
  {100, -84}, // 14
  {102, -85}, // 15
  {104, -87}, // 16
  {106, -88}, // 17
  {109, -90}, // 18
  {112, -93}, // 19
  {115, -96}, // 20
  {117, -99}, // 21
  {118, -102}, // 22
  {118, -105}, // 23
  {118, -107}, // 24
  {118, -109}, // 25
  {118, -110}, // 26
  {118, -112}, // 27
  {119, -115}, // 28
  {120, -118}, // 29
  {121, -120}, // 30
  {120, -122}, // 31
  {119, -123}, // 32
  {116, -123}, // 33
  {113, -121}, // 34
  {109, -119}, // 35
  {106, -116}, // 36
  {104, -114}, // 37
  {101, -111}, // 38
  {99, -110}, // 39
  {97, -108}, // 40
  {94, -106}, // 41
  {91, -104}, // 42
  {88, -101}, // 43
  {86, -98}, // 44
  {84, -94}, // 45
  {83, -91}, // 46
  {83, -89}, // 47
  {84, -87}, // 48
  {85, -86}, // 49
  {85, -85}, // 50
  {85, -83}, // 51
};

// Number of configurations
int numConfigurations = sizeof(configurations) / sizeof(configurations[0]);

// Current configuration index
int currentConfiguration = 0;

// Time threshold for considering the arm as "reached" a configuration
const unsigned long configurationReachedTime = 100; // milliseconds

// Variables to track the time when the arm reaches a configuration
unsigned long configurationReachedStartTime = 0;
bool configurationReached = false;

// Tolerance of the system in degrees
int tolerance = 1;

/**
 * @brief Setup function that initializes the system.
 */
void setup()
{
    Serial.begin(BAUD_RATE);

    // Initialize encoders
    // Adjust the encoder resolution values with descriptive names
    const int INIT_VALUE_1 = 175;
    const int INIT_VALUE_2 = 265;
    encoder1.initializeEncoder(ANGLE_OFFSET_1, INIT_VALUE_1);
    encoder2.initializeEncoder(ANGLE_OFFSET_2, INIT_VALUE_2);

    // Set pin modes for motor control
    pinMode(ENB1, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Initialize motors to a known state
   motorWrite1(0);
   motorWrite2(0);
}

/**
 * @brief Loop function that runs continuously and controls the arm movement.
 */
void loop() {
  // Read the current configuration
  int ref1 = configurations[currentConfiguration][0];
  int ref2 = configurations[currentConfiguration][1];

  // Move the arm based on the current configuration
  moveArm(ref1, ref2);

  // Move to the next configuration
  currentConfiguration = (currentConfiguration + 1) % numConfigurations;
}


/**
 * @brief Moves the arm to a given configuration.
 * @param ref1 The reference value for joint 1.
 * @param ref2 The reference value for joint 2.
 */
 void moveArm(int ref1, int ref2) {
  while (!armReachedConfiguration(ref1, ref2)) {
    // Get current angles
    float angle1 = getAngle1();
    float angle2 = getAngle2();

    // Calculate errors
    float error1 = ref1 - angle1;
    float error2 = ref2 - angle2;

    // Proportional control gains
    float kp1 = 1.2;
    float kp2 = 1.5;

    // Calculate PWM values based on errors and gains
    float pwm1 = adjustPwm(kp1 * error1, minAbsPwm(angle1));
    float pwm2 = adjustPwm(kp2 * error2, 30);

    // Print angles for debugging
    Serial.print(angle1);
    Serial.print(",");
    Serial.print(angle2);
    Serial.println(";");

    // Safety checks for the first joint
    if (angle1 <= 0 || angle1 >= 270) {
      pwm1 = 100;
    } else if (angle1 >= 180) {
      pwm1 = -100;
    }

    // Check if error is within tolerance and apply appropriate action
    if (abs(error1) < 1) {
      brakeMotor1();
    } else {
      motorWrite1(pwm1);
    }

    // Check if error is within tolerance and apply appropriate action
    if (abs(error2) < 1) {
      brakeMotor2();
    } else {
      motorWrite2(pwm2);
    }
  }
}


/**
 * @brief Checks if the arm has reached a given configuration.
 * @param ref1 The reference value for joint 1.
 * @param ref2 The reference value for joint 2.
 * @return True if the arm has reached the configuration, false otherwise.
 */
bool armReachedConfiguration(int ref1, int ref2) {
  // Calculate the error between the references and the current angles
  float error1 = ref1 - getAngle1();
  float error2 = ref2 - getAngle2();

  // Check if the absolute errors are below a certain threshold
  bool angle1Reached = (abs(error1) < tolerance);
  bool angle2Reached = (abs(error2) < tolerance);

  // Check if the arm has reached the desired configuration for the first time
  if (angle1Reached && angle2Reached && !configurationReached) {
    configurationReachedStartTime = millis(); // Record the start time
    configurationReached = true;
  }

  // Check if the arm has been in the desired configuration for the specified time
  if (configurationReached && (millis() - configurationReachedStartTime >= configurationReachedTime)) {
    configurationReached = false; // Reset the configuration reached flag
    return true; // Arm has reached the configuration
  }

  return false; // Arm has not yet reached the configuration
}

/**
 * @brief Gets the current angle of joint 1.
 * @return The current angle of joint 1.
 */
float getAngle1() {
  return (encoder1.getAngleDeplacedBy(-270) * -1);
}

/**
 * @brief Gets the current angle of joint 2.
 * @return The current angle of joint 2.
 */
float getAngle2() {
  return (encoder2.readRawAngle() * -1) - 10;
}

/**
 * @brief Adjusts the PWM value based on a minimum absolute PWM threshold.
 * @param pwm The PWM value to adjust.
 * @param minAbsPwm The minimum absolute PWM threshold.
 * @return The adjusted PWM value.
 */
float adjustPwm(float pwm, float minAbsPwm) {
  if (abs(pwm) < minAbsPwm) {
      return minAbsPwm * (pwm / abs(pwm));
  } else {
      return pwm;
  }
}


/**
 * @brief Calculates the minimum absolute PWM threshold based on the angle.
 * @param ang The angle value.
 * @return The calculated minimum absolute PWM threshold.
 */
float minAbsPwm(float ang) {
    float absPwm = 0;
    // y = mx+b
    if (ang < 90) {
        absPwm = (-100 / 90) * ang + 100;
    } else {
        absPwm = (100 / 90) * ang - 80;
    }

    if (absPwm < 70) {
        absPwm = 70;
    }

    return absPwm;
}

/**
 * @brief Brakes Motor 1.
 *
 * This function stops Motor 1 by setting both control pins to LOW.
 */
void brakeMotor1() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

/**
 * @brief Brakes Motor 2.
 *
 * This function stops Motor 2 by setting both control pins to LOW.
 */
void brakeMotor2() {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

/**
 * @brief Controls the motor movement for joint 1.
 * @param pwm The PWM value for motor 1.
 */
void motorWrite1(float pwm) {
    if (pwm < 0)
        motorDirection1 = false;
    else
        motorDirection1 = true;

    digitalWrite(IN1, !motorDirection1);
    digitalWrite(IN2, motorDirection1);
    analogWrite(ENB1, abs(pwm) / 100 * 255);
}

/**
 * @brief Controls the motor movement for joint 2.
 * @param pwm The PWM value for motor 2.
 */
void motorWrite2(float pwm) {
    if (pwm < 0)
        motorDirection2 = false;
    else
        motorDirection2 = true;

    digitalWrite(IN3, !motorDirection2);
    digitalWrite(IN4, motorDirection2);
    analogWrite(ENB2, abs(pwm) / 100 * 255);
}
