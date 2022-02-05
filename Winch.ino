//
// Define version information
#define WINCH_CONTROL_VERSION "0.1"
#define WINCH_CONTROL_AUTHOR  "N.Ammann"
//
// Define persistent variables in EEPROM using EEPROM-Storage library!
#include <EEPROM-Storage.h>
EEPROMStorage<unsigned int> engineRunTimeSinceLastMaintenanceEEPROM(0, 0); // This variable stores engine run time since last maintenance in seconds. It is stored in EEPROM at positions 0 (4 + 1 bytes)
EEPROMStorage<unsigned int> engineRunTimeTotalEEPROM(5, 0);                // This variable stores total engine run time in seconds. It is stored in EEPROM at positions 5 (4 + 1 bytes)
EEPROMStorage<unsigned int> totalRunsEEPROM(10, 0);                        // This variable stores the total number of runs. It is stored in EEPROM at positions 10 (4 + 1 bytes)
EEPROMStorage<unsigned int> throttleServoMinEEPROM(15,  900);              // This variable stores the PWM value for the minimal angle. It is stored in EEPROM at positions 15 (4 + 1 bytes)
EEPROMStorage<unsigned int> throttleServoMaxEEPROM(20, 2000);              // This variable stores the PWM value for the maximal angle. It is stored in EEPROM at positions 20 (4 + 1 bytes)
EEPROMStorage<bool>         throttleServoInverseEEPROM(25, false);         // This variable indicates if the rotation direction is inversed. It is stored in EEPROM at positions 25 (1 + 1 bytes)
EEPROMStorage<unsigned int> breakServoMinEEPROM(27,  900);                 // This variable stores the PWM value for the minimal angle. It is stored in EEPROM at positions 27 (4 + 1 bytes)
EEPROMStorage<unsigned int> breakServoMaxEEPROM(32, 2000);                 // This variable stores the PWM value for the maximal angle. It is stored in EEPROM at positions 32 (4 + 1 bytes)
EEPROMStorage<bool>         breakServoInverseEEPROM(37, true);             // This variable indicates if the rotation direction is inversed. It is stored in EEPROM at positions 37 (1 + 1 bytes)
EEPROMStorage<float>        throttleMaxTravelEEPROM(39, 0.0f);             // This variable stores the calibrated maximum travel of the throttle servo in mm. It is stored in EEPROM at position 39 (4 + 1 bytes)
EEPROMStorage<float>        breakMaxTravelEEPROM(44, 0.0f);                // This variable stores the calibrated maximum travel of the break servo in mm. It is stored in EEPROM at position 44 (4 + 1 bytes)
//
// Define global variables to enable faster access to EEPROM variables
unsigned int throttleServoMin;
unsigned int throttleServoMax;
bool         throttleServoInverse;
unsigned int breakServoMin;
unsigned int breakServoMax;
bool         breakServoInverse;
float        throttleMaxTravel;
float        breakMaxTravel;
//
// Definition of IO pins
const int throttleServoPin =  2;
const int breakServoPin    =  3;
const int buzzerPin        =  4;
const int buttonPin        =  7;
const int potentiometerPin = A0;
//
// Definition of I2C addresses and include I2C functionality
#include <Wire.h>
const int i2cAddressACC = 0x1C;
const int i2cAddressLCD = 0x27;
const int i2cAddressMAG = 0x36;
//
// Definition of servo functionality
#include <Servo.h>
Servo throttleServo;
float __throttleServoTravel;
Servo breakServo;
float __breakServoTravel;
#define CALIBRATION_ANGLE 30.2f
//
// Definition of the LCD display driver
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(i2cAddressLCD, 20, 4);
//
// Definition of accelerometer driver
#include "SparkFun_MMA8452Q.h"
MMA8452Q acc;
//
// Definition of magnetic rotation encoder driver
#include "AS5600.h"
AMS_5600 encoder(i2cAddressMAG);
//
// Define winch properties
#define SPOOL_DIAMETER 0.140f // Spool diameter in m
#define ROPE_LENGTH 300.0f // Total rope length in m
#define MAX_SERVO_TRAVEL 64.5f // Maximal travel of servo arm in mm
#define MAX_INCREMENT_REDUCTION 0.1f // Reduce max valid angular increment to not violate Nyquist
#define ANGULAR_INCREMENT_DEADBAND 0.1f // Angular increments below this value are ignored
//
// Define constants
class WinchState
{
  public:
    enum State
    {
      SPOOL_UP,
      SHREDDING,
      SPOOL_DOWN,
      HALT
    };

  bool operator ==(const WinchState& other) const
  {
    return this->_state == other._state;
  }

  bool operator !=(const WinchState& other) const
  {
    return this->_state != other._state;
  }

  WinchState()
  {
    this->_state = WinchState::HALT;
    this->_changed = millis();
  }

  WinchState& operator =(const WinchState::State& state)
  {
    this->_state = state;
    this->_changed = millis();
    return *this;
  }

  operator WinchState::State() const
  {
    return this->_state;
  }

  long lastChanged() const
  {
    return this->_changed;
  }

  private:
      State _state;
      long _changed;
};
//
// Define global variables
bool         lastButtonState;
unsigned int loopCounter;
word         lastEncoderReading;
float        lastAngularIncrement;
float        revolutionCounter;
WinchState   winchState;
//
// Define control loop
#define CONTROL_LOOP_FREQ_HZ 50
#define CONTROL_LOOP_INTERVAL (1000 / CONTROL_LOOP_FREQ_HZ)
long lastMillis;
//
// Define boot process
void setup() {
  bool setupSucceeded = true;
  //
  // Load calibrated variables from EEPROM to RAM
  throttleServoMin     = throttleServoMinEEPROM;
  throttleServoMax     = throttleServoMaxEEPROM;
  throttleServoInverse = throttleServoInverseEEPROM;
  breakServoMin        = breakServoMinEEPROM;
  breakServoMax        = breakServoMaxEEPROM;
  breakServoInverse    = breakServoInverseEEPROM;
  throttleMaxTravel    = throttleMaxTravelEEPROM;
  breakMaxTravel       = breakMaxTravelEEPROM;
  //
  // Initialize IO pins
  pinMode(buttonPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  //
  // Initialize I2C bus communication
  Wire.begin();
  //
  // Initialize serial output for debugging
  Serial.begin(115200);
  while (!Serial);
  //
  // Write welcome message to serial output
  Serial.print(F("Winch Control v")); Serial.println(F(WINCH_CONTROL_VERSION));
  Serial.print(F("Created by: ")); Serial.println(F(WINCH_CONTROL_AUTHOR));
  //
  // Display some debug information
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));
  //
  // Search for LCD display
  Wire.beginTransmission(i2cAddressLCD);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("LCD display not found!"));
    setupSucceeded = false;
  }
  //
  // Initialize LCD display
  lcd.init();
  lcd.backlight();
  //
  // Write welcome message to LCD
  lcd.setCursor(0, 0);
  lcd.print(F("Winch Control v"));
  lcd.setCursor(15, 0);
  lcd.print(F(WINCH_CONTROL_VERSION));
  lcd.setCursor(0, 1);
  lcd.print(F("Created by:"));
  lcd.setCursor(12, 1);
  lcd.print(F(WINCH_CONTROL_AUTHOR));
  //
  // Initialize accelerometer
  if (!acc.begin(Wire, i2cAddressACC)) {
    Serial.println(F("Accelerometer not found!"));
    setupSucceeded = false;
  }
  //
  // Initialize magnetic rotational encoder
  if(encoder.detectMagnet() == 1) {
    switch (encoder.getMagnetStrength()) {
      case 1:
        Serial.println(F("Rotational encoder reading is too weak!"));
        // setupSucceeded = false;
        break;
      case 2:
        Serial.print(F("Rotational encoder reading is good: "));
        Serial.println(encoder.getMagnitude());
        break;
      case 3:
        Serial.println(F("Rotational encoder reading is too strong!"));
        // setupSucceeded = false;
        break;
    }
    lastEncoderReading = encoder.getRawAngle();
    
  } else {
    Serial.println(F("Rotational encoder has no magnetic reading!"));
    setupSucceeded = false;
  }
  //
  // Initialize variables
  lastButtonState      = LOW;
  loopCounter          = 0;
  revolutionCounter    = 0.0;
  lastAngularIncrement = 0.0;
  winchState           = WinchState::State::HALT;
  //
  // Check if setup succeeded
  if (!setupSucceeded) {
    //
    // Print warning message
    Serial.println(F("Failure during boot!"));
    lcd.setCursor(0, 3);
    lcd.print(F("Failure during boot!"));
    //
    // Halt program
    haltProgram();
  } else {
    //
    // Just wait a bit so that the welcome message is readable
    idle(3000);
  }
  //
  // Check if we enter the calibration
  bool servosNeedCalibration = !throttleServoMinEEPROM.isInitialized() ||
                               !throttleServoMaxEEPROM.isInitialized() ||
                               !throttleServoInverseEEPROM.isInitialized() ||
                               !breakServoMinEEPROM.isInitialized() ||
                               !breakServoMaxEEPROM.isInitialized() ||
                               !breakServoInverseEEPROM.isInitialized();
  bool travelNeedCalibration = !throttleMaxTravelEEPROM.isInitialized() ||
                               !breakMaxTravelEEPROM.isInitialized();
  if (servosNeedCalibration || travelNeedCalibration || digitalRead(buttonPin) == HIGH) {
    //
    // Display message
    Serial.println(F("Entering calibration mode ..."));
    lcd.setCursor(0, 1);
    lcd.print(F("Calibration Mode:   "));
    //
    // Wait until button is released
    while (digitalRead(buttonPin) == HIGH);
    //
    // Check if we enter servo calibration
    if (servosNeedCalibration || getBool(F("Calibrate Servos?   "))) {
      //
      // Initialize servos with max thinkable values
      throttleServo.attach(throttleServoPin, 100u, 3000u);
      breakServo.attach(breakServoPin, 100u, 3000u);
      //
      // Calibrate minimal value of throttle servo
      Serial.println(F("Calibrate minimal value of throttle servo ..."));
      throttleServoMin = getNumber(F("Min. Throttle Servo:"), 100u, 3000u, &setThrottleServoMicroseconds);
      throttleServoMinEEPROM = throttleServoMin;
      Serial.print(F("Set minimal value of throttle servo to ")); Serial.print(throttleServoMin); Serial.println(F(" µs!"));
      //
      // Calibrate maximal value of throttle servo
      Serial.println(F("Calibrate maximal value of throttle servo ..."));
      throttleServoMax = getNumber(F("Max. Throttle Servo:"), throttleServoMin, 3000u, &setThrottleServoMicroseconds);
      throttleServoMaxEEPROM = throttleServoMax;
      Serial.print(F("Set maximal value of throttle servo to ")); Serial.print(throttleServoMax); Serial.println(F(" µs!"));
      //
      // Check if throttle servo should be inverted
      Serial.println(F("Set inversion of break servo ..."));
      throttleServoInverse = getBool(F("Inv. Throttle Servo:"));
      throttleServoInverseEEPROM = throttleServoInverse;
      Serial.print(F("Inversion of throttle servo ")); Serial.print(throttleServoInverse ? F("enabled") : F("disabled")); Serial.println(F("!"));
      //
      // Calibrate minimal value of break servo
      Serial.println(F("Calibrate minimal value of break servo ..."));
      breakServoMin = getNumber(F("Min. Break Servo:  "), 100u, 3000u, &setBreakServoMicroseconds);
      breakServoMinEEPROM = breakServoMin;
      Serial.print(F("Set minimal value of break servo to ")); Serial.print(breakServoMin); Serial.println(F(" µs!"));
      //
      // Calibrate maximal value of break servo
      Serial.println(F("Calibrate maximal value of break servo ..."));
      breakServoMax = getNumber(F("Max. Break Servo:  "), breakServoMin, 3000u, &setBreakServoMicroseconds);
      breakServoMaxEEPROM = breakServoMax;
      Serial.print(F("Set maximal value of break servo to ")); Serial.print(breakServoMax); Serial.println(F(" µs!"));
      //
      // Check if break servo should be inverted
      Serial.println(F("Set inversion of break servo ..."));
      breakServoInverse = getBool(F("Inv. Break Servo:  "));
      breakServoInverseEEPROM = breakServoInverse;
      Serial.print(F("Inversion of break servo ")); Serial.print(breakServoInverse ? F("enabled") : F("disabled")); Serial.println(F("!"));
      //
      // Unload servos with calibration config and load servos again.
      throttleServo.detach();
      breakServo.detach();
      //
      // When calibrating servos, travel needs to be calibrated also
      travelNeedCalibration = true;
    }
    //
    // Initialize servos
    throttleServo.attach(throttleServoPin, throttleServoMin, throttleServoMax);
    breakServo.attach(breakServoPin, breakServoMin, breakServoMax);
    throttleServo.writeMicroseconds(throttleServoInverse ? throttleServoMax : throttleServoMin);
    breakServo.writeMicroseconds(breakServoInverse ? breakServoMax : breakServoMin);
    //
    // Check if we enter servo calibration
    if (travelNeedCalibration || getBool(F("Calibrate Travel?   "))) {
      //
      // Calibrate maximal travel of throttle servo
      Serial.println(F("Calibrate maximal travel of throttle servo ..."));
      throttleMaxTravel = getNumber(F("Max. Throttle Travel"), 0.0f, MAX_SERVO_TRAVEL, &setThrottleServoTravel);
      throttleMaxTravelEEPROM = throttleMaxTravel;
      Serial.print(F("Set maximal travel of throttle servo to ")); Serial.print(throttleMaxTravel); Serial.println(F(" mm!"));
      //
      // Calibrate maximal travel of break servo
      Serial.println(F("Calibrate maximal travel of break servo ..."));
      breakMaxTravel = getNumber(F("Max. Break Travel:  "), 0.0f, MAX_SERVO_TRAVEL, &setBreakServoTravel);
      breakMaxTravelEEPROM = breakMaxTravel;
      Serial.print(F("Set maximal travel of break servo to ")); Serial.print(breakMaxTravel); Serial.println(F(" mm!"));
    }
  } else {
    //
    // Initialize servos
    throttleServo.attach(throttleServoPin, throttleServoMin, throttleServoMax);
    breakServo.attach(breakServoPin, breakServoMin, breakServoMax);
    throttleServo.writeMicroseconds(throttleServoInverse ? throttleServoMax : throttleServoMin);
    breakServo.writeMicroseconds(breakServoInverse ? breakServoMax : breakServoMin);
  }
  //
  // Initialize timing
  lastMillis = millis();
}

void updateRopeStatus(float& ropeVelocity, float& ropeLength)
{
  //
  // Get encoder reading and transform to angular increment from last reading
  float currentEncoderReading = encoder.getRawAngle();
  float angularIncrement = (currentEncoderReading - lastEncoderReading) * 0.087f;
  lastEncoderReading = currentEncoderReading;
  //
  // Check if we have an edge case 0/360
  if (angularIncrement > +180.0f) {
    angularIncrement -= 360.0f;
  } else if (angularIncrement < -180.0f) {
    angularIncrement += 360.0f;
  }
  //
  // Appy a deadband of 0.1 deg to angular increment
  if (abs(angularIncrement) < ANGULAR_INCREMENT_DEADBAND) {
    angularIncrement = 0.0f;
  }
  //
  // Check if increment is 10% lower that maximum (Nyquist)
  if (abs(angularIncrement) > 180.0f * (1.0f - MAX_INCREMENT_REDUCTION)) {
    //
    // We lost track of RPM due to to high rotational velocity -> STOP
    haltWinch();
    //
    // Print warning message
    Serial.println(F("Lost track of spool rotation due to to too high rotational velocity!"));
    lcd.setCursor(0, 3);
    lcd.print(F("Failure: Nyquist!"));
    //
    // Halt program
    haltProgram();
  }
  //
  // Increment revolution counter
  revolutionCounter += angularIncrement / 360.0f;
  //
  // Convert angular increment to angular velocity
  const float omega = angularIncrement * CONTROL_LOOP_FREQ_HZ; // Angular velocity in deg/s
  //
  // Calculate rope velocity based on angular velocity, revolution counter and spool diameter
  // TODO: This term is going to be dependent on the number of revolutions
  ropeVelocity = (M_PI * SPOOL_DIAMETER / 360.0f) * omega; // Rope velocity in m/s
  //
  // Calculate rope length based on revolution counter and spool diameter
  // TODO: This term is going to be non linear in the numbers of revolutions
  ropeLength = revolutionCounter * (M_PI * SPOOL_DIAMETER); // Rope length in m
}

void loop() {
  //
  // The following code is executed with the frequency of CONTROL_LOOP_FREQ_HZ
  long currentMillis = millis();
  if ((currentMillis - lastMillis) < CONTROL_LOOP_INTERVAL) return;
  lastMillis = currentMillis;
  //
  // Increase loop counter
  loopCounter++;
  //
  // Update rope status
  float ropeVelocity, ropeLength;
  updateRopeStatus(ropeVelocity, ropeLength);
  //
  // Check state of winch ...
  switch (winchState) {
    case WinchState::SPOOL_UP:
      {
      }
      break;
    case WinchState::SHREDDING:
      {
      }
      break;
    case WinchState::SPOOL_DOWN:
      {
      }
      break;
    case WinchState::HALT:
    default:
      {
      }
      break;
  }









  





  
//  int buttonState = digitalRead(buttonPin);
  int potiValue = analogRead(potentiometerPin);
//  if (buttonState == HIGH) {
//    if (lastButtonState != buttonState) {
//      Serial.print(F("Number of button presses: "));
//      Serial.println(++totalRuns);
//    }
//  } else {
//  }
//
//  lastButtonState = buttonState;
//
  setThrottleServoTravel(map(potiValue, 0, 1023, 0.0f, 64.5f));
  setBreakServoTravel(map(potiValue, 0, 1023, 0.0f, 64.5f));
  //if (loopCounter % 100 == 0) {
    lcd.setCursor(0, 3);
    lcd.print(F("Travel:      mm"));
    lcd.setCursor(8, 3);
    lcd.print(map(potiValue, 0, 1023, 0.0f, 64.5f));
  //}
//  
//
//  if (acc.available()) {      // Wait for new data from accelerometer
//    // Acceleration of x, y, and z directions in g units
//    Serial.print(acc.getCalculatedX(), 3);
//    Serial.print(F("\t"));
//    Serial.print(acc.getCalculatedY(), 3);
//    Serial.print(F("\t"));
//    Serial.print(acc.getCalculatedZ(), 3);
//    Serial.println();
//  }
}
//
// Define helper functions
void haltProgram()
{
  while (true) {
    tone(buzzerPin, 2000);
    idle(500);
    noTone(buzzerPin);
    idle(500);
  }
}

void haltWinch()
{
  //
  // Put throttle to zero
  setThrottleServoTravel(0.0f);
  //
  // Enable break;
  setBreakServoTravel(breakMaxTravel);
}

unsigned int getThrottleServoMicroseconds()
{
  return throttleServo.readMicroseconds();
}

void setThrottleServoMicroseconds(unsigned int us)
{
  throttleServo.writeMicroseconds(us);
}

float getThrottleServoTravel()
{
  return __throttleServoTravel;
}

void setThrottleServoTravel(float travel)
{
  //
  // Check if travel is in range
  if (travel > throttleMaxTravel) {
    travel = throttleMaxTravel;
  } else if (travel < 0.0f) {
    travel = 0.0f;
  }
  //
  // Store to global variable to read the value back
  __throttleServoTravel = travel;
  // Use the following equation between the angle of the servos and the travel
  // X = norm([28.3; 55.0] - 38.1 * [sin(x); cos(x)]);
  // Using sampling, inversion and fitting of that data to create a 5th degree polynom.
  // That polynom maps the intended way of travel to a commanded angle.
  //
  // Precalculate x to the power of n
  float x1 =      travel;
  float x2 = x1 * travel;
  float x3 = x2 * travel;
  float x4 = x3 * travel;
  float x5 = x4 * travel;
  //
  // Calculate angle using polynom
  float angle;
  angle  =  32.5230000f;
  angle +=   3.2765000f * x1;
  angle += - 0.1215900f * x2;
  angle +=   0.0037905f * x3;
  angle += - 5.5008e-5f * x4;
  angle +=   3.1089e-7f * x5;
  //
  // Check angle to be in range
  if (angle > 180.0f - CALIBRATION_ANGLE) {
    angle = 180.0f - CALIBRATION_ANGLE;
  } else if (angle < CALIBRATION_ANGLE) {
    angle = CALIBRATION_ANGLE;
  }
  //
  // Transform angle to PWM signal
  if (throttleServoInverse) {
    throttleServo.writeMicroseconds(map(angle, CALIBRATION_ANGLE, 180 - CALIBRATION_ANGLE, throttleServoMax, throttleServoMin));
  } else {
    throttleServo.writeMicroseconds(map(angle, CALIBRATION_ANGLE, 180 - CALIBRATION_ANGLE, throttleServoMin, throttleServoMax));
  }
}

unsigned int getBreakServoMicroseconds()
{
  return breakServo.readMicroseconds();
}

void setBreakServoMicroseconds(unsigned int us)
{
  breakServo.writeMicroseconds(us);
}

float getBreakServoTravel()
{
  return __breakServoTravel;
}

void setBreakServoTravel(float travel)
{
  //
  // Check if travel is in range
  if (travel > breakMaxTravel) {
    travel = breakMaxTravel;
  } else if (travel < 0.0f) {
    travel = 0.0f;
  }
  //
  // Store to global variable to read the value back
  __breakServoTravel = travel;
  // Use the following equation between the angle of the servos and the travel
  // X = norm([28.3; 55.0] - 38.1 * [sin(x); cos(x)]);
  // Using sampling, inversion and fitting of that data to create a 5th degree polynom.
  // That polynom maps the intended way of travel to a commanded angle.
  //
  // Precalculate x to the power of n
  float x1 =      travel;
  float x2 = x1 * travel;
  float x3 = x2 * travel;
  float x4 = x3 * travel;
  float x5 = x4 * travel;
  //
  // Calculate angle using polynom
  float angle;
  angle  =  32.5230000f;
  angle +=   3.2765000f * x1;
  angle += - 0.1215900f * x2;
  angle +=   0.0037905f * x3;
  angle += - 5.5008e-5f * x4;
  angle +=   3.1089e-7f * x5;
  //
  // Check angle to be in range
  if (angle > 180.0f - CALIBRATION_ANGLE) {
    angle = 180.0f - CALIBRATION_ANGLE;
  } else if (angle < CALIBRATION_ANGLE) {
    angle = CALIBRATION_ANGLE;
  }
  //
  // Transform angle to PWM signal
  if (breakServoInverse) {
    breakServo.writeMicroseconds(map(angle, CALIBRATION_ANGLE, 180 - CALIBRATION_ANGLE, breakServoMax, breakServoMin));
  } else {
    breakServo.writeMicroseconds(map(angle, CALIBRATION_ANGLE, 180 - CALIBRATION_ANGLE, breakServoMin, breakServoMax));
  }
}

template< typename T >
float map(T x, T in_min, T in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline bool getBool(const __FlashStringHelper* text)
{
  //
  // Initialize value
  bool value = false;
  //
  // Print text
  lcd.setCursor(0, 2);
  lcd.print(text);
  //
  // Read value from potentiometer as long as the button is not pressed
  while (digitalRead(buttonPin) == LOW) {
    value = map(analogRead(potentiometerPin), 0, 1023, 0, 1);
    lcd.setCursor(0, 3);
    lcd.print(F("Value:              "));
    lcd.setCursor(7, 3);
    lcd.print(value ? F("True") : F("False"));
    idle(100);
  }
  //
  // Wait until button is released
  while (digitalRead(buttonPin) == HIGH);
  //
  // Return value
  return value;
}

inline int getNumber(const __FlashStringHelper* text, const int minimum, const int maximum, void (*f)(int))
{
  //
  // Initialize value
  int value = 0;
  //
  // Print text
  lcd.setCursor(0, 2);
  lcd.print(text);
  //
  // Read value from potentiometer as long as the button is not pressed
  while (digitalRead(buttonPin) == LOW) {
    value = map(analogRead(potentiometerPin), 0, 1023, minimum, maximum);
    if (f) f(value);
    lcd.setCursor(0, 3);
    lcd.print(F("Value:              "));
    lcd.setCursor(7, 3);
    lcd.print(value);
    idle(100);
  }
  //
  // Wait until button is released
  while (digitalRead(buttonPin) == HIGH);
  //
  // Return value
  return value;
}

inline unsigned int getNumber(const __FlashStringHelper* text, const unsigned int minimum, const unsigned int maximum, void (*f)(unsigned int))
{
  //
  // Initialize value
  unsigned int value = 0;
  //
  // Print text
  lcd.setCursor(0, 2);
  lcd.print(text);
  //
  // Read value from potentiometer as long as the button is not pressed
  while (digitalRead(buttonPin) == LOW) {
    value = map(analogRead(potentiometerPin), 0, 1023, minimum, maximum);
    if (f) f(value);
    lcd.setCursor(0, 3);
    lcd.print(F("Value:              "));
    lcd.setCursor(7, 3);
    lcd.print(value);
    idle(100);
  }
  //
  // Wait until button is released
  while (digitalRead(buttonPin) == HIGH);
  //
  // Return value
  return value;
}

inline float getNumber(const __FlashStringHelper* text, const float minimum, const float maximum, void (*f)(float))
{
  //
  // Initialize value
  float value = 0;
  //
  // Print text
  lcd.setCursor(0, 2);
  lcd.print(text);
  //
  // Read value from potentiometer as long as the button is not pressed
  while (digitalRead(buttonPin) == LOW) {
    value = map(analogRead(potentiometerPin), 0, 1023, minimum, maximum);
    if (f) f(value);
    lcd.setCursor(0, 3);
    lcd.print(F("Value:              "));
    lcd.setCursor(7, 3);
    lcd.print(value);
    idle(100);
  }
  //
  // Wait until button is released
  while (digitalRead(buttonPin) == HIGH);
  //
  // Return value
  return value;
}

void idle(const unsigned int duration)
{
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < duration);
}

void clearEEPROM()
{
  for (unsigned int i = 0; i < EEPROM.length(); ++i) {
    EEPROM.update(i, 0xFF);
  }
}
