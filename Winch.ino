//
// Define version information
#define WINCH_CONTROL_AUTHOR  "N.Ammann"
//
// Define winch properties
#define ENGINE_MAINTENANCE_INTERVAL 25 // Interval for engine maintenance in hours
#define SPOOL_DIAMETER 0.225f // Spool diameter in m
#define ENCODER_RESOLUTION 0.087890625f // Resolution of the encoder in deg (360.0f / 4096 = 0.087890625 
#define ROPE_LENGTH 300.0f // Total rope length in m
#define SERVO_MIN_PWM 1000u // Minimum PWM signal
#define SERVO_MAX_PWM 1800u // Maximum PWM signal
#define SAFETY_MARGIN 0.1f // Factor to increase safety margins
#define ANGULAR_INCREMENT_DEADBAND 0.1f // Angular increments below this value are ignored
#define EMERGENCY_STOPPING_DISTANCE 10.0f // Minimum distance the winch needs to come to a complete stop (zero throttle and break) in m.
#define EMERGENCY_STOPPING_TIME 2000 // Minimum time the winch needs to come to a complete stop (zero throttle and break) in ms.
#define STOPPING_DISTANCE -1000.0f // TODO: This value has to be verified! Distance the winch needs to come to a complete stop (spool down then break) in m.
#define STOPPING_TIME 2200 // TODO: This value has to be verified! Time the winch needs to come to a complete stop (spool down then break) in ms.
#define SPOOL_DOWN_TIME 1500 // Time in milliseconds to let the spool decelerate
#define MINIMAL_ACCELERATION  0.5f // TODO: This value has to be verified! Minimal acceleration of the rope to reach desired velocity in m/s^2
#define MAXIMAL_ACCELERATION 10.0f // Maximal acceleration of the rope to reach desired velocity in m/s^2
#define MINIMAL_VELOCITY 5.0f // Minimal configurable velocity in km/h
#define MAXIMAL_VELOCITY 40.0f // Maximal configurable velocity in km/h
#define SPOOL_UP_TIME 3800 // Time in milliseconds to let the spool spin up in idle (0.8s) / let the rope get tight (3s)
#define ENGINE_RUNNING_TIME_WINDOW 2 // Time horizon to check if engine is running in seconds
#define ENGINE_VIBRATION_THRESHOLD 1.25f // Squared norm of acceleration threshold to detect engine vibrations
#define SPOOL_UP_BEEP_TIME 500 // Time in milliseconds for the duration of the spool up beep
#define START_SIGNAL_DURATION 3 // Minimum time the button has to be pressed to start winch in seconds
#define EMERGENCY_STOP_SIGNAL_DURATION 0.1f // Minimum time the button has to be pressed to start breaking in seconds
#define CONF_SIGNAL_DURATION 0.1f // Minimum time the button has to be pressed to switch to configuration state in seconds
//
// Define persistent variables in EEPROM using EEPROM-Storage library!
#include <EEPROM-Storage.h>
EEPROMStorage<unsigned int> engineRunTimeSinceLastMaintenanceEEPROM(0, 0); // This variable stores engine run time since last maintenance in seconds. It is stored in EEPROM at positions 0 (4 + 1 bytes)
EEPROMStorage<unsigned int> engineRunTimeTotalEEPROM(5, 0);                // This variable stores total engine run time in seconds. It is stored in EEPROM at positions 5 (4 + 1 bytes)
EEPROMStorage<unsigned int> totalRunsEEPROM(10, 0);                        // This variable stores the total number of runs. It is stored in EEPROM at positions 10 (4 + 1 bytes)
EEPROMStorage<unsigned int> throttleServoMinEEPROM(15,  SERVO_MIN_PWM);    // This variable stores the PWM value for the minimal angle. It is stored in EEPROM at positions 15 (4 + 1 bytes)
EEPROMStorage<unsigned int> throttleServoMaxEEPROM(20, SERVO_MAX_PWM);     // This variable stores the PWM value for the maximal angle. It is stored in EEPROM at positions 20 (4 + 1 bytes)
EEPROMStorage<bool>         throttleServoInverseEEPROM(25, false);         // This variable indicates if the rotation direction is inversed. It is stored in EEPROM at positions 25 (1 + 1 bytes)
EEPROMStorage<unsigned int> breakServoMinEEPROM(27,  SERVO_MIN_PWM);       // This variable stores the PWM value for the minimal angle. It is stored in EEPROM at positions 27 (4 + 1 bytes)
EEPROMStorage<unsigned int> breakServoMaxEEPROM(32, SERVO_MAX_PWM);        // This variable stores the PWM value for the maximal angle. It is stored in EEPROM at positions 32 (4 + 1 bytes)
EEPROMStorage<bool>         breakServoInverseEEPROM(37, false);            // This variable indicates if the rotation direction is inversed. It is stored in EEPROM at positions 37 (1 + 1 bytes)
EEPROMStorage<float>        controllerKpEEPROM(39, 0.0f);                  // This variable stores the P gain of the PID controller. It is stored in EEPROM at position 49 (4 + 1 bytes)
EEPROMStorage<float>        controllerKiEEPROM(44, 0.0f);                  // This variable stores the I gain of the PID controller. It is stored in EEPROM at position 54 (4 + 1 bytes)
EEPROMStorage<float>        controllerKdEEPROM(49, 0.0f);                  // This variable stores the D gain of the PID controller. It is stored in EEPROM at position 59 (4 + 1 bytes)
EEPROMStorage<float>        desiredVelocityEEPROM(54, 30.0f);              // This variable stores the desired velocity im km/h. It is stored in EEPROM at position 64 (4 + 1 bytes)
EEPROMStorage<float>        accelerationEEPROM(59, 2.0f);                  // This variable stores the acceleration towards the desired velocity im m/s^2. It is stored in EEPROM at position 69 (4 + 1 bytes)
//
// Define global variables to enable faster access to EEPROM variables
unsigned int throttleServoMin;
unsigned int throttleServoMax;
bool         throttleServoInverse;
unsigned int breakServoMin;
unsigned int breakServoMax;
bool         breakServoInverse;
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
float __throttleServoValue;
Servo breakServo;
float __breakServoValue;
#define CALIBRATION_ANGLE 30.2f
//
// Definition of the LCD display driver
#include "LCD_Wrapper.hpp"
LCD_Wrapper lcd(i2cAddressLCD, 20, 4);
//
// Create custom characters
// Inverted ' '
#define INVERTED_SPACE_SYMBOL 0
byte invertedSpace[] = {
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F,
  0x1F
};
// Inverted '<'
#define INVERTED_LEFT_SYMBOL 1
byte invertedLeft[] = {
  0x1D,
  0x1B,
  0x17,
  0x0F,
  0x17,
  0x1B,
  0x1D,
  0x1F
};
// Inverted '>'
#define INVERTED_RIGHT_SYMBOL 2
byte invertedRight[] = {
  0x17,
  0x1B,
  0x1D,
  0x1E,
  0x1D,
  0x1B,
  0x17,
  0x1F
};
// Inverted 'E'
#define INVERTED_E_SYMBOL 3
byte invertedE[] = {
  0x00,
  0x0F,
  0x0F,
  0x01,
  0x0F,
  0x0F,
  0x00,
  0x1F
};
// Inverted 'S'
#define INVERTED_S_SYMBOL 4
byte invertedS[] = {
  0x10,
  0x0F,
  0x0F,
  0x11,
  0x1E,
  0x1E,
  0x01,
  0x1F
};
// Inverted 'C'
#define INVERTED_C_SYMBOL 5
byte invertedC[] = {
  0x11,
  0x0E,
  0x0F,
  0x0F,
  0x0F,
  0x0E,
  0x11,
  0x1F
};
// Inverted 'O'
#define INVERTED_O_SYMBOL 6
byte invertedO[] = {
  0x11,
  0x0E,
  0x0E,
  0x0E,
  0x0E,
  0x0E,
  0x11,
  0x1F
};
// Inverted 'K'
#define INVERTED_K_SYMBOL 7
byte invertedK[] = {
  0x0E,
  0x0D,
  0x0B,
  0x07,
  0x0B,
  0x0D,
  0x0E,
  0x1F
};
//
// Definition of accelerometer driver
#include "SparkFun_MMA8452Q.h"
MMA8452Q acc;
//
// Definition of magnetic rotation encoder driver
#include "AS5600.h"
AMS_5600 encoder(i2cAddressMAG);
//
// Define control loop
#define CONTROL_LOOP_FREQ_HZ 50 // Frequency of the control loop in Hz
#define CONTROL_LOOP_INTERVAL (1000 / CONTROL_LOOP_FREQ_HZ)
#define LCD_UPDATE_RATE 2 // Frequency of the display update in Hz
long lastMillis;
//
// Define configuration items
enum ConfigurationItems
{
  VELOCITY               = 0,
  ACCELERATION           = 1,
  P_GAIN                 = 2,
  I_GAIN                 = 3,
  D_GAIN                 = 4,
  THROTTLE_SERVO_MIN     = 5,
  THROTTLE_SERVO_MAX     = 6,
  THROTTLE_SERVO_INVERSE = 7,
  BREAK_SERVO_MIN        = 8,
  BREAK_SERVO_MAX        = 9,
  BREAK_SERVO_INVERSE    = 10,
  RUNS_TOTAL             = 11,
  RUN_TIME_TOTAL         = 12,
  RUN_TIME_MAINTENANCE   = 13,
  REGISTER_MAINTENANCE   = 14,
  RESET_ALL              = 15
};
enum SelectedItem
{
  LEFT  = 0,
  OK    = 1,
  ESC   = 2,
  RIGHT = 3
};
//
// Define math constants
#define SQ(x) ((x)*(x))
//
// For log
#include <math.h>
//
// Define state classes
class WinchState
{
  public:
    enum State
    {
      CONFIGURATION     = -1,
      STANDBY           =  0,
      SPOOL_UP_REJECTED =  1,
      SPOOL_UP_WARNING  =  2,
      SPOOL_UP          =  3,
      SHREDDING         =  4,
      SPOOL_DOWN        =  5
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
    this->_state = WinchState::STANDBY;
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

class EngineState
{
  public:
    enum State
    {
      OFF = 0,
      ON  = 1
    };

  bool operator ==(const EngineState& other) const
  {
    return this->_state == other._state;
  }

  bool operator !=(const EngineState& other) const
  {
    return this->_state != other._state;
  }

  EngineState()
  {
    this->_state = EngineState::OFF;
    this->_changed = millis();
  }

  EngineState& operator =(const EngineState::State& state)
  {
    this->_state = state;
    this->_changed = millis();
    return *this;
  }

  operator EngineState::State() const
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
unsigned int       buttonState;
unsigned long      buttonHighCount;
bool               waitForButtonRelease;
unsigned long      loopCounter;
unsigned int       engineVibrationCounter;
word               lastEncoderReading;
float              revolutionCounter;
WinchState         winchState;
EngineState        engineState;
ConfigurationItems activeConfiguration;
SelectedItem       selectedItem;
bool               selectingValue;
float              commandedVelocity;
float              desiredVelocity;
float              acceleration;
//
// Define PID controller values
#define MINIMAL_P_GAIN 0.0f
#define MAXIMAL_P_GAIN 0.1f // TODO: This value has to be verified! 
float controllerKp;
#define MINIMAL_I_GAIN 0.0f
#define MAXIMAL_I_GAIN 0.1f // TODO: This value has to be verified! 
float controllerKi;
#define MINIMAL_D_GAIN 0.0f
#define MAXIMAL_D_GAIN 0.1f // TODO: This value has to be verified! 
float controllerKd;
float currentError;
float lastError;
float integralError;
//
// Define feed forward controller
float calculateFeedForwardComponent(float commandedVelocity)
{
  //
  // Check if commanded velocity is in range
  if (commandedVelocity < MINIMAL_VELOCITY) return 0.0f;
  if (commandedVelocity > MAXIMAL_VELOCITY) return 1.0f;
  //
  // Calculate feed forward component based on logarithmic fitting
  return log(commandedVelocity / 1.31994f) / 3.32239f;
}
//
// Utility functions
void idle(const unsigned int duration)
{
  unsigned long currentMillis = millis();
  while (millis() - currentMillis < duration);
}

void haltProgram()
{
  while (true) {
    tone(buzzerPin, 2000);
    idle(500);
    noTone(buzzerPin);
    idle(500);
  }
}

void printErrorMessage(const __FlashStringHelper* text)
{
  Serial.print(F("Error: ")); Serial.println(text);
  lcd.setCursor(0, 1);
  lcd.print(F("   ERROR MESSAGE:   "));
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(0, 3);
  lcd.print(F("                    "));
  lcd.setCursor(0, 3);
  lcd.print(text);
  lcd.updateDisplay();
}

void printInfoMessage(const __FlashStringHelper* text)
{
  lcd.setCursor(0, 1);
  lcd.print(F("    INFO MESSAGE:   "));
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(0, 3);
  lcd.print(F("                    "));
  lcd.setCursor(0, 3);
  lcd.print(text);
}

void printErrorMessageAndHaltProgram(const __FlashStringHelper* text)
{
  //
  // Print error message
  printErrorMessage(text);
  //
  // Halt program
  haltProgram();
}

unsigned int getThrottleServoMicroseconds()
{
  return throttleServo.readMicroseconds();
}

void setThrottleServoMicroseconds(unsigned int us)
{
  throttleServo.writeMicroseconds(us);
}

float getThrottleServo()
{
  return __throttleServoValue;
}

void setThrottleServo(float value)
{
  //
  // Check if value is in range
  if (value > 1.0f) {
    value = 1.0f;
  } else if (value < 0.0f) {
    value = 0.0f;
  }
  //
  // Store to global variable to read the value back
  __throttleServoValue = value;
  //
  // Transform value to PWM signal
  if (throttleServoInverse) {
    throttleServo.writeMicroseconds(map(value, 0.0f, 1.0f, throttleServoMax, throttleServoMin));
  } else {
    throttleServo.writeMicroseconds(map(value, 0.0f, 1.0f, throttleServoMin, throttleServoMax));
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

float getBreakServo()
{
  return __breakServoValue;
}

void setBreakServo(float value)
{
  //
  // Check if value is in range
  if (value > 1.0f) {
    value = 1.0f;
  } else if (value < 0.0f) {
    value = 0.0f;
  }
  //
  // Store to global variable to read the value back
  __breakServoValue = value;
  //
  // Transform angle to PWM signal
  if (breakServoInverse) {
    breakServo.writeMicroseconds(map(value, 0.0f, 1.0f, breakServoMax, breakServoMin));
  } else {
    breakServo.writeMicroseconds(map(value, 0.0f, 1.0f, breakServoMin, breakServoMax));
  }
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
    value = (map(analogRead(potentiometerPin), 0, 1023, 0, 10) > 5);
    lcd.setCursor(0, 3);
    lcd.print(F("Value:              "));
    lcd.setCursor(7, 3);
    lcd.print(value ? F("True") : F("False"));
    lcd.updateDisplay();
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
    lcd.updateDisplay();
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
    lcd.updateDisplay();
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
    lcd.updateDisplay();
    idle(100);
  }
  //
  // Wait until button is released
  while (digitalRead(buttonPin) == HIGH);
  //
  // Return value
  return value;
}

void haltWinch()
{
  //
  // Put throttle to zero
  setThrottleServo(0.0f);
  //
  // Enable break
  setBreakServo(1.0f);
}

void releaseWinch()
{
  //
  // Put throttle to zero
  setThrottleServo(0.0f);
  //
  // Release break
  setBreakServo(0.0f);
}

bool buttonPressedFor(unsigned int counter)
{
  if (buttonState == HIGH && buttonHighCount > counter) {
    waitForButtonRelease = true;
    buttonHighCount = 0;
    return true;
  } else {
    return false;
  }
}

bool buttonPressedForAndReleased(unsigned int counter)
{
  if (buttonState == LOW && buttonHighCount > counter) {
    buttonHighCount = 0;
    return true;
  } else {
    return false;
  }
}

void clearEEPROM()
{
  for (unsigned int i = 0; i < EEPROM.length(); ++i) {
    EEPROM.update(i, 0xFF);
  }
}

void updateButton()
{
  if (digitalRead(buttonPin) == LOW) {
    buttonState = LOW;
    waitForButtonRelease = false;
  } else {
    if (buttonState == LOW || waitForButtonRelease) {
      buttonHighCount = 0;
    } else {
      buttonHighCount++;
    }
    buttonState = HIGH;
  }
}

void updateEngineState()
{
  //
  // Check if new data is available
  if (acc.available()) {
    //
    // Get new data
    acc.read();
    Serial.print(acc.cx); Serial.print(';');
    Serial.print(acc.cy); Serial.print(';');
    Serial.print(acc.cz); Serial.print(';');
    //
    // Calculate squared norm of acceleration vector
    float norm2 = SQ(acc.cx) + SQ(acc.cy) + SQ(acc.cz);
    Serial.print(norm2); Serial.print(';');
    //
    // Check if squared norm is above defined threshold for specified time
    if (norm2 >= ENGINE_VIBRATION_THRESHOLD) {
      if (engineVibrationCounter < ENGINE_RUNNING_TIME_WINDOW * CONTROL_LOOP_FREQ_HZ) {
        engineVibrationCounter++;
      }
    } else if (engineVibrationCounter > 0) {
      engineVibrationCounter--;
    }
    //
    // Check if engine state has to be changed.
    // Change value of counter to introduce hysteresis.
    if (engineState != EngineState::State::ON) {
      if (engineVibrationCounter > ENGINE_RUNNING_TIME_WINDOW * CONTROL_LOOP_FREQ_HZ / 2) {
        engineState = EngineState::State::ON;
        engineVibrationCounter = ENGINE_RUNNING_TIME_WINDOW * CONTROL_LOOP_FREQ_HZ;
      }
    } else {
      if (engineVibrationCounter < ENGINE_RUNNING_TIME_WINDOW * CONTROL_LOOP_FREQ_HZ / 2) {
        engineRunTimeTotalEEPROM                += (millis() - engineState.lastChanged()) / 1000;
        engineRunTimeSinceLastMaintenanceEEPROM += (millis() - engineState.lastChanged()) / 1000;
        engineState = EngineState::State::OFF;
        engineVibrationCounter = 0;
      }
    }
  } else {
    //
    // We lost the accelerometer -> HALT
    haltWinch();
    printErrorMessageAndHaltProgram(F("   NO ACC READING   "));
  }
  Serial.print(engineState); Serial.print(';');
  Serial.print(engineVibrationCounter); Serial.print(';');
}

void displayRopeStatus(const float& ropeVelocity, const float& ropeLength_m)
{
  lcd.setCursor(0, 2);
  lcd.print(F("Velocity:      Rope:"));
  lcd.setCursor(0, 3);
  lcd.print(F("     km/h          m"));
  lcd.setCursor(0, 3);
  if (ropeVelocity < 10) lcd.print(" ");
  lcd.print(ropeVelocity, 1);
  lcd.setCursor(12, 3);
  if (-99.95f <  ropeLength_m && ropeLength_m < 999.95f) lcd.print(" ");
  if ( -9.95f <  ropeLength_m && ropeLength_m <  99.95f) lcd.print(" ");
  if (  0.00f <= ropeLength_m && ropeLength_m <   9.95f) lcd.print(" ");
  lcd.print(ropeLength_m, 1);
}

template< typename T >
float map(T x, T in_min, T in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void displayMenu(int idx = -1)
{
  lcd.setCursor(0, 3);
  if (idx == 0) {
    lcd.write(INVERTED_SPACE_SYMBOL);
    lcd.write(INVERTED_LEFT_SYMBOL);
    lcd.write(INVERTED_LEFT_SYMBOL);
    lcd.write(INVERTED_SPACE_SYMBOL);
  } else {
    lcd.print(F(" << "));
  }
  lcd.print(F("|"));
  if (idx == 1) {
    lcd.write(INVERTED_SPACE_SYMBOL);
    lcd.write(INVERTED_O_SYMBOL);
    lcd.write(INVERTED_K_SYMBOL);
    lcd.write(INVERTED_SPACE_SYMBOL);
  } else {
    lcd.print(F(" OK "));
  }
  lcd.print(F("|"));
  if (idx == 2) {
    lcd.write(INVERTED_SPACE_SYMBOL);
    lcd.write(INVERTED_E_SYMBOL);
    lcd.write(INVERTED_S_SYMBOL);
    lcd.write(INVERTED_C_SYMBOL);
    lcd.write(INVERTED_SPACE_SYMBOL);
  } else {
    lcd.print(F(" ESC "));
  }
  lcd.print(F("|"));
  if (idx == 3) {
    lcd.write(INVERTED_SPACE_SYMBOL);
    lcd.write(INVERTED_RIGHT_SYMBOL);
    lcd.write(INVERTED_RIGHT_SYMBOL);
    lcd.write(INVERTED_SPACE_SYMBOL);
  } else {
    lcd.print(F(" >> "));
  }
}
//
// Define boot process
void setup() {
  //
  // Load calibrated variables from EEPROM to RAM
  throttleServoMin     = throttleServoMinEEPROM;
  throttleServoMax     = throttleServoMaxEEPROM;
  throttleServoInverse = throttleServoInverseEEPROM;
  breakServoMin        = breakServoMinEEPROM;
  breakServoMax        = breakServoMaxEEPROM;
  breakServoInverse    = breakServoInverseEEPROM;
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
  Serial.println(F("Winch Control"));
  Serial.print(F("Created by: ")); Serial.println(F(WINCH_CONTROL_AUTHOR));
  //
  // Display some debug information
  Serial.print(F("CPU Frequency: ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));
  Serial.print(F("Control Loop Frequency: ")); Serial.print(CONTROL_LOOP_FREQ_HZ); Serial.println(F(" Hz"));
  Serial.print(F("Control Loop Rate: ")); Serial.print(CONTROL_LOOP_INTERVAL); Serial.println(F(" ms"));
  Serial.print(F("Throttle Servo Min PWM: ")); Serial.print(throttleServoMinEEPROM); Serial.println(F(" µs"));
  Serial.print(F("Throttle Servo Max PWM: ")); Serial.print(throttleServoMaxEEPROM); Serial.println(F(" µs"));
  Serial.print(F("Throttle Servo Inverse: ")); Serial.print(throttleServoInverseEEPROM ? F("True") : F("False")); Serial.println(F(""));
  Serial.print(F("Break Servo Min PWM: ")); Serial.print(breakServoMinEEPROM); Serial.println(F(" µs"));
  Serial.print(F("Break Servo Max PWM: ")); Serial.print(breakServoMaxEEPROM); Serial.println(F(" µs"));
  Serial.print(F("Break Servo Inverse: ")); Serial.print(breakServoInverseEEPROM ? F("True") : F("False")); Serial.println(F(""));
  Serial.print(F("Controller Kp: ")); Serial.print(controllerKpEEPROM); Serial.println(F(""));
  Serial.print(F("Controller Ki: ")); Serial.print(controllerKiEEPROM); Serial.println(F(""));
  Serial.print(F("Controller Kd: ")); Serial.print(controllerKdEEPROM); Serial.println(F(""));
  Serial.print(F("Desired Velocity: ")); Serial.print(desiredVelocityEEPROM); Serial.println(F(" km/h"));
  Serial.print(F("Acceleration: ")); Serial.print(accelerationEEPROM); Serial.println(F(" m/s^2"));
  Serial.print(F("Engine Run Time Total: ")); Serial.print(engineRunTimeTotalEEPROM); Serial.println(F(" s"));
  Serial.print(F("Engine Run Time Since Maintenance: ")); Serial.print(engineRunTimeSinceLastMaintenanceEEPROM); Serial.println(F(" s"));
  Serial.print(F("Total Runs: ")); Serial.print(totalRunsEEPROM); Serial.println(F(""));
  //
  // Search for LCD display
  Wire.beginTransmission(i2cAddressLCD);
  if (Wire.endTransmission() != 0) {
    printErrorMessageAndHaltProgram(F("   LCD NOT FOUND!   "));
  }
  //
  // Initialize LCD display
  lcd.init();
  lcd.backlight();
  //
  // Add custom characters
  lcd.createChar(INVERTED_SPACE_SYMBOL, invertedSpace);
  lcd.createChar(INVERTED_LEFT_SYMBOL, invertedLeft);
  lcd.createChar(INVERTED_RIGHT_SYMBOL, invertedRight);
  lcd.createChar(INVERTED_E_SYMBOL, invertedE);
  lcd.createChar(INVERTED_S_SYMBOL, invertedS);
  lcd.createChar(INVERTED_C_SYMBOL, invertedC);
  lcd.createChar(INVERTED_O_SYMBOL, invertedO);
  lcd.createChar(INVERTED_K_SYMBOL, invertedK);
  //
  // Write welcome message to LCD
  lcd.setCursor(0, 0);
  lcd.print(F("Winch Control"));
  lcd.setCursor(0, 1);
  lcd.print(F("Created by:"));
  lcd.setCursor(12, 1);
  lcd.print(F(WINCH_CONTROL_AUTHOR));
  lcd.setCursor(0, 3);
  lcd.print(F("  Happy Shredding!  "));
  lcd.updateDisplay();
  //
  // Initialize accelerometer
  if (!acc.begin(Wire, i2cAddressACC)) {
    printErrorMessageAndHaltProgram(F("   ACC NOT FOUND!   "));
  }
  //
  // Search for magnetic rotational encoder
  Wire.beginTransmission(i2cAddressMAG);
  if (Wire.endTransmission() != 0) {
    printErrorMessageAndHaltProgram(F(" ENCODER NOT FOUND! "));
  }
  //
  // Initialize magnetic rotational encoder
  if(encoder.detectMagnet() == 1) {
    switch (encoder.getMagnetStrength()) {
      case 1:
        // printErrorMessageAndHaltProgram(F("    MAG TOO WEAK    "));
        break;
      case 2:
        break;
      case 3:
        // printErrorMessageAndHaltProgram(F("   MAG TOO STRONG   "));
        break;
    }
    lastEncoderReading = encoder.getRawAngle();
  } else {
    //
    // No communication with encoder established!
    printErrorMessageAndHaltProgram(F("  NO MAG DETECTED!  "));
  }
  //
  // Initialize variables
  buttonState            = LOW;
  buttonHighCount        = 0;
  waitForButtonRelease   = false;
  loopCounter            = 0;
  engineVibrationCounter = 0;
  revolutionCounter      = 0.0f;
  winchState             = WinchState::State::STANDBY;
  engineState            = EngineState::State::OFF;
  activeConfiguration     = ConfigurationItems::VELOCITY;
  selectedItem           = SelectedItem::OK;
  selectingValue         = false;
  commandedVelocity      = 0.0f;
  desiredVelocity        = desiredVelocityEEPROM;
  acceleration           = 100000000; // For PID tuning set value very high! // accelerationEEPROM;
  //
  // Set PID values
  // const float K_krit = 12.5f;
  // const float T_krit = 2.0f;
  // controllerKp = 0.60f * K_krit;
  // controllerKi = 1.20f * K_krit / T_krit;
  // controllerKd = 3.00f * K_krit * T_krit / 40.0f;
  controllerKp = controllerKpEEPROM;
  controllerKi = controllerKiEEPROM;
  controllerKd = controllerKdEEPROM;
  //
  // Just wait a bit so that the welcome message is readable
  idle(3000);
  //
  // Check if we have to inform about some important stuff
  if (((ENGINE_MAINTENANCE_INTERVAL - 1) * 60) <= engineRunTimeSinceLastMaintenanceEEPROM / 60) {
    printInfoMessage(F("MAINTENANCE REQUIRED"));
    //
    // Just wait a bit so that the info message is readable
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
  if (servosNeedCalibration) {
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
    if (servosNeedCalibration) {
      //
      // Initialize servos with max thinkable values
      throttleServo.attach(throttleServoPin, SERVO_MIN_PWM, SERVO_MAX_PWM);
      breakServo.attach(breakServoPin, SERVO_MIN_PWM, SERVO_MAX_PWM);
      //
      // Calibrate minimal value of throttle servo
      Serial.println(F("Calibrate minimal value of throttle servo ..."));
      throttleServoMin = getNumber(F("Min. Throttle Servo:"), SERVO_MIN_PWM, SERVO_MAX_PWM, &setThrottleServoMicroseconds);
      throttleServoMinEEPROM = throttleServoMin;
      Serial.print(F("Set minimal value of throttle servo to ")); Serial.print(throttleServoMin); Serial.println(F(" us!"));
      //
      // Calibrate maximal value of throttle servo
      Serial.println(F("Calibrate maximal value of throttle servo ..."));
      throttleServoMax = getNumber(F("Max. Throttle Servo:"), throttleServoMin, SERVO_MAX_PWM, &setThrottleServoMicroseconds);
      throttleServoMaxEEPROM = throttleServoMax;
      Serial.print(F("Set maximal value of throttle servo to ")); Serial.print(throttleServoMax); Serial.println(F(" us!"));
      //
      // Check if throttle servo should be inverted
      Serial.println(F("Set inversion of break servo ..."));
      throttleServoInverse = getBool(F("Inv. Throttle Servo:"));
      throttleServoInverseEEPROM = throttleServoInverse;
      Serial.print(F("Inversion of throttle servo ")); Serial.print(throttleServoInverse ? F("enabled") : F("disabled")); Serial.println(F("!"));
      //
      // Calibrate minimal value of break servo
      Serial.println(F("Calibrate minimal value of break servo ..."));
      breakServoMin = getNumber(F("Min. Break Servo:   "), SERVO_MIN_PWM, SERVO_MAX_PWM, &setBreakServoMicroseconds);
      breakServoMinEEPROM = breakServoMin;
      Serial.print(F("Set minimal value of break servo to ")); Serial.print(breakServoMin); Serial.println(F(" us!"));
      //
      // Calibrate maximal value of break servo
      Serial.println(F("Calibrate maximal value of break servo ..."));
      breakServoMax = getNumber(F("Max. Break Servo:   "), breakServoMin, SERVO_MAX_PWM, &setBreakServoMicroseconds);
      breakServoMaxEEPROM = breakServoMax;
      Serial.print(F("Set maximal value of break servo to ")); Serial.print(breakServoMax); Serial.println(F(" us!"));
      //
      // Check if break servo should be inverted
      Serial.println(F("Set inversion of break servo ..."));
      breakServoInverse = getBool(F("Inv. Break Servo:   "));
      breakServoInverseEEPROM = breakServoInverse;
      Serial.print(F("Inversion of break servo ")); Serial.print(breakServoInverse ? F("enabled") : F("disabled")); Serial.println(F("!"));
      //
      // Unload servos with calibration config and load servos again.
      throttleServo.detach();
      breakServo.detach();
    }
    //
    // Initialize servos
    throttleServo.attach(throttleServoPin, throttleServoMin, throttleServoMax);
    breakServo.attach(breakServoPin, breakServoMin, breakServoMax);
    throttleServo.writeMicroseconds(throttleServoInverse ? throttleServoMax : throttleServoMin);
    breakServo.writeMicroseconds(breakServoInverse ? breakServoMax : breakServoMin);
  } else {
    //
    // Initialize servos
    throttleServo.attach(throttleServoPin, throttleServoMin, throttleServoMax);
    breakServo.attach(breakServoPin, breakServoMin, breakServoMax);
    haltWinch();
  }
  //
  // Print header of log
  Serial.println(F("Time [ms];dt [ms];LoopCounter;CurrentEncoderReading [0.087890625 deg];AngularIncrementRaw [deg];AngularIncrement [deg];RevolutionCounter;RopeVelocity [km/h];RopeLength [m];WinchState;DesiredVelocity [km/h];CommandedVelocity [km/h];CurrentError [km/h];IntegralError [km/h];DifferentialError [km/h];ProportionalComponent;IntegralComponent;DifferentialComponent;FeedForwardComponent;ThrottleServoSetpoint;ThrottleServoMicroseconds [us];BreakServoSetpoint;BreakServoMicroseconds [us];AccX [g];AccY [g];AccZ [g];norm(Acc)^2 [g^2];EngineState;EngineVibrationCounter;ProcessingTime [ms];"));
  //
  // Initialize timing
  lastMillis = millis();
}

void updateRopeStatus(float& ropeVelocity, float& ropeLength)
{
  //
  // Get encoder reading and transform to angular increment from last reading
  float currentEncoderReading = encoder.getRawAngle();
  if (loopCounter == 1) {
    lastEncoderReading = currentEncoderReading;
  }
  float angularIncrement = (currentEncoderReading - lastEncoderReading) * ENCODER_RESOLUTION; // deg
  lastEncoderReading = currentEncoderReading;
  Serial.print(currentEncoderReading); Serial.print(';');
  Serial.print(angularIncrement); Serial.print(';');
  //
  // Check if we have an edge case [-180, +180]
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
  if (abs(angularIncrement) > 180.0f * (1.0f - SAFETY_MARGIN)) {
    //
    // We lost track of RPM due to to high rotational velocity -> HALT
    haltWinch();
    printErrorMessageAndHaltProgram(F("Violation of Nyquist"));
  }
  Serial.print(angularIncrement); Serial.print(';');
  //
  // Increment revolution counter
  revolutionCounter += angularIncrement / 360.0f;
  //
  // Convert angular increment to angular velocity
  const float omega = angularIncrement * CONTROL_LOOP_FREQ_HZ; // Angular velocity in deg/s
  //
  // Calculate rope velocity based on angular velocity, revolution counter and spool diameter
  // TODO: This term is going to be dependent on the number of revolutions
  ropeVelocity = (M_PI * SPOOL_DIAMETER / 100.0f) * abs(omega); // Rope velocity in km/h
  //
  // Calculate rope length based on revolution counter and spool diameter
  // TODO: This term is going to be non linear in the numbers of revolutions
  ropeLength = revolutionCounter * (M_PI * SPOOL_DIAMETER); // Rope length in m
  Serial.print(revolutionCounter); Serial.print(';');
  Serial.print(ropeVelocity); Serial.print(';');
  Serial.print(ropeLength); Serial.print(';');
}

void loop() {
  //
  // The following code is executed with the frequency of CONTROL_LOOP_FREQ_HZ
  long currentMillis = millis();
  if ((currentMillis - lastMillis) < CONTROL_LOOP_INTERVAL) return;
  //
  // Output a tone, if we missed our time frame
  // if ((currentMillis - lastMillis) > CONTROL_LOOP_INTERVAL) {
  //   tone(buzzerPin, 2000);
  // } else {
  //   noTone(buzzerPin);
  // }
  Serial.print(currentMillis); Serial.print(';');
  Serial.print((currentMillis - lastMillis)); Serial.print(';');
  lastMillis = currentMillis;
  //
  // Increase loop counter
  loopCounter++;
  Serial.print(loopCounter); Serial.print(';');
  //
  // Update rope status
  float ropeVelocity, ropeLength;
  updateRopeStatus(ropeVelocity, ropeLength);
  //
  // Check state of winch ...
  switch (winchState) {
    case WinchState::SPOOL_UP:
      {
        //
        // Release the break
        setBreakServo(0.0f);
        //
        // Check if we want to abort the run
        if (buttonPressedFor(EMERGENCY_STOP_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
          winchState = WinchState::STANDBY;
        }
        //
        // Check the remaining rope length to initiate stop
        if (ropeLength < (1.0f + SAFETY_MARGIN) * STOPPING_DISTANCE) {
          winchState = WinchState::SPOOL_DOWN;
        }
        //
        // Check if spool up time is reached
        if (millis() - winchState.lastChanged() > SPOOL_UP_TIME) {
          //
          // Switch to SHREDDING mode
          winchState = WinchState::SHREDDING;
          //
          // Increase number of total runs
          totalRunsEEPROM++;
        }
      }
      break;
    case WinchState::SHREDDING:
      {
        //
        // We are shredding so control the speed
        currentError = commandedVelocity - ropeVelocity;
        //
        // Calculate the proportional component
        float proportionalComponent = controllerKp * currentError;
        //
        // Calculate the integral component
        float integralComponent = controllerKi * integralError;
        //
        // Calculate differential component
        float differentialComponent = controllerKd * (currentError - lastError);
        //
        // Calculate feed forward component
        float feedForwardComponent = calculateFeedForwardComponent(commandedVelocity);
        //
        // Summing all components of the PID controller
        float throttle = proportionalComponent + integralComponent + differentialComponent + feedForwardComponent;
        setThrottleServo(throttle);
        //
        // Update variables for integral component
        // Only update integral part if throttle is not saturated
        if (0.0f <= throttle && throttle <= 1.0f) {
            integralError += currentError;
        }
        //
        // Update variables for differential component
        lastError = currentError;
        //
        // Check if we are still in the acceleration phase
        if (desiredVelocity > commandedVelocity) {
          //
          // Increase velocity gently
          commandedVelocity += acceleration / CONTROL_LOOP_FREQ_HZ * 3.6f; // Convert m/s to km/h
          //
          // Keep velocity in desired range
          if (desiredVelocity < commandedVelocity) {
            commandedVelocity = desiredVelocity;
          }
        }
        //
        // Check if we want to abort the run
        if (buttonPressedFor(EMERGENCY_STOP_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
          winchState = WinchState::SPOOL_DOWN;
        }
        //
        // Check the remaining rope length to initiate stop
        if (ropeLength < (1.0f + SAFETY_MARGIN) * STOPPING_DISTANCE) {
          winchState = WinchState::SPOOL_DOWN;
        }
      }
      break;
    case WinchState::SPOOL_DOWN:
      {
        //
        // Check if throttle is still open
        if (getThrottleServo() > (1.0f / (SPOOL_DOWN_TIME / 1000.0f * CONTROL_LOOP_FREQ_HZ))) {
          //
          // Reduce throttle
          setThrottleServo(getThrottleServo() - (1.0f / (SPOOL_DOWN_TIME / 1000.0f  * CONTROL_LOOP_FREQ_HZ)));
          //
          // Update winch state to store time of last throttle decrease
          winchState = WinchState::SPOOL_DOWN;
        } else {
          setThrottleServo(0.0f);
          //
          // Check if spool down time is reached
          if (millis() - winchState.lastChanged() > SPOOL_DOWN_TIME) {
            //
            // Switch to STANDBY mode to enable the break
            winchState = WinchState::STANDBY;
          }
        }
        //
        // Check if we want to abort the run
        if (buttonPressedFor(EMERGENCY_STOP_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
          winchState = WinchState::STANDBY;
        }
        //
        // Set desired velocity to zero
        commandedVelocity = 0.0f;
      }
      break;
    case WinchState::SPOOL_UP_WARNING:
      {
        //
        // Get millis since start of spool up phase
        long millisSinceStart = millis() - winchState.lastChanged();
        //
        // Play tone
        if ((0 * SPOOL_UP_BEEP_TIME < millisSinceStart && millisSinceStart < 1 * SPOOL_UP_BEEP_TIME) ||
            (2 * SPOOL_UP_BEEP_TIME < millisSinceStart && millisSinceStart < 3 * SPOOL_UP_BEEP_TIME) ||
            (4 * SPOOL_UP_BEEP_TIME < millisSinceStart && millisSinceStart < 5 * SPOOL_UP_BEEP_TIME)) {
          tone(buzzerPin, 2000);
        } else if (6 * SPOOL_UP_BEEP_TIME < millisSinceStart && millisSinceStart < 8 * SPOOL_UP_BEEP_TIME) {
          tone(buzzerPin, 2500);
        } else {
          noTone(buzzerPin);
        }
        //
        // Check if we want to abort the run
        if (buttonPressedFor(EMERGENCY_STOP_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
          winchState = WinchState::STANDBY;
        }
        //
        // After beeping go to spool up phase
        if (millisSinceStart >= 8 * SPOOL_UP_BEEP_TIME) {
          //
          // Transfer to spool up state
          winchState = WinchState::SPOOL_UP;
        }
      }
	  break;
    case WinchState::SPOOL_UP_REJECTED:
      {
        //
        // Get millis since start of spool up phase
        long millisSinceStart = millis() - winchState.lastChanged();
        //
        // Play tone
        if (0 * SPOOL_UP_BEEP_TIME < millisSinceStart && millisSinceStart < 1 * SPOOL_UP_BEEP_TIME) {
          tone(buzzerPin, 2000);
        } else if (1 * SPOOL_UP_BEEP_TIME < millisSinceStart && millisSinceStart < 3 * SPOOL_UP_BEEP_TIME) {
          tone(buzzerPin, 1000);
        } else {
          noTone(buzzerPin);
        }
        //
        // After beeping go to standby again
        if (millisSinceStart >= 3 * SPOOL_UP_BEEP_TIME) {
          //
          // Transfer to standby state
          winchState = WinchState::STANDBY;
        }
      }
	  break;
    case WinchState::CONFIGURATION:
    	{
        //
        // If engine is running halt winch, otherwise release spool to pull rope out
        if (engineState == EngineState::State::OFF) {
          if (selectingValue) {
            bool confirmed = false;
            switch (activeConfiguration)
            {
              case ConfigurationItems::VELOCITY:
                desiredVelocity = map(analogRead(potentiometerPin), 0, 1023, MINIMAL_VELOCITY, MAXIMAL_VELOCITY);
                break;
  
              case ConfigurationItems::ACCELERATION:
                acceleration = 100000000; // For PID tuning set value very high! // map(analogRead(potentiometerPin), 0, 1023, MINIMAL_ACCELERATION, MAXIMAL_ACCELERATION);
                break;
  
              case ConfigurationItems::P_GAIN:
                controllerKp = map(analogRead(potentiometerPin), 0, 1023, MINIMAL_P_GAIN, MAXIMAL_P_GAIN);
                break;
  
              case ConfigurationItems::I_GAIN:
                controllerKi = map(analogRead(potentiometerPin), 0, 1023, MINIMAL_I_GAIN, MAXIMAL_I_GAIN);
                break;
  
              case ConfigurationItems::D_GAIN:
                controllerKd = map(analogRead(potentiometerPin), 0, 1023, MINIMAL_D_GAIN, MAXIMAL_D_GAIN);
                break;

              case ConfigurationItems::THROTTLE_SERVO_MIN:
                throttleServoMin = map(analogRead(potentiometerPin), 0, 1023, SERVO_MIN_PWM, SERVO_MAX_PWM);
                setThrottleServoMicroseconds(throttleServoMin);
                break;

              case ConfigurationItems::THROTTLE_SERVO_MAX:
                throttleServoMax = map(analogRead(potentiometerPin), 0, 1023, throttleServoMin, SERVO_MAX_PWM);
                setThrottleServoMicroseconds(throttleServoMax);
                break;

              case ConfigurationItems::THROTTLE_SERVO_INVERSE:
                throttleServoInverse = (map(analogRead(potentiometerPin), 0, 1023, 0, 10) > 5);
                break;

              case ConfigurationItems::BREAK_SERVO_MIN:
                breakServoMin = map(analogRead(potentiometerPin), 0, 1023, SERVO_MIN_PWM, SERVO_MAX_PWM);
                setBreakServoMicroseconds(breakServoMin);
                break;

              case ConfigurationItems::BREAK_SERVO_MAX:
                breakServoMax = map(analogRead(potentiometerPin), 0, 1023, breakServoMin, SERVO_MAX_PWM);
                setBreakServoMicroseconds(breakServoMax);
                break;

              case ConfigurationItems::BREAK_SERVO_INVERSE:
                breakServoInverse = (map(analogRead(potentiometerPin), 0, 1023, 0, 10) > 5);
                break;

              case ConfigurationItems::REGISTER_MAINTENANCE:
              case ConfigurationItems::RESET_ALL:
                confirmed = (map(analogRead(potentiometerPin), 0, 1023, 0, 10) > 5);
                break;
              
              default:
                break;
            }
            if (buttonPressedForAndReleased(CONF_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
              //
              // We have confirmed our selected value
              selectingValue = false;
              //
              // Wite values to EEPROM
              desiredVelocityEEPROM = desiredVelocity;
              accelerationEEPROM = acceleration;
              controllerKpEEPROM = controllerKp;
              controllerKiEEPROM = controllerKi;
              controllerKdEEPROM = controllerKd;
              throttleServoMaxEEPROM = throttleServoMax;
              throttleServoMinEEPROM = throttleServoMin;
              breakServoMaxEEPROM = breakServoMax;
              breakServoMinEEPROM = breakServoMin;
              //
              // ?
              if (confirmed) {
                switch (activeConfiguration)
                {
                  case ConfigurationItems::REGISTER_MAINTENANCE:
                    engineRunTimeSinceLastMaintenanceEEPROM = 0;
                    break;

                  case ConfigurationItems::RESET_ALL:
                    clearEEPROM();
                    printErrorMessageAndHaltProgram(F("REBOOT RESET EEPROM!"));
                    break;
                    
                  default:
                    break;
                }
              }
            }
          } else {
            selectedItem = (SelectedItem) map(analogRead(potentiometerPin), 0, 1023, (int) LEFT, (int) RIGHT + 0.99f);
            if (buttonPressedForAndReleased(CONF_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
              switch (selectedItem)
              {
                case SelectedItem::LEFT:
                  {
                    int i = (int) activeConfiguration - 1;
                    if (i >= 0)
                      activeConfiguration = (ConfigurationItems) i;
                  }
                  break;
                case SelectedItem::RIGHT:
                  {
                    int i = (int) activeConfiguration + 1;
                    if (i <= (int) ConfigurationItems::RESET_ALL)
                      activeConfiguration = (ConfigurationItems) i;
                  }
                  break;
                case SelectedItem::OK:
                  {
                    selectingValue = true;
                  }
                  break;
                case SelectedItem::ESC:
                default:
                  {
                    winchState = WinchState::STANDBY;
                  }
                  break;
              }
            }
          }
          break;
        } else {
          //
          // Engine is running so leave configuration state
          winchState = WinchState::STANDBY;
          //
          // No break here to fall back to standby state
        }
			}
    case WinchState::STANDBY:
    default:
      {
        //
        // If engine is running halt winch, otherwise release spool to pull rope out
        if (engineState == EngineState::State::ON) {
          haltWinch();
        } else {
          releaseWinch();
        }
        //
        // Set desired velocity to zero
        commandedVelocity = 0.0f;
        //
        // Reset PID values
        currentError  = 0.0f;
        integralError = 0.0f;
        lastError     = 0.0f;
        //
        // User interaction
        if (buttonPressedFor(START_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
          if ((ropeLength >= (1.0f + SAFETY_MARGIN) * STOPPING_DISTANCE) && // <- Rope length is sufficient
              (engineState == EngineState::State::ON)) { // <- Engine is running
            winchState = WinchState::SPOOL_UP_WARNING;
          } else {
            winchState = WinchState::SPOOL_UP_REJECTED;
          }
        }
        if (buttonPressedForAndReleased(CONF_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
          winchState = WinchState::CONFIGURATION;
          activeConfiguration = ConfigurationItems::VELOCITY;
        }
      }
      break;
  }
  Serial.print(winchState); Serial.print(';');
  Serial.print(desiredVelocity); Serial.print(';');
  Serial.print(commandedVelocity); Serial.print(';');
  Serial.print(currentError); Serial.print(';');
  Serial.print(integralError); Serial.print(';');
  Serial.print(currentError - lastError); Serial.print(';');
  Serial.print(controllerKp * currentError); Serial.print(';');
  Serial.print(controllerKi * integralError); Serial.print(';');
  Serial.print(controllerKd * (currentError - lastError)); Serial.print(';');
  Serial.print(calculateFeedForwardComponent(commandedVelocity)); Serial.print(';');
  Serial.print(getThrottleServo()); Serial.print(';');
  Serial.print(getThrottleServoMicroseconds()); Serial.print(';');
  Serial.print(getBreakServo()); Serial.print(';');
  Serial.print(getBreakServoMicroseconds()); Serial.print(';');
  //
  // Update button status
  updateButton();
  //
  // Check engine state
  updateEngineState();
  //
  // If engine turned off after the winch state changed, put the winch to standby again
  if (engineState == EngineState::State::OFF && winchState > WinchState::State::STANDBY && engineState.lastChanged() > winchState.lastChanged()) {
    winchState = WinchState::STANDBY;
  }
  //
  // Update display winch state line
  if (loopCounter % (CONTROL_LOOP_FREQ_HZ / LCD_UPDATE_RATE) == 0) {
    lcd.setCursor(0, 1);
    switch (winchState) {
      case WinchState::SPOOL_UP_WARNING:
      case WinchState::SPOOL_UP:
        lcd.print(F("Spool Up:           "));
        displayRopeStatus(ropeVelocity, ropeLength);
        break;
      case WinchState::SHREDDING:
        lcd.print(F("Shredding:          "));
        displayRopeStatus(ropeVelocity, ropeLength);
        break;
      case WinchState::SPOOL_DOWN:
        lcd.print(F("Spool Down:         "));
        displayRopeStatus(ropeVelocity, ropeLength);
        break;
      case WinchState::CONFIGURATION:
        {
          lcd.print(F("Configuration:      "));
          switch (activeConfiguration) {
            case ConfigurationItems::VELOCITY:
              {
                lcd.print(F("Velocity:           "));
                lcd.setCursor(10, 2);
                lcd.print(desiredVelocity, 1);
                lcd.print(F(" km/h"));
              }
              break;
            case ConfigurationItems::ACCELERATION:
              {
                lcd.print(F("Accel.:             "));
                lcd.setCursor(8, 2);
                lcd.print(acceleration, 1);
                lcd.print(F(" m/s2"));
              }
              break;
            case ConfigurationItems::P_GAIN:
              {
                lcd.print(F("P-Gain.:            "));
                lcd.setCursor(9, 2);
                lcd.print(controllerKp, 4);
              }
              break;
            case ConfigurationItems::I_GAIN:
              {
                lcd.print(F("I-Gain.:            "));
                lcd.setCursor(9, 2);
                lcd.print(controllerKi, 4);
              }
              break;
            case ConfigurationItems::D_GAIN:
              {
                lcd.print(F("D-Gain.:            "));
                lcd.setCursor(9, 2);
                lcd.print(controllerKd, 4);
              }
              break;
            case ConfigurationItems::THROTTLE_SERVO_MIN:
              {
                lcd.print(F("THROT MIN:          "));
                lcd.setCursor(11, 2);
                lcd.print(throttleServoMin);
                lcd.print(F(" us"));
              }
              break;
            case ConfigurationItems::THROTTLE_SERVO_MAX:
              {
                lcd.print(F("THROT MAX:          "));
                lcd.setCursor(11, 2);
                lcd.print(throttleServoMax);
                lcd.print(F(" us"));
              }
              break;
            case ConfigurationItems::THROTTLE_SERVO_INVERSE:
              {
                lcd.print(F("THROT INV:          "));
                lcd.setCursor(11, 2);
                lcd.print(throttleServoInverse ? F("True") : F("False"));
              }
              break;
            case ConfigurationItems::BREAK_SERVO_MIN:
              {
                lcd.print(F("BRK MIN:            "));
                lcd.setCursor(9, 2);
                lcd.print(breakServoMin);
                lcd.print(F(" us"));
              }
              break;
            case ConfigurationItems::BREAK_SERVO_MAX:
              {
                lcd.print(F("BRK MAX:            "));
                lcd.setCursor(9, 2);
                lcd.print(breakServoMax);
                lcd.print(F(" us"));
              }
              break;
            case ConfigurationItems::BREAK_SERVO_INVERSE:
              {
                lcd.print(F("BRK INV:            "));
                lcd.setCursor(9, 2);
                lcd.print(breakServoInverse ? F("True") : F("False"));
              }
              break;
            case ConfigurationItems::RUNS_TOTAL:
              {
                lcd.print(F("Total Runs:         "));
                lcd.setCursor(12, 2);
                lcd.print(totalRunsEEPROM);
              }
              break;
            case ConfigurationItems::RUN_TIME_TOTAL:
              {
                lcd.print(F("Total Runtime:      "));
                lcd.setCursor(15, 2);
                      unsigned long m = engineRunTimeTotalEEPROM / 60;
                const unsigned long h = m / 60;
                                    m = m - h * 60;
                if (h < 10) lcd.print(F("0"));
                lcd.print(h);
                lcd.print(F(":"));
                if (m < 10) lcd.print(F("0"));
                lcd.print(m);
              }
              break;
            case ConfigurationItems::RUN_TIME_MAINTENANCE:
              {
                lcd.print(F("Maintenance in      "));
                lcd.setCursor(15, 2);
                if ((ENGINE_MAINTENANCE_INTERVAL * 60) <= engineRunTimeSinceLastMaintenanceEEPROM / 60) {
                  lcd.print(F("00:00"));
                } else {
                        unsigned long m = (ENGINE_MAINTENANCE_INTERVAL * 60) - engineRunTimeSinceLastMaintenanceEEPROM / 60;
                  const unsigned long h = m / 60;
                                      m = m - h * 60;
                  if (h < 10) lcd.print(F("0"));
                  lcd.print(h);
                  lcd.print(F(":"));
                  if (m < 10) lcd.print(F("0"));
                  lcd.print(m);
                }
              }
              break;
            case ConfigurationItems::REGISTER_MAINTENANCE:
              {
                lcd.print(F("Reset Maintenance   "));
                if (selectingValue) {
                  lcd.setCursor(17, 2);
                  lcd.print(F(": "));
                  if (map(analogRead(potentiometerPin), 0, 1023, 0, 10) > 5) {
                    lcd.print(F("Y"));
                  } else {
                    lcd.print(F("N"));
                  }
                }
              }
              break;
            case ConfigurationItems::RESET_ALL:
            default:
              {
                lcd.print(F("Reset EEPROM        "));
                if (selectingValue) {
                  lcd.setCursor(12, 2);
                  lcd.print(F(":      "));
                  if (map(analogRead(potentiometerPin), 0, 1023, 0, 10) > 5) {
                    lcd.print(F("Y"));
                  } else {
                    lcd.print(F("N"));
                  }
                }
              }
          }
          if (selectingValue) {
            displayMenu(OK);
          } else {
            displayMenu(selectedItem);
          }
        }
        break;
      case WinchState::SPOOL_UP_REJECTED:
      case WinchState::STANDBY:
      default:
        if (engineState == EngineState::State::ON) {
          lcd.print(F("Standby: Breaking   "));
        } else {
          lcd.print(F("Standby: Freerunning"));
        }
        displayRopeStatus(ropeVelocity, ropeLength);
        break;
    }
  }
  lcd.updateDisplay(1);
  Serial.print(millis() - currentMillis); Serial.print(';');
  Serial.println();
}
