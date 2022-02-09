//
// Define version information
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
EEPROMStorage<float>         throttleMaxTravelEEPROM(39, 0.0f);             // This variable stores the calibrated maximum travel of the throttle servo in mm. It is stored in EEPROM at position 39 (4 + 1 bytes)
EEPROMStorage<float>         breakMaxTravelEEPROM(44, 0.0f);                // This variable stores the calibrated maximum travel of the break servo in mm. It is stored in EEPROM at position 44 (4 + 1 bytes)
//
// Define global variables to enable faster access to EEPROM variables
unsigned int throttleServoMin;
unsigned int throttleServoMax;
bool         throttleServoInverse;
unsigned int breakServoMin;
unsigned int breakServoMax;
bool         breakServoInverse;
float         throttleMaxTravel;
float         breakMaxTravel;
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
#include "LCD_Wrapper.hpp"
LCD_Wrapper lcd(i2cAddressLCD, 20, 4);
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
#define SAFETY_MARGIN 0.1f // Percentage to increase safety margins 
#define ANGULAR_INCREMENT_DEADBAND 0.1f // Angular increments below this value are ignored
#define EMERGENCY_STOP_SIGNAL_DURATION 0.5f // Minimum time the button has to be pressed to start breaking in seconds
#define MINIMAL_STOPPING_DISTANCE 25.0f // Minimum distance the winch needs to come to a complete stop in m.
#define THROTTLE_DOWN_TIME 2.0f // Time in seconds to fully throttle down
#define SPOOL_DOWN_TIME 2000 // Time in milliseconds to let the spool decelerate
#define DEFAULT_ACCELERATION 2.0f // Acceleration of the rope to reach desired velocity in m/s^2
#define SPOOL_UP_TIME 5000 // Time in milliseconds to let the spool spin up in idle / let the rope get tight
#define LCD_UPDATE_RATE 2 // Frequency of the display update in Hz
#define ENGINE_RUNNING_TIME_WINDOW 2 // Time horizon to check if engine is running in seconds
#define ENGINE_VIBRATION_THRESHOLD 2.25f // Squared norm of acceleration threshold to detect engine vibrations
//
// Define constants
class WinchState
{
  public:
    enum State
    {
      STANDBY       = 0,
      SPOOL_UP      = 1,
      SHREDDING     = 2,
      SPOOL_DOWN    = 3
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
unsigned int  buttonState;
unsigned long buttonHighCount;
bool          waitForButtonRelease;
unsigned long loopCounter;
unsigned int  engineVibrationCounter;
word          lastEncoderReading;
float          lastAngularIncrement;
float          revolutionCounter;
WinchState    winchState;
EngineState   engineState;
float          commandedVelocity;
float          desiredVelocity;
float          acceleration;
//
// Define control loop
#define CONTROL_LOOP_FREQ_HZ 50 // Frequency of the control loop in Hz
#define CONTROL_LOOP_INTERVAL (1000 / CONTROL_LOOP_FREQ_HZ)
long lastMillis;
//
// Define PID controller values
float controllerKp;
float controllerKi;
float controllerKd;
float lastError;
float integralError;
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
  Serial.print(F("Winch Control"));
  Serial.print(F("Created by: ")); Serial.println(F(WINCH_CONTROL_AUTHOR));
  //
  // Display some debug information
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));
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
  // Write welcome message to LCD
  lcd.setCursor(0, 0);
  lcd.print(F("Winch Control"));
  lcd.setCursor(0, 1);
  lcd.print(F("Created by:"));
  lcd.setCursor(12, 1);
  lcd.print(F(WINCH_CONTROL_AUTHOR));
  lcd.updateDisplay();
  //
  // Initialize accelerometer
  if (!acc.begin(Wire, i2cAddressACC)) {
    printErrorMessageAndHaltProgram(F("   ACC NOT FOUND!   "));
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
    printErrorMessageAndHaltProgram(F(" ENCODER NOT FOUND! "));
  }
  //
  // Initialize variables
  buttonState            = LOW;
  buttonHighCount        = 0;
  waitForButtonRelease   = false;
  loopCounter            = 0;
  engineVibrationCounter = 0;
  revolutionCounter      = 0.0f;
  lastAngularIncrement   = 0.0f;
  winchState             = WinchState::State::STANDBY;
  engineState            = EngineState::State::OFF;
  commandedVelocity      = 0.0f;
  desiredVelocity        = 0.0f;
  acceleration           = DEFAULT_ACCELERATION;
  //
  // Set PID values
  const float K_krit = 12.5f;
  const float T_krit = 2.0f;
  controllerKp  = 0.60f * K_krit;
  controllerKi  = 1.20f * K_krit / T_krit;
  controllerKd  = 3.00f * K_krit * T_krit / 40.0f;
  //
  // Just wait a bit so that the welcome message is readable
  idle(3000);
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
    haltWinch();
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
  if (abs(angularIncrement) > 180.0f * (1.0f - SAFETY_MARGIN)) {
    //
    // We lost track of RPM due to to high rotational velocity -> HALT
    haltWinch();
    printErrorMessageAndHaltProgram(F("Violation of Nyquist"));
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
  //
  // Output a tone, if we missed our time frame
  if ((currentMillis - lastMillis) > CONTROL_LOOP_INTERVAL) {
    tone(buzzerPin, 1000);
  } else {
    noTone(buzzerPin);
  }
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
        //
        // Release the break
        setBreakServoTravel(0.0f);
        //
        // Check if we want to abort the run
        if (buttonPressedFor(EMERGENCY_STOP_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
          winchState = WinchState::STANDBY;
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
        float error = commandedVelocity - ropeVelocity;
        //
        // Calculate the proportional component
        float proportionalComponent = controllerKp * error;
        //
        // Calculate the integral component
        float integralComponent = controllerKi * integralError;
        //
        // Calculate differential component
        float differentialComponent = controllerKd * (error - lastError);
        //
        // Summing all components of the PID controller
        float throttleTravel = proportionalComponent + integralComponent + differentialComponent;
        setThrottleServoTravel(throttleTravel);
        //
        // Update variables for integral component
        // Only update integral part if throttle travel is not saturated
        if (0.0f <= throttleTravel && throttleTravel <= throttleMaxTravel) {
            integralError += error;
        }
        //
        // Update variables for differential component
        lastError = error;
        //
        // Check if we are still in the acceleration phase
        if (desiredVelocity > commandedVelocity) {
          //
          // Increase velocity gently
          commandedVelocity += acceleration / CONTROL_LOOP_FREQ_HZ;
          //
          // Keep velocity in desired range
          if (desiredVelocity < commandedVelocity) {
            commandedVelocity = desiredVelocity;
          }
        }
        //
        // Check if we want to abort the run
        if (buttonPressedFor(EMERGENCY_STOP_SIGNAL_DURATION * CONTROL_LOOP_FREQ_HZ)) {
          winchState = WinchState::STANDBY;
        }
        //
        // Check the remaining rope length to initiate stop
        if (ropeLength < (1.0f + SAFETY_MARGIN) * MINIMAL_STOPPING_DISTANCE) {
          winchState = WinchState::SPOOL_DOWN;
        }
      }
      break;
    case WinchState::SPOOL_DOWN:
      {
        //
        // Check if throttle is still open
        if (getThrottleServoTravel() > (throttleMaxTravel / (THROTTLE_DOWN_TIME * CONTROL_LOOP_FREQ_HZ))) {
          //
          // Reduce throttle
          setThrottleServoTravel(getThrottleServoTravel() - (throttleMaxTravel / (THROTTLE_DOWN_TIME * CONTROL_LOOP_FREQ_HZ)));
        } else {
          //
          // Update winch state to store time of reaching full throttle down
          winchState = WinchState::SPOOL_DOWN;
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
        desiredVelocity   = 0.0f;
      }
      break;
    case WinchState::STANDBY:
    default:
      {
        //
        // Ensure winch is in STANDBY mode
        haltWinch();
        //
        // Set desired velocity to zero
        commandedVelocity = 0.0f;
        desiredVelocity   = 0.0f;
        //
        // Reset PID values
        integralError = 0.0f;
        lastError     = 0.0f;
      }
      break;
  }
  //
  // Update button status
  updateButton();
  //
  // Check engine state
  updateEngineState();
  //
  // If engine turned off after the winch state changed, but the winch to standby again
  if (engineState == EngineState::State::OFF && winchState > WinchState::State::STANDBY && engineState.lastChanged() > winchState.lastChanged()) {
    winchState = WinchState::STANDBY;
  }
  //
  // Update display with X Hz
  if (loopCounter % (CONTROL_LOOP_FREQ_HZ / LCD_UPDATE_RATE) == 0) {
    lcd.setCursor(0, 1);
    switch (winchState) {
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
      case WinchState::STANDBY:
      default:
        lcd.print(F("Standby:            "));
        displayRopeStatus(ropeVelocity, ropeLength);
        break;
    }
  }
  lcd.updateDisplay(1);
}
//
// Define helper functions
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

bool buttonClickedFor(unsigned int counter)
{
  if (buttonState == LOW && buttonHighCount > counter) {
    buttonHighCount = 0;
    return true;
  } else {
    return false;
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
    //
    // Calculate squared norm of acceleration vector
    float norm2 = SQ(acc.cx) + SQ(acc.cy) + SQ(acc.cz);
    //
    // Check if squared norm is above defined threshold for specified time
    if (norm2 >= ENGINE_VIBRATION_THRESHOLD) {
      if (engineVibrationCounter < ENGINE_RUNNING_TIME_WINDOW * CONTROL_LOOP_FREQ_HZ) {
        engineVibrationCounter++;
      }
    } else if (engineVibrationCounter > 0) {
      engineVibrationCounter--;
    }
    if (2 * engineVibrationCounter >= ENGINE_RUNNING_TIME_WINDOW * CONTROL_LOOP_FREQ_HZ) {
      if (engineState != EngineState::State::ON) {
        engineState = EngineState::State::ON;
      }
    } else {
      engineRunTimeTotalEEPROM                += (millis() - engineState.lastChanged()) / 1000;
      engineRunTimeSinceLastMaintenanceEEPROM += (millis() - engineState.lastChanged()) / 1000;
      engineState = EngineState::State::OFF;
    }
  } else {
    //
    // We lost the accelerometer -> HALT
    haltWinch();
    printErrorMessageAndHaltProgram(F("   NO ACC READING   "));
  }
}

void displayRopeStatus(const float& ropeVelocity, const float& ropeLength)
{
  lcd.setCursor(0, 2);
  lcd.print(F("Velocity:      Rope:"));
  lcd.setCursor(0, 3);
  lcd.print(F("     km/h          m"));
  lcd.setCursor(0, 3);
  if (abs(ropeVelocity) < 10) lcd.print(" ");
  lcd.print(abs(ropeVelocity), 1);
  lcd.setCursor(12, 3);
  if (-99.95f <  ropeLength && ropeLength < 999.95f) lcd.print(" ");
  if ( -9.95f <  ropeLength && ropeLength <  99.95f) lcd.print(" ");
  if (  0.00f <= ropeLength && ropeLength <   9.95f) lcd.print(" ");
  lcd.print(ropeLength, 1);
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

void printErrorMessageAndHaltProgram(const __FlashStringHelper* text)
{
  //
  // Print error message
  printErrorMessage(text);
  //
  // Halt program
  haltProgram();
}

void printErrorMessage(const __FlashStringHelper* text)
{
  Serial.print(F("Error: ")); Serial.println(text);
  lcd.setCursor(0, 1);
  lcd.print(F("       ERROR:       "));
  lcd.setCursor(0, 2);
  lcd.print(F("                    "));
  lcd.setCursor(0, 3);
  lcd.print(F("                    "));
  lcd.setCursor(0, 3);
  lcd.print(text);
  lcd.updateDisplay();
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
