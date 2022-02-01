//
// Define version information
#define WINCH_CONTROL_VERSION "0.1"
#define WINCH_CONTROL_AUTHOR  "N.Ammann"
//
// Define persistent variables in EEPROM using EEPROM-Storage library!
#include <EEPROM-Storage.h>
EEPROMStorage<unsigned int> engineRunTimeSinceLastMaintenance(0, 0); // This variable stores engine run time since last maintenance in seconds. It is stored in EEPROM at positions 0 (4 + 1 bytes)
EEPROMStorage<unsigned int> engineRunTimeTotal(5, 0);                // This variable stores total engine run time in seconds. It is stored in EEPROM at positions 5 (4 + 1 bytes)
EEPROMStorage<unsigned int> totalRuns(10, 0);                        // This variable stores the total number of runs. It is stored in EEPROM at positions 10 (4 + 1 bytes)
EEPROMStorage<unsigned int> throttleServoMin(15,  900);              // This variable stores the PWM value for the minimal angle. It is stored in EEPROM at positions 15 (4 + 1 bytes)
EEPROMStorage<unsigned int> throttleServoMax(20, 2000);              // This variable stores the PWM value for the maximal angle. It is stored in EEPROM at positions 20 (4 + 1 bytes)
EEPROMStorage<bool>         throttleServoInverse(25, false);         // This variable indicates if the rotation direction is inversed. It is stored in EEPROM at positions 25 (1 + 1 bytes)
EEPROMStorage<unsigned int> breakServoMin(27,  900);                 // This variable stores the PWM value for the minimal angle. It is stored in EEPROM at positions 27 (4 + 1 bytes)
EEPROMStorage<unsigned int> breakServoMax(32, 2000);                 // This variable stores the PWM value for the maximal angle. It is stored in EEPROM at positions 32 (4 + 1 bytes)
EEPROMStorage<bool>         breakServoInverse(37, true);             // This variable indicates if the rotation direction is inversed. It is stored in EEPROM at positions 37 (1 + 1 bytes)
EEPROMStorage<float>        throttleMaxTravel(39, 0.0f);             // This variable stores the calibrated maximum travel of the throttle servo in mm. It is stored in EEPROM at position 39 (4 + 1 bytes)
EEPROMStorage<float>        breakMaxTravel(44, 0.0f);                // This variable stores the calibrated maximum travel of the break servo in mm. It is stored in EEPROM at position 44 (4 + 1 bytes)
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
Servo breakServo;
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
// Define math constants
// #define M_PI 3.14159265358979323846f
//
// Define global variables
bool         lastButtonState;
unsigned int loopCounter;
word         lastEncoderReading;
float        lastAngularIncrement;
float        revolutionCounter;
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
  Serial.print("Winch Control v"); Serial.println(WINCH_CONTROL_VERSION);
  Serial.print("Created by: "); Serial.println(WINCH_CONTROL_AUTHOR);
  //
  // Display some debug information
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));
  //
  // Search for LCD display
  Wire.beginTransmission(i2cAddressLCD);
  if (Wire.endTransmission() != 0) {
    Serial.println("LCD display not found!");
    setupSucceeded = false;
  }
  //
  // Initialize LCD display
  lcd.init();
  lcd.backlight();
  //
  // Write welcome message to LCD
  lcd.setCursor(0, 0);
  lcd.print("Winch Control v");
  lcd.setCursor(15, 0);
  lcd.print(WINCH_CONTROL_VERSION);
  lcd.setCursor(0, 1);
  lcd.print("Created by:");
  lcd.setCursor(12, 1);
  lcd.print(WINCH_CONTROL_AUTHOR);
  //
  // Initialize accelerometer
  if (!acc.begin(Wire, i2cAddressACC)) {
    Serial.println("Accelerometer not found!");
    setupSucceeded = false;
  }
  //
  // Initialize magnetic rotational encoder
  if(encoder.detectMagnet() == 1) {
    switch (encoder.getMagnetStrength()) {
      case 1:
        Serial.println("Rotational encoder reading is too weak!");
        // setupSucceeded = false;
        break;
      case 2:
        Serial.print("Rotational encoder reading is good: ");
        Serial.println(encoder.getMagnitude());
        break;
      case 3:
        Serial.println("Rotational encoder reading is too strong!");
        // setupSucceeded = false;
        break;
    }
    lastEncoderReading = encoder.getRawAngle();
    
  } else {
    Serial.println("Rotational encoder has no magnetic reading!");
    setupSucceeded = false;
  }
  //
  // Initialize variables
  lastButtonState      = LOW;
  loopCounter          = 0;
  revolutionCounter    = 0.0;
  lastAngularIncrement = 0.0;
  //
  // Check if setup succeeded
  if (!setupSucceeded) {
    //
    // Print warning message
    Serial.println("Failure during boot!");
    lcd.setCursor(0, 3);
    lcd.print("Failure during boot!");
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
  bool servosNeedCalibration = !throttleServoMin.isInitialized() ||
                               !throttleServoMax.isInitialized() ||
                               !throttleServoInverse.isInitialized() ||
                               !breakServoMin.isInitialized() ||
                               !breakServoMax.isInitialized() ||
                               !breakServoInverse.isInitialized();
  bool travelNeedCalibration = !throttleMaxTravel.isInitialized() ||
                               !breakMaxTravel.isInitialized();
  if (servosNeedCalibration || travelNeedCalibration || digitalRead(buttonPin) == HIGH) {
    //
    // Display message
    Serial.println("Entering calibration mode ...");
    lcd.setCursor(0, 1);
    lcd.print("Calibration Mode:   ");
    //
    // Wait until button is released
    while (digitalRead(buttonPin) == HIGH);
    //
    // Check if we enter servo calibration
    if (servosNeedCalibration || getBool("Calibrate Servos?   ")) {
      //
      // Initialize servos with max thinkable values
      throttleServo.attach(throttleServoPin, 100u, 3000u);
      breakServo.attach(breakServoPin, 100u, 3000u);
      //
      // Calibrate minimal value of throttle servo
      Serial.println("Calibrate minimal value of throttle servo ...");
      throttleServoMin = getNumber("Min. Throttle Servo:", 100u, 3000u, &setThrottleServoMicroseconds);
      Serial.print("Set minimal value of throttle servo to "); Serial.print(throttleServoMin); Serial.println(" µs!");
      //
      // Calibrate maximal value of throttle servo
      Serial.println("Calibrate maximal value of throttle servo ...");
      throttleServoMax = getNumber("Max. Throttle Servo:", throttleServoMin.get(), 3000u, &setThrottleServoMicroseconds);
      Serial.print("Set maximal value of throttle servo to "); Serial.print(throttleServoMax); Serial.println(" µs!");
      //
      // Check if throttle servo should be inverted
      Serial.println("Set inversion of break servo ...");
      throttleServoInverse = getBool("Inv. Throttle Servo:");
      Serial.print("Inversion of throttle servo "); Serial.print(throttleServoInverse ? "enabled" : "disabled"); Serial.println("!");
      //
      // Calibrate minimal value of break servo
      Serial.println("Calibrate minimal value of break servo ...");
      breakServoMin = getNumber("Min. Break Servo:  ", 100u, 3000u, &setBreakServoMicroseconds);
      Serial.print("Set minimal value of break servo to "); Serial.print(breakServoMin); Serial.println(" µs!");
      //
      // Calibrate maximal value of break servo
      Serial.println("Calibrate maximal value of break servo ...");
      breakServoMax = getNumber("Max. Break Servo:  ", breakServoMin.get(), 3000u, &setBreakServoMicroseconds);
      Serial.print("Set maximal value of break servo to "); Serial.print(breakServoMax); Serial.println(" µs!");
      //
      // Check if break servo should be inverted
      Serial.println("Set inversion of break servo ...");
      breakServoInverse = getBool("Inv. Break Servo:  ");
      Serial.print("Inversion of break servo "); Serial.print(breakServoInverse ? "enabled" : "disabled"); Serial.println("!");
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
    if (travelNeedCalibration || getBool("Calibrate Travel?   ")) {
      //
      // Calibrate maximal travel of throttle servo
      Serial.println("Calibrate maximal travel of throttle servo ...");
      throttleMaxTravel = getNumber("Max. Throttle Travel", 0.0f, MAX_SERVO_TRAVEL, &setThrottleServoTravel);
      Serial.print("Set maximal travel of throttle servo to "); Serial.print(throttleMaxTravel); Serial.println(" mm!");
      //
      // Calibrate maximal travel of break servo
      Serial.println("Calibrate maximal travel of break servo ...");
      breakMaxTravel = getNumber("Max. Break Travel:  ", 0.0f, MAX_SERVO_TRAVEL, &setBreakServoTravel);
      Serial.print("Set maximal travel of break servo to "); Serial.print(breakMaxTravel); Serial.println(" mm!");
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
    emergencyStop();
    //
    // Print warning message
    Serial.println("Lost track of spool rotation due to to too high rotational velocity!");
    lcd.setCursor(0, 3);
    lcd.print("Failure: Nyquist!");
    //
    // Halt program
    haltProgram();
  }
  //
  // Increment revolution counter
  revolutionCounter += angularIncrement / 360.0f;
  //
  // Convert angular increment to angular velocity
  float omega = angularIncrement * CONTROL_LOOP_FREQ_HZ; // Angular velocity in deg/s
  //
  // Calculate rope velocity based on angular velocity, revolution counter and spool diameter
  // TODO: This term is going to be dependent on the number of revolutions
  float ropeVelocity = (M_PI * SPOOL_DIAMETER / 360.0f) * omega; // Rope velocity in m/s
  //
  // Calculate rope length based on revolution counter and spool diameter
  // TODO: This term is going to be non linear in the numbers of revolutions
  float ropeLength = revolutionCounter * (M_PI * SPOOL_DIAMETER); // Rope length in m










  





  
//  int buttonState = digitalRead(buttonPin);
  int potiValue = analogRead(potentiometerPin);
//  if (buttonState == HIGH) {
//    if (lastButtonState != buttonState) {
//      Serial.print("Number of button presses: ");
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
    lcd.print("Travel:      mm");
    lcd.setCursor(8, 3);
    lcd.print(map(potiValue, 0, 1023, 0.0f, 64.5f));
  //}
//  
//
//  if (acc.available()) {      // Wait for new data from accelerometer
//    // Acceleration of x, y, and z directions in g units
//    Serial.print(acc.getCalculatedX(), 3);
//    Serial.print("\t");
//    Serial.print(acc.getCalculatedY(), 3);
//    Serial.print("\t");
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

void emergencyStop()
{
  //
  // Put throttle to zero
  setThrottleServoTravel(0.0f);
  //
  // Enable break;
  setBreakServoTravel(breakMaxTravel);
}

void setThrottleServoMicroseconds(unsigned int us)
{
  throttleServo.writeMicroseconds(us);
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

void setBreakServoMicroseconds(int us)
{
  breakServo.writeMicroseconds(us);
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

inline bool getBool(const char* text)
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
    lcd.print("Value:              ");
    lcd.setCursor(7, 3);
    lcd.print(value ? "True" : "False");
    idle(100);
  }
  //
  // Wait until button is released
  while (digitalRead(buttonPin) == HIGH);
  //
  // Return value
  return value;
}

template< typename T >
inline T getNumber(const char* text, const T minimum, const T maximum, void (*func) (T))
{
  //
  // Initialize value
  T value = 0;
  //
  // Print text
  lcd.setCursor(0, 2);
  lcd.print(text);
  //
  // Read value from potentiometer as long as the button is not pressed
  while (digitalRead(buttonPin) == LOW) {
    value = map(analogRead(potentiometerPin), 0, 1023, minimum, maximum);
    if (func) (*func)(value);
    lcd.setCursor(0, 3);
    lcd.print("Value:              ");
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
