#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

//command for establishing serial connection for keyboard control: python -m serial.tools.miniterm COM13 115200 --raw
// modify this command according to your serial port and baud rate and run it in a terminal to interact with the controller.

// --- Servo Configuration ---
const int ServoPin = A0;
Servo servo;
float servoStepSize = 1.2f;             // Degrees per 20ms tick (adjust with +/-)
const float SERVO_MIN_ANGLE = 0.0f;
const float SERVO_MAX_ANGLE = 180.0f;
float currentServoAngle = 0.0f;

enum ServoState {
  SERVO_IDLE,
  SERVO_CW,   // Increasing angle (Closing)
  SERVO_CCW   // Decreasing angle (Opening)
};
ServoState servoState = SERVO_IDLE;
unsigned long lastServoJogTime = 0;
unsigned long lastServoTickTime = 0;
const unsigned long SERVO_TICK_INTERVAL_MS = 20; // 50 Hz servo update rate

// Driver TB6560
const float microstepping = 8.0f;

// --- Stepper Pin definitions & Specs ---
const int Nema23stepPin = A1; //on the driver it's written as CLK+
const int Nema23dirPin  = A2; //on the driver it's written as CW+
const int Nema23enPin   = 52; //on the driver it's written as EN+
const float Nema23angleperstep = 1.8f;
const float Nema23anglepermicrostep = Nema23angleperstep / microstepping;
const float Nema23MaxSpeed = 1200.0f;
const float Nema23MaxCurrent = 4.4f; 
const float Nema23MaxAcceleration = 400.0f; 
const float Nema23MotorShaftPulleyDiameter = 21.5f; 
const float Nema23DrivenPulleyDiameter = 124.0f; 
const float Nema23OutputtoInputPulleyRatio = Nema23DrivenPulleyDiameter / Nema23MotorShaftPulleyDiameter;

const int GearstepPin = A3;
const int GeardirPin  = A4;
const int GearenPin   = 50;
const float Gearangleperstep = 0.35f; 
const float Gearanglepermicrostep = Gearangleperstep / microstepping; 
const float GearMaxSpeed = 3200.0f; 
const float GearMaxCurrent = 1.4f; 
const float GearMaxAcceleration = 1600.0f; 
const float GearMotorShaftPulleyDiameter = 21.5f; 
const float GearDrivenPulleyDiameter = 45.0f;     
const float GearOutputtoInputPulleyRatio = GearDrivenPulleyDiameter / GearMotorShaftPulleyDiameter;

const int Wrist_roll_stepPin = A5;
const int Wrist_roll_dirPin  = A7;
const int Wrist_roll_enPin   = 53;
const float Wrist_roll_angleperstep = 1.80f; 
const float Wrist_roll_anglepermicrostep = Wrist_roll_angleperstep / microstepping; 
const float Wrist_roll_MaxSpeed = 1600.0f; 
const float Wrist_roll_MaxCurrent = 1.4f; 
const float Wrist_roll_MaxAcceleration = 1600.0f; 
const float Wrist_roll_MotorShaftPulleyDiameter = 1.0f; 
const float Wrist_roll_DrivenPulleyDiameter = 1.0f; 
const float Wrist_roll_OutputtoInputPulleyRatio = Wrist_roll_DrivenPulleyDiameter / Wrist_roll_MotorShaftPulleyDiameter;

const int Wrist_pitch_stepPin = A8;
const int Wrist_pitch_dirPin  = A9;
const int Wrist_pitch_enPin   = 51;
const float Wrist_pitch_angleperstep = 1.80f; 
const float Wrist_pitch_anglepermicrostep = Wrist_pitch_angleperstep / microstepping; 
const float Wrist_pitch_MaxSpeed = 1600.0f; 
const float Wrist_pitch_MaxCurrent = 1.4f; 
const float Wrist_pitch_MaxAcceleration = 1600.0f; 
const float Wrist_pitch_MotorShaftPulleyDiameter = 1.0f; 
const float Wrist_pitch_DrivenPulleyDiameter = 1.0f; 
const float Wrist_pitch_OutputtoInputPulleyRatio = Wrist_pitch_DrivenPulleyDiameter / Wrist_pitch_MotorShaftPulleyDiameter;

const int MonitorPin = 2;

class MotorController : public AccelStepper {
public:
  enum MotorState {
    IDLE,
    CW,
    CCW
  };

private:
  float anglePerMicroStep;
  float pulleyRatio;
  float gearRatio;

  float currentMotorPulleyAngle = 0.0f;
  float currentDrivenPulleyAngle = 0.0f;
  long  currentStep = 0;

  float maxConfiguredSpeed;
  float currentTargetSpeed;
  float speedStepDelta;

  float maxCurrent;
  float idleCurrent;

  int _dirPin;
  int enablePin;
  bool driverEnabled = false;
  MotorState currentState;

public:
  MotorController(int stepPin, int dirPin, int enPin,  
                  float anglePerMicroStep, 
                  float maxSpd, float maxAcc, float maxCur,
                  float OutputtoInputPulleyRatio, 
                  float gearRatio = 1.0f)
    : AccelStepper(AccelStepper::DRIVER, stepPin, dirPin),
      anglePerMicroStep(anglePerMicroStep),
      pulleyRatio(OutputtoInputPulleyRatio > 0.0f ? OutputtoInputPulleyRatio : 1.0f),
      gearRatio(gearRatio > 0.0f ? gearRatio : 1.0f),
      maxConfiguredSpeed(maxSpd),
      currentTargetSpeed(maxSpd * 0.5f),
      speedStepDelta(100.0f),
      maxCurrent(maxCur),
      _dirPin(dirPin),
      enablePin(enPin)
  {
    pinMode(enablePin, OUTPUT);
    setDriverEnabled(true);

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    setMaxSpeed(maxSpd);
    setAcceleration(maxAcc);
    setIdleCurrent(maxCurrent);
    
    this->currentState = IDLE;
  }

  int directionPin() {
    return digitalRead(_dirPin);
  }

  void setIdleCurrent(float current) {
    idleCurrent = current / 2.0f;
  }

  void setDriverEnabled(bool enable) {
    digitalWrite(enablePin, enable ? LOW : HIGH);
    driverEnabled = enable;
  }

  bool isDriverEnabled() const {
    return driverEnabled;
  }

  MotorState getState() const {
    return currentState;
  }

  void setState(MotorState state) {
    currentState = state;
    updateSpeed();
  }

  void increaseSpeed() {
    currentTargetSpeed += speedStepDelta;
    if (currentTargetSpeed > maxConfiguredSpeed) {
      currentTargetSpeed = maxConfiguredSpeed;
    }
    updateSpeed();
  }

  void decreaseSpeed() {
    currentTargetSpeed -= speedStepDelta;
    if (currentTargetSpeed < 50.0f) {
      currentTargetSpeed = 50.0f;
    }
    updateSpeed();
  }

  float getTargetSpeed() const {
    return currentTargetSpeed;
  }

  void currentPositionUpdate() {
    currentStep = currentPosition();
    currentMotorPulleyAngle = (float)currentStep * anglePerMicroStep;

    float totalRatio = pulleyRatio * gearRatio; 
    if (totalRatio > 0.0f) {
      currentDrivenPulleyAngle = currentMotorPulleyAngle / totalRatio;
    }
  }

  float getDrivenPulleyAngle() {
    return currentDrivenPulleyAngle;
  }

  long getCurrentPosition() {
    return currentStep;
  }

  void updateSpeed() {
    switch (currentState) {
      case IDLE:
        setSpeed(0.0f);
        break;
      case CW:
        setSpeed(currentTargetSpeed);
        break;
      case CCW:
        setSpeed(-currentTargetSpeed);
        break;
    }
  }
};

// --- Instances ---
MotorController NEMA23stepper(
  Nema23stepPin, Nema23dirPin, Nema23enPin, 
  Nema23anglepermicrostep, 
  Nema23MaxSpeed, Nema23MaxAcceleration, Nema23MaxCurrent, 
  Nema23OutputtoInputPulleyRatio
); 

MotorController Gearstepper(
  GearstepPin, GeardirPin, GearenPin, 
  Gearanglepermicrostep, 
  GearMaxSpeed, GearMaxAcceleration, GearMaxCurrent, 
  GearOutputtoInputPulleyRatio
);    

MotorController WristStepper_roll(
  Wrist_roll_stepPin, Wrist_roll_dirPin, Wrist_roll_enPin, 
  Wrist_roll_anglepermicrostep, 
  Wrist_roll_MaxSpeed, Wrist_roll_MaxAcceleration, Wrist_roll_MaxCurrent, 
  Wrist_roll_OutputtoInputPulleyRatio
);

MotorController WristStepper_pitch(
  Wrist_pitch_stepPin, Wrist_pitch_dirPin, Wrist_pitch_enPin, 
  Wrist_pitch_anglepermicrostep, 
  Wrist_pitch_MaxSpeed, Wrist_pitch_MaxAcceleration, Wrist_pitch_MaxCurrent, 
  Wrist_pitch_OutputtoInputPulleyRatio
);

MotorController* motors[] = { &NEMA23stepper, &Gearstepper, &WristStepper_roll, &WristStepper_pitch };
const uint8_t NUM_STEPPERS = sizeof(motors) / sizeof(motors[0]);
const uint8_t TOTAL_ACTUATORS = NUM_STEPPERS + 1; // 4 Steppers + 1 Servo

const char* motorNames[TOTAL_ACTUATORS] = { 
  "NEMA23", 
  "GearStepper", 
  "WristStepper_roll", 
  "WristStepper_pitch",
  "GripperServo"
};

uint8_t activeMotorIndex = 0; 
unsigned long lastTelemetryPrint = 0;
unsigned long lastJogCommandTime = 0;
const unsigned long JOG_TIMEOUT_MS = 250;

void handleSerialInput();

void setup() {
  Serial.begin(115200);

  // Initialize Servo
  servo.attach(ServoPin);
  servo.write((int)round(currentServoAngle));

  pinMode(MonitorPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    motors[i]->setMinPulseWidth(20);
    motors[i]->setDriverEnabled(true);
    motors[i]->setState(MotorController::IDLE);
  }

  Serial.println(F("--- Multi-Axis Controller (Steppers + Servo) ---"));
  Serial.println(F("Select Axis:   [1] NEMA23 | [2] Gear | [3] Wrist Roll | [4] Wrist Pitch | [5] Servo"));
  Serial.println(F("Jogging:       [D] CW/Close | [A] CCW/Open (Hold to run, release to lock)"));
  Serial.println(F("Speed Tuning:  [+] Increase Speed/Step | [-] Decrease Speed/Step"));
  Serial.println(F("Other:         [E] Toggle Enable/Power | [W] Zero Position | [S] Stop"));
  Serial.println(F("Servo Direct:  [O] Open | [C] or [P] Close (Operates servo anytime)"));
  Serial.print(F("Active Axis: "));
  Serial.println(motorNames[activeMotorIndex]);
}

void loop() {
  handleSerialInput();

  // Watchdog for Steppers
  if (activeMotorIndex < NUM_STEPPERS) {
    MotorController* activeMotor = motors[activeMotorIndex];
    if (activeMotor->getState() != MotorController::IDLE) {
      if (millis() - lastJogCommandTime > JOG_TIMEOUT_MS) {
        activeMotor->setState(MotorController::IDLE);
        Serial.print(motorNames[activeMotorIndex]);
        Serial.print(F(" held at Position: "));
        Serial.println(activeMotor->getCurrentPosition());
      }
    }
  }

  // Watchdog for Servo
  if (servoState != SERVO_IDLE) {
    if (millis() - lastServoJogTime > JOG_TIMEOUT_MS) {
      servoState = SERVO_IDLE;
      Serial.print(F("GripperServo held at Angle: "));
      Serial.print(currentServoAngle, 1);
      Serial.println(F(" deg"));
    }
  }

  // Servo non-blocking update tick (50 Hz rate)
  if (millis() - lastServoTickTime >= SERVO_TICK_INTERVAL_MS) {
    lastServoTickTime = millis();
    if (servoState == SERVO_CW) {
      currentServoAngle += servoStepSize;
      if (currentServoAngle > SERVO_MAX_ANGLE) currentServoAngle = SERVO_MAX_ANGLE;
      if (servo.attached()) servo.write((int)round(currentServoAngle));
    } else if (servoState == SERVO_CCW) {
      currentServoAngle -= servoStepSize;
      if (currentServoAngle < SERVO_MIN_ANGLE) currentServoAngle = SERVO_MIN_ANGLE;
      if (servo.attached()) servo.write((int)round(currentServoAngle));
    }
  }

  // Run all active steppers
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    motors[i]->currentPositionUpdate();

    if (!motors[i]->isDriverEnabled()) {
      if (motors[i]->getState() != MotorController::IDLE) {
        motors[i]->setState(MotorController::IDLE);
      }
      continue;
    }

    motors[i]->runSpeed();
  }

  digitalWrite(LED_BUILTIN, digitalRead(MonitorPin) == LOW ? HIGH : LOW);

  // Telemetry stream while moving
  if (millis() - lastTelemetryPrint >= 250) {
    lastTelemetryPrint = millis();
    if (activeMotorIndex < NUM_STEPPERS) {
      MotorController* activeMotor = motors[activeMotorIndex];
      if (activeMotor->getState() != MotorController::IDLE) {
        Serial.print(motorNames[activeMotorIndex]);
        Serial.print(F(" -> Step: "));
        Serial.print(activeMotor->getCurrentPosition());
        Serial.print(F(" | Speed: "));
        Serial.print(activeMotor->getTargetSpeed());
        Serial.print(F(" steps/s | Direction: "));
        Serial.println(activeMotor->directionPin() == HIGH ? F("CW") : F("CCW"));
      }
    } else {
      if (servoState != SERVO_IDLE) {
        Serial.print(motorNames[activeMotorIndex]);
        Serial.print(F(" -> Angle: "));
        Serial.print(currentServoAngle, 1);
        Serial.print(F(" deg | StepSize: "));
        Serial.print(servoStepSize, 2);
        Serial.print(F(" deg/tick | Direction: "));
        Serial.println(servoState == SERVO_CW ? F("CW (Close)") : F("CCW (Open)"));
      }
    }
  }
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r' || c == ' ') continue;

    // Switch Active Axis: '1' through TOTAL_ACTUATORS ('5')
    if (c >= '1' && c < ('1' + TOTAL_ACTUATORS)) {
      if (activeMotorIndex < NUM_STEPPERS) {
        motors[activeMotorIndex]->setState(MotorController::IDLE);
      } else {
        servoState = SERVO_IDLE;
      }
      
      activeMotorIndex = c - '1';
      Serial.print(F("Switched to Active Axis: ["));
      Serial.print(c);
      Serial.print(F("] "));
      Serial.print(motorNames[activeMotorIndex]);

      if (activeMotorIndex < NUM_STEPPERS) {
        Serial.print(F(" | Speed: "));
        Serial.print(motors[activeMotorIndex]->getTargetSpeed());
        Serial.println(F(" steps/s"));
      } else {
        Serial.print(F(" | Current Angle: "));
        Serial.print(currentServoAngle, 1);
        Serial.print(F(" deg | StepSize: "));
        Serial.print(servoStepSize, 2);
        Serial.println(F(" deg/tick"));
      }
      continue;
    }

    // Direct Servo Jogging (independent of active axis)
    if (c == 'O' || c == 'o') {
      servoState = SERVO_CCW;
      lastServoJogTime = millis();
      continue;
    }
    if (c == 'P' || c == 'p' || c == 'C' || c == 'c') {
      servoState = SERVO_CW;
      lastServoJogTime = millis();
      continue;
    }

    // Stepper Axis Controls
    if (activeMotorIndex < NUM_STEPPERS) {
      MotorController* activeMotor = motors[activeMotorIndex];

      switch (c) {
        case 'D':
        case 'd':
          if (activeMotor->isDriverEnabled()) {
            activeMotor->setState(MotorController::CW);
            lastJogCommandTime = millis();
          }
          break;

        case 'A':
        case 'a':
          if (activeMotor->isDriverEnabled()) {
            activeMotor->setState(MotorController::CCW);
            lastJogCommandTime = millis();
          }
          break;

        case '+':
        case '=':
          activeMotor->increaseSpeed();
          Serial.print(motorNames[activeMotorIndex]);
          Serial.print(F(" Speed increased to: "));
          Serial.print(activeMotor->getTargetSpeed());
          Serial.println(F(" steps/s"));
          break;

        case '-':
        case '_':
          activeMotor->decreaseSpeed();
          Serial.print(motorNames[activeMotorIndex]);
          Serial.print(F(" Speed decreased to: "));
          Serial.print(activeMotor->getTargetSpeed());
          Serial.println(F(" steps/s"));
          break;

        case 'S':
        case 's':
          activeMotor->setState(MotorController::IDLE);
          break;

        case 'E':
        case 'e': {
          bool newState = !activeMotor->isDriverEnabled();
          activeMotor->setDriverEnabled(newState);
          Serial.print(motorNames[activeMotorIndex]);
          Serial.println(newState ? F(" driver: ENABLED") : F(" driver: DISABLED"));
          break;
        }

        case 'W':
        case 'w':
          activeMotor->setCurrentPosition(0);
          Serial.print(motorNames[activeMotorIndex]);
          Serial.println(F(" position zeroed."));
          break;
      }
    } 
    // Servo Axis Controls (Active Motor 5)
    else {
      switch (c) {
        case 'D':
        case 'd':
          servoState = SERVO_CW;
          lastServoJogTime = millis();
          break;

        case 'A':
        case 'a':
          servoState = SERVO_CCW;
          lastServoJogTime = millis();
          break;

        case '+':
        case '=':
          servoStepSize += 0.2f;
          if (servoStepSize > 8.0f) servoStepSize = 8.0f;
          Serial.print(motorNames[activeMotorIndex]);
          Serial.print(F(" StepSize increased to: "));
          Serial.print(servoStepSize, 2);
          Serial.println(F(" deg/tick"));
          break;

        case '-':
        case '_':
          servoStepSize -= 0.2f;
          if (servoStepSize < 0.2f) servoStepSize = 0.2f;
          Serial.print(motorNames[activeMotorIndex]);
          Serial.print(F(" StepSize decreased to: "));
          Serial.print(servoStepSize, 2);
          Serial.println(F(" deg/tick"));
          break;

        case 'S':
        case 's':
          servoState = SERVO_IDLE;
          Serial.print(motorNames[activeMotorIndex]);
          Serial.print(F(" stopped at Angle: "));
          Serial.print(currentServoAngle, 1);
          Serial.println(F(" deg"));
          break;

        case 'E':
        case 'e': {
          if (servo.attached()) {
            servo.detach();
            Serial.println(F("GripperServo detached (unpowered/relaxed)."));
          } else {
            servo.attach(ServoPin);
            servo.write((int)round(currentServoAngle));
            Serial.println(F("GripperServo attached (holding position)."));
          }
          break;
        }

        case 'W':
        case 'w':
          currentServoAngle = 0.0f;
          if (servo.attached()) servo.write(0);
          Serial.print(motorNames[activeMotorIndex]);
          Serial.println(F(" angle zeroed to 0 deg."));
          break;
      }
    }
  }
}