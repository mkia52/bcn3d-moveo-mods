#include <Arduino.h>
#include <AccelStepper.h>

//driver TB6560

const float anglePerStep = 1.8; // degrees per step for NEMA23
const float microstepping = 8; // microstepping setting (1, 2, 4, 8, 16, etc.)
const float anglePermicroStep = anglePerStep / microstepping; // degrees per microstep

const int Nema23stepPin = 54; // Pin A0 (X_STEP_PIN)
const int Nema23dirPin  = 55; // Pin A1 (X_DIR_PIN)
const int Nema23enPin   = 52; // Pin 38 (X_ENABLE_PIN)
const float Nema23AngleLimit = 60; 
const int Nema23MaxSpeed = 400; // Max speed in steps per second
const int Nema23MaxCurrent = 4.4; // Max current in Amperes
const int Nema23MaxAcceleration = 100; // Acceleration in steps per second squared

const int GearstepPin = 57;
const int GeardirPin = 58;
const int GearenPin = 53;
const float GearAngleLimit = 60; 
const int GearMaxSpeed = 3200; // Max speed in steps per second
const int GearMaxCurrent = 1.4; // Max current in Amperes
const int GearMaxAcceleration = 1600; // Acceleration in steps per second squared

const int MonitorPin = 2; // Pin 2 (X_LIMIT_PIN)

class MotorController : public AccelStepper {
public:
  enum MotorState {
    IDLE,
    CW,
    CCW
  };

private:
  float motorAngleMaxLimit;
  long motorStepMaxLimit;
  float anglePerMicroStep;

  float currentAngle = 0.0f;
  long currentStep = 0;

  float maxCurrent;
  float idleCurrent;

  int _dirPin;

  int enablePin;
  bool driverEnabled = false;
  MotorState currentState;

public:
  MotorController(int stepPin, int dirPin, int enPin, float angleLimit, float anglePerStep, float maxSpd, float maxCur)
    : AccelStepper(AccelStepper::DRIVER, stepPin, dirPin),
      motorAngleMaxLimit(angleLimit),
      anglePerMicroStep(anglePerStep),
      maxCurrent(maxCur),
      _dirPin(dirPin),
      enablePin(enPin)
  {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW); // Active-LOW enable

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    setMotorStepMaxLimit();
    setMaxSpeed(maxSpd);         // Inherited from AccelStepper
    setIdleCurrent(maxCurrent);
    
    this->currentState = IDLE;   // Assigned to the member variable
  }

  int directionPin(){
    return digitalRead(_dirPin);
  }

  void setMotorStepMaxLimit() {
    if (anglePerMicroStep > 0.0f) {
      motorStepMaxLimit = (long)(motorAngleMaxLimit / anglePerMicroStep);
    }
  }

  void setIdleCurrent(float current) {
    idleCurrent = current / 2.0f;
  }

  void setDriverEnabled(bool enable) {
    digitalWrite(enablePin, enable ? LOW : HIGH);
    driverEnabled = enable;
  }
  
  bool isDriverEnabled() {
    return driverEnabled;
  }

  MotorState getState() const {
    return currentState;
  }

  void setState(MotorState state) {
    currentState = state;
    updateSpeed();
  }

  long getStepMaxLimit() const {
    return motorStepMaxLimit;
  }

  void moveToAngle(float targetAngle) {
    if (targetAngle < -motorAngleMaxLimit || targetAngle > motorAngleMaxLimit) {
      Serial.println("Target angle out of bounds.");
      return;
    }

    long targetStep = (long)(targetAngle / anglePerMicroStep);
    moveTo(targetStep);
  }

  void currentPositionUpdate() {
    currentStep = currentPosition();
    currentAngle = currentStep * anglePerMicroStep;
  }

  void updateSpeed() {
    switch (currentState) {
      case IDLE:
        setSpeed(0);
        break;
      case CW:
        setSpeed(maxSpeed());
        break;
      case CCW:
        setSpeed(-maxSpeed());
        break;
    }
  }
};

MotorController NEMA23stepper(Nema23stepPin, Nema23dirPin, Nema23enPin, Nema23AngleLimit, anglePermicroStep, Nema23MaxSpeed, Nema23MaxCurrent); //eerything here is red lined with xx is not a type

MotorController Gearstepper(GearstepPin, GeardirPin, GearenPin, GearAngleLimit, anglePermicroStep, GearMaxSpeed, GearMaxCurrent); //same here with red lines and xx is not a type

// --- Motor Collection ---
MotorController* motors[] = { &NEMA23stepper, &Gearstepper };
const uint8_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);
const char* motorNames[2] = { "NEMA23", "GearStepper"};

uint8_t activeMotorIndex = 0; // Currently selected motor (0 = NEMA23, 1 = Gear)
unsigned long lastTelemetryPrint = 0;

void handleSerialInput();

void setup() {
  Serial.begin(115200);

  pinMode(MonitorPin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize all motors in the array
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motors[i]->setMinPulseWidth(20); // 20µs optocoupler pulse for TB6560
    motors[i]->setDriverEnabled(true);
    motors[i]->setState(MotorController::IDLE);
  }

  Serial.println(F("--- Multi-Stepper Controller Online ---"));
  Serial.println(F("Select Motor: [1] NEMA23 | [2] Gear"));
  Serial.println(F("Control:      [D] CW     | [A] CCW   | [S] Stop | [E] Toggle Enable | [W] Zero Pos"));
  Serial.print(F("Active Motor: "));
  Serial.println(motorNames[activeMotorIndex]);
}

void loop() {
  handleSerialInput();

  // Run all active, enabled motors
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    if (motors[i]->getState() != MotorController::IDLE && motors[i]->isDriverEnabled()) {
      motors[i]->runSpeed();
    }
  }

  // Non-blocking telemetry output every 250ms for the selected motor
  if (millis() - lastTelemetryPrint >= 250) {
    lastTelemetryPrint = millis();
    MotorController* current = motors[activeMotorIndex];

    if (current->getState() != MotorController::IDLE) {
      Serial.print(motorNames[activeMotorIndex]);
      Serial.print(F(" -> Step: "));
      Serial.print(current->currentPosition());
      Serial.print(F(" | Angle: "));
      Serial.println(current->currentPosition() * anglePermicroStep);
      Serial.print(F(" | Direction: "));
      Serial.println(current->directionPin() == HIGH ? F("CW") : F("CCW"));
    }
  }

  // Limit/monitor indicator
  digitalWrite(LED_BUILTIN, digitalRead(MonitorPin) == LOW ? HIGH : LOW);
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '\r' || cmd == '\n') {
      continue;
    }

    // Switch active motor via digit keys
    if (cmd >= '1' && cmd < ('1' + NUM_MOTORS)) {
      activeMotorIndex = cmd - '1';
      Serial.print(F("Switched control to: "));
      Serial.println(motorNames[activeMotorIndex]);
      return;
    }

    MotorController* activeMotor = motors[activeMotorIndex];

    switch (cmd) {
      case 'E':
      case 'e': {
        bool newState = !activeMotor->isDriverEnabled();
        activeMotor->setDriverEnabled(newState);
        Serial.print(motorNames[activeMotorIndex]);
        Serial.print(F(" driver: "));
        Serial.println(newState ? F("ENABLED") : F("DISABLED"));
        break;
      }

      case 'D':
      case 'd':
        if (!activeMotor->isDriverEnabled()) {
          Serial.println(F("Driver disabled. Press 'E' to enable."));
          break;
        }
        activeMotor->setState(MotorController::CW);
        Serial.print(motorNames[activeMotorIndex]);
        Serial.println(F(" moving CW"));
        break;

      case 'A':
      case 'a':
        if (!activeMotor->isDriverEnabled()) {
          Serial.println(F("Driver disabled. Press 'E' to enable."));
          break;
        }
        activeMotor->setState(MotorController::CCW);
        Serial.print(motorNames[activeMotorIndex]);
        Serial.println(F(" moving CCW"));
        break;

      case 'S':
      case 's':
        activeMotor->setState(MotorController::IDLE);
        Serial.print(motorNames[activeMotorIndex]);
        Serial.print(F(" stopped at position: "));
        Serial.println(activeMotor->currentPosition());
        break;

      case 'W':
      case 'w':
        Serial.print(motorNames[activeMotorIndex]);
        Serial.print(F(" zeroed. Previous: "));
        Serial.println(activeMotor->currentPosition());
        activeMotor->setCurrentPosition(0);
        break;

      default:
        Serial.print(F("Unknown command: '"));
        Serial.print(cmd);
        Serial.println(F("'. Use 1/2 to select motor, and D, A, S, E, W to control."));
        break;
    }
  }
}