#include <Arduino.h>
#include <AccelStepper.h>

//driver TB6560

const float microstepping = 8.0f; // microstepping setting (1, 2, 4, 8, 16, etc.)

const int Nema23stepPin = 54; // Pin A0 (X_STEP_PIN)
const int Nema23dirPin  = 55; // Pin A1 (X_DIR_PIN)
const int Nema23enPin   = 52; // Pin 38 (X_ENABLE_PIN)
const float Nema23angleperstep = 1.8f; // degrees per step
const float Nema23anglepermicrostep = Nema23angleperstep / microstepping; // degrees per microstep
const float Nema23AngleLimit = 55.0f; 
const float Nema23MaxSpeed = 1200.0f; // Max speed in steps per second
const float Nema23MaxCurrent = 4.4f; // Max current in Amperes
const float Nema23MaxAcceleration = 400.0f; // Acceleration in steps per second squared
const float Nema23MotorShaftPulleyDiameter = 21.5f; // in mm
const float Nema23DrivenPulleyDiameter = 124.0f; // in mm
const float Nema23OutputtoInputPulleyRatio = Nema23DrivenPulleyDiameter / Nema23MotorShaftPulleyDiameter; // Driven Pulley Diameter / Motor Pulley Diameter

const int GearstepPin = 57; // Pin A3
const int GeardirPin = 58; // Pin A4
const int GearenPin = 53;
const float Gearangleperstep = 0.35f; // degrees per step
const float Gearanglepermicrostep = Gearangleperstep / microstepping; // degrees per microstep
const float GearAngleLimit = 60.0f; 
const float GearMaxSpeed = 3200.0f; // Max speed in steps per second
const float GearMaxCurrent = 1.4f; // Max current in Amperes
const float GearMaxAcceleration = 1600.0f; // Acceleration in steps per second squared
const float GearMotorShaftPulleyDiameter = 21.5f; // in mm
const float GearDrivenPulleyDiameter = 45.0f; // in mm
const float GearOutputtoInputPulleyRatio = GearDrivenPulleyDiameter / GearMotorShaftPulleyDiameter; // Driven Pulley Diameter / Motor Pulley Diameter

const int MonitorPin = 2; // Pin 2 (X_LIMIT_PIN)

class MotorController : public AccelStepper {
public:
  enum MotorState {
    IDLE,
    CW,
    CCW
  };

private:
  float drivenPulleyAngleLimit;
  float motorAngleMaxLimit;
  long  motorStepMaxLimit;
  float anglePerMicroStep;
  float pulleyRatio;
  float gearRatio;

  float currentMotorPulleyAngle = 0.0f;
  float currentDrivenPulleyAngle = 0.0f;
  long  currentStep = 0;

  float maxCurrent;
  float idleCurrent;

  int _dirPin;
  int enablePin;
  bool driverEnabled = false;
  MotorState currentState;

public:
  MotorController(int stepPin, int dirPin, int enPin,  
                  float drivenAngleLimit, float anglePerMicroStep, 
                  float maxSpd, float maxAcc, float maxCur,
                  float OutputtoInputPulleyRatio, 
                  float gearRatio = 1.0f)
    : AccelStepper(AccelStepper::DRIVER, stepPin, dirPin),
      drivenPulleyAngleLimit(drivenAngleLimit),
      anglePerMicroStep(anglePerMicroStep),
      pulleyRatio(OutputtoInputPulleyRatio > 0.0f ? OutputtoInputPulleyRatio : 1.0f),
      gearRatio(gearRatio > 0.0f ? gearRatio : 1.0f),
      maxCurrent(maxCur),
      _dirPin(dirPin),
      enablePin(enPin)
  {
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW); // Active-LOW enable

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    setMaxSpeed(maxSpd);
    setAcceleration(maxAcc);
    setMotorAngleMaxLimit();
    setIdleCurrent(maxCurrent);
    
    this->currentState = IDLE;
  }

  void setMotorAngleMaxLimit() {
    // Total reduction: motor turns (pulleyRatio * gearRatio) times per 1 driven turn
    float totalRatio = pulleyRatio * gearRatio;
    motorAngleMaxLimit = drivenPulleyAngleLimit * totalRatio;
    setMotorStepMaxLimit();
  }

  void setMotorStepMaxLimit() {
    if (anglePerMicroStep > 0.0f) {
      motorStepMaxLimit = lroundf(motorAngleMaxLimit / anglePerMicroStep);
    }
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

  long getStepMaxLimit() const {
    return motorStepMaxLimit;
  }

  int moveToAngle(float targetDrivenAngle = 0.0f) {
    currentPositionUpdate(); 

    int lastTelemetery = millis();

    bool targetAngleExceeds = (fabs(targetDrivenAngle) > fabs(drivenPulleyAngleLimit));

    if (targetAngleExceeds) {
      Serial.println(F("Target angle exceeds driven pulley limit."));
      return 1;
    }

    // Convert target driven angle -> required motor shaft angle
    float targetMotorAngle = targetDrivenAngle * pulleyRatio * gearRatio;

    // Convert angle to nearest integer step
    long targetStep = lroundf(targetMotorAngle / anglePerMicroStep);
    
    moveTo(targetStep);

    while(distanceToGo() != 0) {
      run();
      currentPositionUpdate();
      if(millis() - lastTelemetery >= 500){
        lastTelemetery = millis();
        Serial.print(F(" -> Step: "));
        Serial.print(getCurrentPosition());
        Serial.print(F(" | Angle: "));
        Serial.println(getDrivenPulleyAngle());
        Serial.print(F(" | Direction: "));
        Serial.println(directionPin() == HIGH ? F("CW") : F("CCW"));
      }
    }
    setState(IDLE);
    return 0;
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
  
  int currentAnglelimitCheck(void) {
    if (currentDrivenPulleyAngle > drivenPulleyAngleLimit || currentDrivenPulleyAngle < -drivenPulleyAngleLimit) {
      //Serial.println(F("Current angle exceeds driven pulley limit."));
      return 1;
    }
    return 0;
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
        setSpeed(maxSpeed());
        break;
      case CCW:
        setSpeed(-maxSpeed());
        break;
    }
  }
};

MotorController NEMA23stepper(Nema23stepPin, Nema23dirPin, Nema23enPin, Nema23AngleLimit, Nema23anglepermicrostep, Nema23MaxAcceleration, Nema23MaxSpeed, Nema23MaxCurrent, Nema23OutputtoInputPulleyRatio); 
MotorController Gearstepper(GearstepPin, GeardirPin, GearenPin, GearAngleLimit, Gearanglepermicrostep, GearMaxSpeed, GearMaxAcceleration, GearMaxCurrent, GearOutputtoInputPulleyRatio); 

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
    motors[i]->currentPositionUpdate(); // Update current position and angles

    if(motors[i]->isDriverEnabled() == false) {
      if(motors[i]->getState() != MotorController::IDLE) {
        motors[i]->setState(MotorController::IDLE);
        Serial.print(motorNames[i]);
        Serial.println(F(" driver is disabled. Stopping motor."));
      }
      continue;
    }
    
    bool exceedsLimit = motors[i]->currentAnglelimitCheck() == 1;

    if(exceedsLimit) {

      bool recovering = (motors[i]->getState() == MotorController::CW && motors[i]->getDrivenPulleyAngle() < 0) || (motors[i]->getState() == MotorController::CCW && motors[i]->getDrivenPulleyAngle() > 0);
      
      if(recovering) {
         motors[i]->runSpeed();
        }
      else if(motors[i]->getState() != MotorController::IDLE) {
        Serial.print(motorNames[i]);
        Serial.println(F(" current angle exceeds limit. Stopping motor."));
        motors[i]->setState(MotorController::IDLE);
      }
    }
    else{
      motors[i]->runSpeed();
    }


  }

  // Non-blocking telemetry output every 250ms for the selected motor
  if (millis() - lastTelemetryPrint >= 250) {
    lastTelemetryPrint = millis();
    MotorController* current = motors[activeMotorIndex];

    if (current->getState() != MotorController::IDLE) {
      current->currentPositionUpdate(); // Ensure we have the latest position
      Serial.print(motorNames[activeMotorIndex]);
      Serial.print(F(" -> Step: "));
      Serial.print(current->getCurrentPosition());
      Serial.print(F(" | Angle: "));
      Serial.println(current->getDrivenPulleyAngle());
      Serial.print(F(" | Direction: "));
      Serial.println(current->directionPin() == HIGH ? F("CW") : F("CCW"));
    }
  }

  // Limit/monitor indicator
  digitalWrite(LED_BUILTIN, digitalRead(MonitorPin) == LOW ? HIGH : LOW);
}

void handleSerialInput() {
  const byte MAX_CMD_LEN = 32;
  static char serialBuffer[MAX_CMD_LEN]; // static retains state between loop calls
  static byte bufIndex = 0;

  while (Serial.available() > 0) {
    char incomingByte = (char)Serial.read();

    // Check if the user pressed Enter
    if (incomingByte == '\n' || incomingByte == '\r') {
      if (bufIndex > 0) {
        serialBuffer[bufIndex] = '\0'; // Seal string
        
        // --- PROCESS COMMAND ONLY ONCE PER FULL LINE ---
        const char* input = serialBuffer;
        while (*input == ' ') input++; // Skip leading spaces

        if (*input != '\0') {
          char cmd = input[0];
          Serial.println(cmd);

          if (cmd >= '1' && cmd < ('1' + NUM_MOTORS)) {
            activeMotorIndex = cmd - '1';
            Serial.print(F("Active Motor: "));
            Serial.println(motorNames[activeMotorIndex]);
          } else {
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
                Serial.print(F(" stopped at Position: "));
                Serial.print(activeMotor->getCurrentPosition());
                Serial.print(F(" | Angle: "));
                Serial.println(activeMotor->getDrivenPulleyAngle());
                break;

              case 'W':
              case 'w':
                Serial.print(motorNames[activeMotorIndex]);
                Serial.print(F(" zeroed. Previous: "));
                Serial.println(activeMotor->getCurrentPosition());
                activeMotor->setCurrentPosition(0);
                break;

              case 'C':
              case 'c': {
                if (!activeMotor->isDriverEnabled()) {
                  Serial.println(F("Driver disabled. Press 'E' to enable."));
                  break;
                }
                input++; // Move past 'c' or 'C'
                while (*input == ' ') input++;
                if (*input == '\0') { // Correct empty check
                  Serial.println(F("Error: provide an angle, e.g. C 45"));
                  break;
                }
                float targetAngleInput = atof(input);
                //Serial.print("Registered Angle: ");
                //Serial.println(targetAngleInput);
                if(activeMotor->moveToAngle(targetAngleInput) == 0){
                  Serial.println("Success");
                  Serial.print(motorNames[activeMotorIndex]);
                  Serial.print(F(" stopped at Position: "));
                  Serial.print(activeMotor->getCurrentPosition());
                  Serial.print(F(" | Angle: "));
                  Serial.println(activeMotor->getDrivenPulleyAngle());
                }
                else{
                  Serial.println("Failure");
                }
                break;}
              
              default:
                Serial.print(F("Unknown command: '"));
                Serial.print(cmd);
                Serial.println(F("'. Use 1/2 to select motor, and D, A, S, E, W to control."));
                break;
            }
          }
        }

        bufIndex = 0; // Reset buffer index for next command
      }
    } else {
      if (bufIndex < MAX_CMD_LEN - 1) {
        serialBuffer[bufIndex++] = incomingByte;
      } else {
        Serial.println(F("Command too long!"));
        bufIndex = 0;
      }
    }
  }
}
