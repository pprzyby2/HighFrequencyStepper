#include "HighFrequencyStepper.h"
#include "SPI.h"

// Constructor
HighFrequencyStepper::HighFrequencyStepper() {
    stepperCount = 0;
    globalEnable = false;
    
    // Initialize arrays to nullptr
    for (uint8_t i = 0; i < MAX_STEPPERS; i++) {
        pwmSteppers[i] = nullptr;
        pulseCounters[i] = nullptr;
        tmc2209Drivers[i] = nullptr;
        uartPorts[i] = nullptr;
    }
}

// Destructor
HighFrequencyStepper::~HighFrequencyStepper() {
    // Clean up allocated instances
    for (uint8_t i = 0; i < MAX_STEPPERS; i++) {
        if (pwmSteppers[i]) {
            delete pwmSteppers[i];
            pwmSteppers[i] = nullptr;
        }
        if (pulseCounters[i]) {
            delete pulseCounters[i];
            pulseCounters[i] = nullptr;
        }
        if (tmc2209Drivers[i]) {
            delete tmc2209Drivers[i];
            tmc2209Drivers[i] = nullptr;
        }
        // Note: uartPorts[i] points to HardwareSerial instances, don't delete
    }
}

// Helper method to validate stepper index
bool HighFrequencyStepper::validateStepperIndex(uint8_t index) const {
    return (index < MAX_STEPPERS && index < stepperCount && pwmSteppers[index] != nullptr);
}

// Add a stepper configuration
bool HighFrequencyStepper::addStepper(uint8_t index, const StepperConfig& config) {
    if (index >= MAX_STEPPERS) {
        Serial.println("ERROR: Stepper index out of range");
        return false;
    }
    
    // Store configuration
    configs[index] = config;
    
    // Create PulseCounter instance
    //pulseCounters[index] = new PulseCounter(config.pcntUnit, config.stepCountPin, config.dirPin);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    pinMode(config.encoderSettings.pinA, INPUT_PULLUP);
    pinMode(config.encoderSettings.pinB, INPUT_PULLUP);
    pinMode(config.encoderSettings.pinZ, INPUT_PULLUP);
    
    pulseCounters[index] = new ESP32Encoder();
    pulseCounters[index]->setFilter(1023);
    if (config.encoderSettings.attachMode == 1) {
        pulseCounters[index]->attachSingleEdge(config.encoderSettings.pinA, config.encoderSettings.pinB);
    } else if (config.encoderSettings.attachMode == 2) {
        pulseCounters[index]->attachHalfQuad(config.encoderSettings.pinA, config.encoderSettings.pinB);
    } else if (config.encoderSettings.attachMode == 4) {
        pulseCounters[index]->attachFullQuad(config.encoderSettings.pinA, config.encoderSettings.pinB);
    } else {
        Serial.println("ERROR: Invalid encoder attach mode");
        delete pulseCounters[index];
        pulseCounters[index] = nullptr;
        return false;
    }
    if (!pulseCounters[index]) {
        Serial.println("ERROR: Failed to create PulseCounter instance");
        delete pwmSteppers[index];
        pwmSteppers[index] = nullptr;
        return false;
    }
    configs[index].encoderToMicrostepRatio = float(config.stepsPerRev * config.microsteps) / float(config.encoderSettings.resolution * config.encoderSettings.attachMode);
        attachInterrupt(digitalPinToInterrupt(config.encoderSettings.pinZ), []() {
        // Handle Z pin interrupt (e.g., reset position)
    }, RISING);

    // Create PWMStepper instance
    pwmSteppers[index] = new PWMStepper(pulseCounters[index], configs[index].encoderToMicrostepRatio, config.stepPin, config.dirPin, config.enablePin, config.ledcChannel);
    pwmSteppers[index]->setStepperEnabledHigh(config.stepperEnabledHigh);
    if (!pwmSteppers[index]) {
        Serial.println("ERROR: Failed to create PWMStepper instance");
        return false;
    }
        
    // Configure UART port
    if (config.driverSettings.driverType == TMC2209_DRIVER) {
        if (config.driverSettings.uartConfig.uart == nullptr) {
            Serial.println("ERROR: UART not configured for TMC2209 driver");
            return false;
        }
        uartPorts[index] = configs[index].driverSettings.uartConfig.uart;    
        tmc2209Drivers[index] = new TMC2209Stepper(uartPorts[index], config.driverSettings.uartConfig.rSense, config.driverSettings.uartConfig.driverAddress);
    } else if (config.driverSettings.driverType == TMC2240_DRIVER) {
        TMC2240Stepper *tmc2240Driver = new TMC2240Stepper(
            config.driverSettings.spiConfig.pinCS, 
            config.driverSettings.spiConfig.pinMOSI, 
            config.driverSettings.spiConfig.pinMISO, 
            config.driverSettings.spiConfig.pinSCK);
        uartPorts[index] = nullptr; // No UART for TMC2240
        tmc2240Drivers[index] = tmc2240Driver;
    } else {
        uartPorts[index] = nullptr; // No TMC driver
        tmc2209Drivers[index] = nullptr;
        tmc2240Drivers[index] = nullptr;
    }
        
    // Update stepper count
    if (index >= stepperCount) {
        stepperCount = index + 1;
    }

    Serial.printf("Added stepper %d successfully\n", index);

    return true;
}

void configureTMC2209Driver(TMC2209Stepper* driver, uint16_t rmsCurrent, uint16_t microsteps);

// Initialize a specific stepper
bool HighFrequencyStepper::initializeStepper(uint8_t index) {
    if (!validateStepperIndex(index)) {
        Serial.println("ERROR: Invalid stepper index for initialization");
        return false;
    }

    // Initialize PulseCounter
    pulseCounters[index]->setCount(0);
    
    // Initialize PWMStepper
    double maxFreq = (configs[index].maxRPM / 60.0) * configs[index].microsteps * configs[index].stepsPerRev; // Convert RPM to Hz 
    double acceleration = rpmToFrequency(index, 60.0 * configs[index].rpsAcceleration); // Convert RPS^2 to steps/s^2
    pwmSteppers[index]->setMaxFreq(maxFreq);
    pwmSteppers[index]->setAcceleration(acceleration);
    pwmSteppers[index]->setInvertDirection(configs[index].invertDirection);
    pwmSteppers[index]->setStepperEnabledHigh(configs[index].stepperEnabledHigh);
    pwmSteppers[index]->setTargetFrequency(0);
    pwmSteppers[index]->setTargetPosition(0);
    pwmSteppers[index]->begin();
    
    
    // Initialize TMC2209
    if (tmc2209Drivers[index]) {
        tmc2209Drivers[index]->begin();
        configureTMC2209Driver(tmc2209Drivers[index], configs[index].rmsCurrent, configs[index].microsteps);
    } else if (tmc2240Drivers[index]) {
        TMC2240Stepper *tmc2240Driver = tmc2240Drivers[index];
        while (true) {
            enableStepper(index);
            tmc2240Driver->defaults();
            tmc2240Driver->rms_current(configs[index].rmsCurrent, 0.5); // Set motor RMS current
            tmc2240Drivers[index]->microsteps(configs[index].microsteps); // Set microsteps
            uint16_t checkMicrosteps = tmc2240Drivers[index]->microsteps();

            bool stealthChop = true;
            if (stealthChop) {
                tmc2240Driver->en_pwm_mode(true);
                tmc2240Driver->pwm_autoscale(true);
                tmc2240Driver->pwm_grad(1); // Slope of the PWM current
                tmc2240Driver->pwm_meas_sd_enable(true); // Enable automatic current scaling
                tmc2240Driver->pwm_freq(1); // PWM frequency setting
                tmc2240Driver->toff(3);              // Enable driver with good chopper frequency
                tmc2240Driver->TBL(2);              // Set blank time to 24 clocks
                tmc2240Driver->hstrt(4);             // HSTRT: 4 is good for high speed
                tmc2240Driver->hend(0);              // Fast current decay
                tmc2240Drivers[index]->TPWMTHRS(100);             // Threshold (number of clk cycles between microsteps) for switching to SpreadCycle
                //tmc2240Driver->automatic_tuning(true); // Enable automatic tuning
                uint32_t chopConf = tmc2240Driver->CHOPCONF();
                // Function to set bits in a uint32_t reference parameter
                auto setBits = [](uint32_t& value, uint8_t newBits, uint8_t startPos, uint8_t endPos) {
                    uint8_t numBits = endPos - startPos + 1;
                    uint32_t mask = ((1UL << numBits) - 1) << startPos;
                    value = (value & ~mask) | ((uint32_t(newBits) << startPos) & mask);
                };

                setBits(chopConf, 5, 0, 3); // toff = 5
                setBits(chopConf, 2, 15, 16); // tbl = 2
                setBits(chopConf, 0, 4, 6); // hstrt = 0
                setBits(chopConf, 0, 7, 10); // hend = 0
                setBits(chopConf, 0, 28, 28); // intpol = true
                tmc2240Driver->CHOPCONF(chopConf); 
            } else {
                tmc2240Driver->en_pwm_mode(false);
                tmc2240Drivers[index]->toff(5);
                tmc2240Drivers[index]->TBL(2);               // Set blank time to 24 clocks
                tmc2240Driver->hstrt(0);             // HSTRT: 4 is good for high speed
                tmc2240Driver->hend(0);              // Fast current decay
                //tmc2240Drivers[index]->en_pwm_mode(true); // Enable extremely quiet stepping
                // // For maximum speed with TMC2240:
                tmc2240Drivers[index]->THIGH(15);                  // High threshold voltage
                tmc2240Drivers[index]->TCOOLTHRS(20);                 // 0 = always use SpreadCycle
                tmc2240Drivers[index]->TPWMTHRS(50);             // Threshold (number of clk cycles between microsteps) for switching to SpreadCycle
                tmc2240Drivers[index]->vhighfs(true);
                tmc2240Drivers[index]->chm(0);                  // 0 - Use SpreadCycle chopper, 1 - Use classic chopper
                tmc2240Drivers[index]->hysteresis_start(0);      // HSTRT: 4 is good for high speed
                tmc2240Drivers[index]->hysteresis_end(0);        // Fast current decay
                tmc2240Drivers[index]->irun(31);                 // Disable coolStep
                tmc2240Drivers[index]->ihold(8);      // Hold current = ~50% of run current
                tmc2240Drivers[index]->iholddelay(5);           // Delay before
                
                tmc2240Drivers[index]->intpol(false);            // No interpolation delay            
                tmc2240Drivers[index]->pwm_autoscale(false);     // Consistent performance

            }
            
            tmc2240Drivers[index]->begin();

            if (tmc2240Drivers[index]->test_connection() == 0 && checkMicrosteps == configs[index].microsteps) {//configs[index].rmsCurrent) {
                Serial.printf("TMC2240 driver %d connected successfully\n", index);
                Serial.printf("Driver RMS Current: %d mA\n", tmc2240Drivers[index]->rms_current());
                Serial.printf("Driver Microsteps: %d\n", tmc2240Drivers[index]->microsteps());
                break;
            } else {
                Serial.printf("TMC2240 driver %d connection failed, retrying...\n", index);
                delay(1000);
            }
        }


    } else {
        Serial.println("WARNING: TMC2209 driver not initialized");
    }
    
    // Initialize status
    status[index].isInitialized = true;
    status[index].currentPosition = 0;
    status[index].targetPosition = 0;

    Serial.printf("Stepper %d initialized successfully\n", index);

    return true;
}

void configureTMC2209Driver(TMC2209Stepper* driver, uint16_t rmsCurrent, uint16_t microsteps) {
    if (!driver) return;
    driver->defaults();
    driver->rms_current(rmsCurrent);
    driver->microsteps(microsteps);
    driver->VACTUAL(0);

    driver->irun(31);                     // Run current = 100% of rms_current
    driver->hold_multiplier(10);          // Hold current = ~50% of run current
    driver->iholddelay(5);                // Delay before reducing to hold: 5 * 2^18 clocks
    driver->en_spreadCycle(true);         // TRUE for high RPM
    // // High-speed optimized settings:
    // driver->toff(5);                      // Enable with balanced chopper freq (~37 kHz)
    // driver->blank_time(24);               // Standard blank time

    // // SpreadCycle for high speed
    
    // driver->TCOOLTHRS(40);                 // 0 = always use SpreadCycle
    // driver->TPWMTHRS(50);                 // Threshold for switching to SpreadCycle
    // driver->SGTHRS(30);                    // Set stallGuard threshold

    // // Disable interpolation for maximum speed
    // driver->intpol(true);                // FALSE for high speed

    // // High-speed chopper tuning
    // driver->hysteresis_start(4);          // HSTRT: 4 is good for high speed
    // driver->hysteresis_end(0);            // HEND: 0 for fast decay
    // driver->semin(0);                     // Disable coolStep for max speed

    // // Current control - FIXED METHOD NAMES:
    // driver->pwm_autoscale(true);         // 
}

// Initialize all steppers
bool HighFrequencyStepper::initializeAll() {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!initializeStepper(i)) {
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}


// Set maximum RPM
bool HighFrequencyStepper::setMaxRPM(uint8_t index, double rpm) {
    if (!validateStepperIndex(index)) return false;

    configs[index].maxRPM = rpm;
    pwmSteppers[index]->setMaxFreq(getMaxFrequency(index)); // Convert RPM to Hz

    Serial.printf("Stepper %d max RPM set to %.2f RPM\n", index, rpm);
    return true;
}

bool HighFrequencyStepper::setAcceleration(uint8_t index, double rpsAcceleration) {
    if (!validateStepperIndex(index)) return false;
    
    configs[index].rpsAcceleration = rpsAcceleration;
    pwmSteppers[index]->setAcceleration(rpmToFrequency(index, 60.0 * rpsAcceleration)); // Convert RPS^2 to steps/s^2

    Serial.printf("Stepper %d acceleration set to %.2f RPS^2\n", index, rpsAcceleration);

    return true;
}

bool HighFrequencyStepper::setName(uint8_t index, const String& name) {
    if (!validateStepperIndex(index)) return false;
    configs[index].name = name;
    return true;
}

String HighFrequencyStepper::getName(uint8_t index) const {
    if (!validateStepperIndex(index)) return String("");
    return configs[index].name;
}

uint8_t HighFrequencyStepper::getStepPin(uint8_t index) const {
    if (!validateStepperIndex(index)) return 255;
    return configs[index].stepPin;
}

uint8_t HighFrequencyStepper::getDirPin(uint8_t index) const {
    if (!validateStepperIndex(index)) return 255;
    return configs[index].dirPin;
}

uint8_t HighFrequencyStepper::getEnablePin(uint8_t index) const {
    if (!validateStepperIndex(index)) return 255;
    return configs[index].enablePin;
}

uint8_t HighFrequencyStepper::getStepCountPin(uint8_t index) const {
    if (!validateStepperIndex(index)) return 255;
    return configs[index].encoderSettings.pinA;
}

uint16_t HighFrequencyStepper::getMicrostepsPerRevolution(uint8_t index) const {
    if (!validateStepperIndex(index)) return 0;
    return configs[index].stepsPerRev * configs[index].microsteps;
}

double HighFrequencyStepper::getMaxFrequency(uint8_t index) const {
    if (!validateStepperIndex(index)) return 0;
    return configs[index].maxRPM * configs[index].microsteps * configs[index].stepsPerRev / 60.0;
}

double HighFrequencyStepper::getMaxRPM(uint8_t index) const {
    if (!validateStepperIndex(index)) return 0;
    return configs[index].maxRPM;
}

double HighFrequencyStepper::getAcceleration(uint8_t index) const {
    if (!validateStepperIndex(index)) return 0;
    return configs[index].rpsAcceleration;
}

bool HighFrequencyStepper::getInvertDirection(uint8_t index) const {
    if (!validateStepperIndex(index)) return false;
    return configs[index].invertDirection;
}



// Move to absolute position
bool HighFrequencyStepper::moveToPosition(uint8_t index, int32_t position, double frequency, bool blocking, bool correctPosition) {
    if (!validateStepperIndex(index)) return false;

    if (frequency == 0 || frequency > getMaxFrequency(index)) frequency = getMaxFrequency(index);

    int32_t currentPos = getPosition(index);
    int32_t steps = position - currentPos;
    if (steps == 0) return true; // Already at position    

    // Update target position
    status[index].targetPosition = position;
    status[index].isMoving = true;
    status[index].targetFrequency = frequency;
    
    pwmSteppers[index]->moveToPosition(position, frequency);

    int maxTimeMs = 10000 + (abs(steps) / (frequency)) * 1000 * 5; // Estimated max time with buffer
    uint32_t startTime = millis();
    int prevError = abs(getPosition(index) - position);
    if (blocking) {
        int loopCounter = 0;
        int currentError = prevError;
        while (currentError > configs[index].encoderToMicrostepRatio && pwmSteppers[index]->isMovingToPosition()) {
            vTaskDelay(10); // Yield to other tasks
            currentError = abs(getPosition(index) - position);
            // Timeout check            
            if (loopCounter % 100 == 0) {
                //Serial.printf("Current TSTEP: %d\n", tmc2240Drivers[index]->TSTEP());
                //pwmSteppers[index]->update();
                //Serial.printf("Stepper %d moving... Current: %d, Target: %d\n", index, getPosition(index), position);
                //pwmSteppers[index]->printStatus();
            }
            if (loopCounter % 100 == 0) {
                if (prevError <= currentError && (millis() - startTime) > maxTimeMs) {
                    Serial.printf("ERROR: moveToPosition timeout for stepper %d\n", index);
                    return false;
                }
                prevError = currentError;
            }
            loopCounter++;
        }
        status[index].isMoving = false;
    }
    if (correctPosition) {
        return moveToPosition(index, position, configs[index].microsteps, blocking, false);
    } else {
        return true;
    }
}


// Move relative steps
bool HighFrequencyStepper::moveRelative(uint8_t index, int32_t steps, double frequency, bool blocking) {
    return moveToPosition(index, getPosition(index) + steps, frequency, blocking);
}

bool HighFrequencyStepper::moveToAngle(uint8_t index, double angleDegrees, double frequency, bool blocking) {
    if (!validateStepperIndex(index)) return false;
    double stepsPerRev = getMicrostepsPerRevolution(index);
    int32_t targetPosition = (int32_t)((angleDegrees / 360.0) * stepsPerRev);
    return moveToPosition(index, targetPosition, frequency, blocking);
}

bool HighFrequencyStepper::moveToAngleRelative(uint8_t index, double angleDegrees, double frequency, bool blocking) {
    if (!validateStepperIndex(index)) return false;
    double stepsPerRev = getMicrostepsPerRevolution(index);
    int32_t steps = (int32_t)((angleDegrees / 360.0) * stepsPerRev);
    return moveRelative(index, steps, frequency, blocking);
}

bool HighFrequencyStepper::accelerateToFrequency(uint8_t index, double frequency, bool direction, bool waitForCompletion) { 
    if (!validateStepperIndex(index)) return false;

    if (frequency > getMaxFrequency(index)) frequency = getMaxFrequency(index);

    status[index].isMoving = true;
    status[index].currentFrequency = frequency;
    
    if (configs[index].invertDirection) {
        direction = !direction;
    }
    pwmSteppers[index]->setDirection(direction);
    //tmc2209Drivers[index]->VACTUAL((1.0/0.72)*frequency * (direction ? -1 : 1));

    pwmSteppers[index]->accelerateToFrequency(frequency);
    
    if (waitForCompletion) {
        // Wait until target frequency is reached
        int loopCounter = 0;
        int prevError = abs(pwmSteppers[index]->getFrequency() - frequency);
        int currentError = prevError;
        int maxTimeMs = 10000;
        uint32_t startTime = millis();        
        while (currentError > configs[index].encoderToMicrostepRatio) {
            vTaskDelay(10); // Yield to other tasks
            currentError = abs(pwmSteppers[index]->getFrequency() - frequency);
            // Timeout check            
            if (loopCounter % 100 == 0) {
                //Serial.printf("Current TSTEP: %d, Temp: %f\n", tmc2240Drivers[index]->TSTEP(), tmc2240Drivers[index]->get_chip_temperature());
                //tmc2209Drivers[index]->TSTEP();
                //Serial.printf("Stepper %d moving... Current: %d, Target: %d\n", index, getPosition(index), position);
                //pwmSteppers[index]->printStatus();
            }
            if (loopCounter % 100 == 0) {
                if (prevError <= currentError && (millis() - startTime) > maxTimeMs) {
                    Serial.printf("ERROR: accelerateToFrequency timeout for stepper %d\n", index);
                    return false;
                }
                prevError = currentError;
            }
            loopCounter++;
        }
    }
    
    return true;
}

bool HighFrequencyStepper::accelerateToAngularSpeed(uint8_t index, double angularSpeed, bool direction, bool waitForCompletion) {
    if (!validateStepperIndex(index)) return false;

    double stepsPerRev = getMicrostepsPerRevolution(index);
    double frequency = (angularSpeed / 360.0) * stepsPerRev; // Convert angular speed (deg/s) to frequency (Hz)

    return accelerateToFrequency(index, frequency, direction, waitForCompletion);
}

// Start continuous movement
bool HighFrequencyStepper::moveAtFrequency(uint8_t index, double frequency, bool direction) {
    if (!validateStepperIndex(index)) return false;

    if (frequency > getMaxFrequency(index)) frequency = getMaxFrequency(index);

    status[index].isMoving = true;
    status[index].currentFrequency = frequency;
    
    if (configs[index].invertDirection) {
        direction = !direction;
    }
    pwmSteppers[index]->setDirection(direction);
    pwmSteppers[index]->moveAtFrequency(frequency);

    Serial.printf("Stepper %d started continuous movement at %.1f Hz\n", index, frequency);
    
    return true;
}

bool HighFrequencyStepper::moveAtAngularSpeed(uint8_t index, double angularSpeed, bool direction) {
    if (!validateStepperIndex(index)) return false;

    double stepsPerRev = getMicrostepsPerRevolution(index);
    double frequency = (angularSpeed / 360.0) * stepsPerRev; // Convert angular speed (deg/s) to frequency (Hz)

    return moveAtFrequency(index, frequency, direction);
}

// Stop specific stepper
bool HighFrequencyStepper::stop(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    pwmSteppers[index]->stopPWM();
    status[index].isMoving = false;
    status[index].currentFrequency = 0;
    
    updatePosition(index);
    
    Serial.print("Stepper ");
    Serial.print(index);
    Serial.println(" stopped");
    
    return true;
}

// Stop all steppers
bool HighFrequencyStepper::stopAll() {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!stop(i)) {
                allSuccess = false;
            }
        }
    }
    
    return allSuccess;
}

// Emergency stop - immediate stop of all steppers
bool HighFrequencyStepper::emergencyStop() {
    Serial.println("EMERGENCY STOP ACTIVATED!");
    
    // Disable all steppers immediately
    disableAll();
    stopAll();
    
    return true;
}

// Get current position
int32_t HighFrequencyStepper::getPosition(uint8_t index) {
    if (!validateStepperIndex(index)) return 0;
    
    // Get position from pulse counter
    int32_t pulseCount = (int) (pulseCounters[index]->getCount() * configs[index].encoderToMicrostepRatio);
    
    // Update internal position tracking
    status[index].currentPosition = pulseCount;
    
    return pulseCount;
}

double HighFrequencyStepper::getAngle(uint8_t index) {
    if (!validateStepperIndex(index)) return 0.0;
    
    int32_t position = getPosition(index);
    double stepsPerRev = getMicrostepsPerRevolution(index);
    double angleDegrees = (double(position) / stepsPerRev) * 360.0;
    
    return angleDegrees;
}

// Update position tracking
void HighFrequencyStepper::updatePosition(uint8_t index) {
    if (!validateStepperIndex(index)) return;
    
    // Update current position from pulse counter
    status[index].currentPosition = getPosition(index);
    
    // Check if movement is complete
    if (pwmSteppers[index]->getFrequency() == 0) {
        status[index].isMoving = false;
        status[index].currentFrequency = 0;
    }
}

// Enable stepper
bool HighFrequencyStepper::enableStepper(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    pwmSteppers[index]->enable();
    status[index].isEnabled = true;
    
    return true;
}

// Disable stepper
bool HighFrequencyStepper::disableStepper(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    pwmSteppers[index]->disable();
    status[index].isEnabled = false;
    status[index].isMoving = false;
    
    return true;
}

// Enable all steppers
bool HighFrequencyStepper::enableAll() {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!enableStepper(i)) {
                allSuccess = false;
            }
        }
    }
    
    globalEnable = allSuccess;
    return allSuccess;
}

// Disable all steppers
bool HighFrequencyStepper::disableAll() {
    bool allSuccess = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!disableStepper(i)) {
                allSuccess = false;
            }
        }
    }
    
    globalEnable = false;
    return allSuccess;
}

// Check if stepper is enabled
bool HighFrequencyStepper::isEnabled(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    return status[index].isEnabled;
}

// Set position (zero or calibrate)
bool HighFrequencyStepper::setPosition(uint8_t index, int32_t position) {
    if (!validateStepperIndex(index)) return false;
    
    pulseCounters[index]->setCount(position);
    status[index].currentPosition = position;
    status[index].targetPosition = position;
    return true;
}

// Get stepper status
StepperStatus HighFrequencyStepper::getStatus(uint8_t index) {
    if (!validateStepperIndex(index)) {
        return StepperStatus(); // Return default status
    }
    
    // Update position before returning status
    updatePosition(index);
    
    // Update additional status information
    status[index].stallGuard = isStallDetected(index);
    
    return status[index];
}

// Print status for one stepper
void HighFrequencyStepper::printStatus(uint8_t index) {
    if (!validateStepperIndex(index)) {
        Serial.println("Invalid stepper index");
        return;
    }

    pwmSteppers[index]->printStatus();
}

// Print status for all steppers
void HighFrequencyStepper::printAllStatus() {
    Serial.println("\n=== HIGH FREQUENCY STEPPER STATUS ===");
    Serial.printf("Stepper Count: %d\n", stepperCount);
    Serial.printf("Global Enable: %s\n", globalEnable ? "ON" : "OFF");
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            printStatus(i);
        }
    }
}

bool HighFrequencyStepper::isInLEDCMode(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    return pwmSteppers[index]->getMode() == MODE_LEDC;
}

// Check if stall is detected
bool HighFrequencyStepper::isStallDetected(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    
    // Check TMC StallGuard status
    if (!tmc2209Drivers[index]) return false;
    return false;
    uint32_t drv_status = tmc2209Drivers[index]->DRV_STATUS();
    return (drv_status & 0x1000000) != 0; // StallGuard flag
}

// Self test for one stepper
bool HighFrequencyStepper::selfTest(uint8_t index) {
    if (!validateStepperIndex(index)) {
        Serial.println("Self-test failed: Invalid stepper index");
        return false;
    }

    Serial.printf("Self-test for stepper %d...\n", index);

    // Test 1: Communication with TMC driver
    if (tmc2209Drivers[index]) {
        int previousMicrosteps = tmc2209Drivers[index]->microsteps();
        int expectMicrosteps = 32; // Any value except 256 (default return value if no communication)
        tmc2209Drivers[index]->microsteps(expectMicrosteps);
        int actualMicrosteps = tmc2209Drivers[index]->microsteps(); // Read back
        // Restore previous microsteps
        Serial.printf("TMC Driver Version: 0x%08X\n", tmc2209Drivers[index]->version());
        tmc2209Drivers[index]->microsteps(previousMicrosteps);
        if (expectMicrosteps != actualMicrosteps) {
            Serial.println("FAIL: TMC communication error");
            return false;
        } else {
            Serial.printf("TMC Driver Version: 0x%08X\n", tmc2209Drivers[index]->version());
        }
    } else if (tmc2240Drivers[index]) {
        enableStepper(index);
        //tmc2240Drivers[index]->en_pwm_mode(1);
        Serial.printf("Test connection: %d\n", tmc2240Drivers[index]->test_connection());
        Serial.printf("TMC2240 Driver Version: 0x enabled %s\n", tmc2240Drivers[index]->isEnabled() ? "YES" : "NO");
        // Serial.printf("TMC2240 Driver PINS:\n   DRV_ENN: %s\n   DIR: %s\n   STEP: %s\n  UART ENN: %s\n",
        //               tmc2240Drivers[index]->drv_enn() == configs[index].enablePin ? "HIGH" : "LOW",
        //               tmc2240Drivers[index]->dir() == configs[index].dirPin ? "HIGH" : "LOW",
        //               tmc2240Drivers[index]->step() == configs[index].stepPin ? "HIGH" : "LOW",
        //               tmc2240Drivers[index]->uart_en() == configs[index].driverSettings.spiConfig.pinCS ? "HIGH" : "LOW");
        // disableStepper(index); // Disable after test
        // Serial.printf("TMC2240 Driver DRV_ENN: %s\n", tmc2240Drivers[index]->drv_enn() ? "YES" : "NO");
    } else {
        Serial.println("WARN: No TMC driver instance");
    }
    
    // Test 2: Enable/disable functionality
    enableStepper(index);
    if (!isEnabled(index)) {
        Serial.println("FAIL: Enable function");
        return false;
    }
    
    // Test 3: Small movement test
    int32_t startPos = getPosition(index);
    moveToPosition(index, startPos + 1000, getMaxFrequency(index), true); // Move 100 steps at max frequency
    int32_t endPos = getPosition(index);

    if (abs(endPos - startPos - 1000) > configs[index].encoderToMicrostepRatio) { // Allow configurable step tolerance
        Serial.printf("FAIL: Movement accuracy (Expected: %d, Actual: %d)\n", startPos + 1000, endPos);
        return false;
    }
    
    // Return to start position
    moveToPosition(index, startPos, getMaxFrequency(index), true); // Move back to start position

    Serial.println("PASS: Self-test completed successfully");
    return true;
}

// Self test for all steppers
bool HighFrequencyStepper::selfTestAll() {
    bool allPass = true;
    Serial.println("Starting self-test for all steppers...");
    for (uint8_t i = 0; i < stepperCount; i++) {
        if (pwmSteppers[i] != nullptr) {
            if (!selfTest(i)) {
                allPass = false;
            }
        }
    }
    Serial.printf("%s\n", allPass ? "All steppers passed self-test!" : "Some steppers failed self-test!");
    return allPass;
}

// Additional methods for completeness...
bool HighFrequencyStepper::isMoving(uint8_t index) {
    if (!validateStepperIndex(index)) return false;
    updatePosition(index);
    return pwmSteppers[index]->isMoving();
}

int32_t HighFrequencyStepper::getTargetPosition(uint8_t index) {
    if (!validateStepperIndex(index)) return 0;
    return pwmSteppers[index]->getTargetPosition();
}

double HighFrequencyStepper::getCurrentFrequency(uint8_t index) {
    if (!validateStepperIndex(index)) return 0.0;
    return pwmSteppers[index]->getFrequency();
}

double HighFrequencyStepper::toAngle(uint8_t index, int32_t position) {
    if (!validateStepperIndex(index)) return 0.0;
    double stepsPerRev = getMicrostepsPerRevolution(index);
    double angleDegrees = (double(position) / stepsPerRev) * 360.0;
    return angleDegrees;
}

int32_t HighFrequencyStepper::toPosition(uint8_t index, double angleDegrees) {
    if (!validateStepperIndex(index)) return 0;
    double stepsPerRev = getMicrostepsPerRevolution(index);
    int32_t position = (int32_t)((angleDegrees / 360.0) * stepsPerRev);
    return position;
}

bool HighFrequencyStepper::isAtPosition(uint8_t index, int32_t tolerance) {
    if (!validateStepperIndex(index)) return false;
    updatePosition(index);
    return abs(status[index].currentPosition - status[index].targetPosition) <= tolerance;
}

StepperConfig HighFrequencyStepper::getConfig(uint8_t index) const {
    if (index < MAX_STEPPERS) {
        return configs[index];
    }
    return StepperConfig(); // Return default config for invalid index
}

/**
 * TMC2209 Specific Settings
 */
bool HighFrequencyStepper::setMicrosteps(uint8_t index, uint16_t microsteps) {
    if (!validateStepperIndex(index)) return false;
    
    // Validate microsteps value
    if (microsteps != 1 && microsteps != 2 && microsteps != 4 && microsteps != 8 && 
        microsteps != 16 && microsteps != 32 && microsteps != 64 && microsteps != 128 && microsteps != 256) {
        Serial.println("ERROR: Invalid microsteps value");
        return false;
    }
    
    configs[index].microsteps = microsteps;

    // Adjust max frequency accordingly
    double maxFreq = (configs[index].maxRPM / 60.0) * configs[index].microsteps * configs[index].stepsPerRev; // Convert RPM to Hz 
    double acceleration = rpmToFrequency(index, 60.0 * configs[index].rpsAcceleration); // Convert RPS^2 to steps/s^2
    configs[index].encoderToMicrostepRatio = float(configs[index].stepsPerRev * configs[index].microsteps) / float(configs[index].encoderSettings.resolution * configs[index].encoderSettings.attachMode);
    pwmSteppers[index]->setMaxFreq(maxFreq);
    pwmSteppers[index]->setAcceleration(acceleration);
    pwmSteppers[index]->setEncoderScale(configs[index].encoderToMicrostepRatio);

    Serial.printf("Stepper %d microsteps set to %d, acceleration set to %f, max frequency set to %f, encoder to microstep ratio set to %f\n", index, microsteps, acceleration, maxFreq, configs[index].encoderToMicrostepRatio);
    if (!tmc2209Drivers[index]) {
        return false;
    } else {
        tmc2209Drivers[index]->microsteps(microsteps);
        return true;
    }
}

bool HighFrequencyStepper::setRMSCurrent(uint8_t index, uint16_t currentMA) {
    if (!validateStepperIndex(index)) return false;
    
    configs[index].rmsCurrent = currentMA;
    if (!tmc2209Drivers[index]) return false;
    tmc2209Drivers[index]->rms_current(currentMA);

    Serial.printf("Stepper %d RMS current set to %d mA\n", index, currentMA);

    return true;
}

bool HighFrequencyStepper::setSpreadCycle(uint8_t index, bool enable) {
    if (!validateStepperIndex(index)) return false;
    if (!tmc2209Drivers[index]) return false;
    tmc2209Drivers[index]->en_spreadCycle(enable);
    return true;
}

bool HighFrequencyStepper::isSpreadCycleEnabled(uint8_t index) const {
    if (!validateStepperIndex(index)) return false;
    if (!tmc2209Drivers[index]) return false;
    return tmc2209Drivers[index]->en_spreadCycle();
}

bool HighFrequencyStepper::setHybridThreshold(uint8_t index, uint8_t threshold) {
    if (!validateStepperIndex(index)) return false;
    if (!tmc2209Drivers[index]) return false;
    tmc2209Drivers[index]->TCOOLTHRS(threshold);
    return true;
}

bool HighFrequencyStepper::setCoolStep(uint8_t index, uint16_t value) {
    if (!validateStepperIndex(index)) return false;
    if (!tmc2209Drivers[index]) return false;
    tmc2209Drivers[index]->semin(value); // TODO: Validate the method and value
    return true;
}

// stallguard 255 is most sensitive value, 0 is least sensitive
bool HighFrequencyStepper::setStallGuardThreshold(uint8_t index, uint16_t threshold) {
    if (!validateStepperIndex(index)) return false;
    if (!tmc2209Drivers[index]) return false;
    tmc2209Drivers[index]->SGTHRS(threshold);
    return true;
}

uint16_t HighFrequencyStepper::getMicrosteps(uint8_t index) const {
    if (!validateStepperIndex(index)) return 0;
    return configs[index].microsteps;
}

uint16_t HighFrequencyStepper::getRMSCurrent(uint8_t index) const {
    if (!validateStepperIndex(index)) return 0;
    return configs[index].rmsCurrent;
}

float HighFrequencyStepper::getRSense(uint8_t index) const {
    if (!validateStepperIndex(index)) return 0;
    return configs[index].driverSettings.uartConfig.rSense;
}

HardwareSerial* HighFrequencyStepper::getUART(uint8_t index) const {
    if (!validateStepperIndex(index)) return nullptr;
    return uartPorts[index];
}

uint8_t HighFrequencyStepper::getDriverAddress(uint8_t index) const {
    if (!validateStepperIndex(index)) return 0;
    return configs[index].driverSettings.uartConfig.driverAddress;
}

