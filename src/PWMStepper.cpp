#include "PWMStepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <vector>
#include <algorithm>



// Global instance pointer for timer callback
PWMStepper* currentStepperInstance = nullptr;

// Define frequency threshold (512 Hz)
const double PWMStepper::FREQUENCY_THRESHOLD = 512.0;
const uint64_t PWM_STEPPER_TIMER_DELAY = 500; // 0.5ms
esp_timer_handle_t pwm_stepper_timer = nullptr;
static std::vector<PWMStepper*> pwmStepperInstances;

static void ARDUINO_ISR_ATTR onPWMStepperTimer(void *arg)
{
    for (auto stepper : pwmStepperInstances) {
        stepper->update();
    }
}

void initTimers() {
    if (pwm_stepper_timer != nullptr) {
        return; // Already initialized
    }
    const esp_timer_create_args_t periodic_pwm_stepper_timer_args = {
        .callback = &onPWMStepperTimer, // link the call back
        .arg = nullptr,                 // no argument
        .name = "pwm-stepper-timer"      
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_pwm_stepper_timer_args, &pwm_stepper_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_stepper_timer, PWM_STEPPER_TIMER_DELAY)); // Start PWM stepper timer with a delay of 10ms
}

// Constructor
PWMStepper::PWMStepper(ESP32Encoder* encoder, float encoderScale, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t ledcChannel) {
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->enablePin = enablePin;
    this->ledcChannel = ledcChannel;
    this->ledcResolution = 8;    // 8-bit resolution (0-255)
    this->direction = true;      // Forward by default
    this->currentFreq = 0;
    this->targetFreq = 0;
    this->targetPosition = 0;
    this->acceleration = 0;
    this->encoder = encoder;
    this->encoderScale = encoderScale;
    this->targetPosition = 0;
    this->state = STEPPER_OFF;
    this->mode = MODE_LEDC;
}

// Destructor - ensure proper cleanup
PWMStepper::~PWMStepper() {
    stopPWM();
    if (currentStepperInstance == this) {
        currentStepperInstance = nullptr;
    }
    pwmStepperInstances.erase(std::remove(pwmStepperInstances.begin(), pwmStepperInstances.end(), this), pwmStepperInstances.end());
}

// Initialize the stepper
void PWMStepper::begin() {
    // Configure GPIO pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    
    // Set initial states
    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, direction ? HIGH : LOW);
    disable();
    
    // Setup LEDC channel
    ledcSetup(ledcChannel, ledcFrequency, ledcResolution);
    ledcAttachPin(stepPin, ledcChannel);

    pwmStepperInstances.push_back(this);
    initTimers();

    positionHistory.reserve(MAX_POSITION_HISTORY);
    updateTimes.reserve(MAX_POSITION_HISTORY);
    for (size_t i = 0; i < MAX_POSITION_HISTORY; ++i) {
        positionHistory[i] = encoder->getCount();
        updateTimes[i] = micros();
    }
    state = STEPPER_IDLE;

    Serial.println("PWMStepper initialized successfully!");
    Serial.print("Step Pin: "); Serial.println(stepPin);
    Serial.print("Dir Pin: "); Serial.println(dirPin);
    Serial.print("Enable Pin: "); Serial.println(enablePin);
    Serial.print("LEDC Channel: "); Serial.println(ledcChannel);
    Serial.print("Frequency Threshold: "); Serial.print(FREQUENCY_THRESHOLD); Serial.println(" Hz");
    Serial.printf("PWM Stepper instances: %d\n", (int)pwmStepperInstances.size());
}

void ARDUINO_ISR_ATTR PWMStepper::update() {
    float currentPosition = encoderScale * encoder->getCount();
    uint64_t currentTime = micros();
    positionHistory[updateNumber % MAX_POSITION_HISTORY] = currentPosition;
    updateTimes[updateNumber % MAX_POSITION_HISTORY] = currentTime;

    if (mode == MODE_TIMER && currentFreq > 0) {
        uint64_t microsSinceLastSpeedChange = currentTime - lastSpeedChangeMicros;
        double expectedMicrostepPeriod = 1000000.0 / abs(currentFreq * 2); // Period in microseconds
        uint64_t expectedNumberOfSteps = (uint64_t) (double(microsSinceLastSpeedChange) / expectedMicrostepPeriod);
        if (expectedNumberOfSteps > stepsSinceLastSpeedChange) {
            int stepPinState = digitalRead(stepPin); // Read current state
            pinMode(stepPin, OUTPUT);
            digitalWrite(stepPin, !stepPinState); // Start new period with toggled state   
            stepsSinceLastSpeedChange++;     
        }
    }
    if (state == STEPPER_MOVE_TO_POSITION) {
        bool actualDirection = invertDirection ? !direction : direction;
        if (actualDirection && currentPosition >= targetPosition || !actualDirection && currentPosition <= targetPosition) {
            stopPWM();
            decelerating = false;
            state = STEPPER_IDLE;
            currentFreq = 0;
        }
    }
    int freqUpdateInterval = 100;
    updateNumber++;
    if (updateNumber % freqUpdateInterval != 0) {
        return;
    }
    int64_t error = targetPosition - currentPosition;
    double unitFrequencyChange = acceleration * (freqUpdateInterval * PWM_STEPPER_TIMER_DELAY / 1000000.0);

    // int64_t previousPosition = positionHistory[(updateNumber - 1 + MAX_POSITION_HISTORY) % MAX_POSITION_HISTORY];
    // uint64_t previousTime = updateTimes[(updateNumber - 1 + MAX_POSITION_HISTORY) % MAX_POSITION_HISTORY];
    // recentFreq = (currentPosition - previousPosition) * 1000000.0 / (currentTime - previousTime + 1); // +1 to avoid div by zero
    switch (state) {
        case STEPPER_OFF:
            return;
        case STEPPER_IDLE:
            return; // Do nothing if not running
        case STEPPER_MOVE_TO_POSITION:

            // if (error != 0) {
            //     bool dir = error > 0;
            //     if (invertDirection) {
            //         dir = !dir;
            //     }
            //     setDirection(dir);
            // }
            if (abs(error) >= int(encoderScale) + 1) {                
                if (acceleration != 0) {
                    double decelerationUpdates = (currentFreq / unitFrequencyChange);
                    double estimatedDecelerationTime = decelerationUpdates * (freqUpdateInterval * PWM_STEPPER_TIMER_DELAY / 1000000.0);
                    decelerationDistance = acceleration * estimatedDecelerationTime * estimatedDecelerationTime / 2.0; // (currentFreq * currentFreq) / (2.0 * acceleration);
                    if (abs(error) < decelerationDistance * 1.5) { // Add slight buffer to deceleration
                        targetFreq = 500; // Decelerate
                        //decelerating = true;
                        //currentFreq = targetFreq;
                    } else {
                        targetFreq = maxFreq; // Accelerate
                    }
                    double newFreq = currentFreq;
                
                    //if (!decelerating) {
                    if (newFreq < targetFreq) {
                        newFreq += unitFrequencyChange; // Convert delay to seconds
                        // if (newFreq > targetFreq) {
                        //     newFreq = targetFreq;
                        // }
                        // //Serial.printf("Accelerating to %.2f Hz, deceleration distance: %.2f, error: %d\n", newFreq, decelerationDistance, error);                                                                            
                    } else {
                        newFreq -= unitFrequencyChange; // Convert delay to seconds
                        // if (newFreq < targetFreq) {
                        //     newFreq = targetFreq;
                        // }
                    }
                    startPWM(newFreq);
                }
            }  
            break; // Handle below
        case STEPPER_MOVE_WITH_FREQUENCY:
            if (acceleration != 0) {
                double newFreq = currentFreq;
                if (newFreq < targetFreq) {
                    newFreq += unitFrequencyChange; // Convert delay to seconds
                    if (newFreq > targetFreq) {
                        newFreq = targetFreq;
                    }
                    startPWM(newFreq);
                } else if (newFreq > targetFreq) {
                    newFreq -= unitFrequencyChange; // Convert delay to seconds
                    if (newFreq < targetFreq) {
                        newFreq = targetFreq;
                    }
                    startPWM(newFreq);
                }
            } else {
                onSpeedChange(targetFreq, direction);
            }
            break; // Handle below
    }
}

// Set direction
void PWMStepper::setDirection(bool dir) {
    digitalWrite(dirPin, dir ? HIGH : LOW);
    onSpeedChange(currentFreq, dir);
}

// Enable the stepper driver
void PWMStepper::enable() {
    if (stepperEnabledHigh) {
        digitalWrite(enablePin, HIGH);  // Active HIGH
    } else {
        digitalWrite(enablePin, LOW);  // Active LOW
    }
    state = STEPPER_IDLE;
}

// Disable the stepper driver
void PWMStepper::disable() {
    if (stepperEnabledHigh) {
        digitalWrite(enablePin, LOW);  // Active HIGH
    } else {
        digitalWrite(enablePin, HIGH);  // Active LOW
    }
    stopPWM();  // Also stop any running PWM
    state = STEPPER_OFF;
}

void PWMStepper::accelerateToFrequency(double frequency) {
    targetFreq = frequency;
    state = STEPPER_MOVE_WITH_FREQUENCY;
    targetPosition = 0; // Clear target position
}

void PWMStepper::moveAtFrequency(double frequency) {
    targetFreq = frequency;
    state = STEPPER_MOVE_WITH_FREQUENCY;
    targetPosition = 0; // Clear target position
    onSpeedChange(frequency, direction);
    startPWM(frequency);
    Serial.printf("Moving at frequency: %.2f Hz in %s direction with %s mode\n", frequency, direction ? "forward" : "reverse", mode == MODE_LEDC ? "LEDC" : "Timer");
}

void PWMStepper::moveToPosition(int64_t position, double frequency) {
    int32_t currentPos = getPosition();
    int32_t steps = position - currentPos;
    if (steps == 0) return; // Already at position    

    bool direction = steps > 0;
    if (invertDirection) {
        direction = !direction;
    }

    decelerating = false;
    setTargetPosition(position);
    setTargetFrequency(frequency);
    setDirection(direction);
    state = STEPPER_MOVE_TO_POSITION;
}

// Start PWM at specified frequency (steps per second) - Dual Mode
void PWMStepper::startPWM(double frequency) {
    onSpeedChange(frequency, direction);
    
    // Choose mode based on frequency threshold
    if (frequency >= FREQUENCY_THRESHOLD) {
        // High frequency - use LEDC mode
        mode = MODE_LEDC;
        stopTimerMode(); // Stop Timer mode if running
        startLEDCMode(frequency);
    } else {
        // Low frequency - use Timer mode
        mode = MODE_TIMER;
        stopLEDCMode(); // Stop LEDC mode if running
        startTimerMode(frequency);

    }
}

// Stop PWM - Dual Mode
void PWMStepper::stopPWM() {
    setTargetFrequency(0);
    if (mode == MODE_LEDC) {
        stopLEDCMode();
    } else {
        stopTimerMode();
    }
    
    digitalWrite(stepPin, LOW); // Ensure pin is LOW
    if (state != STEPPER_OFF) {
        state = STEPPER_IDLE;
    }
    onSpeedChange(0, direction);
    
}

void PWMStepper::setTargetFrequency(double frequency) {
    targetFreq = frequency;
    state = STEPPER_MOVE_WITH_FREQUENCY;
}

// Set frequency while running
void PWMStepper::setFrequency(double frequency) {
    if (state == STEPPER_MOVE_WITH_FREQUENCY) {
        startPWM(frequency);  // Restart with new frequency
    } else {
        onSpeedChange(frequency, direction);
    }
}

void PWMStepper::setAcceleration(double accel) {
    acceleration = accel;
}

void PWMStepper::setTargetPosition(int64_t position) {
    targetPosition = position;
    state = STEPPER_MOVE_TO_POSITION;
}

// Get current status
bool PWMStepper::isEnabled() const {
    if (digitalRead(enablePin) == LOW) {
        return !stepperEnabledHigh; // Active LOW
    } else {
        return stepperEnabledHigh; // Active HIGH
    }
}

bool PWMStepper::getDirection() const {
    return direction;
}

double PWMStepper::getFrequency() const {
    return currentFreq;
}

StepperMode PWMStepper::getMode() const {
    return mode;
}

void PWMStepper::printStatus() const {
    Serial.printf("[%d] ", updateNumber);
    Serial.printf("Mode: %s, ", mode == MODE_LEDC ? "LEDC" : "Timer");
    Serial.printf("Enabled: %s, ", isEnabled() ? "Yes" : "No");
    Serial.printf("Direction: %s, ", direction ? "Forward" : "Reverse");
    Serial.printf("Current Freq: %.2f Hz, ", currentFreq);
    Serial.printf("Target Freq: %.2f Hz, ", targetFreq);
    Serial.printf("Acceleration: %.2f steps/s², ", acceleration);
    Serial.printf("Dec Distance: %.2f steps, ", decelerationDistance);
    Serial.printf("Current Pos: %ld, ", (long) getPosition());
    Serial.printf("Target Pos: %ld, ", (long) targetPosition);
    Serial.printf("State: ");
    switch (state) {
        case STEPPER_OFF:
            Serial.println("OFF");
            break;
        case STEPPER_IDLE:
            Serial.println("IDLE");
            break;
        case STEPPER_MOVE_TO_POSITION:
            Serial.println("MOVING TO POSITION");
            break;
        case STEPPER_MOVE_WITH_FREQUENCY:
            Serial.println("MOVING WITH FREQUENCY");
            break;
    }
}

// LEDC Mode Methods
void PWMStepper::startLEDCMode(double frequency) {
    // Reconfigure LEDC with new frequency and resolution
    ledcSetup(ledcChannel, frequency, 4); // 4-bit resolution (0-15) - we use half the range for 50% duty cycle anyway
    ledcAttachPin(stepPin, ledcChannel);
    
    // Set 50% duty cycle for square wave
    uint32_t dutyCycle = 8;  // 50% duty cycle
    ledcWrite(ledcChannel, dutyCycle);
}

void PWMStepper::stopLEDCMode() {
    ledcWrite(ledcChannel, 0);  // 0% duty cycle
    ledcDetachPin(stepPin);
}

// Timer Mode Methods
void PWMStepper::startTimerMode(double frequency) {
    mode = MODE_TIMER;
    pinMode(stepPin, OUTPUT);
}

void PWMStepper::stopTimerMode() {
    digitalWrite(stepPin, LOW); // Ensure pin is LOW
}

void PWMStepper::onSpeedChange(double newSpeed, bool dir) {
    if (newSpeed != currentFreq || dir != direction) {
        lastSpeedChangeMicros = micros();
        stepsSinceLastSpeedChange = 0;
    }
    currentFreq = newSpeed;
    direction = dir;
}

// Move a specific number of steps
void PWMStepper::step(uint32_t steps, double frequency, bool dir) {
    if (steps == 0) return;
    
    setDirection(dir);
    enable();
    
    if (frequency >= FREQUENCY_THRESHOLD) {
        // Use LEDC mode for high frequencies
        startPWM(frequency);
        
        // Calculate duration in milliseconds
        uint32_t duration = (steps * 1000) / frequency;
        
        Serial.print("LEDC stepping ");
        Serial.print(steps);
        Serial.print(" steps at ");
        Serial.print(frequency);
        Serial.print(" Hz for ");
        Serial.print(duration);
        Serial.println(" ms");
        
        delay(duration);
        stopPWM();
    } else {
        // Use Timer mode for low frequencies
        //stepTimerMode(steps, frequency, dir);
    }
}

// Move steps with automatic direction (positive = forward, negative = reverse)
void PWMStepper::moveSteps(int32_t steps, double frequency) {
    if (steps == 0) return;
    
    bool dir = steps > 0;
    uint32_t absSteps = abs(steps);
    
    step(absSteps, frequency, dir);
}

// Set LEDC resolution (1-16 bits)
void PWMStepper::setLEDCResolution(uint8_t resolution) {
    if (resolution >= 1 && resolution <= 16) {
        ledcResolution = resolution;
        Serial.print("LEDC resolution set to "); 
        Serial.print(resolution); 
        Serial.println(" bits");
    }
}

// Set LEDC channel (0-15)
void PWMStepper::setLEDCChannel(uint8_t channel) {
    if (channel <= 15) {
        ledcChannel = channel;
        Serial.print("LEDC channel set to "); 
        Serial.println(channel);
    }
}

// Get maximum LEDC frequency for current resolution
double PWMStepper::getMaxLEDCFrequency() const {
    // ESP32 LEDC clock source is typically 80MHz
    const double clockSource = 80000000.0; // 80MHz
    return clockSource / (1 << ledcResolution);
}