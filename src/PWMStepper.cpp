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
    const esp_timer_create_args_t periodic_pwm_stepper_timer_args = {
        .callback = &onPWMStepperTimer, // link the call back
        .arg = nullptr,                 // no argument
        .name = "pwm-stepper-timer"      
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_pwm_stepper_timer_args, &pwm_stepper_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_stepper_timer, PWM_STEPPER_TIMER_DELAY)); // Start PWM stepper timer with a delay of 10ms
}

// Constructor
PWMStepper::PWMStepper(ESP32Encoder* encoder, int encoderScale, uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t ledcChannel) {
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->enablePin = enablePin;
    this->ledcChannel = ledcChannel;
    this->ledcFrequency = 1000;  // Default 1kHz
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
    if (pwm_stepper_timer == nullptr) {
        initTimers();
    }

    positionHistory.reserve(MAX_POSITION_HISTORY);
    updateTimes.reserve(MAX_POSITION_HISTORY);
    for (size_t i = 0; i < MAX_POSITION_HISTORY; ++i) {
        positionHistory[i] = encoder->getCount();
        updateTimes[i] = micros();
    }

    Serial.println("PWMStepper initialized successfully!");
    Serial.print("Step Pin: "); Serial.println(stepPin);
    Serial.print("Dir Pin: "); Serial.println(dirPin);
    Serial.print("Enable Pin: "); Serial.println(enablePin);
    Serial.print("LEDC Channel: "); Serial.println(ledcChannel);
    Serial.print("Frequency Threshold: "); Serial.print(FREQUENCY_THRESHOLD); Serial.println(" Hz");
    Serial.printf("PWM Stepper instances: %d\n", (int)pwmStepperInstances.size());
}

void ARDUINO_ISR_ATTR PWMStepper::update() {
    int64_t currentPosition = encoder->getCount() * encoderScale;
    uint64_t currentTime = micros();
    positionHistory[updateNumber % MAX_POSITION_HISTORY] = currentPosition;
    updateTimes[updateNumber % MAX_POSITION_HISTORY] = currentTime;
    // int64_t previousPosition = positionHistory[(updateNumber - 1 + MAX_POSITION_HISTORY) % MAX_POSITION_HISTORY];
    // uint64_t previousTime = updateTimes[(updateNumber - 1 + MAX_POSITION_HISTORY) % MAX_POSITION_HISTORY];
    // recentFreq = (currentPosition - previousPosition) * 1000000.0 / (currentTime - previousTime + 1); // +1 to avoid div by zero
    switch (state) {
        case STEPPER_OFF:
            return;
        case STEPPER_IDLE:
            return; // Do nothing if not running
        case STEPPER_MOVE_TO_POSITION:
            if (abs(currentPosition - targetPosition) >= encoderScale) {
                int64_t error = targetPosition - currentPosition;
                targetFreq = std::min(maxFreq, int64_t(abs(0.5 * error))); // Simple proportional control
                double newFreq = currentFreq;
                if (acceleration != 0) {
                    if (newFreq < targetFreq) {
                        newFreq += acceleration * (PWM_STEPPER_TIMER_DELAY / 1000000.0); // Convert delay to seconds
                        if (newFreq > targetFreq) {
                            newFreq = targetFreq;
                        }
                        startPWM(newFreq);
                    } else if (newFreq > targetFreq) {
                        newFreq -= acceleration * (PWM_STEPPER_TIMER_DELAY / 1000000.0); // Convert delay to seconds
                        if (newFreq < targetFreq) {
                            newFreq = targetFreq;
                        }
                        startPWM(newFreq);
                    }
                }        
            }  else {
                // Reached target
                stopPWM();
                state = STEPPER_IDLE;
                currentFreq = 0;
            }
            break; // Handle below
        case STEPPER_MOVE_WITH_FREQUENCY:
            if (acceleration != 0) {
                if (currentFreq < targetFreq) {
                    currentFreq += acceleration * (PWM_STEPPER_TIMER_DELAY / 1000000.0); // Convert delay to seconds
                    if (currentFreq > targetFreq) {
                        currentFreq = targetFreq;
                    }
                    startPWM(currentFreq);
                } else if (currentFreq > targetFreq) {
                    currentFreq -= acceleration * (PWM_STEPPER_TIMER_DELAY / 1000000.0); // Convert delay to seconds
                    if (currentFreq < targetFreq) {
                        currentFreq = targetFreq;
                    }
                    startPWM(currentFreq);
                }
            } else {
                currentFreq = targetFreq;
            }
            break; // Handle below
    }


    if (mode == MODE_TIMER && currentFreq > 0) {
        uint64_t microsSinceLastSpeedChange = currentTime - lastSpeedChangeMicros;
        double expectedMicrostepPeriod = 1000000.0 / abs(currentFreq * 2); // Period in microseconds
        uint64_t expectedNumberOfSteps = (uint64_t) (double(microsSinceLastSpeedChange) / expectedMicrostepPeriod);
        if (expectedNumberOfSteps <= stepsSinceLastSpeedChange) {
            return; // Not time for the next step yet
        }
        int stepPinState = digitalRead(stepPin); // Read current state
        pinMode(stepPin, OUTPUT);
        digitalWrite(stepPin, !stepPinState); // Start new period with toggled state   
        stepsSinceLastSpeedChange++;     
    }

    updateNumber++;
}

// Set direction
void PWMStepper::setDirection(bool dir) {
    direction = dir;
    digitalWrite(dirPin, direction ? HIGH : LOW);
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
    onSpeedChange(frequency, direction);
}

void PWMStepper::moveAtFrequency(double frequency) {
    targetFreq = frequency;
    state = STEPPER_MOVE_WITH_FREQUENCY;
    targetPosition = 0; // Clear target position
    onSpeedChange(frequency, direction);
    startPWM(frequency);
    Serial.printf("Moving at frequency: %.2f Hz in %s direction with %s mode\n", frequency, direction ? "forward" : "reverse", mode == MODE_LEDC ? "LEDC" : "Timer");
}

// Start PWM at specified frequency (steps per second) - Dual Mode
void PWMStepper::startPWM(double frequency) {
    
    currentFreq = frequency;
    
    // Choose mode based on frequency threshold
    if (frequency >= FREQUENCY_THRESHOLD) {
        // High frequency - use LEDC mode
        mode = MODE_LEDC;
        startLEDCMode(frequency);
    } else {
        // Low frequency - use Timer mode
        mode = MODE_TIMER;
        stopLEDCMode(); // Stop LEDC mode if running
        //startTimerMode(frequency);
    }

    onSpeedChange(frequency, direction);
}

// Stop PWM - Dual Mode
void PWMStepper::stopPWM() {
    setTargetFrequency(0);
    currentFreq = 0;
    if (mode == MODE_LEDC) {
        stopLEDCMode();
    } else {
        stopTimerMode();
    }
    
    digitalWrite(stepPin, LOW); // Ensure pin is LOW
    if (state != STEPPER_OFF) {
        state = STEPPER_IDLE;
    }
    currentFreq = 0;
    
    Serial.println("PWM stopped");
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
        ledcFrequency = frequency;
        currentFreq = frequency;
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
    return digitalRead(enablePin) == LOW;  // Active LOW
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

// LEDC Mode Methods
void PWMStepper::startLEDCMode(double frequency) {
    // Auto-adjust resolution based on frequency for optimal performance
    uint8_t optimalResolution = ledcResolution;    
   
    // Update LEDC frequency
    ledcFrequency = frequency;
    
    // Reconfigure LEDC with new frequency and resolution
    ledcSetup(ledcChannel, ledcFrequency, 4); // 4-bit resolution (0-15) - we use half the range for 50% duty cycle anyway
    ledcAttachPin(stepPin, ledcChannel);
    
    // Set 50% duty cycle for square wave
    uint32_t dutyCycle = 8;  // 50% duty cycle
    ledcWrite(ledcChannel, dutyCycle);
    onSpeedChange(frequency, direction);
}

void PWMStepper::stopLEDCMode() {
    ledcWrite(ledcChannel, 0);  // 0% duty cycle
    ledcDetachPin(stepPin);
}

// Timer Mode Methods
void PWMStepper::startTimerMode(double frequency) {
    mode = MODE_TIMER;
    currentFreq = frequency;
    onSpeedChange(frequency, direction);
}

void PWMStepper::stopTimerMode() {
    mode = MODE_LEDC; // Default back to LEDC mode
    digitalWrite(stepPin, LOW); // Ensure pin is LOW
}

void PWMStepper::onSpeedChange(double newSpeed, bool dir) {
    if (newSpeed != currentFreq || dir != direction) {
        lastSpeedChangeMicros = micros();
        stepsSinceLastSpeedChange = 0;
    }
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