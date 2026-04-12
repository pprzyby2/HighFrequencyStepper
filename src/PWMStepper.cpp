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

// Helper to try configuring LEDC timer with given bits/clock
static bool try_config_ledc_timer(ledc_mode_t speedMode,
                                  ledc_timer_t timer,
                                  uint8_t bits,
                                  uint32_t freq,
                                  ledc_clk_cfg_t clk)
{
    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode       = speedMode;
    tcfg.timer_num        = timer;
    tcfg.duty_resolution  = (ledc_timer_bit_t)bits;
    tcfg.freq_hz          = freq;
    tcfg.clk_cfg          = clk;
    return ledc_timer_config(&tcfg) == ESP_OK;
}

void initStepperTimers() {
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
    this->currentFreq = 0;
    this->targetFreq = 0;
    this->targetPosition = 0;
    this->acceleration = 0;
    this->encoder = encoder;
    this->encoderScale = encoderScale;
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
    digitalWrite(dirPin, getDirection() ? HIGH : LOW);
    disable();
    
    // Setup LEDC channel
    ledcSetup(ledcChannel, ledcFrequency, ledcResolution);
    ledcAttachPin(stepPin, ledcChannel);
    stopLEDCMode();

    pwmStepperInstances.push_back(this);
    initStepperTimers();

    positionHistory.reserve(MAX_POSITION_HISTORY);
    updateTimes.reserve(MAX_POSITION_HISTORY);
    for (size_t i = 0; i < MAX_POSITION_HISTORY; ++i) {
        positionHistory[i] = encoder->getCount();
        updateTimes[i] = micros();
    }
    state = STEPPER_IDLE;

    Serial.println("PWMStepper initialized successfully!");
    Serial.printf("Step Pin: %d\n", stepPin);
    Serial.printf("Dir Pin: %d\n", dirPin);
    Serial.printf("Enable Pin: %d\n", enablePin);
    Serial.printf("LEDC Channel: %d\n", ledcChannel);
    Serial.printf("Frequency Threshold: %d Hz\n", FREQUENCY_THRESHOLD);
    Serial.printf("PWM Stepper instances: %d\n", (int)pwmStepperInstances.size());
}

void ARDUINO_ISR_ATTR PWMStepper::update() {
    float currentPosition = encoderScale * encoder->getCount();
    uint64_t currentTime = micros();

    // Update position history for frequency estimation
    positionHistory[updateNumber % MAX_POSITION_HISTORY] = currentPosition;
    updateTimes[updateNumber % MAX_POSITION_HISTORY] = currentTime;

    // Handle timer mode stepping if state is active and frequency is above threshold
    // Timer mode is triggered when frequency is below 500 Hz for better low-speed peformance.
    // In timer mode, we manually toggle the step pin at the correct intervals based on the current frequency.
    if (state != STEPPER_IDLE && state != STEPPER_OFF && mode == MODE_TIMER && currentFreq != 0) {
        // Calculate how many steps we should have taken since the last speed change based on the current frequency
        uint64_t microsSinceLastSpeedChange = currentTime - lastSpeedChangeMicros;
        double expectedMicrostepPeriod = 1000000.0 / abs(currentFreq * 2); // Period in microseconds for a full step (times 2 because we toggle 0-1 and 1-0 for each step)
        uint64_t expectedNumberOfSteps = (uint64_t) (double(microsSinceLastSpeedChange) / expectedMicrostepPeriod);
        if (expectedNumberOfSteps > stepsSinceLastSpeedChange) {
            int stepPinState = digitalRead(stepPin); // Read current state
            pinMode(stepPin, OUTPUT);
            digitalWrite(stepPin, !stepPinState); // Start new period with toggled state   
            stepsSinceLastSpeedChange++;     
        }
    }
    // If timer mode is not active, we rely on LEDC to generate the step pulses, so we only need to monitor position and handle acceleration/deceleration logic for moveToPosition mode.


    // Handle overshooting target position in moveToPosition mode. Just stop the motor if we have reached or passed the target position. 
    // This is a safety measure to prevent runaway if something goes wrong with the acceleration/deceleration logic.
    if (state == STEPPER_MOVE_TO_POSITION) {
        bool actualDirection = currentPosition < targetPosition; // true = forward, false = reverse
        setDirection(actualDirection); // Ensure direction is correct based on target frequency
        // if (actualDirection && currentPosition >= targetPosition || !actualDirection && currentPosition <= targetPosition) {
        //     stopPWM();
        //     state = STEPPER_IDLE;
        //     currentFreq = 0;
        // }
    }

    // Periodically update the encoder frequency in order to accelerate/decelerate based on actual movement. 
    // This can help compensate for missed steps or stalls, especially at low speeds. We only update the frequency every N updates to avoid excessive calculations and noise in the frequency estimation.
    int freqUpdateInterval = 50;
    updateNumber++;
    // Only update frequency every freqUpdateInterval updates to reduce noise and computational load
    if (updateNumber % freqUpdateInterval != 0) {
        return;
    }

    // Calculate empirical frequency from encoder counts. We use the position history to calculate how many steps have been taken over a certain time period, 
    // which gives us an estimate of the actual frequency. This can help us detect stalls or missed steps.
    if (updateNumber < MAX_POSITION_HISTORY) {
        encoderFrequency = abs((currentPosition - positionHistory[0]) * 1000000.0 / (currentTime - updateTimes[0]));
    } else {
        encoderFrequency = abs((currentPosition - positionHistory[updateNumber % MAX_POSITION_HISTORY]) * 1000000.0 / (currentTime - updateTimes[updateNumber % MAX_POSITION_HISTORY]));
    }
    if (currentFreq > FREQUENCY_THRESHOLD && encoderFrequency < 50) {
        // Possible stall or missed steps
        //startPWM(encoderFrequency);
    }


    int64_t error = targetPosition - currentPosition;
    // acceleration is in steps/s^2. We need to convert it to frequency change per update interval. 
    // The frequency change per second is acceleration * current frequency, so the frequency change per update interval is acceleration * current frequency * (update interval in seconds). 
    //We can simplify this by using the fact that frequency is proportional to speed, so we can just use acceleration * (update interval in seconds) as the frequency change per update interval.
    double unitFrequencyChange = acceleration * (freqUpdateInterval * PWM_STEPPER_TIMER_DELAY / 1000000.0);

    switch (state) {
        case STEPPER_OFF:
            return;
        case STEPPER_IDLE:
            return; // Do nothing if not running
        case STEPPER_MOVE_TO_POSITION:
            if (abs(error) >= int(encoderScale) + 1) {    
                // Acceleration should not be zero for moveToPosition mode.
                if (acceleration != 0) {
                    // Calculate how many updates it would take to decelerate from current frequency to 0 at the given acceleration. 
                    // This gives us an estimate of how long it will take to stop if we start decelerating now. 
                    // We can then calculate how far we would travel during that time at the current speed, which gives us an estimate of the deceleration distance. 
                    // If we are within that distance from the target position, we should start decelerating to avoid overshooting.
                    double decelerationUpdates = (abs(currentFreq) / unitFrequencyChange);
                    double estimatedDecelerationTime = decelerationUpdates * (freqUpdateInterval * PWM_STEPPER_TIMER_DELAY / 1000000.0);
                    decelerationDistance = acceleration * pow(estimatedDecelerationTime, 2) / 2.0; // (currentFreq * currentFreq) / (2.0 * acceleration);
                    if (abs(error) < decelerationDistance * 1.5) { // Add slight buffer to deceleration
                        targetFreq = 400; // Decelerate below timer mode threshold for better low-speed control. We can also calculate a dynamic target frequency based on how close we are to the target position, but for simplicity we will just use a fixed low frequency here.
                        //decelerating = true;
                        //currentFreq = targetFreq;
                    } else {
                        targetFreq = maxFreq; // Accelerate
                    }
                    double newFreq = abs(currentFreq);
                
                    if (newFreq < targetFreq) {
                        newFreq += unitFrequencyChange; // Convert delay to seconds
                        if (newFreq > targetFreq) {
                            newFreq = targetFreq;
                        }
                        // //Serial.printf("Accelerating to %.2f Hz, deceleration distance: %.2f, error: %d\n", newFreq, decelerationDistance, error);                                                                            
                    } else {
                        newFreq -= unitFrequencyChange; // Convert delay to seconds
                        if (newFreq < targetFreq) {
                            newFreq = targetFreq;
                        }
                    }
                    if (currentFreq < 0) {
                        newFreq = -newFreq;
                    }
                    startPWM(newFreq);
                }
            } else {
                stopPWM();
                state = STEPPER_IDLE;
                currentFreq = 0;
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
                onSpeedChange(targetFreq);
            }
            break; // Handle below
    }
}

// Set direction
void PWMStepper::setDirection(bool dir) {
    if (invertDirection) {
        dir = !dir;
    }
    digitalWrite(dirPin, dir ? HIGH : LOW);
    //onSpeedChange(currentFreq);
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
    onSpeedChange(frequency);
    startPWM(frequency);
    Serial.printf("Moving at frequency: %.2f Hz in %s direction with %s mode\n", frequency, getDirection() ? "forward" : "reverse", mode == MODE_LEDC ? "LEDC" : "Timer");
}

void PWMStepper::moveToPosition(int64_t position, double frequency) {
    int32_t currentPos = getPosition();
    int32_t steps = position - currentPos;
    if (steps == 0) {
        return; // Already at position    
    } 

    setTargetPosition(position);
    setTargetFrequency(frequency);
    state = STEPPER_MOVE_TO_POSITION;
}

bool PWMStepper::isMovingToPosition() const {
    return state == STEPPER_MOVE_TO_POSITION;
}

// Start PWM at specified frequency (steps per second) - Dual Mode
void PWMStepper::startPWM(double frequency) {
    onSpeedChange(frequency);
    
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
    onSpeedChange(0);
    
}

void PWMStepper::setTargetFrequency(double frequency) {
    targetFreq = frequency;
}

void PWMStepper::setAcceleration(double accel) {
    acceleration = accel;
}

void PWMStepper::setTargetPosition(int64_t position) {
    targetPosition = position;
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
    bool currentDir = digitalRead(dirPin);
    int speed = currentFreq;
    if (abs(speed) < 400) {
        // If speed is very low, determine direction based on target position vs current position
        int64_t currentPosition = getPosition();
        return currentPosition < targetPosition; // true = forward, false = reverse
    } else {
        // For higher speeds, determine direction based on current frequency sign
        return invertDirection ? speed < 0 : speed > 0; // true = forward, false = reverse
    }
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
    Serial.printf("Direction: %s, ", getDirection() ? "Forward" : "Reverse");
    Serial.printf("Current Freq: %.2f Hz, ", currentFreq);
    Serial.printf("Encoder Freq: %.2f Hz, ", encoderFrequency);
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

// Robust LEDC setup that auto-picks clock and resolution
void PWMStepper::startLEDCMode(double frequency)
{
    // Detach first to avoid "not initialized" errors
    ledcDetachPin(stepPin);

    // Try fast clocks first on S3: 160 MHz, then 80 MHz, then AUTO (as last resort)
    const ledc_clk_cfg_t clocksToTry[] = {
        LEDC_USE_APB_CLK,     // 80 MHz
        LEDC_AUTO_CLK         // may select REF_TICK (1 MHz) -> low max freq
    };

    // Start from requested or current resolution, clamp 1..15
    uint8_t startBits = ledcResolution > 0 ? ledcResolution : 8;
    if (startBits < 1) startBits = 1;
    if (startBits > 15) startBits = 15;

    bool configured = false;
    uint8_t chosenBits = startBits;
    ledc_clk_cfg_t chosenClk = LEDC_USE_APB_CLK;

    for (ledc_clk_cfg_t clk : clocksToTry) {
        for (int bits = startBits; bits >= 1; --bits) {
            if (try_config_ledc_timer(ledcSpeedMode, ledcTimer, bits, (uint32_t)frequency, clk)) {
                chosenBits = (uint8_t)bits;
                chosenClk = clk;
                configured = true;
                break;
            }
        }
        if (configured) break;
    }

    if (!configured) {
        ESP_LOGE("PWMStepper", "LEDC setup failed for %.1f Hz at all clocks/resolutions, using Timer mode", frequency);
        mode = MODE_TIMER;
        startTimerMode(frequency);
        return;
    }

    // Configure channel on the selected timer
    ledc_channel_config_t c = {};
    c.gpio_num   = stepPin;
    c.speed_mode = ledcSpeedMode;
    c.channel    = (ledc_channel_t)ledcChannel;
    c.intr_type  = LEDC_INTR_DISABLE;
    c.timer_sel  = ledcTimer;
    c.duty       = 1u << (chosenBits - 1); // ~50%
    c.hpoint     = 0;
    c.flags.output_invert = 0;

    esp_err_t err = ledc_channel_config(&c);
    if (err != ESP_OK) {
        // ESP_LOGE("PWMStepper", "LEDC channel config failed (%d), falling back to Timer mode", (int)err);
        mode = MODE_TIMER;
        startTimerMode(frequency);
        return;
    }

    // Write initial duty to start output
    ledcResolutionBits = chosenBits;
    ledcClk = chosenClk;
    ledcFrequency = (uint32_t)frequency;

    //ledcUpdateDuty(ledcSpeedMode, (ledc_channel_t)ledcChannel);
    ledcWrite(ledcChannel, 1u << (chosenBits - 1)); // ~50%

    mode = MODE_LEDC;
    // ESP_LOGI("PWMStepper", "LEDC OK: f=%.1f Hz, res=%u-bit, clk=%s",
    //          frequency,
    //          (unsigned)chosenBits,
    //          (chosenClk == LEDC_USE_PLL_DIV_CLK ? "PLL160" :
    //           chosenClk == LEDC_USE_APB_CLK ? "APB80" : "AUTO"));
}




void PWMStepper::stopLEDCMode() {
    // Ensure channel is stopped and pin is released
    ledc_stop(ledcSpeedMode, (ledc_channel_t)ledcChannel, 0 /*idle level*/);

    //ledcWrite(ledcChannel, 0);  // 0% duty cycle
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

void PWMStepper::onSpeedChange(double newSpeed) {
    if (newSpeed != currentFreq) {
        lastSpeedChangeMicros = micros();
        stepsSinceLastSpeedChange = 0;
    }
    currentFreq = newSpeed;
}

// Move a specific number of steps
void PWMStepper::step(uint32_t steps, double frequency, bool dir) {
    if (steps == 0) return;
    
    enable();
    setDirection(dir);
    startPWM(frequency);
    
    // Calculate duration in milliseconds
    uint32_t duration = (steps * 1000) / frequency;

    delay(duration);
    stopPWM();
    
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