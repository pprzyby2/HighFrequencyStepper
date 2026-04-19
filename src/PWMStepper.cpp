#include "PWMStepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <vector>
#include <algorithm>



// Define frequency threshold (512 Hz) - below this we use timer mode for better low-speed control
const double PWMStepper::FREQUENCY_THRESHOLD = 1024.0;
const double PWMStepper::DECELERATION_FREQUENCY = 1000.0; // Target freq when decelerating near position

// Constants for position control
static const int64_t DECEL_SCALING_DISTANCE = 800; // Distance (steps) over which acceleration scales
static const int64_t MIN_DECEL_ERROR = 50; // Minimum error (steps) before starting deceleration

const uint64_t PWM_STEPPER_TIMER_DELAY = 500; // 0.5ms
esp_timer_handle_t pwm_stepper_timer = nullptr;
static std::vector<PWMStepper*> pwmStepperInstances;

// LEDC timer allocation tracking
static const uint8_t MAX_LEDC_TIMERS = 4;
static uint8_t allocatedLedcTimers = 0;

static void ARDUINO_ISR_ATTR onPWMStepperTimer(void *arg)
{
    for (auto stepper : pwmStepperInstances) {
        stepper->update();
    }
}

// Process pending LEDC commands for all steppers - call from loop()
void processAllStepperCommands() {
    for (auto stepper : pwmStepperInstances) {
        stepper->processCommands();
    }
}

// FreeRTOS task for processing stepper commands
static TaskHandle_t stepperCommandTaskHandle = nullptr;
static const uint32_t COMMAND_TASK_DELAY_MS = 1; // 1ms between command processing

static void stepperCommandTask(void* parameter) {
    for (;;) {
        processAllStepperCommands();
        vTaskDelay(pdMS_TO_TICKS(COMMAND_TASK_DELAY_MS));
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
    
    // Create ISR timer for update()
    const esp_timer_create_args_t periodic_pwm_stepper_timer_args = {
        .callback = &onPWMStepperTimer, // link the call back
        .arg = nullptr,                 // no argument
        .name = "pwm-stepper-timer"      
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_pwm_stepper_timer_args, &pwm_stepper_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pwm_stepper_timer, PWM_STEPPER_TIMER_DELAY));
    
    // Create FreeRTOS task for processing LEDC commands (runs on core 1)
    if (stepperCommandTaskHandle == nullptr) {
        xTaskCreatePinnedToCore(
            stepperCommandTask,        // Task function
            "StepperCmdTask",          // Task name
            4096,                      // Stack size
            nullptr,                   // Parameters
            1,                         // Priority (low, but higher than idle)
            &stepperCommandTaskHandle, // Task handle
            1                          // Core 1 (main loop is typically on core 1)
        );
        Serial.println("Stepper command processing task started on core 1");
    }
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
    
    // Allocate LEDC timer sequentially
    if (allocatedLedcTimers >= MAX_LEDC_TIMERS) {
        Serial.println("ERROR: All 4 LEDC timers are already allocated! Cannot create more PWMStepper instances.");
        this->ledcTimer = LEDC_TIMER_0; // Fallback, but won't work correctly
    } else {
        this->ledcTimer = static_cast<ledc_timer_t>(allocatedLedcTimers);
        allocatedLedcTimers++;
        Serial.printf("Allocated LEDC timer %d (total: %d/%d)\n", this->ledcTimer, allocatedLedcTimers, MAX_LEDC_TIMERS);
    }
}

// Destructor - ensure proper cleanup
PWMStepper::~PWMStepper() {
    stopPWM();
    pwmStepperInstances.erase(std::remove(pwmStepperInstances.begin(), pwmStepperInstances.end(), this), pwmStepperInstances.end());
    
    // Release LEDC timer
    if (allocatedLedcTimers > 0) {
        allocatedLedcTimers--;
    }
}

// Initialize the stepper
void PWMStepper::begin() {
    // Configure GPIO pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    
    // Set initial states
    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, HIGH);
    disable();
    
    // Setup LEDC channel
    ledcSetup(ledcChannel, ledcFrequency, 1);
    ledcAttachPin(stepPin, ledcChannel);
    stopLEDCMode();

    pwmStepperInstances.push_back(this);
    initStepperTimers();

    positionHistory.resize(MAX_POSITION_HISTORY, encoder->getCount());
    updateTimes.resize(MAX_POSITION_HISTORY, micros());
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
    // Read 64-bit variables atomically at the start of ISR
    portENTER_CRITICAL_ISR(&mux);
    int64_t localTargetPosition = targetPosition;
    double localTargetFreq = targetFreq;
    double localAcceleration = acceleration;
    portEXIT_CRITICAL_ISR(&mux);
    
    float currentPosition = encoderScale * encoder->getCount();
    uint64_t currentTime = micros();
    int64_t error = 0;
    bool reachedTarget = false;
    bool direction = getDirection(); // true = forward, false = reverse
    if (state == STEPPER_MOVE_TO_POSITION) {
        error = localTargetPosition - currentPosition;
        reachedTarget = abs(error) < int(encoderScale); // Consider reached if within 1 step (scaled by encoder)
    }

    // Update position history for frequency estimation
    positionHistory[updateNumber % MAX_POSITION_HISTORY] = currentPosition;
    updateTimes[updateNumber % MAX_POSITION_HISTORY] = currentTime;

    // Update direction based on current frequency and target position. 
    setDirectionPin(direction); // Ensure direction is correct based on target frequency

    // Handle timer mode stepping if state is active and frequency is above threshold
    // Timer mode is triggered when frequency is below 500 Hz for better low-speed peformance.
    // In timer mode, we manually toggle the step pin at the correct intervals based on the current frequency.
    if (mode == MODE_TIMER && state != STEPPER_IDLE && state != STEPPER_OFF && currentFreq != 0) {
        // Calculate how many steps we should have taken since the last speed change based on the current frequency
        uint64_t microsSinceLastSpeedChange = currentTime - lastSpeedChangeMicros;
        double expectedMicrostepPeriod = 1000000.0 / abs(currentFreq * 2); // Period in microseconds for a full step (times 2 because we toggle 0-1 and 1-0 for each step)
        uint64_t expectedNumberOfSteps = (uint64_t) (double(microsSinceLastSpeedChange) / expectedMicrostepPeriod);
        if (expectedNumberOfSteps > stepsSinceLastSpeedChange && !reachedTarget) {
            int stepPinState = digitalRead(stepPin); // Read current state
            pinMode(stepPin, OUTPUT);
            digitalWrite(stepPin, !stepPinState); // Start new period with toggled state   
            stepsSinceLastSpeedChange++;     
        }
    }
    // If timer mode is not active, we rely on LEDC to generate the step pulses, so we only need to monitor position and handle acceleration/deceleration logic for moveToPosition mode.

    // Periodically update the encoder frequency in order to accelerate/decelerate based on actual movement. 
    // This can help compensate for missed steps or stalls, especially at low speeds. We only update the frequency every N updates to avoid excessive calculations and noise in the frequency estimation.
    int freqUpdateInterval = 10;
    updateNumber++;
    // Only update frequency every freqUpdateInterval updates to reduce noise and computational load
    if (updateNumber % freqUpdateInterval != 0) {
        return;
    }
    // if (updateNumber % 5000 == 0) {
    //     Serial.printf("Current Position: %.2f, Target Position: %d, Current Freq: %.2f, Target Freq: %.2f, Encoder Freq: %.2f, state: %s\n", 
    //         currentPosition, targetPosition, currentFreq, targetFreq, encoderFrequency, getStateName(state).c_str());
    // }

    // Calculate empirical frequency from encoder counts. We use the position history to calculate how many steps have been taken over a certain time period, 
    // which gives us an estimate of the actual frequency. This can help us detect stalls or missed steps.
    if (updateNumber < MAX_POSITION_HISTORY) {
        encoderFrequency = (currentPosition - positionHistory[0]) * 1000000.0 / (currentTime - updateTimes[0]);
    } else {
        encoderFrequency = (currentPosition - positionHistory[updateNumber % MAX_POSITION_HISTORY]) * 1000000.0 / (currentTime - updateTimes[updateNumber % MAX_POSITION_HISTORY]);
    }

    // acceleration is in steps/s^2. We need to convert it to frequency change per update interval. 
    // The frequency change per second is acceleration * current frequency, so the frequency change per update interval is acceleration * current frequency * (update interval in seconds). 
    // We can simplify this by using the fact that frequency is proportional to speed, so we can just use acceleration * (update interval in seconds) as the frequency change per update interval.
    double unitFrequencyChange = localAcceleration * (freqUpdateInterval * PWM_STEPPER_TIMER_DELAY / 1000000.0);
    
    switch (state) {
        case STEPPER_OFF:
            return;
        case STEPPER_IDLE:
            return; // Do nothing if not running
        case STEPPER_MOVE_TO_POSITION:
            unitFrequencyChange = unitFrequencyChange * min(abs(error), DECEL_SCALING_DISTANCE) / (double)DECEL_SCALING_DISTANCE;
            if (!reachedTarget) {
                // Acceleration should not be zero for moveToPosition mode.
                if (localAcceleration != 0) {
                    // Calculate how many updates it would take to decelerate from current frequency to 0 at the given acceleration. 
                    // This gives us an estimate of how long it will take to stop if we start decelerating now. 
                    // We can then calculate how far we would travel during that time at the current speed, which gives us an estimate of the deceleration distance. 
                    // If we are within that distance from the target position, we should start decelerating to avoid overshooting.                    
                    double decelerationUpdates = (abs(currentFreq) / unitFrequencyChange);
                    double estimatedDecelerationTime = decelerationUpdates * (freqUpdateInterval * PWM_STEPPER_TIMER_DELAY / 1000000.0);
                    decelerationDistance = localAcceleration * pow(estimatedDecelerationTime, 2) / 2.0; // (currentFreq * currentFreq) / (2.0 * acceleration);
                    if (abs(error) < decelerationDistance * 1.5 || abs(error) < MIN_DECEL_ERROR) {
                        localTargetFreq = DECELERATION_FREQUENCY; // Decelerate below timer mode threshold for better low-speed control
                    } else {
                        localTargetFreq = maxFreq; // Accelerate
                    }
                    localTargetFreq = abs(localTargetFreq) * (error > 0 ? 1 : -1); // Ensure target frequency has correct sign based on error direction
                    double newFreq = currentFreq;
                
                    if (newFreq < localTargetFreq) {
                        newFreq += unitFrequencyChange; // Convert delay to seconds
                        if (newFreq > localTargetFreq) {
                            newFreq = localTargetFreq;
                        }
                        // //Serial.printf("Accelerating to %.2f Hz, deceleration distance: %.2f, error: %d\n", newFreq, decelerationDistance, error);                                                                            
                    } else {
                        newFreq -= unitFrequencyChange; // Convert delay to seconds
                        if (newFreq < localTargetFreq) {
                            newFreq = localTargetFreq;
                        }
                    }
                    requestStartPWM(newFreq);
                }
            } else {
                requestStopPWM();
                pendingNewState = STEPPER_IDLE;
                pendingStateChange = true;
                currentFreq = 0;
            }
            break; // Handle below
        case STEPPER_MOVE_WITH_FREQUENCY:
            {
                double newFreq = currentFreq;
                if (localAcceleration != 0) {
                    if (newFreq < localTargetFreq) {
                        newFreq += unitFrequencyChange; // Convert delay to seconds
                        if (newFreq > localTargetFreq) {
                            newFreq = localTargetFreq;
                        }
                    } else if (newFreq > localTargetFreq) {
                        newFreq -= unitFrequencyChange; // Convert delay to seconds
                        if (newFreq < localTargetFreq) {
                            newFreq = localTargetFreq;
                        }
                    }
                }
                requestStartPWM(newFreq);
            }
            break; // Handle below
    }
}

// Set direction
void PWMStepper::setDirectionPin(bool dir) {
    if (invertDirection) {
        dir = !dir;
    }
    digitalWrite(dirPin, dir ? HIGH : LOW);
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
    // If we are currently in moveToPosition mode, we want to switch to moveWithFrequency mode immediately with the new target frequency.
    // Acceleration won't be used in this case since we are directly setting the current frequency to the target frequency.
    startPWM(frequency);
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
    if (abs(frequency) >= FREQUENCY_THRESHOLD) {
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

// ISR-safe: request PWM start (sets flag, actual LEDC work done in processCommands)
void PWMStepper::requestStartPWM(double frequency) {
    pendingNewFrequency = frequency;
    pendingFrequencyChange = true;
    pendingStop = false;
    
    // Update speed tracking immediately (safe for ISR)
    onSpeedChange(frequency);
}

// ISR-safe: request PWM stop (sets flag, actual LEDC work done in processCommands)
void PWMStepper::requestStopPWM() {
    pendingStop = true;
    pendingFrequencyChange = false;
}

// Process pending PWM commands - call this from main loop, NOT from ISR
void PWMStepper::processCommands() {
    // Handle pending state change
    if (pendingStateChange) {
        state = pendingNewState;
        pendingStateChange = false;
    }
    
    // Handle pending stop
    if (pendingStop) {
        pendingStop = false;
        setTargetFrequency(0);
        if (mode == MODE_LEDC) {
            stopLEDCMode();
        } else {
            stopTimerMode();
        }
        digitalWrite(stepPin, LOW);
        if (state != STEPPER_OFF) {
            state = STEPPER_IDLE;
        }
        onSpeedChange(0);
        return;
    }
    
    // Handle pending frequency change
    if (pendingFrequencyChange) {
        double frequency = pendingNewFrequency;
        pendingFrequencyChange = false;
        
        // Choose mode based on frequency threshold
        if (abs(frequency) >= FREQUENCY_THRESHOLD) {
            // High frequency - use LEDC mode
            if (mode != MODE_LEDC) {
                stopTimerMode();
            }
            mode = MODE_LEDC;
            startLEDCMode(frequency);
        } else {
            // Low frequency - use Timer mode
            if (mode != MODE_TIMER) {
                stopLEDCMode();
            }
            mode = MODE_TIMER;
            startTimerMode(frequency);
        }
    }
}

void PWMStepper::setTargetFrequency(double frequency) {
    portENTER_CRITICAL(&mux);
    targetFreq = frequency;
    portEXIT_CRITICAL(&mux);
}

void PWMStepper::setAcceleration(double accel) {
    portENTER_CRITICAL(&mux);
    acceleration = accel;
    portEXIT_CRITICAL(&mux);
}

void PWMStepper::setTargetPosition(int64_t position) {
    portENTER_CRITICAL(&mux);
    targetPosition = position;
    portEXIT_CRITICAL(&mux);
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
    int speed = currentFreq;
    if (state == STEPPER_MOVE_TO_POSITION && abs(speed) < DECELERATION_FREQUENCY) {
        // In moveToPosition mode at low speeds, determine direction based on target position vs current position rather than frequency, 
        // because it is safe to switch direction at low speeds and move towards the target position even if the frequency is not perfectly stable yet. 
        // This helps to ensure we are moving in the correct direction towards the target position, especially when we are starting from a stop (speed = 0) or when we are very
        // close to the target and the frequency is low.
        int64_t currentPosition = getPosition();
        return currentPosition < targetPosition; // true = forward, false = reverse
    } else {
        // For higher speeds, determine direction based on current frequency sign
        return speed > 0; // true = forward, false = reverse
    }
}

double PWMStepper::getFrequency() const {
    portENTER_CRITICAL(&mux);
    double freq = currentFreq;
    portEXIT_CRITICAL(&mux);
    return freq;
}

int64_t PWMStepper::getTargetPosition() const {
    portENTER_CRITICAL(&mux);
    int64_t pos = targetPosition;
    portEXIT_CRITICAL(&mux);
    return pos;
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
            if (try_config_ledc_timer(ledcSpeedMode, ledcTimer, bits, (uint32_t)abs(frequency), clk)) {
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
    ledcFrequency = (uint32_t) abs(frequency);

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

// Set LEDC resolution (1-16 bits)
void PWMStepper::setLEDCResolution(uint8_t resolution) {
    if (resolution >= 1 && resolution <= 16) {
        ledcResolution = resolution;
        //Serial.printf("LEDC resolution set to %u bits\n", resolution);
    }
}

// Set LEDC channel (0-15)
void PWMStepper::setLEDCChannel(uint8_t channel) {
    if (channel <= 15) {
        ledcChannel = channel;
        //Serial.printf("LEDC channel set to %u\n", channel);
    }
}

// Get maximum LEDC frequency for current resolution
double PWMStepper::getMaxLEDCFrequency() const {
    // ESP32 LEDC clock source is typically 80MHz
    const double clockSource = 80000000.0; // 80MHz
    return clockSource / (1 << ledcResolution);
}