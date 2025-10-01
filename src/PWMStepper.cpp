#include "PWMStepper.h"

// Global instance pointer for timer callback
PWMStepper* currentStepperInstance = nullptr;

// Define frequency threshold (512 Hz)
const double PWMStepper::FREQUENCY_THRESHOLD = 512.0;

// Constructor
PWMStepper::PWMStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin, uint8_t ledcChannel) {
    this->stepPin = stepPin;
    this->dirPin = dirPin;
    this->enablePin = enablePin;
    this->ledcChannel = ledcChannel;
    this->ledcFrequency = 1000;  // Default 1kHz
    this->ledcResolution = 8;    // 8-bit resolution (0-255)
    this->isRunning = false;
    this->direction = true;      // Forward by default
    this->currentFreq = 0;
    this->isLEDCMode = true;     // Default to LEDC mode
    this->stepTimer = nullptr;
    this->timerStepState = false;
    this->timerStepsRemaining = 0;
    this->timerRunning = false;
}

// Destructor - ensure proper cleanup
PWMStepper::~PWMStepper() {
    stopPWM();
    if (currentStepperInstance == this) {
        currentStepperInstance = nullptr;
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
    digitalWrite(dirPin, direction ? HIGH : LOW);
    digitalWrite(enablePin, HIGH);  // Disabled by default (active LOW)
    
    // Setup LEDC channel
    ledcSetup(ledcChannel, ledcFrequency, ledcResolution);
    ledcAttachPin(stepPin, ledcChannel);

    // Initialize timer (will be configured when needed)
    stepTimer = nullptr;
    
    Serial.println("PWMStepper initialized successfully!");
    Serial.print("Step Pin: "); Serial.println(stepPin);
    Serial.print("Dir Pin: "); Serial.println(dirPin);
    Serial.print("Enable Pin: "); Serial.println(enablePin);
    Serial.print("LEDC Channel: "); Serial.println(ledcChannel);
    Serial.print("Frequency Threshold: "); Serial.print(FREQUENCY_THRESHOLD); Serial.println(" Hz");
}

// Set direction
void PWMStepper::setDirection(bool dir) {
    direction = dir;
    digitalWrite(dirPin, direction ? HIGH : LOW);
}

// Enable the stepper driver
void PWMStepper::enable() {
    digitalWrite(enablePin, LOW);  // Active LOW
}

// Disable the stepper driver
// Disable the stepper driver
void PWMStepper::disable() {
    digitalWrite(enablePin, HIGH); // Active LOW
    stopPWM();  // Also stop any running PWM
}

// Start PWM at specified frequency (steps per second) - Dual Mode
void PWMStepper::startPWM(double frequency) {
    if (frequency == 0) {
        stopPWM();
        return;
    }
    
    currentFreq = frequency;
    
    // Choose mode based on frequency threshold
    if (frequency >= FREQUENCY_THRESHOLD) {
        // High frequency - use LEDC mode
        Serial.print("Using LEDC mode for frequency: ");
        Serial.print(frequency);
        Serial.println(" Hz");
        
        isLEDCMode = true;
        stopTimerMode(); // Stop timer mode if running
        startLEDCMode(frequency);
    } else {
        // Low frequency - use Timer mode
        Serial.print("Using Timer mode for frequency: ");
        Serial.print(frequency);
        Serial.println(" Hz");
        
        isLEDCMode = false;
        stopLEDCMode(); // Stop LEDC mode if running
        startTimerMode(frequency);
    }
    
    isRunning = true;
}

// Stop PWM - Dual Mode
void PWMStepper::stopPWM() {
    if (isLEDCMode) {
        stopLEDCMode();
    } else {
        stopTimerMode();
    }
    
    digitalWrite(stepPin, LOW); // Ensure pin is LOW
    isRunning = false;
    currentFreq = 0;
    
    Serial.println("PWM stopped");
}

// Set frequency while running
void PWMStepper::setFrequency(double frequency) {
    if (isRunning) {
        startPWM(frequency);  // Restart with new frequency
    } else {
        ledcFrequency = frequency;
        currentFreq = frequency;
    }
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

bool PWMStepper::isInLEDCMode() const {
    return isLEDCMode;
}

// LEDC Mode Methods
void PWMStepper::startLEDCMode(double frequency) {
    // Auto-adjust resolution based on frequency for optimal performance
    uint8_t optimalResolution = ledcResolution;
    
    // Calculate optimal resolution for the requested frequency
    // ESP32 LEDC clock is typically 80MHz
    if (frequency > 312500) {      // > 312.5kHz requires 7-bit or less
        optimalResolution = 7;     // Max ~625kHz
    } else if (frequency > 156250) { // > 156.25kHz requires 8-bit or less  
        optimalResolution = 8;     // Max ~312.5kHz
    } else if (frequency > 78125) {  // > 78.125kHz requires 9-bit or less
        optimalResolution = 9;     // Max ~156.25kHz
    } else {
        optimalResolution = 10;    // Max ~78.125kHz, good resolution
    }
    
    // Update resolution if it changed
    if (optimalResolution != ledcResolution) {
        ledcResolution = optimalResolution;
        Serial.print("Auto-adjusted LEDC resolution to ");
        Serial.print(ledcResolution);
        Serial.print(" bits for ");
        Serial.print(frequency);
        Serial.println(" Hz");
    }
    
    // Update LEDC frequency
    ledcFrequency = frequency;
    
    // Reconfigure LEDC with new frequency and resolution
    ledcSetup(ledcChannel, ledcFrequency, 4); // 4-bit resolution (0-15) - we use half the range for 50% duty cycle anyway
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
    // Calculate timer period in microseconds
    // For square wave, we need to toggle every half period
    uint32_t halfPeriodUs = (uint32_t)(500000.0 / frequency);  // 500000 = 1000000/2
    
    // Stop existing timer if running
    stopTimerMode();
    int stepPinState = digitalRead(stepPin); // Read current state
    pinMode(stepPin, OUTPUT);
    digitalWrite(stepPin, !stepPinState); // Start new period with toggled state
    
    // Create and configure timer
    stepTimer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1MHz), count up
    if (stepTimer == nullptr) {
        Serial.println("Failed to create timer!");
        return;
    }
    
    // Attach interrupt - only after successful timer creation
    timerAttachInterrupt(stepTimer, &PWMStepper::onStepTimer, true);
    
    timerAlarmWrite(stepTimer, halfPeriodUs, true);  // Set alarm value, auto-reload
    // Set up for continuous running
    timerStepState = false;
    timerRunning = true;
    timerStepsRemaining = 0; // 0 means continuous
    interruptCount = 0;
    
    // Set the current instance for the callback
    currentStepperInstance = this;
    
    timerAlarmEnable(stepTimer);  // Enable the alarm
}

void PWMStepper::stopTimerMode() {
    if (stepTimer != nullptr) {
        timerAlarmDisable(stepTimer);
        timerDetachInterrupt(stepTimer);  // Properly detach interrupt
        timerEnd(stepTimer);
        stepTimer = nullptr;
        timerRunning = false;
    }
}

// Static timer callback function
void IRAM_ATTR PWMStepper::onStepTimer() {
    // Safety check - ensure instance exists and timer is still running
    if (currentStepperInstance != nullptr && currentStepperInstance->timerRunning) {
        currentStepperInstance->handleStepTimer();
    }
}



// Timer callback function
void IRAM_ATTR PWMStepper::handleStepTimer() {
    // Additional safety check within the interrupt
    if (!timerRunning || stepTimer == nullptr) {
        return;
    }
    
    // Toggle step pin
    timerStepState = !timerStepState;
    digitalWrite(stepPin, !digitalRead(stepPin));
    interruptCount++;
    
    // If counting steps, decrement on rising edge
    if (timerStepsRemaining > 0 && timerStepState) {
        timerStepsRemaining--;
        if (timerStepsRemaining == 0) {
            timerRunning = false;
            if (stepTimer != nullptr) {
                timerAlarmDisable(stepTimer);
            }
        }
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
        stepTimerMode(steps, frequency, dir);
    }
}

// Move steps with automatic direction (positive = forward, negative = reverse)
void PWMStepper::moveSteps(int32_t steps, double frequency) {
    if (steps == 0) return;
    
    bool dir = steps > 0;
    uint32_t absSteps = abs(steps);
    
    step(absSteps, frequency, dir);
}

// Timer mode specific step function
void PWMStepper::stepTimerMode(uint32_t steps, double frequency, bool dir) {
    if (steps == 0) return;
    
    setDirection(dir);
    enable();
    
    // Calculate timer period in microseconds
    uint32_t halfPeriodUs = (uint32_t)(500000.0 / frequency);
    
    // Stop existing timer if running
    stopTimerMode();
    
    // Create and configure timer for precise step count
    stepTimer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1MHz), count up
    if (stepTimer == nullptr) {
        Serial.println("Failed to create timer for stepping!");
        return;
    }
    
    timerAttachInterrupt(stepTimer, &PWMStepper::onStepTimer, true);
    
    timerAlarmWrite(stepTimer, halfPeriodUs, true);
    
    // Set up for specific step count
    timerStepState = false;
    timerRunning = true;
    timerStepsRemaining = steps;
    currentStepperInstance = this;
    
    Serial.print("Timer stepping ");
    Serial.print(steps);
    Serial.print(" steps at ");
    Serial.print(frequency);
    Serial.print(" Hz (period: ");
    Serial.print(halfPeriodUs * 2);
    Serial.println(" μs)");
    
    timerAlarmEnable(stepTimer);
    
    // Wait for completion
    while (timerStepsRemaining > 0 && timerRunning) {
        delay(1);
    }
    
    // Clean up
    stopTimerMode();
    
    digitalWrite(stepPin, LOW); // Ensure pin is LOW
    Serial.println("Timer stepping completed");
}

// Check if timer step is complete
bool PWMStepper::isTimerStepComplete() const {
    return (timerStepsRemaining == 0 || !timerRunning);
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