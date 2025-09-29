#include "PWMStepper.h"

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
    
    Serial.println("PWMStepper initialized successfully!");
    Serial.print("Step Pin: "); Serial.println(stepPin);
    Serial.print("Dir Pin: "); Serial.println(dirPin);
    Serial.print("Enable Pin: "); Serial.println(enablePin);
    Serial.print("LEDC Channel: "); Serial.println(ledcChannel);
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
void PWMStepper::disable() {
    digitalWrite(enablePin, HIGH); // Active LOW
    stopPWM();  // Also stop any running PWM
}

// Start PWM at specified frequency (steps per second)
void PWMStepper::startPWM(uint32_t frequency) {
    if (frequency == 0) {
        stopPWM();
        return;
    }
    
    // Update LEDC frequency
    ledcFrequency = frequency;
    currentFreq = frequency;
    
    // Reconfigure LEDC with new frequency
    ledcSetup(ledcChannel, ledcFrequency, ledcResolution);
    ledcAttachPin(stepPin, ledcChannel);
    
    // Set 50% duty cycle for square wave
    uint32_t dutyCycle = (1 << ledcResolution) / 2;  // 50% duty cycle
    ledcWrite(ledcChannel, dutyCycle);
    
    isRunning = true;
    
    Serial.print("PWM started at "); 
    Serial.print(frequency); 
    Serial.println(" Hz");
}

// Stop PWM
void PWMStepper::stopPWM() {
    ledcWrite(ledcChannel, 0);  // 0% duty cycle
    digitalWrite(stepPin, LOW); // Ensure pin is LOW
    isRunning = false;
    currentFreq = 0;
    
    Serial.println("PWM stopped");
}

// Set frequency while running
void PWMStepper::setFrequency(uint32_t frequency) {
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

uint32_t PWMStepper::getFrequency() const {
    return currentFreq;
}

// Move a specific number of steps
void PWMStepper::step(uint32_t steps, uint32_t frequency, bool dir) {
    if (steps == 0) return;
    
    setDirection(dir);
    enable();
    startPWM(frequency);
    
    // Calculate duration in milliseconds
    uint32_t duration = (steps * 1000) / frequency;
    
    Serial.print("Stepping "); 
    Serial.print(steps); 
    Serial.print(" steps at "); 
    Serial.print(frequency); 
    Serial.print(" Hz for "); 
    Serial.print(duration); 
    Serial.println(" ms");
    
    delay(duration);
    stopPWM();
}

// Move steps with automatic direction (positive = forward, negative = reverse)
void PWMStepper::moveSteps(int32_t steps, uint32_t frequency) {
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