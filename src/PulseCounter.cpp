#include "PulseCounter.h"
#include "esp_log.h"

// Static member initialization
int16_t PulseCounter::overflowCount = 0;
PulseCounter* PulseCounter::instance = nullptr;

// Constructor
PulseCounter::PulseCounter(pcnt_unit_t unit, uint8_t pulsePin, uint8_t ctrlPin) {
    this->pcntUnit = unit;
    this->pcntChannel = PCNT_CHANNEL_0;
    this->pulsePin = pulsePin;
    this->ctrlPin = ctrlPin;
    this->currentCount = 0;
    this->totalPosition = 0;
    this->lastPosition = 0;
    this->isInitialized = false;
    this->countingEnabled = false;
    
    // Set static instance for interrupt handling
    instance = this;
}

// Initialize the pulse counter
bool PulseCounter::begin() {
    // Configure PCNT unit
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pulsePin,
        .ctrl_gpio_num = ctrlPin,
        .lctrl_mode = PCNT_MODE_REVERSE,    // Reverse counting direction when control signal is low
        .hctrl_mode = PCNT_MODE_KEEP,       // Keep counting direction when control signal is high
        .pos_mode = PCNT_COUNT_INC,         // Count up on positive edge
        .neg_mode = PCNT_COUNT_DIS,         // Don't count on negative edge
        .counter_h_lim = 10000,             // High limit
        .counter_l_lim = -10000,            // Low limit
        .unit = pcntUnit,
        .channel = pcntChannel,
    };
    
    // Initialize PCNT unit
    esp_err_t err = pcnt_unit_config(&pcnt_config);

    // gpio_matrix_out(pulsePin, SIG_GPIO_OUT_IDX, false, false);      // Normal output
    // gpio_matrix_in(pulsePin, PCNT_SIG_CH0_IN0_IDX, false);         // Also route to PCNT input
    // gpio_matrix_out(ctrlPin, SIG_GPIO_OUT_IDX, false, false);      // Normal output  
    // gpio_matrix_in(ctrlPin, PCNT_CTRL_CH0_IN0_IDX, false);        // Also route to PCNT control    

    if (err != ESP_OK) {
        Serial.print("PCNT unit config failed: ");
        Serial.println(esp_err_to_name(err));
        return false;
    }
    
    // Set glitch filter (100 APB clock cycles)
    pcnt_set_filter_value(pcntUnit, 100);
    pcnt_filter_enable(pcntUnit);
    
    // Enable events on reaching limits
    pcnt_event_enable(pcntUnit, PCNT_EVT_H_LIM);
    pcnt_event_enable(pcntUnit, PCNT_EVT_L_LIM);
    
    // Initialize the counter value to zero
    pcnt_counter_clear(pcntUnit);
    
    isInitialized = true;
    
    Serial.println("PulseCounter initialized successfully!");
    Serial.print("Pulse Pin: GPIO"); Serial.println(pulsePin);
    Serial.print("Control Pin: GPIO"); Serial.println(ctrlPin);
    Serial.print("PCNT Unit: "); Serial.println(pcntUnit);
    
    return true;
}

// Start counting
void PulseCounter::start() {
    if (!isInitialized) return;
    
    pcnt_counter_clear(pcntUnit);
    pcnt_counter_resume(pcntUnit);
    countingEnabled = true;
    
    Serial.println("Pulse counting started");
}

// Stop counting
void PulseCounter::stop() {
    if (!isInitialized) return;
    
    pcnt_counter_pause(pcntUnit);
    countingEnabled = false;
    
    Serial.println("Pulse counting stopped");
}

// Reset counter
void PulseCounter::reset() {
    if (!isInitialized) return;
    
    pcnt_counter_clear(pcntUnit);
    currentCount = 0;
    totalPosition = 0;
    lastPosition = 0;
    overflowCount = 0;
    
    Serial.println("Pulse counter reset");
}

// Pause counting
void PulseCounter::pause() {
    if (!isInitialized) return;
    
    pcnt_counter_pause(pcntUnit);
    countingEnabled = false;
}

// Resume counting
void PulseCounter::resume() {
    if (!isInitialized) return;
    
    pcnt_counter_resume(pcntUnit);
    countingEnabled = true;
}

// Get raw count from PCNT unit
int16_t PulseCounter::getRawCount() {
    if (!isInitialized) return 0;
    
    pcnt_get_counter_value(pcntUnit, &currentCount);
    return currentCount;
}

// Get current position (including overflow handling)
int32_t PulseCounter::getPosition() {
    if (!isInitialized) return 0;
    
    int16_t count = getRawCount();
    totalPosition = (int32_t)overflowCount * 20000 + count;  // 20000 = high_lim - low_lim
    return totalPosition;
}

// Get absolute position since initialization
int32_t PulseCounter::getAbsolutePosition() {
    return getPosition();
}

// Set current position
void PulseCounter::setPosition(int32_t position) {
    totalPosition = position;
    lastPosition = position;
    overflowCount = position / 20000;
    int16_t remainder = position % 20000;
    
    pcnt_counter_clear(pcntUnit);
    // Note: We can't directly set PCNT counter value, so we adjust with overflowCount
    
    Serial.print("Position set to: "); Serial.println(position);
}

// Reset position to zero
void PulseCounter::resetPosition() {
    setPosition(0);
}

// Get direction based on control pin
bool PulseCounter::getDirection() {
    return digitalRead(ctrlPin) == HIGH;
}

// Get position change since last call
int32_t PulseCounter::getPositionChange() {
    int32_t currentPos = getPosition();
    int32_t change = currentPos - lastPosition;
    lastPosition = currentPos;
    return change;
}

// Set count mode for positive and negative edges
void PulseCounter::setCountMode(pcnt_count_mode_t pos_mode, pcnt_count_mode_t neg_mode) {
    if (!isInitialized) return;
    
    pcnt_set_mode(pcntUnit, pcntChannel, pos_mode, neg_mode, PCNT_MODE_KEEP, PCNT_MODE_REVERSE);
}

// Set control mode for high and low control signal
void PulseCounter::setCtrlMode(pcnt_ctrl_mode_t high_mode, pcnt_ctrl_mode_t low_mode) {
    if (!isInitialized) return;
    
    pcnt_set_mode(pcntUnit, pcntChannel, PCNT_COUNT_INC, PCNT_COUNT_DIS, high_mode, low_mode);
}

// Set counting limits
void PulseCounter::setLimits(int16_t low_limit, int16_t high_limit) {
    if (!isInitialized) return;
    
    pcnt_set_event_value(pcntUnit, PCNT_EVT_H_LIM, high_limit);
    pcnt_set_event_value(pcntUnit, PCNT_EVT_L_LIM, low_limit);
    
    Serial.print("Limits set: "); Serial.print(low_limit); 
    Serial.print(" to "); Serial.println(high_limit);
}

// Enable glitch filter
void PulseCounter::enableFilter(uint16_t filter_val) {
    if (!isInitialized) return;
    
    pcnt_set_filter_value(pcntUnit, filter_val);
    pcnt_filter_enable(pcntUnit);
    
    Serial.print("Filter enabled with value: "); Serial.println(filter_val);
}

// Disable glitch filter
void PulseCounter::disableFilter() {
    if (!isInitialized) return;
    
    pcnt_filter_disable(pcntUnit);
    Serial.println("Filter disabled");
}

// Check if counting is running
bool PulseCounter::isRunning() const {
    return countingEnabled;
}

// Check if counter is enabled
bool PulseCounter::isEnabled() const {
    return isInitialized;
}

// Print current status
void PulseCounter::printStatus() {
    if (!isInitialized) {
        Serial.println("PulseCounter not initialized");
        return;
    }
    
    Serial.println("=== PulseCounter Status ===");
    Serial.print("Raw Count: "); Serial.println(getRawCount());
    Serial.print("Position: "); Serial.println(getPosition());
    Serial.print("Direction: "); Serial.println(getDirection() ? "Forward" : "Reverse");
    Serial.print("Running: "); Serial.println(isRunning() ? "Yes" : "No");
    Serial.print("Overflow Count: "); Serial.println(overflowCount);
    Serial.println("===========================");
}

// Enable interrupts for overflow detection
void PulseCounter::enableInterrupt() {
    if (!isInitialized) return;
    
    // Install ISR service if not already installed
    pcnt_isr_service_install(0);
    
    // Add ISR handler for this unit
    pcnt_isr_handler_add(pcntUnit, pcnt_intr_handler, (void*)this);
    
    Serial.println("PCNT interrupt enabled");
}

// Disable interrupts
void PulseCounter::disableInterrupt() {
    if (!isInitialized) return;
    
    // Remove ISR handler for this unit
    pcnt_isr_handler_remove(pcntUnit);
    Serial.println("PCNT interrupt disabled");
}

// Calculate steps per second
int32_t PulseCounter::getStepsPerSecond(uint32_t timeWindow) {
    static uint32_t lastTime = 0;
    static int32_t lastPos = 0;
    
    uint32_t currentTime = millis();
    int32_t currentPos = getPosition();
    
    if (lastTime == 0) {
        lastTime = currentTime;
        lastPos = currentPos;
        return 0;
    }
    
    uint32_t deltaTime = currentTime - lastTime;
    if (deltaTime >= timeWindow) {
        int32_t deltaPos = currentPos - lastPos;
        int32_t stepsPerSec = (deltaPos * 1000) / deltaTime;
        
        lastTime = currentTime;
        lastPos = currentPos;
        
        return stepsPerSec;
    }
    
    return 0; // Not enough time elapsed
}

// Wait for a specific number of steps
void PulseCounter::waitForSteps(uint32_t steps, uint32_t timeout) {
    uint32_t startTime = millis();
    int32_t startPos = getPosition();
    int32_t targetPos = startPos + steps;
    
    while (millis() - startTime < timeout) {
        if (abs(getPosition() - targetPos) <= 1) {
            break;
        }
        delay(1);
    }
}

// Check if reached target position within tolerance
bool PulseCounter::hasReachedPosition(int32_t targetPosition, uint32_t tolerance) {
    return abs(getPosition() - targetPosition) <= tolerance;
}

// Interrupt handler for overflow detection
void IRAM_ATTR PulseCounter::pcnt_intr_handler(void* arg) {
    // Get interrupt status for our PCNT unit
    pcnt_evt_type_t evt_type = PCNT_EVT_H_LIM;
    
    // Check if our unit triggered the interrupt
    if (instance != nullptr) {
        pcnt_unit_t unit = instance->pcntUnit;
        
        // Read current counter value to determine event type
        int16_t count;
        pcnt_get_counter_value(unit, &count);
        
        // Check for high limit overflow
        if (count >= 9999) {  // Near high limit
            overflowCount++;
            pcnt_counter_clear(unit);
        }
        // Check for low limit overflow  
        else if (count <= -9999) {  // Near low limit
            overflowCount--;
            pcnt_counter_clear(unit);
        }
    }
}