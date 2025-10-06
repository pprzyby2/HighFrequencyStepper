#include "PulseCounter.h"
#include "esp_log.h"
#include "soc/pcnt_struct.h"  // For PCNT register access

#define PCNT_H_LIM_VAL      INT16_MAX
#define PCNT_L_LIM_VAL      INT16_MIN

static pcnt_isr_handle_t s_isr_handle = NULL;
// Static member initialization - support for 4 pulse counters
PulseCounter* PulseCounter::instances[PCNT_UNIT_MAX] = {nullptr};

// Static variables for interrupt handling - one for each PCNT unit
static portMUX_TYPE s_mutex[PCNT_UNIT_MAX] = {
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED
};
static int32_t s_totalPosition[PCNT_UNIT_MAX] = {0, 0, 0, 0, 0, 0, 0, 0};

// Constructor
PulseCounter::PulseCounter(pcnt_unit_t unit, uint8_t pulsePin, uint8_t ctrlPin) {
    this->pcntUnit = unit;
    this->pcntChannel = PCNT_CHANNEL_0;
    this->pulsePin = pulsePin;
    this->ctrlPin = ctrlPin;
    this->lastPosition = 0;
    this->isInitialized = false;
    this->countingEnabled = false;
    
    // Register this instance for the specific PCNT unit
    if (unit < PCNT_UNIT_MAX) {
        instances[unit] = this;
    }
}

// Initialize the pulse counter
bool PulseCounter::begin(bool invertDirection) {
    // Configure PCNT unit
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pulsePin,
        .ctrl_gpio_num = ctrlPin,
        .lctrl_mode = invertDirection ? PCNT_MODE_KEEP : PCNT_MODE_REVERSE,    // Reverse counting direction when control signal is low
        .hctrl_mode = invertDirection ? PCNT_MODE_REVERSE : PCNT_MODE_KEEP,       // Keep counting direction when control signal is high
        .pos_mode = PCNT_COUNT_INC,         // Count up on positive edge
        .neg_mode = PCNT_COUNT_DIS,         // Don't count on negative edge
        .counter_h_lim = PCNT_H_LIM_VAL,             // High limit
        .counter_l_lim = PCNT_L_LIM_VAL,            // Low limit
        .unit = pcntUnit,
        .channel = pcntChannel,
    };    

    // Initialize PCNT unit
    esp_err_t err = pcnt_unit_config(&pcnt_config);

    if (err != ESP_OK) {
        Serial.print("PCNT unit config failed: ");
        Serial.println(esp_err_to_name(err));
        return false;
    }

    // Configure GPIO pins mapping to PCNT
    //gpio_matrix_out(pulsePin, SIG_GPIO_OUT_IDX, false, false);      // Normal output
    //gpio_matrix_in(pulsePin, PCNT_SIG_CH0_IN0_IDX, false);         // Also route to PCNT input
    gpio_matrix_out(ctrlPin, SIG_GPIO_OUT_IDX, false, false);      // Normal output  
    gpio_matrix_in(ctrlPin, PCNT_CTRL_CH0_IN0_IDX, false);        // Also route to PCNT control        
    
    // Set glitch filter (100 APB clock cycles)
    pcnt_set_filter_value(pcntUnit, 100);
    pcnt_filter_enable(pcntUnit);
    
    // Enable events on reaching limits
    pcnt_event_enable(pcntUnit, PCNT_EVT_H_LIM);
    pcnt_event_enable(pcntUnit, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(pcntUnit);
    pcnt_counter_clear(pcntUnit);    
    
    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &s_isr_handle);
    pcnt_intr_enable(pcntUnit);
    
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
    portENTER_CRITICAL(&s_mutex[pcntUnit]);
    s_totalPosition[pcntUnit] = 0;
    portEXIT_CRITICAL(&s_mutex[pcntUnit]);
    lastPosition = 0;
    
    Serial.print("Pulse counter unit ");
    Serial.print(pcntUnit);
    Serial.println(" reset");
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
    int16_t currentCount;
    pcnt_get_counter_value(pcntUnit, &currentCount);
    return currentCount;
}

// Get current position (including overflow handling)
int32_t PulseCounter::getPosition() {
    if (!isInitialized) return 0;

    int16_t count = getRawCount();
    portENTER_CRITICAL(&s_mutex[pcntUnit]);
    int32_t position = s_totalPosition[pcntUnit] + count;
    portEXIT_CRITICAL(&s_mutex[pcntUnit]);
    return position;
}

// Get absolute position since initialization
int32_t PulseCounter::getAbsolutePosition() {
    return getPosition();
}

// Set current position
void PulseCounter::setPosition(int32_t position) {
    pcnt_counter_clear(pcntUnit);
    portENTER_CRITICAL(&s_mutex[pcntUnit]);
    s_totalPosition[pcntUnit] = position;
    portEXIT_CRITICAL(&s_mutex[pcntUnit]);
    lastPosition = position;
    
    Serial.print("Unit ");
    Serial.print(pcntUnit);
    Serial.print(" position set to: ");
    Serial.println(position);
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
    Serial.println("===========================");
}

// Enable interrupts for overflow detection
void PulseCounter::enableInterrupt() {
    // if (!isInitialized) return;
    
    // // Install ISR service if not already installed
    // pcnt_isr_service_install(0);
    
    // // Add ISR handler for this unit
    // pcnt_isr_handler_add(pcntUnit, pcnt_intr_handler, (void*)this);
    
    // Serial.println("PCNT interrupt enabled");
}

// Disable interrupts
void PulseCounter::disableInterrupt() {
    if (!isInitialized) return;
    
    // Remove ISR handler for this unit
    pcnt_isr_handler_remove(pcntUnit);
    Serial.println("PCNT interrupt disabled");
}

// Calculate steps per second
double PulseCounter::getStepsPerSecond(uint32_t timeWindow) {
    static uint32_t lastTime = 0;
    static int32_t lastPos = 0;
    
    uint32_t currentTime = millis();
    int32_t currentPos = getPosition();
    
    if (lastTime == 0) {
        lastTime = currentTime;
        lastPos = currentPos;
        return 0;
    }
    
    int32_t deltaTime = currentTime - lastTime;
    if (deltaTime >= timeWindow) {
        int32_t deltaPos = currentPos - lastPos;
        double stepsPerSec = (deltaPos * 1000.0) / deltaTime;
        
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

// Interrupt handler for overflow detection - handles all PCNT units
void IRAM_ATTR pcnt_intr_handler(void* arg) {
    uint32_t intr_status = PCNT.int_st.val;
    int unit;
    uint32_t status; // information on the event type that caused the interrupt
    
    for (unit = 0; unit < PCNT_UNIT_MAX; unit++) {
        if (intr_status & (BIT(unit))) {
            pcnt_get_event_status((pcnt_unit_t)unit, &status);
            PCNT.int_clr.val = BIT(unit);
            
            // Handle overflow for the specific unit
            if (PulseCounter::instances[unit] != nullptr) {
                portENTER_CRITICAL(&s_mutex[unit]);
                if (status & PCNT_EVT_H_LIM) { 
                    s_totalPosition[unit] += PCNT_H_LIM_VAL; 
                } 
                if (status & PCNT_EVT_L_LIM) { 
                    s_totalPosition[unit] += PCNT_L_LIM_VAL; 
                } 
                portEXIT_CRITICAL(&s_mutex[unit]);
            }
        }
    }
}