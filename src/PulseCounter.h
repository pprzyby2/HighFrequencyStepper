#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include <Arduino.h>
#include "driver/pcnt.h"

void IRAM_ATTR pcnt_intr_handler(void* arg);

class PulseCounter {
private:
    pcnt_unit_t pcntUnit;
    pcnt_channel_t pcntChannel;
    uint8_t pulsePin;
    uint8_t ctrlPin;
    
    int32_t lastPosition;
    bool isInitialized;
    bool countingEnabled;
    
public:
    // Static array to hold instances for interrupt handling
    static PulseCounter* instances[PCNT_UNIT_MAX];
    
    // Constructor
    PulseCounter(pcnt_unit_t unit = PCNT_UNIT_0, uint8_t pulsePin = GPIO_NUM_19, uint8_t ctrlPin = GPIO_NUM_21);
    
    // Initialize the pulse counter
    bool begin();
    
    // Control functions
    void start();
    void stop();
    void reset();
    void pause();
    void resume();
    
    // Position tracking
    int16_t getRawCount();
    int32_t getPosition();
    int32_t getAbsolutePosition();
    void setPosition(int32_t position);
    void resetPosition();
    
    // Direction tracking
    bool getDirection();
    int32_t getPositionChange();
    
    // Configuration
    void setCountMode(pcnt_count_mode_t pos_mode, pcnt_count_mode_t neg_mode);
    void setCtrlMode(pcnt_ctrl_mode_t high_mode, pcnt_ctrl_mode_t low_mode);
    void setLimits(int16_t low_limit, int16_t high_limit);
    void enableFilter(uint16_t filter_val);
    void disableFilter();
    
    // Status
    bool isRunning() const;
    bool isEnabled() const;
    void printStatus();
    
    // Advanced features
    void enableInterrupt();
    void disableInterrupt();
    double getStepsPerSecond(uint32_t timeWindow = 1000);
    
    // Utility functions
    void waitForSteps(uint32_t steps, uint32_t timeout = 10000);
    bool hasReachedPosition(int32_t targetPosition, uint32_t tolerance = 1);
};

#endif // PULSECOUNTER_H