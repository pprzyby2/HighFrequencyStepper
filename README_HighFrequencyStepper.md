# HighFrequencyStepper Class

A comprehensive stepper motor controller class that integrates PWMStepper, PulseCounter, and TMC2209Stepper to provide high-performance stepper motor control with position feedback and advanced TMC features.

## Features

- **Multi-stepper support**: Control up to 4 stepper motors simultaneously
- **Dual-mode operation**: Automatic switching between LEDC (high frequency) and Timer (low frequency) modes
- **Position feedback**: Real-time position tracking using pulse counters
- **TMC2209 integration**: Advanced stepper driver features like StallGuard, temperature monitoring, and current control
- **Configuration management**: Structured configuration for easy setup and management
- **Safety features**: Emergency stop, stall detection, and comprehensive error handling
- **Self-testing**: Built-in diagnostic functions
- **Serial control**: Interactive control via serial commands

## Classes Integration

### PWMStepper
- Provides dual-mode PWM generation (LEDC/Timer)
- Automatic frequency-based mode selection (≥512 Hz = LEDC, <512 Hz = Timer)
- High-frequency capability (up to ~625 kHz with auto-resolution adjustment)

### PulseCounter
- Real-time position tracking using ESP32 PCNT peripheral
- Direction-aware counting
- Overflow handling for extended range

### TMC2209Stepper
- Silent stepper operation with StealthChop
- Configurable microsteps (1-256)
- Current control and thermal protection
- StallGuard for sensorless homing

## Configuration Structure

```cpp
struct StepperConfig {
    // Pin assignments
    uint8_t stepPin;         // Step signal pin
    uint8_t dirPin;          // Direction pin
    uint8_t enablePin;       // Enable pin (can be shared)
    uint8_t stepCountPin;    // Pulse counter pin (usually same as stepPin)
    uint8_t txPin;           // UART TX for TMC communication
    uint8_t rxPin;           // UART RX for TMC communication
    
    // TMC2209 configuration
    uint8_t driverAddress;   // TMC address (0b00, 0b01, 0b10, 0b11)
    float rSense;            // Current sense resistor (typically 0.11Ω)
    
    // Motor parameters
    uint16_t microsteps;     // Microsteps per full step (1, 2, 4, 8, 16, 32, 64, 128, 256)
    uint16_t rmsCurrent;     // RMS current in mA
    uint16_t stepsPerRev;    // Steps per revolution (typically 200)
    double maxFrequency;     // Maximum frequency in Hz
    
    // Hardware assignments
    uint8_t ledcChannel;     // LEDC channel (0-15)
    pcnt_unit_t pcntUnit;    // PCNT unit (PCNT_UNIT_0 to PCNT_UNIT_7)
};
```

## Basic Usage

### 1. Include Headers
```cpp
#include "HighFrequencyStepper.h"
```

### 2. Create Instance
```cpp
HighFrequencyStepper stepperController;
```

### 3. Configure Steppers
```cpp
// Configuration for Stepper 0
StepperConfig config0;
config0.stepPin = 19;
config0.dirPin = 21;
config0.enablePin = 23;
config0.stepCountPin = 22;
config0.txPin = 17;
config0.rxPin = 16;
config0.driverAddress = 0b00;
config0.rSense = 0.11f;
config0.microsteps = 16;
config0.rmsCurrent = 800;
config0.maxFrequency = 100000;
config0.ledcChannel = 0;
config0.pcntUnit = PCNT_UNIT_0;

// Add stepper to controller
stepperController.addStepper(0, config0);
```

### 4. Initialize and Enable
```cpp
void setup() {
    stepperController.initializeAll();
    stepperController.enableAll();
    
    // Perform self-test
    if (stepperController.selfTestAll()) {
        Serial.println("All steppers ready!");
    }
}
```

### 5. Control Motors
```cpp
// Move to absolute position
stepperController.moveToPosition(0, 1000, 10000); // stepper 0, position 1000, 10kHz

// Move relative steps
stepperController.moveRelative(0, 500, 5000); // stepper 0, 500 steps, 5kHz

// Start continuous movement
stepperController.startContinuous(0, 1000, true); // stepper 0, 1kHz, forward

// Stop movement
stepperController.stop(0);
```

## API Reference

### Initialization Methods
- `bool addStepper(uint8_t index, const StepperConfig& config)` - Add stepper configuration
- `bool initializeStepper(uint8_t index)` - Initialize specific stepper
- `bool initializeAll()` - Initialize all configured steppers

### Configuration Methods
- `bool setMicrosteps(uint8_t index, uint16_t microsteps)` - Set microsteps
- `bool setRMSCurrent(uint8_t index, uint16_t currentMA)` - Set RMS current
- `bool setMaxFrequency(uint8_t index, double frequency)` - Set maximum frequency

### Movement Methods
- `bool moveToPosition(uint8_t index, int32_t position, double frequency = 0)` - Move to absolute position
- `bool moveRelative(uint8_t index, int32_t steps, double frequency = 0)` - Move relative steps
- `bool startContinuous(uint8_t index, double frequency, bool direction = true)` - Start continuous movement
- `bool stop(uint8_t index)` - Stop specific stepper
- `bool stopAll()` - Stop all steppers
- `bool emergencyStop()` - Emergency stop all

### Position and Status Methods
- `int32_t getPosition(uint8_t index)` - Get current position
- `bool isMoving(uint8_t index)` - Check if stepper is moving
- `bool isAtPosition(uint8_t index, int32_t tolerance = 1)` - Check if at target position
- `StepperStatus getStatus(uint8_t index)` - Get comprehensive status

### Enable/Disable Methods
- `bool enableStepper(uint8_t index)` - Enable specific stepper
- `bool disableStepper(uint8_t index)` - Disable specific stepper
- `bool enableAll()` - Enable all steppers
- `bool disableAll()` - Disable all steppers

### Diagnostic Methods
- `void printStatus(uint8_t index)` - Print status for one stepper
- `void printAllStatus()` - Print status for all steppers
- `bool selfTest(uint8_t index)` - Self-test specific stepper
- `bool selfTestAll()` - Self-test all steppers

### Advanced TMC Features
- `bool enableStallGuard(uint8_t index, uint8_t threshold = 10)` - Enable stall detection
- `bool isStallDetected(uint8_t index)` - Check for stall condition
- `float getTemperature(uint8_t index)` - Get driver temperature

## Serial Commands

When using the example code, you can control steppers via serial commands:

- `status` - Show all stepper status
- `move <stepper> <position> <frequency>` - Move stepper to position
- `stop` - Stop all steppers
- `enable` - Enable all steppers
- `disable` - Disable all steppers
- `test` - Run self-test on all steppers
- `zero` - Zero all positions
- `help` - Show command help

### Example Commands
```
move 0 1000 10000    // Move stepper 0 to position 1000 at 10kHz
move 1 -500 5000     // Move stepper 1 to position -500 at 5kHz
status               // Show all stepper status
stop                 // Stop all steppers
```

## Pin Configuration Examples

### Single Stepper Setup
```cpp
StepperConfig config;
config.stepPin = 19;        // Step signal
config.dirPin = 21;         // Direction
config.enablePin = 23;      // Enable (active LOW)
config.stepCountPin = 22;   // Pulse counter (separate pin recommended)
config.txPin = 17;          // TMC UART TX
config.rxPin = 16;          // TMC UART RX
```

### Multi-Stepper Setup
```cpp
// Stepper 0
config0.stepPin = 19; config0.dirPin = 21; config0.enablePin = 23;
config0.txPin = 17; config0.rxPin = 16; config0.driverAddress = 0b00;

// Stepper 1
config1.stepPin = 18; config1.dirPin = 5; config1.enablePin = 23; // Shared enable
config1.txPin = 25; config1.rxPin = 26; config1.driverAddress = 0b01;

// Stepper 2 & 3 can share UART with different addresses
config2.txPin = 17; config2.rxPin = 16; config2.driverAddress = 0b10; // Same UART as stepper 0
config3.txPin = 25; config3.rxPin = 26; config3.driverAddress = 0b11; // Same UART as stepper 1
```

## Performance Characteristics

### Frequency Ranges
- **Timer Mode**: 1 Hz - 511 Hz (precise, low frequencies)
- **LEDC Mode**: 512 Hz - 625 kHz (high frequencies with auto-resolution)

### Position Accuracy
- **32-bit position tracking** with pulse counters
- **±1 step accuracy** with proper mechanical setup
- **Real-time feedback** for closed-loop control

### Update Rates
- **Position updates**: ~1 kHz typical
- **Status monitoring**: ~10 Hz typical
- **Stall detection**: Real-time via TMC interrupt

## Error Handling

The class provides comprehensive error handling:
- Parameter validation for all methods
- TMC communication error detection
- Position tracking error detection
- Timeout handling for movements
- Emergency stop functionality

## Memory Usage

- **RAM**: ~500 bytes per stepper (includes buffers and status)
- **Flash**: ~15 KB for the complete class
- **PCNT Units**: 1 per stepper (8 total available)
- **LEDC Channels**: 1 per stepper (16 total available)
- **UART Ports**: Shared among steppers with different addresses

## Example Projects

See `example_HighFrequencyStepper.cpp.example` for a comprehensive example showing:
- Multi-stepper configuration
- Interactive serial control
- Automated demonstration sequences
- Error handling and diagnostics
- Self-testing procedures

## Troubleshooting

### Common Issues
1. **TMC Communication Error**: Check UART pins and baud rate
2. **Position Drift**: Verify pulse counter pin connections
3. **High Frequency Issues**: Check LEDC resolution auto-adjustment
4. **Stall Detection**: Tune StallGuard threshold for your motor

### Debug Output
Enable detailed debug output by calling `printAllStatus()` regularly to monitor:
- Position tracking accuracy
- TMC driver status
- Movement completion
- Error conditions

## Future Enhancements

- Trajectory planning and smooth acceleration
- Synchronized multi-axis movements
- Configuration save/load from EEPROM
- Web interface for remote control
- Integration with other motion control protocols