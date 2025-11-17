# LED Status Indicator System

This system uses the ESP32-S3's built-in RGB LED (WS2812/NeoPixel) on pin 48 to provide visual feedback about the stepper motor system's state.

## Features

- **Automatic Status Detection**: Monitors all stepper motors and displays appropriate colors
- **Priority-Based Display**: Shows the most critical status when multiple motors have different states
- **Custom Colors**: Supports manual RGB control for custom status indication
- **Blinking Effects**: Optional blinking for attention-grabbing notifications
- **Rainbow Animation**: Built-in rainbow cycle for testing
- **Brightness Control**: Adjustable LED brightness (0-255)

## Status Color Mapping

| Status | Color | Description |
|--------|-------|-------------|
| `LED_IDLE` | Green | System idle, motors enabled and ready |
| `LED_MOVING` | Blue | Motor(s) actively moving |
| `LED_ERROR` | Red | Error detected (initialization failure, invalid commands) |
| `LED_STALL` | Yellow | Stall detected via StallGuard |
| `LED_INITIALIZING` | Cyan | System initializing |
| `LED_DISABLED` | Orange | Motor(s) disabled |
| `LED_WARNING` | Magenta | Warning condition |
| `LED_OFF` | Off | LED turned off |

## Priority System

When using `updateFromAllSteppers()`, the system displays the highest priority state:

```
HIGHEST: Stall Guard Active (Yellow) - Any motor stall detected
         ↓
         Motor Moving (Blue) - Any motor actively moving
         ↓
         Not Initialized (Cyan) - Any motor not initialized
         ↓
         Disabled (Orange) - All motors disabled
         ↓
LOWEST:  Idle (Green) - All motors ready but stationary
```

## Basic Usage

### 1. Initialization

```cpp
#include "LEDStatusIndicator.h"

// Create LED status indicator (pin 48 for ESP32-S3, brightness 50/255)
LEDStatusIndicator ledStatus(48, 50);

void setup() {
    // Initialize LED
    if (!ledStatus.begin()) {
        Serial.println("LED initialization failed!");
    }
}
```

### 2. Automatic Status Updates

```cpp
void loop() {
    // Update LED based on all stepper states (recommended)
    ledStatus.updateFromAllSteppers(stepperController);
    
    // Or monitor a specific stepper
    // ledStatus.updateFromStepper(stepperController, 0);
    
    // Required if using blink effects
    ledStatus.update();
}
```

### 3. Manual Status Control

```cpp
// Set specific status
ledStatus.setStatus(LED_MOVING);    // Blue - motor moving
ledStatus.setStatus(LED_ERROR);     // Red - error condition
ledStatus.setStatus(LED_IDLE);      // Green - ready

// Custom RGB colors
ledStatus.setRGB(255, 128, 0);      // Custom orange

// Turn off
ledStatus.turnOff();
```

### 4. Blinking Effects

```cpp
// Enable blinking (interval in milliseconds)
ledStatus.enableBlink(500);         // Blink every 500ms
ledStatus.setStatus(LED_ERROR);     // Red blinking error

// Call update() in loop() for blinking to work
void loop() {
    ledStatus.update();
}

// Disable blinking
ledStatus.disableBlink();
```

### 5. Brightness Control

```cpp
// Set brightness (0-255)
ledStatus.setBrightness(100);       // Medium brightness
ledStatus.setBrightness(255);       // Maximum brightness
ledStatus.setBrightness(10);        // Very dim
```

### 6. Rainbow Effect

```cpp
// Continuous rainbow cycle
for (int i = 0; i < 256; i++) {
    ledStatus.setRainbow(i);
    delay(10);
}
```

### 7. Test All Colors

```cpp
// Run complete test sequence (all colors + rainbow + blink)
ledStatus.test();
```

## Integration with Test System

The LED status indicator is automatically integrated with the test menu:

```
=== TEST MENU ===
13. LED Status Test        // Test all LED colors and effects
```

During motor tests, the LED automatically indicates:
- **Blue** when motors are moving
- **Green** when tests complete successfully
- **Yellow** if stall detected
- **Red** if errors occur

## Advanced Usage

### Error Handling Example

```cpp
void operationWithError() {
    ledStatus.setStatus(LED_ERROR);
    ledStatus.enableBlink(250);     // Fast blink for error
    
    Serial.println("ERROR: Operation failed!");
    
    // Wait for user acknowledgment
    while (Serial.available() == 0) {
        ledStatus.update();
        delay(10);
    }
    
    ledStatus.disableBlink();
    ledStatus.setStatus(LED_IDLE);
}
```

### Custom Status Notification

```cpp
void customNotification() {
    // Custom color (purple)
    ledStatus.setRGB(128, 0, 128);
    ledStatus.enableBlink(1000);    // Slow blink
    
    delay(5000);                    // Show for 5 seconds
    
    ledStatus.disableBlink();
    ledStatus.setStatus(LED_IDLE);
}
```

### Startup Sequence

```cpp
void setup() {
    Serial.begin(115200);
    
    // Initialize LED
    ledStatus.begin();
    ledStatus.setStatus(LED_INITIALIZING);  // Cyan during init
    
    // Initialize steppers
    stepperController.initializeAll();
    
    // Run LED test
    ledStatus.test();
    
    // Ready
    ledStatus.setStatus(LED_IDLE);          // Green when ready
}
```

## API Reference

### Constructor
```cpp
LEDStatusIndicator(uint8_t ledPin = 48, uint8_t ledBrightness = 50);
```

### Core Methods
- `bool begin()` - Initialize the LED (call in setup)
- `void setStatus(LEDStatus status)` - Set LED status
- `void updateFromStepper(HighFrequencyStepper& controller, uint8_t index)` - Update from specific stepper
- `void updateFromAllSteppers(HighFrequencyStepper& controller)` - Update from all steppers
- `void update()` - Update blink state (call in loop)

### Color Control
- `void setRGB(uint8_t r, uint8_t g, uint8_t b)` - Set custom RGB color
- `void setRainbow(uint8_t wheelPos)` - Set rainbow color (0-255)
- `void turnOff()` - Turn LED off

### Settings
- `void setBrightness(uint8_t brightness)` - Set brightness (0-255)
- `void enableBlink(uint16_t intervalMs = 500)` - Enable blinking
- `void disableBlink()` - Disable blinking

### Utility
- `void test()` - Run complete test sequence
- `LEDStatus getStatus()` - Get current status
- `uint8_t getBrightness()` - Get current brightness

## Hardware Requirements

- **ESP32-S3** with built-in RGB LED (WS2812) on pin 48
- Or any ESP32 with external WS2812/NeoPixel LED
- Library: Adafruit NeoPixel (automatically installed via platformio.ini)

## Pin Configuration

Default pin for ESP32-S3-DevKitM-1: **Pin 48**

For other boards or external LEDs, change the pin in initialization:
```cpp
LEDStatusIndicator ledStatus(YOUR_PIN, 50);
```

## Troubleshooting

**LED not lighting up:**
1. Check pin number matches your hardware (48 for ESP32-S3)
2. Verify `ledStatus.begin()` returns true
3. Try increasing brightness: `ledStatus.setBrightness(255)`
4. Test with: `ledStatus.test()`

**Wrong colors:**
1. Some LEDs use different color order (RGB vs GRB)
2. Modify NeoPixel initialization in `LEDStatusIndicator.cpp` if needed

**LED stays on one color:**
1. Make sure `ledStatus.update()` is called in loop()
2. Check if manual `setStatus()` is overriding automatic updates
3. Verify steppers are properly initialized

## Performance Notes

- LED updates every 100ms in automatic mode (configurable in main.cpp)
- Minimal CPU overhead (~0.1%)
- No impact on stepper timing or precision
- Blink update requires `update()` call in loop (every 10-50ms recommended)

## Examples

See `LEDStatusIndicator_Example.cpp.txt` for comprehensive usage examples.
