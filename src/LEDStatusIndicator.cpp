#include "LEDStatusIndicator.h"

// Constructor
LEDStatusIndicator::LEDStatusIndicator(uint8_t ledPin, uint8_t ledBrightness)
    : pin(ledPin), brightness(ledBrightness), currentStatus(LED_OFF),
      lastBlinkTime(0), blinkState(false), blinkInterval(0) {
    pixel = nullptr;
}

// Destructor
LEDStatusIndicator::~LEDStatusIndicator() {
    if (pixel != nullptr) {
        delete pixel;
    }
}

// Initialization
bool LEDStatusIndicator::begin() {
    // Create NeoPixel object (1 pixel on ESP32-S3)
    pixel = new Adafruit_NeoPixel(1, pin, NEO_GRB + NEO_KHZ800);
    
    if (pixel == nullptr) {
        Serial.println("ERROR: Failed to create NeoPixel object");
        return false;
    }
    
    pixel->begin();
    pixel->setBrightness(brightness);
    pixel->show(); // Initialize to off
    
    Serial.println("LED Status Indicator initialized on pin " + String(pin));
    return true;
}

// Get color for status
LEDStatusIndicator::Color LEDStatusIndicator::getColorForStatus(LEDStatus status) {
    switch (status) {
        case LED_IDLE:
            return Color(0, 255, 0);      // Green
        case LED_MOVING:
            return Color(0, 0, 255);      // Blue
        case LED_ERROR:
            return Color(255, 0, 0);      // Red
        case LED_STALL:
            return Color(255, 255, 0);    // Yellow
        case LED_INITIALIZING:
            return Color(0, 255, 255);    // Cyan
        case LED_DISABLED:
            return Color(255, 165, 0);    // Orange
        case LED_WARNING:
            return Color(255, 0, 255);    // Magenta
        case LED_OFF:
        default:
            return Color(0, 0, 0);        // Black (off)
    }
}

// Apply color to LED
void LEDStatusIndicator::setColor(Color color) {
    if (pixel != nullptr) {
        pixel->setPixelColor(0, pixel->Color(color.r, color.g, color.b));
        pixel->show();
    }
}

void LEDStatusIndicator::setColor(uint8_t r, uint8_t g, uint8_t b) {
    if (pixel != nullptr) {
        pixel->setPixelColor(0, pixel->Color(r, g, b));
        pixel->show();
    }
}

// Set status
void LEDStatusIndicator::setStatus(LEDStatus status) {
    currentStatus = status;
    Color color = getColorForStatus(status);
    setColor(color);
}

// Update from single stepper
void LEDStatusIndicator::updateFromStepper(HighFrequencyStepper& controller, uint8_t stepperIndex) {
    if (!controller.isValidIndex(stepperIndex)) {
        setStatus(LED_ERROR);
        return;
    }
    
    StepperStatus status = controller.getStatus(stepperIndex);
    
    // Priority: Error > Stall > Moving > Disabled > Idle
    if (status.stallGuard) {
        setStatus(LED_STALL);
    } else if (status.isMoving) {
        setStatus(LED_MOVING);
    } else if (!status.isEnabled) {
        setStatus(LED_DISABLED);
    } else if (!status.isInitialized) {
        setStatus(LED_INITIALIZING);
    } else {
        setStatus(LED_IDLE);
    }
}

// Update from all steppers (show highest priority state)
void LEDStatusIndicator::updateFromAllSteppers(HighFrequencyStepper& controller) {
    uint8_t stepperCount = controller.getStepperCount();
    
    if (stepperCount == 0) {
        setStatus(LED_OFF);
        return;
    }
    
    // Track states across all steppers
    bool anyStall = false;
    bool anyMoving = false;
    bool anyDisabled = false;
    bool anyNotInitialized = false;
    bool allIdle = true;
    
    for (uint8_t i = 0; i < stepperCount; i++) {
        StepperStatus status = controller.getStatus(i);
        
        if (status.stallGuard) {
            anyStall = true;
            allIdle = false;
        }
        if (status.isMoving) {
            anyMoving = true;
            allIdle = false;
        }
        if (!status.isEnabled) {
            anyDisabled = true;
        }
        if (!status.isInitialized) {
            anyNotInitialized = true;
            allIdle = false;
        }
    }
    
    // Priority: Stall > Moving > Initializing > Disabled > Idle
    if (anyStall) {
        setStatus(LED_STALL);
    } else if (anyMoving) {
        setStatus(LED_MOVING);
    } else if (anyNotInitialized) {
        setStatus(LED_INITIALIZING);
    } else if (anyDisabled) {
        setStatus(LED_DISABLED);
    } else if (allIdle) {
        setStatus(LED_IDLE);
    }
}

// Brightness control
void LEDStatusIndicator::setBrightness(uint8_t newBrightness) {
    brightness = newBrightness;
    if (pixel != nullptr) {
        pixel->setBrightness(brightness);
        pixel->show();
    }
}

// Blinking control
void LEDStatusIndicator::enableBlink(uint16_t intervalMs) {
    blinkInterval = intervalMs;
    lastBlinkTime = millis();
    blinkState = false;
}

void LEDStatusIndicator::disableBlink() {
    blinkInterval = 0;
    blinkState = false;
    // Restore current status color
    setStatus(currentStatus);
}

// Update blink state (call in loop)
void LEDStatusIndicator::update() {
    if (blinkInterval > 0) {
        unsigned long currentTime = millis();
        if (currentTime - lastBlinkTime >= blinkInterval) {
            lastBlinkTime = currentTime;
            blinkState = !blinkState;
            
            if (blinkState) {
                Color color = getColorForStatus(currentStatus);
                setColor(color);
            } else {
                setColor(0, 0, 0); // Off
            }
        }
    }
}

// Direct color control
void LEDStatusIndicator::setRGB(uint8_t r, uint8_t g, uint8_t b) {
    currentStatus = LED_OFF; // Mark as custom color
    setColor(r, g, b);
}

// Rainbow wheel effect
void LEDStatusIndicator::setRainbow(uint8_t wheelPos) {
    wheelPos = 255 - wheelPos;
    if (wheelPos < 85) {
        setRGB(255 - wheelPos * 3, 0, wheelPos * 3);
    } else if (wheelPos < 170) {
        wheelPos -= 85;
        setRGB(0, wheelPos * 3, 255 - wheelPos * 3);
    } else {
        wheelPos -= 170;
        setRGB(wheelPos * 3, 255 - wheelPos * 3, 0);
    }
}

// Turn off LED
void LEDStatusIndicator::turnOff() {
    setStatus(LED_OFF);
}

// Test cycle through all colors
void LEDStatusIndicator::test() {
    Serial.println("\n=== LED Status Test ===");
    
    LEDStatus states[] = {
        LED_IDLE, LED_MOVING, LED_ERROR, LED_STALL,
        LED_INITIALIZING, LED_DISABLED, LED_WARNING, LED_OFF
    };
    
    const char* names[] = {
        "IDLE (Green)", "MOVING (Blue)", "ERROR (Red)", "STALL (Yellow)",
        "INITIALIZING (Cyan)", "DISABLED (Orange)", "WARNING (Magenta)", "OFF"
    };
    
    for (int i = 0; i < 8; i++) {
        Serial.println(String("Testing: ") + names[i]);
        setStatus(states[i]);
        delay(1000);
    }
    
    // Rainbow test
    Serial.println("Testing: Rainbow cycle");
    for (int i = 0; i < 256; i++) {
        setRainbow(i);
        delay(10);
    }
    
    // Blink test
    Serial.println("Testing: Blink (5 seconds)");
    setStatus(LED_MOVING);
    enableBlink(250);
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        update();
        delay(10);
    }
    disableBlink();
    
    turnOff();
    Serial.println("=== LED Test Complete ===\n");
}
