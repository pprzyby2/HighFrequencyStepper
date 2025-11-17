# Manual Motor Control Guide

## Overview

The manual motor control mode provides real-time interactive control of stepper motors through serial commands. This is useful for:
- Testing motor configurations
- Calibration and setup
- Manual positioning
- Speed testing
- Debugging motor issues

## Accessing Manual Control

From the main test menu, select option **14**:
```
=== TEST MENU ===
...
14. Manual Motor Control
...
```

## Manual Control Commands

### Motor Control
| Command | Description |
|---------|-------------|
| `e` | **Enable** stepper motor |
| `d` | **Disable** stepper motor |
| `s` | **Start/Resume** movement at current speed and direction |
| `x` | **Stop** movement (motor remains enabled) |

### Direction Control
| Command | Description |
|---------|-------------|
| `r` | **Reverse** direction (toggles CW ↔ CCW) |

### Speed Control
| Command | Description |
|---------|-------------|
| `+` | **Increase** speed by 10 Hz |
| `-` | **Decrease** speed by 10 Hz |
| `*` | **Increase** speed by 100 Hz (fast) |
| `/` | **Decrease** speed by 100 Hz (fast) |

### Multi-Stepper Control
| Command | Description |
|---------|-------------|
| `n` | Select **next** stepper (cycles through all configured steppers) |

### Status & Utility
| Command | Description |
|---------|-------------|
| `p` | Show **position** and detailed status |
| `z` | **Zero** position counter |
| `h` or `?` | Show **help** menu |
| `q` | **Quit** manual control mode and return to test menu |

### TMC2209 Driver Controls
*(Only available when controlling a TMC2209-equipped stepper)*

| Command | Description |
|---------|-------------|
| `c` | **Increase** RMS current by 50 mA |
| `C` | **Decrease** RMS current by 50 mA |
| `t` | **Toggle** between StealthChop (quiet) and SpreadCycle (high speed) |
| `g` | Set **StallGuard** threshold (stall detection sensitivity) |
| `m` | Change **microsteps** (1, 2, 4, 8, 16, 32, 64, 128, 256) |
| `i` | Show detailed **TMC2209 driver info** |

## Typical Workflow

### 1. Enter Manual Control Mode
```
Enter test number: 14
✓ Entering Manual Control Mode
```

### 2. Enable Motor
```
Enter command: e
✓ Stepper TMC2209 ENABLED
```

### 3. Start Movement
```
Enter command: s
✓ Moving at 100.00 Hz Forward
```

### 4. Adjust Speed
```
Enter command: +
✓ Speed increased to 110.00 Hz

Enter command: *
✓ Speed increased to 210.00 Hz
```

### 5. Change Direction
```
Enter command: r
✓ Direction changed to Reverse (CCW)
```

### 6. Stop Motor
```
Enter command: x
✓ Motor STOPPED
```

### 7. Exit Manual Control
```
Enter command: q
✓ Exiting manual control mode
```

## Status Display

The manual control menu shows real-time status:

```
=== MANUAL MOTOR CONTROL ===
Stepper: TMC2209 (Index 0)
Speed: 100.00 Hz (360.00 RPM)
Direction: Forward (CW)
Status: ENABLED
Moving: YES
Position: 12345 (123.45 degrees)
```

## LED Status Indicator

During manual control, the RGB LED shows:
- 🟢 **Green** - Motor enabled, idle
- 🔵 **Blue** - Motor moving
- 🟠 **Orange** - Motor disabled
- 🔴 **Red** - Error condition

## Speed Limits

- **Minimum Speed**: 1 Hz
- **Maximum Speed**: Configured `maxRPM` for each stepper
- Speed automatically limited to safe ranges
- Real-time feedback if limits reached

## Multi-Stepper Support

If multiple steppers are configured:
1. Use `n` command to cycle through steppers
2. Current stepper shown in status display
3. Previous stepper automatically stops when switching
4. Each stepper maintains independent settings

## Safety Features

- Motor must be **enabled** before starting movement
- Automatic speed limiting to `maxRPM`
- Emergency stop via `x` command
- All motors stop when exiting manual control

## Advanced Usage Examples

### TMC2209 Current Tuning
```
14         # Enter manual control
i          # Show current TMC2209 settings
c          # Increase RMS current (motor gets stronger but hotter)
c          # Increase again
e          # Enable motor
s          # Start and test
x          # Stop
```

### StealthChop vs SpreadCycle Testing
```
14         # Enter manual control
t          # Switch to StealthChop (quiet mode, good for low/medium speed)
e          # Enable
s          # Start at 100 Hz - should be very quiet
*          # Increase speed
*          # Keep increasing - may lose steps if too fast in StealthChop
x          # Stop
t          # Switch to SpreadCycle (high speed mode, louder)
s          # Start - louder but can handle high speeds
```

### StallGuard Configuration
```
14         # Enter manual control
g          # Set StallGuard threshold
10         # Enter value (0-10 = very sensitive, good for detecting obstacles)
e          # Enable motor
s          # Start moving
           # Motor will stop if it encounters resistance
i          # Check if stall was detected
```

### Microstep Adjustment
```
14         # Enter manual control
m          # Change microsteps
256        # Set to 256 for smoothest operation (but slower max speed)
e          # Enable
s          # Test smooth operation
m          # Change again
16         # Set to 16 for faster operation (but less smooth)
s          # Test faster operation
```

### Calibration Workflow
```
e          # Enable motor
s          # Start at default speed (100 Hz)
+          # Fine-tune speed up
-          # Fine-tune speed down
z          # Zero position at desired location
x          # Stop
```

### High-Speed Testing
```
e          # Enable
s          # Start
*          # Increase by 100 Hz
*          # Increase by 100 Hz
*          # Increase by 100 Hz
p          # Check current speed and status
```

### Direction Testing
```
e          # Enable
s          # Start forward
r          # Reverse to backward
r          # Back to forward
x          # Stop
```

### Multi-Motor Setup
```
e          # Enable stepper 0
s          # Start stepper 0
n          # Switch to stepper 1
e          # Enable stepper 1
s          # Start stepper 1
n          # Switch to stepper 2
e          # Enable stepper 2
s          # Start stepper 2
```

## Troubleshooting

### TMC2209 Specific Issues

#### Motor Not Responding After Current Change
1. Check if current is too low: `i` to see current setting
2. Increase current: `c` several times
3. Typical values: 500-1000 mA for NEMA17, 1000-2000 mA for NEMA23

#### Motor Skipping Steps at High Speed
1. Switch to SpreadCycle mode: `t`
2. StealthChop is limited to ~200-300 RPM for most motors
3. SpreadCycle can handle much higher speeds

#### StallGuard Not Detecting Stalls
1. Check threshold with `i`
2. Lower threshold = more sensitive: `g` then enter 5-20
3. Test by manually blocking the motor

#### Motor Too Hot
1. Check current setting: `i`
2. Reduce current: `C` several times
3. Enable CoolStep (automatic current reduction when idle)

### Motor Not Moving
1. Check if motor is enabled: press `e`
2. Verify status with `p` command
3. Check speed is above 0 Hz
4. Ensure motor is connected properly

### Speed Changes Not Responding
- Speed changes apply immediately if motor is moving
- If stopped, speed is set but motor won't move until `s` pressed

### Position Drift
- Use `z` command to zero position counter
- Check encoder/pulse counter connections
- Verify `encoderToMicrostepRatio` in configuration

### LED Showing Error (Red)
- Check serial output for error messages
- Stop all motors with `x`
- Exit and re-enter manual control mode
- Verify motor configuration in `setup()`

## Technical Details

### TMC2209 Features

#### StealthChop Mode
- **Quiet operation** - nearly silent at low/medium speeds
- **Current reduction** - automatic when idle
- **Speed limit** - typically 200-300 RPM maximum
- **Best for**: Low speed, quiet operation, battery-powered applications

#### SpreadCycle Mode
- **High speed** - can handle maximum motor speeds
- **More torque** - better holding torque
- **Louder** - audible motor noise
- **Best for**: High speed applications, maximum performance

#### Current Settings
- **Minimum**: 100 mA (very weak, usually too low)
- **Typical NEMA17**: 500-1000 mA
- **Typical NEMA23**: 1000-2000 mA
- **Maximum**: 2000 mA (safety limit in software)
- **Formula**: Peak current = RMS current × 1.414

#### StallGuard Threshold
- **Range**: 0-255
- **0-10**: Very sensitive (detects light resistance)
- **50-100**: Moderate (normal operation)
- **200+**: Low sensitivity (only heavy loads)
- **-1**: Disabled (no stall detection)

#### Microsteps
- **Higher values** (128, 256): Smoother, quieter, lower max speed
- **Lower values** (2, 4, 8): Faster, more torque, noisier
- **Typical**: 16 or 32 for most applications
- **Ultra-smooth**: 256 for precision positioning

### Default Settings
- **Initial Speed**: 100 Hz
- **Initial Direction**: Forward (CW)
- **Speed Increment**: 10 Hz (fine), 100 Hz (coarse)
- **Active Stepper**: Index 0 (first configured stepper)

### Real-Time Updates
- Speed changes apply immediately during movement
- Direction changes stop motor briefly, then resume
- LED updates every 100ms
- Position updates continuously

### Command Processing
- Case-insensitive commands (e.g., `E` or `e`)
- Buffered input (press Enter after command)
- Invalid commands show error message
- Help always available with `h` or `?`

## Integration with Test Suite

Manual control complements the automated test suite:
- Use manual control for **exploratory testing**
- Use automated tests for **repeatable validation**
- Manual control helps identify optimal test parameters
- Position zeroing useful before running automated tests

## Best Practices

1. **Always enable before starting**: Prevents accidental movement commands
2. **Use fine adjustments first**: Start with `+/-` before `*/`
3. **Zero position regularly**: Maintain accurate position tracking
4. **Monitor status**: Use `p` to verify motor state
5. **Stop before switching**: Motor auto-stops, but explicit stop is clearer
6. **Exit properly**: Use `q` to ensure clean exit and motor stop

## Future Enhancements

Potential additions:
- [ ] Acceleration profile control
- [ ] Move to absolute position
- [ ] Incremental jogging (single steps)
- [ ] Speed presets (slow/medium/fast)
- [ ] Macro recording/playback
- [ ] Multi-motor synchronized control
- [ ] PID tuning interface
- [ ] Save/load motor configurations

## See Also

- `README_LEDStatusIndicator.md` - LED color meanings and control
- `HighFrequencyStepper.h` - Full API reference
- Test menu options 1-13 for automated testing
