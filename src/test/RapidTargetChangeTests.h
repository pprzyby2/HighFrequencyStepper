#ifndef RAPIDTARGETCHANGETESTS_H
#define RAPIDTARGETCHANGETESTS_H

#include <Arduino.h>
#include "HighFrequencyStepper.h"

/**
 * @brief Test rapid target position changes while motor is moving
 * 
 * This test simulates scenarios where the target position is rapidly changed
 * while the stepper is moving, such as:
 * - User quickly adjusting a slider/encoder
 * - Tracking a fast-moving target
 * - Rapid direction reversals
 * 
 * @param stepper Reference to HighFrequencyStepper controller
 */
void testRapidTargetChanges(HighFrequencyStepper& stepper);

/**
 * @brief Test rapid back-and-forth oscillation
 * 
 * Tests the stepper's ability to handle rapid reversals by oscillating
 * between two positions with decreasing intervals.
 * 
 * @param stepper Reference to HighFrequencyStepper controller
 */
void testRapidOscillation(HighFrequencyStepper& stepper);

/**
 * @brief Test chasing a continuously changing target
 * 
 * Simulates tracking a moving target by continuously updating
 * the target position while the motor is running.
 * 
 * @param stepper Reference to HighFrequencyStepper controller
 */
void testChasingTarget(HighFrequencyStepper& stepper);

/**
 * @brief Test long-duration run for stability
 * 
 * Tests the stepper's ability to maintain accuracy and stability
 * over an extended period of continuous operation.
 * 
 * @param stepper Reference to HighFrequencyStepper controller
 */
void testLongRun(HighFrequencyStepper& stepper);

#endif // RAPIDTARGETCHANGETESTS_H
