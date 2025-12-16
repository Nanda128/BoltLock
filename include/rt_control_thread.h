/* ============================================================================
 * Real-Time Control Thread - Servo Position Control
 *
 * Implements a high-priority real-time control thread for responsive
 * servo motor position control.
 *
 * REAL-TIME CONTROL SYSTEM:
 * A dedicated high-priority thread handles servo commands asynchronously,
 * ensuring the main application remains responsive during servo movements.
 *
 * Key Features:
 * - High Priority Thread: Runs at priority 10 for real-time response
 * - Non-blocking Operation: Commands execute asynchronously
 * - Command Queue: Receives lock/unlock/position commands
 * - Feedback Queue: Reports completion and status
 * - Timeout Handling: Waits for servo to complete movement
 *
 * Thread Communication:
 * - Receives commands via command queue (lock/unlock/position)
 * - Sends feedback via feedback queue (completion, errors)
 * ============================================================================ */

#ifndef RT_CONTROL_THREAD_H
#define RT_CONTROL_THREAD_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* Control Thread Priority */
#define RT_CONTROL_THREAD_PRIORITY 10 // High priority for real-time control
#define RT_CONTROL_THREAD_STACK_SIZE 2048

/* Servo movement timing */
#define SERVO_MOVE_WAIT_MS 500 // Wait time for servo to reach position

/* Servo State */
typedef struct
{
    float current_position; // Current position (degrees)
    float target_position;  // Target position (degrees)
    bool in_motion;         // True if servo is moving
} servo_state_t;

/* Control Thread Statistics */
typedef struct
{
    uint32_t commands_processed; // Commands processed
    uint32_t moves_completed;    // Successful movements
} control_stats_t;

/**
 * Initialize the real-time control thread
 *
 * Creates the control thread with high priority and initializes
 * the PID controller and servo state.
 *
 * Returns: ESP_OK on success, error code on failure
 */
esp_err_t rt_control_thread_init(void);

/**
 * Get current servo position
 *
 * Thread-safe function to get the current servo position
 *
 * Returns: Current position in degrees
 */
float rt_control_get_position(void);

/**
 * Get servo state (thread-safe)
 *
 * Parameters:
 *   state - Buffer to receive servo state
 *
 * Returns: ESP_OK on success
 */
esp_err_t rt_control_get_state(servo_state_t *state);

/**
 * Get control statistics
 *
 * Parameters:
 *   stats - Buffer to receive statistics
 *
 * Returns: ESP_OK on success
 */
esp_err_t rt_control_get_stats(control_stats_t *stats);

/**
 * Emergency stop - immediately halt motion
 */
void rt_control_emergency_stop(void);

#endif // RT_CONTROL_THREAD_H
