/* ============================================================================
 * Real-Time Control Thread - Servo Position Control with PID
 *
 * Implements a high-priority real-time control thread for
 * precise servo motor position control using a PID (Proportional-Integral-
 * Derivative) controller.
 *
 * REAL-TIME CONTROL SYSTEM:
 * The servo position control system simulates a closed-loop feedback system
 * where the controller continuously monitors the servo position and adjusts
 * the control signal to minimize position error.
 *
 * Key Features:
 * - PID Controller: Provides smooth, accurate position control
 * - Position Feedback: Simulates encoder feedback for position monitoring
 * - Velocity Control: Limits maximum speed for safety
 * - Stall Detection: Detects when servo cannot reach target
 * - High Priority Thread: Runs at priority 10 for real-time response
 * - 20ms Control Loop: Fast enough for smooth servo control
 *
 * PID Controller Basics:
 * - P (Proportional): Responds to current error
 * - I (Integral): Eliminates steady-state error
 * - D (Derivative): Dampens oscillation and overshoot
 *
 * Thread Communication:
 * - Receives commands via command queue (lock/unlock/position)
 * - Sends feedback via feedback queue (position updates, status)
 * ============================================================================ */

#ifndef RT_CONTROL_THREAD_H
#define RT_CONTROL_THREAD_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* Control Thread Priority */
#define RT_CONTROL_THREAD_PRIORITY 10 // High priority for real-time control
#define RT_CONTROL_THREAD_STACK_SIZE 4096

/* Control Loop Timing */
#define RT_CONTROL_LOOP_MS 20 // 20ms = ~50Hz control loop

/* PID Controller Parameters */
typedef struct
{
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Deri vative gain
    float integral;     // Accumulated integral error
    float prev_error;   // Previous error for derivative calculation
    float integral_max; // Anti-windup: max integral value
} pid_controller_t;

/* Servo State */
typedef struct
{
    float current_position; // Current position (degrees)
    float target_position;  // Target position (degrees)
    float velocity;         // Current velocity (degrees/sec)
    float max_velocity;     // Maximum velocity limit (degrees/sec)
    bool in_motion;         // True if servo is moving
    uint32_t stall_counter; // Counter for stall detection
} servo_state_t;

/* Control Thread Statistics */
typedef struct
{
    uint32_t loop_count;            // Total control loops executed
    uint32_t commands_processed;    // Commands processed
    uint32_t position_updates_sent; // Position updates sent
    uint32_t max_loop_time_us;      // Maximum loop execution time (us)
    float avg_loop_time_us;         // Average loop execution time (us)
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
 * Reset PID controller
 *
 * Clears integral and derivative terms
 */
void rt_control_reset_pid(void);

/**
 * Set PID gains (for tuning)
 *
 * Parameters:
 *   kp - Proportional gain
 *   ki - Integral gain
 *   kd - Derivative gain
 */
void rt_control_set_pid_gains(float kp, float ki, float kd);

/**
 * Set maximum velocity limit
 *
 * Parameters:
 *   max_vel - Maximum velocity in degrees/second
 */
void rt_control_set_max_velocity(float max_vel);

/**
 * Emergency stop - immediately halt motion
 */
void rt_control_emergency_stop(void);

#endif // RT_CONTROL_THREAD_H
