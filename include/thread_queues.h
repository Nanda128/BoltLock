/* ============================================================================
 * Thread Queues - Bi-directional Communication Between Threads
 *
 * This module implements bi-directional queuing for inter-thread communication
 * between the control thread (high priority, real-time) and agent thread
 * (lower priority, event handling).
 *
 * Queue Architecture:
 * 1. Command Queue: Agent Thread -> Control Thread
 *    - Sends lock/unlock commands with target positions
 *    - High priority to ensure real-time response
 *
 * 2. Feedback Queue: Control Thread -> Agent Thread
 *    - Reports current position, state, errors
 *    - Allows agent to monitor control status
 *
 * Thread Model:
 * - Control Thread: Priority 10 (high) - Real-time servo control with PID
 * - Agent Thread: Priority 5 (normal) - State machine, buttons, network
 * ============================================================================ */

#ifndef THREAD_QUEUES_H
#define THREAD_QUEUES_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h>

/* Command Queue: Agent -> Control Thread */
typedef enum
{
    CMD_MOVE_TO_POSITION, // Move servo to specific position
    CMD_LOCK,             // Lock the door (move to locked position)
    CMD_UNLOCK,           // Unlock the door (move to unlocked position)
    CMD_STOP,             // Emergency stop
    CMD_CALIBRATE,        // Recalibrate servo positions
    CMD_GET_STATUS        // Request current status
} control_command_type_t;

typedef struct
{
    control_command_type_t type; // Command type
    float target_position;       // Target position in degrees (0-180)
    float speed_limit;           // Max speed (degrees/sec), 0 = no limit
    uint32_t timeout_ms;         // Command timeout in milliseconds
    void *user_data;             // Optional user data pointer
} control_command_t;

/* Feedback Queue: Control Thread -> Agent */
typedef enum
{
    FEEDBACK_COMMAND_COMPLETE, // Command completed successfully
    FEEDBACK_ERROR             // Error occurred
} feedback_type_t;

typedef struct
{
    feedback_type_t type;   // Feedback type
    float current_position; // Current servo position (degrees)
    float target_position;  // Target position (degrees)
    float error;            // Position error (degrees)
    uint32_t timestamp_ms;  // Timestamp of feedback
    esp_err_t error_code;   // Error code if applicable
    char message[64];       // Optional message
} control_feedback_t;

/* Queue Handles */
typedef struct
{
    QueueHandle_t command_queue;  // Agent -> Control
    QueueHandle_t feedback_queue; // Control -> Agent
} thread_queues_t;

/**
 * Initialize the bi-directional queue system
 *
 * Creates both command and feedback queues with appropriate sizes
 *
 * Returns: ESP_OK on success, error code on failure
 */
esp_err_t thread_queues_init(void);

/**
 * Get the queue handles structure
 *
 * Returns: Pointer to queue handles (NULL if not initialized)
 */
thread_queues_t *get_thread_queues(void);

/**
 * Send a command to the control thread (non-blocking)
 *
 * Parameters:
 *   cmd - Command to send
 *   timeout_ticks - Max ticks to wait for queue space
 *
 * Returns: ESP_OK if sent, ESP_ERR_TIMEOUT if queue full
 */
esp_err_t send_control_command(const control_command_t *cmd, uint32_t timeout_ticks);

/**
 * Send a command to the control thread (blocking)
 *
 * Parameters:
 *   cmd - Command to send
 *
 * Returns: ESP_OK if sent
 */
esp_err_t send_control_command_blocking(const control_command_t *cmd);

/**
 * Receive feedback from control thread (non-blocking)
 *
 * Parameters:
 *   feedback - Buffer to receive feedback
 *   timeout_ticks - Max ticks to wait for feedback
 *
 * Returns: ESP_OK if received, ESP_ERR_TIMEOUT if no feedback available
 */
esp_err_t receive_control_feedback(control_feedback_t *feedback, uint32_t timeout_ticks);

/**
 * Post feedback from control thread (called by control thread)
 *
 * Parameters:
 *   feedback - Feedback to send
 *   timeout_ticks - Max ticks to wait for queue space
 *
 * Returns: ESP_OK if sent
 */
esp_err_t post_control_feedback(const control_feedback_t *feedback, uint32_t timeout_ticks);

/**
 * Create a simple lock command
 *
 * Returns: Initialized lock command structure
 */
control_command_t create_lock_command(void);

/**
 * Create a simple unlock command
 *
 * Returns: Initialized unlock command structure
 */
control_command_t create_unlock_command(void);

/**
 * Create a position command
 *
 * Parameters:
 *   position - Target position in degrees
 *
 * Returns: Initialized position command structure
 */
control_command_t create_position_command(float position);

#endif // THREAD_QUEUES_H
