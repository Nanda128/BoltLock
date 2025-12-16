/* ============================================================================
 * Thread Queues - Implementation
 * 
 * "Implements bi-directional queuing between control and agent threads" - Mark Spec
 * 
 * Command Queue sends CMD_LOCK, CMD_UNLOCK, CMD_MOVE_TO_POSITION commands from the agent thread to the control thread.
 * Feedback Queue sends FEEDBACK_COMMAND_COMPLETE, FEEDBACK_ERROR messages from the control thread to the agent thread.
 * ============================================================================ */

#include "thread_queues.h"
#include "config.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "THREAD_QUEUES";

// Queue config
#define COMMAND_QUEUE_SIZE 10   // Can hold up to 10 commands
#define FEEDBACK_QUEUE_SIZE 20  // Can hold up to 20 feedback messages

// Queue handles
static thread_queues_t queues = {
    .command_queue = NULL,
    .feedback_queue = NULL
};

/* ----------------------------------------------------------------------------
 * thread_queues_init
 * 
 * Initialize both command and feedback queues
 * ---------------------------------------------------------------------------- */
esp_err_t thread_queues_init(void) {
    // THIS FUNCTION IMPLEMENTS QUEUES BETWEEN THREADS
    // HENCE, BIDIRECTIONAL QUEUEING MECHANISMS TO ALLOW DATA
    // TO BE TRANSFERRED BETWEEN THREADS
    ESP_LOGI(TAG, "Initializing bi-directional thread queues...");
    
    // SYSTEM BUFFERS COMMANDS AND FEEDBACK THROUGH FREERTOS QUEUES
    // THIS WAY, THEY CAN ACCUMULATE EVEN IF NETWORK IS DOWN
    
    // Create command queue (Agent thread -> Control thread)
    queues.command_queue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(control_command_t));
    if (queues.command_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create command queue!");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Command queue created (size: %d)", COMMAND_QUEUE_SIZE);
    
    // Create feedback queue (Control thread -> Agent thread)
    queues.feedback_queue = xQueueCreate(FEEDBACK_QUEUE_SIZE, sizeof(control_feedback_t));
    if (queues.feedback_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create feedback queue!");
        vQueueDelete(queues.command_queue);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "Feedback queue created (size: %d)", FEEDBACK_QUEUE_SIZE);
    
    ESP_LOGI(TAG, "Bi-directional queues initialized successfully");
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * get_thread_queues
 * 
 * Returns pointer to queue handles structure
 * ---------------------------------------------------------------------------- */
thread_queues_t* get_thread_queues(void) {
    return &queues;
}

/* ----------------------------------------------------------------------------
 * send_control_command
 * 
 * Send command to control thread (non-blocking with timeout)
 * ---------------------------------------------------------------------------- */
esp_err_t send_control_command(const control_command_t* cmd, uint32_t timeout_ticks) {
    if (queues.command_queue == NULL) {
        ESP_LOGE(TAG, "Command queue not initialized!");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (cmd == NULL) {
        ESP_LOGE(TAG, "NULL command pointer!");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Try to send command to queue
    if (xQueueSend(queues.command_queue, cmd, timeout_ticks) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full, command dropped");
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGD(TAG, "Command sent: type=%d, target=%.1f°", cmd->type, cmd->target_position);
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * send_control_command_blocking
 * 
 * Send command to control thread (blocking)
 * ---------------------------------------------------------------------------- */
esp_err_t send_control_command_blocking(const control_command_t* cmd) {
    return send_control_command(cmd, portMAX_DELAY);
}

/* ----------------------------------------------------------------------------
 * receive_control_feedback
 * 
 * Receive feedback from control thread
 * ---------------------------------------------------------------------------- */
esp_err_t receive_control_feedback(control_feedback_t* feedback, uint32_t timeout_ticks) {
    if (queues.feedback_queue == NULL) {
        ESP_LOGE(TAG, "Feedback queue not initialized!");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (feedback == NULL) {
        ESP_LOGE(TAG, "NULL feedback pointer!");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Try to receive feedback from queue
    if (xQueueReceive(queues.feedback_queue, feedback, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    ESP_LOGD(TAG, "Feedback received: type=%d, pos=%.1f°", 
            feedback->type, feedback->current_position);
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * post_control_feedback
 * 
 * Post feedback from control thread (called by control thread)
 * ---------------------------------------------------------------------------- */
esp_err_t post_control_feedback(const control_feedback_t* feedback, uint32_t timeout_ticks) {
    if (queues.feedback_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (feedback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Try to send feedback to queue
    if (xQueueSend(queues.feedback_queue, feedback, timeout_ticks) != pdTRUE) {
        // Feedback queue full - drop oldest message and try again
        control_feedback_t dummy;
        xQueueReceive(queues.feedback_queue, &dummy, 0);
        xQueueSend(queues.feedback_queue, feedback, 0);
    }
    
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * Helper Functions for Creating Commands
 * ---------------------------------------------------------------------------- */

control_command_t create_lock_command(void) {
    control_command_t cmd = {
        .type = CMD_LOCK,
        .target_position = SERVO_LOCKED_ANGLE,
        .speed_limit = 0.0f,  // No speed limit
        .timeout_ms = 5000,   // 5 second timeout
        .user_data = NULL
    };
    return cmd;
}

control_command_t create_unlock_command(void) {
    control_command_t cmd = {
        .type = CMD_UNLOCK,
        .target_position = SERVO_UNLOCKED_ANGLE,
        .speed_limit = 0.0f,  // No speed limit
        .timeout_ms = 5000,   // 5 second timeout
        .user_data = NULL
    };
    return cmd;
}

control_command_t create_position_command(float position) {
    control_command_t cmd = {
        .type = CMD_MOVE_TO_POSITION,
        .target_position = position,
        .speed_limit = 0.0f,  // No speed limit
        .timeout_ms = 5000,   // 5 second timeout
        .user_data = NULL
    };
    return cmd;
}
