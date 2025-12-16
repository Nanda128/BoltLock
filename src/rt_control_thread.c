/* ============================================================================
 * Real-Time Control Thread - Implementation
 * 
 * Implements a high-priority servo control thread for responsive,
 * non-blocking servo commands.
 * ============================================================================ */

#include "rt_control_thread.h"
#include "thread_queues.h"
#include "lock_control.h"
#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <string.h>

static const char* TAG = "RT_CONTROL";

static servo_state_t servo_state;
static control_stats_t stats;
static SemaphoreHandle_t state_mutex = NULL;
static TaskHandle_t control_task_handle = NULL;
static bool thread_running = false;

/*
 * ============================================================================
 * Explaining the Real-Time aspect
 * 
 * Servo position tracked through servo_state_t structure which
 * tracks current_position, target_position, and in_motion
 * 
 * Commands are recieved from command queue and executes 
 * the servo movements immediately
 * 
 * Feedback is sent back via feedback queue
 * ============================================================================
*/

/* ----------------------------------------------------------------------------
 * send_command_complete_feedback
 * 
 * Send command completion feedback
 * ---------------------------------------------------------------------------- */
static void send_command_complete_feedback(void) {
    control_feedback_t feedback;
    memset(&feedback, 0, sizeof(feedback));
    
    feedback.type = FEEDBACK_COMMAND_COMPLETE;
    feedback.current_position = servo_state.current_position;
    feedback.target_position = servo_state.target_position;
    feedback.error = 0.0f;
    feedback.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    snprintf(feedback.message, sizeof(feedback.message), 
            "Position reached: %.1f°", servo_state.current_position);
    
    post_control_feedback(&feedback, 0);
}

/* ----------------------------------------------------------------------------
 * send_error_feedback
 * 
 * Send error feedback
 * ---------------------------------------------------------------------------- */
static void send_error_feedback(const char* message, esp_err_t error_code) {
    control_feedback_t feedback;
    memset(&feedback, 0, sizeof(feedback));
    
    feedback.type = FEEDBACK_ERROR;
    feedback.current_position = servo_state.current_position;
    feedback.target_position = servo_state.target_position;
    feedback.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    feedback.error_code = error_code;
    snprintf(feedback.message, sizeof(feedback.message), "%s", message);
    
    post_control_feedback(&feedback, 0);
}


/* ----------------------------------------------------------------------------
 * process_control_command
 * 
 * Process incoming control command and execute servo movement
 * ---------------------------------------------------------------------------- */
static void process_control_command(const control_command_t* cmd) {
    ESP_LOGI(TAG, "Processing command: type=%d, target=%.1f°", 
            cmd->type, cmd->target_position);
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    
    switch (cmd->type) {
        case CMD_LOCK:
        case CMD_UNLOCK:
        case CMD_MOVE_TO_POSITION:
            servo_state.target_position = cmd->target_position;
            servo_state.in_motion = true;
            
            // Update physical servo immediately
            esp_err_t ret = set_lock_position(cmd->target_position);
            
            if (ret == ESP_OK) {
                // Wait for servo to reach position
                xSemaphoreGive(state_mutex);  // Release mutex during delay
                vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_WAIT_MS));
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                
                // Update state
                servo_state.current_position = cmd->target_position;
                servo_state.in_motion = false;
                
                // Send completion feedback
                send_command_complete_feedback();
                stats.moves_completed++;
                
                ESP_LOGI(TAG, "Servo moved to %.1f°", servo_state.current_position);
            } else {
                servo_state.in_motion = false;
                xSemaphoreGive(state_mutex);  // Release before feedback
                send_error_feedback("Failed to move servo", ret);
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                ESP_LOGE(TAG, "Failed to move servo: %s", esp_err_to_name(ret));
            }
            break;
            
        case CMD_STOP:
            servo_state.in_motion = false;
            ESP_LOGI(TAG, "Stop command received");
            break;
            
        case CMD_GET_STATUS:
            // Send current status
            send_command_complete_feedback();
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command type: %d", cmd->type);
            break;
    }
    
    xSemaphoreGive(state_mutex);
    stats.commands_processed++;
}


/* ----------------------------------------------------------------------------
 * rt_control_task
 * 
 * Real-time control task - processes commands at high priority
 * ---------------------------------------------------------------------------- */
static void rt_control_task(void* arg) {
    thread_queues_t* queues = get_thread_queues();
    control_command_t cmd;
    
    ESP_LOGI(TAG, "Real-time control thread started (priority: %d)", 
            RT_CONTROL_THREAD_PRIORITY);
    
    while (thread_running) {
        // Wait for incoming commands (blocking with timeout)
        if (xQueueReceive(queues->command_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            process_control_command(&cmd);
        }
    }
    
    ESP_LOGI(TAG, "Control thread stopped");
    vTaskDelete(NULL);
}


/* ============================================================================
 * Public API Functions
 * ============================================================================ */

/* ----------------------------------------------------------------------------
 * rt_control_thread_init
 * ---------------------------------------------------------------------------- */
esp_err_t rt_control_thread_init(void) {
    ESP_LOGI(TAG, "Initializing real-time control thread...");
    
    // Create mutex for state protection
    state_mutex = xSemaphoreCreateMutex();
    if (state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex!");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize servo state
    memset(&servo_state, 0, sizeof(servo_state));
    servo_state.current_position = SERVO_LOCKED_ANGLE;  // Start locked
    servo_state.target_position = SERVO_LOCKED_ANGLE;
    servo_state.in_motion = false;
    
    // Initialize statistics
    memset(&stats, 0, sizeof(stats));
    
    // Start control thread
    thread_running = true;
    if (xTaskCreate(rt_control_task, "rt_control", 
                    RT_CONTROL_THREAD_STACK_SIZE, NULL, 
                    RT_CONTROL_THREAD_PRIORITY, 
                    &control_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task!");
        vSemaphoreDelete(state_mutex);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Real-time control thread initialized successfully");
    
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * rt_control_get_position
 * ---------------------------------------------------------------------------- */
float rt_control_get_position(void) {
    float position;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    position = servo_state.current_position;
    xSemaphoreGive(state_mutex);
    return position;
}

/* ----------------------------------------------------------------------------
 * rt_control_get_state
 * ---------------------------------------------------------------------------- */
esp_err_t rt_control_get_state(servo_state_t* state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // Monitors servo position through servo_state_t which tracks
    // current_position(up in rt_control_get_position())
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    memcpy(state, &servo_state, sizeof(servo_state_t));
    xSemaphoreGive(state_mutex);
    
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * rt_control_get_stats
 * ---------------------------------------------------------------------------- */
esp_err_t rt_control_get_stats(control_stats_t* out_stats) {
    if (out_stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(out_stats, &stats, sizeof(control_stats_t));
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * rt_control_emergency_stop
 * ---------------------------------------------------------------------------- */
void rt_control_emergency_stop(void) {
    control_command_t cmd = {
        .type = CMD_STOP,
        .target_position = 0,
        .speed_limit = 0,
        .timeout_ms = 0,
        .user_data = NULL
    };
    
    send_control_command(&cmd, 0);
    ESP_LOGW(TAG, "EMERGENCY STOP commanded!");
}
