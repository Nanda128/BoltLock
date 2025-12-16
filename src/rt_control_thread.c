/* ============================================================================
 * Real-Time Control Thread - Implementation
 * 
 * Implements a PID-based servo position control system running in a high-
 * priority thread with real-time constraints.
 * ============================================================================ */

#include "rt_control_thread.h"
#include "thread_queues.h"
#include "lock_control.h"
#include "config.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char* TAG = "RT_CONTROL";

#define DEFAULT_KP 2.0f
#define DEFAULT_KI 0.5f
#define DEFAULT_KD 0.1f
#define INTEGRAL_MAX 50.0f

#define POSITION_TOLERANCE 1.0f     // Acceptable error tolerance (degrees)
#define STALL_THRESHOLD 100         // Stall detection threshold (loops)
#define MAX_VELOCITY_DEFAULT 90.0f  // Default max velocity (deg/sec)
#define POSITION_UPDATE_INTERVAL 10 // Send position update every N loops

#define SERVO_RESPONSE_TIME_MS 500  // Time for servo to reach target
#define FEEDBACK_NOISE 0.5f         // Simulated position noise (degrees)

static pid_controller_t pid;
static servo_state_t servo_state;
static control_stats_t stats;
static SemaphoreHandle_t state_mutex = NULL;
static TaskHandle_t control_task_handle = NULL;
static bool thread_running = false;

/* ----------------------------------------------------------------------------
 * pid_init
 * 
 * Initialize PID controller with default gains
 * ---------------------------------------------------------------------------- */
static void pid_init(void) {
    pid.kp = DEFAULT_KP;
    pid.ki = DEFAULT_KI;
    pid.kd = DEFAULT_KD;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.integral_max = INTEGRAL_MAX;
}

/* ----------------------------------------------------------------------------
 * pid_compute
 * 
 * Compute PID control output
 * 
 * Parameters:
 *   error - Current position error (target - current)
 *   dt - Time step (seconds)
 * 
 * Returns: Control output (velocity in degrees/sec)
 * ---------------------------------------------------------------------------- */
static float pid_compute(float error, float dt) {
    // Proportional term
    float p_term = pid.kp * error;
    
    // Integral term with anti-windup
    pid.integral += error * dt;
    if (pid.integral > pid.integral_max) {
        pid.integral = pid.integral_max;
    } else if (pid.integral < -pid.integral_max) {
        pid.integral = -pid.integral_max;
    }
    float i_term = pid.ki * pid.integral;
    
    // Derivative term
    float d_term = 0.0f;
    if (dt > 0.0001f) {
        d_term = pid.kd * (error - pid.prev_error) / dt;
    }
    pid.prev_error = error;
    
    // Total output
    float output = p_term + i_term + d_term;
    
    return output;
}

/* ----------------------------------------------------------------------------
 * pid_reset
 * 
 * Reset PID controller state
 * ---------------------------------------------------------------------------- */
static void pid_reset(void) {
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
}

/* ============================================================================
 * Servo Simulation Functions
 * ============================================================================ */

/* ----------------------------------------------------------------------------
 * simulate_servo_response
 * 
 * Simulates servo movement with realistic dynamics
 * 
 * This simulates a first-order servo response with velocity limiting
 * 
 * Parameters:
 *   current - Current position
 *   target - Target position
 *   velocity_cmd - Commanded velocity from PID
 *   dt - Time step (seconds)
 * 
 * Returns: New position after time step
 * ---------------------------------------------------------------------------- */
static float simulate_servo_response(float current, float target, 
                                    float velocity_cmd, float dt) {
    // Calculate position error
    float error = target - current;
    
    // Limit velocity command
    float max_vel = servo_state.max_velocity;
    if (velocity_cmd > max_vel) {
        velocity_cmd = max_vel;
    } else if (velocity_cmd < -max_vel) {
        velocity_cmd = -max_vel;
    }
    
    // Calculate position change
    float position_change = velocity_cmd * dt;
    
    // Don't overshoot target
    if (fabsf(position_change) > fabsf(error)) {
        position_change = error;
    }
    
    // Update position
    float new_position = current + position_change;
    
    // Add small noise to simulate encoder feedback
    float noise = ((float)(esp_random() % 100) / 100.0f - 0.5f) * FEEDBACK_NOISE;
    new_position += noise;
    
    // stop at range
    if (new_position < 0.0f) new_position = 0.0f;
    if (new_position > 180.0f) new_position = 180.0f;
    
    // update velocity
    servo_state.velocity = position_change / dt;
    
    return new_position;
}

/* ============================================================================
 * Control Thread Functions
 * ============================================================================ */

/* ----------------------------------------------------------------------------
 * send_position_feedback
 * 
 * Send position update feedback to agent thread
 * ---------------------------------------------------------------------------- */
static void send_position_feedback(void) {
    control_feedback_t feedback;
    memset(&feedback, 0, sizeof(feedback));
    
    feedback.type = FEEDBACK_POSITION_UPDATE;
    feedback.current_position = servo_state.current_position;
    feedback.target_position = servo_state.target_position;
    feedback.error = servo_state.target_position - servo_state.current_position;
    feedback.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    
    post_control_feedback(&feedback, 0);
    stats.position_updates_sent++;
}

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
 * send_stall_feedback
 * 
 * Send stall detection feedback
 * ---------------------------------------------------------------------------- */
static void send_stall_feedback(void) {
    control_feedback_t feedback;
    memset(&feedback, 0, sizeof(feedback));
    
    feedback.type = FEEDBACK_STALL_DETECTED;
    feedback.current_position = servo_state.current_position;
    feedback.target_position = servo_state.target_position;
    feedback.error = servo_state.target_position - servo_state.current_position;
    feedback.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
    feedback.error_code = ESP_ERR_TIMEOUT;
    snprintf(feedback.message, sizeof(feedback.message), 
             "Stall detected at %.1f°", servo_state.current_position);
    
    post_control_feedback(&feedback, 0);
}

/* ----------------------------------------------------------------------------
 * process_control_command
 * 
 * Process incoming control command
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
            servo_state.stall_counter = 0;
            
            if (cmd->speed_limit > 0.0f) {
                servo_state.max_velocity = cmd->speed_limit;
            } else {
                servo_state.max_velocity = MAX_VELOCITY_DEFAULT;
            }
            
            // Reset PID for new command
            pid_reset();
            
            // Update physical servo
            set_lock_position(cmd->target_position);
            break;
            
        case CMD_STOP:
            servo_state.target_position = servo_state.current_position;
            servo_state.in_motion = false;
            pid_reset();
            break;
            
        case CMD_GET_STATUS:
            send_position_feedback();
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command type: %d", cmd->type);
            break;
    }
    
    xSemaphoreGive(state_mutex);
    stats.commands_processed++;
}

/* ----------------------------------------------------------------------------
 * control_loop
 * 
 * Main control loop - runs at fixed rate
 * ---------------------------------------------------------------------------- */
static void control_loop(void) {
    const float dt = RT_CONTROL_LOOP_MS / 1000.0f;  // Time step in seconds
    static uint32_t loop_counter = 0;
    
    uint64_t start_time_us = esp_timer_get_time();
    
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    
    // Calculate position error
    float error = servo_state.target_position - servo_state.current_position;
    
    // Check if we've reached target
    if (fabsf(error) < POSITION_TOLERANCE) {
        if (servo_state.in_motion) {
            // Just reached target
            ESP_LOGI(TAG, "Target reached: %.1f° (error: %.2f°)", 
                     servo_state.current_position, error);
            servo_state.in_motion = false;
            send_command_complete_feedback();
        }
        servo_state.stall_counter = 0;
    } else if (servo_state.in_motion) {
        // Still moving - compute PID output
        float velocity_cmd = pid_compute(error, dt);
        
        // Simulate servo response
        servo_state.current_position = simulate_servo_response(
            servo_state.current_position,
            servo_state.target_position,
            velocity_cmd,
            dt
        );
        
        // Stall detection
        if (fabsf(servo_state.velocity) < 1.0f && fabsf(error) > POSITION_TOLERANCE) {
            servo_state.stall_counter++;
            if (servo_state.stall_counter >= STALL_THRESHOLD) {
                ESP_LOGW(TAG, "Servo stall detected!");
                send_stall_feedback();
                servo_state.in_motion = false;
                servo_state.stall_counter = 0;
            }
        } else {
            servo_state.stall_counter = 0;
        }
        
        // Send periodic position updates
        if (loop_counter % POSITION_UPDATE_INTERVAL == 0) {
            send_position_feedback();
        }
    }
    
    xSemaphoreGive(state_mutex);
    
    // Update statistics
    uint64_t end_time_us = esp_timer_get_time();
    uint32_t loop_time_us = (uint32_t)(end_time_us - start_time_us);
    
    if (loop_time_us > stats.max_loop_time_us) {
        stats.max_loop_time_us = loop_time_us;
    }
    
    stats.avg_loop_time_us = (stats.avg_loop_time_us * stats.loop_count + loop_time_us) 
                             / (stats.loop_count + 1);
    stats.loop_count++;
    
    loop_counter++;
}

/* ----------------------------------------------------------------------------
 * rt_control_task
 * 
 * Real-time control task - runs at high priority
 * ---------------------------------------------------------------------------- */
static void rt_control_task(void* arg) {
    thread_queues_t* queues = get_thread_queues();
    control_command_t cmd;
    
    ESP_LOGI(TAG, "Real-time control thread started (priority: %d)", 
             RT_CONTROL_THREAD_PRIORITY);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (thread_running) {
        // Check for incoming commands (non-blocking)
        if (xQueueReceive(queues->command_queue, &cmd, 0) == pdTRUE) {
            process_control_command(&cmd);
        }
        
        // Execute control loop
        control_loop();
        
        // Wait for next control cycle (fixed rate)
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RT_CONTROL_LOOP_MS));
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
    
    // Initialize PID controller
    pid_init();
    
    // Initialize servo state
    memset(&servo_state, 0, sizeof(servo_state));
    servo_state.current_position = SERVO_LOCKED_ANGLE;  // Start locked
    servo_state.target_position = SERVO_LOCKED_ANGLE;
    servo_state.max_velocity = MAX_VELOCITY_DEFAULT;
    servo_state.in_motion = false;
    servo_state.stall_counter = 0;
    
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
    ESP_LOGI(TAG, "  Control loop: %d ms (%d Hz)", RT_CONTROL_LOOP_MS, 
             1000 / RT_CONTROL_LOOP_MS);
    ESP_LOGI(TAG, "  PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f", 
             pid.kp, pid.ki, pid.kd);
    
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
 * rt_control_reset_pid
 * ---------------------------------------------------------------------------- */
void rt_control_reset_pid(void) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    pid_reset();
    xSemaphoreGive(state_mutex);
}

/* ----------------------------------------------------------------------------
 * rt_control_set_pid_gains
 * ---------------------------------------------------------------------------- */
void rt_control_set_pid_gains(float kp, float ki, float kd) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
    xSemaphoreGive(state_mutex);
    
    ESP_LOGI(TAG, "PID gains updated: Kp=%.2f, Ki=%.2f, Kd=%.2f", kp, ki, kd);
}

/* ----------------------------------------------------------------------------
 * rt_control_set_max_velocity
 * ---------------------------------------------------------------------------- */
void rt_control_set_max_velocity(float max_vel) {
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    servo_state.max_velocity = max_vel;
    xSemaphoreGive(state_mutex);
    
    ESP_LOGI(TAG, "Max velocity set to: %.1f deg/sec", max_vel);
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
