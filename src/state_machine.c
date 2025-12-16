/* ============================================================================
 * State Machine - Manages Lock/Unlock Logic and Auto-Lock Timer
 * 
 * States:
 * - LOCKED: Door is locked
 * - UNLOCKING: Door is in process of unlocking
 * - UNLOCKED: Door is unlocked
 * - ERROR: Something went wrong
 * 
 * Events that trigger state changes:
 * - Button press (physical button)
 * - Timeout (auto-lock timer expires)
 * - Remote unlock (via WiFi/MQTT)
 * - Remote lock (via WiFi/MQTT)
 * ============================================================================ */

#include "state_machine.h"
#include "lock_control.h"
#include "event_logger.h"
#include "network_handler.h"
#include "thread_queues.h"
#include "config.h"
#include "buzzer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char* TAG = "STATE_MACHINE";

static QueueHandle_t sm_event_queue = NULL;

static TimerHandle_t auto_lock_timer = NULL;

/* ----------------------------------------------------------------------------
 * auto_lock_timer_callback
 * 
 * Called automatically when the auto-lock timer expires
 * Sends a TIMEOUT event to the state machine, which will lock the door
 * ---------------------------------------------------------------------------- */
static void auto_lock_timer_callback(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Auto-lock timer expired");
    
    sm_event_data_t event = {
        .event = SM_EVENT_TIMEOUT,
        .data = NULL
    };
    
    xQueueSend(sm_event_queue, &event, 0);
}

/* ----------------------------------------------------------------------------
 * handle_locked_state
 * 
 * Handles events when door is in LOCKED state
 * 
 * Possible actions:
 * - Button press -> Unlock door
 * - Remote unlock command -> Unlock door
 * - Other events -> Ignore
 * ---------------------------------------------------------------------------- */
static void handle_locked_state(sm_event_data_t* event) {
    // Check which event happened
    if (event->event == SM_EVENT_BUTTON_PRESS) {
        ESP_LOGI(TAG, "Button pressed in LOCKED state");
        
        control_command_t cmd = create_unlock_command();
        send_control_command_blocking(&cmd);
        
        unlock_door();              
        buzzer_play_unlock();      // bzzzt beepbeep 
        log_event(EVENT_BUTTON_PRESS, "Physical button unlock");  
        
        if (is_mqtt_connected()) {
            mqtt_publish(MQTT_TOPIC_STATUS, "{\"state\":\"unlocked\",\"method\":\"button\"}");
            mqtt_publish(MQTT_TOPIC_EVENTS, "{\"event\":\"unlock\",\"trigger\":\"button\"}");
        }
        
        if (auto_lock_timer != NULL) {
            xTimerStart(auto_lock_timer, 0);
        }
    } 
    else if (event->event == SM_EVENT_REMOTE_UNLOCK) {
        ESP_LOGI(TAG, "Remote unlock command received");
        
        control_command_t cmd = create_unlock_command();
        send_control_command_blocking(&cmd);
        
        unlock_door();
        buzzer_play_unlock();       // bzzt beepbeep
        log_event(EVENT_REMOTE_UNLOCK, "Remote unlock via network");
        
        if (is_mqtt_connected()) {
            mqtt_publish(MQTT_TOPIC_STATUS, "{\"state\":\"unlocked\",\"method\":\"remote\"}");
            mqtt_publish(MQTT_TOPIC_EVENTS, "{\"event\":\"unlock\",\"trigger\":\"remote\"}");
        }
        
        if (auto_lock_timer != NULL) {
            xTimerStart(auto_lock_timer, 0);
        }
    }
}

/* ----------------------------------------------------------------------------
 * handle_unlocked_state
 * 
 * Handles events when door is in UNLOCKED state
 * 
 * Possible actions:
 * - Timeout -> Auto-lock door
 * - Button press -> Manually lock door
 * - Remote lock command -> Lock door
 * ---------------------------------------------------------------------------- */
static void handle_unlocked_state(sm_event_data_t* event) {
    if (event->event == SM_EVENT_TIMEOUT) {
        ESP_LOGI(TAG, "Auto-lock timeout");
        
        control_command_t cmd = create_lock_command();
        send_control_command_blocking(&cmd);
        
        lock_door();
        buzzer_play_lock();         // bzzt beep
        log_event(EVENT_LOCK, "Auto-lock after timeout");
        
        // Publish status to internet
        if (is_mqtt_connected()) {
            mqtt_publish(MQTT_TOPIC_STATUS, "{\"state\":\"locked\",\"method\":\"auto\"}");
            mqtt_publish(MQTT_TOPIC_EVENTS, "{\"event\":\"lock\",\"trigger\":\"timeout\"}");
        }
    } 
    else if (event->event == SM_EVENT_BUTTON_PRESS) {
        ESP_LOGI(TAG, "Button pressed in UNLOCKED state");
        
        control_command_t cmd = create_lock_command();
        send_control_command_blocking(&cmd);
        
        lock_door();
        buzzer_play_lock();         // bzzt beep
        log_event(EVENT_LOCK, "Manual lock via button");
        
        if (is_mqtt_connected()) {
            mqtt_publish(MQTT_TOPIC_STATUS, "{\"state\":\"locked\",\"method\":\"button\"}");
            mqtt_publish(MQTT_TOPIC_EVENTS, "{\"event\":\"lock\",\"trigger\":\"button\"}");
        }
        
        if (auto_lock_timer != NULL) {
            xTimerStop(auto_lock_timer, 0);
        }
    } 
    else if (event->event == SM_EVENT_REMOTE_LOCK) {
        ESP_LOGI(TAG, "Remote lock command received");
        
        control_command_t cmd = create_lock_command();
        send_control_command_blocking(&cmd);
        
        lock_door();
        buzzer_play_lock();         // bzzt beep
        log_event(EVENT_LOCK, "Remote lock via network");
        
        if (is_mqtt_connected()) {
            mqtt_publish(MQTT_TOPIC_STATUS, "{\"state\":\"locked\",\"method\":\"remote\"}");
            mqtt_publish(MQTT_TOPIC_EVENTS, "{\"event\":\"lock\",\"trigger\":\"remote\"}");
        }
        
        // Stop auto-lock timer
        if (auto_lock_timer != NULL) {
            xTimerStop(auto_lock_timer, 0);
        }
    }
}

/* ----------------------------------------------------------------------------
 * state_machine_task
 * 
 * Background task that runs the state machine
 * 
 * This runs continuously, waiting for events and processing them
 * based on the current state of the lock.
 * ---------------------------------------------------------------------------- */
static void state_machine_task(void* arg) {
    sm_event_data_t event;
    
    ESP_LOGI(TAG, "State machine started");
    
    while (1) {
        if (xQueueReceive(sm_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            lock_state_t current_state = get_lock_state();
            
            ESP_LOGI(TAG, "Processing event %d in state %d", event.event, current_state);
            
            if (current_state == LOCK_STATE_LOCKED) {
                handle_locked_state(&event);
            } 
            else if (current_state == LOCK_STATE_UNLOCKED || 
                    current_state == LOCK_STATE_UNLOCKING) {
                handle_unlocked_state(&event);
            } 
            else if (current_state == LOCK_STATE_ERROR) {
                ESP_LOGE(TAG, "System in ERROR state");
            }
            
            update_status_led();
        }
    }
}

/* ----------------------------------------------------------------------------
 * state_machine_init
 * 
 * Initialize the state machine
 * 
 * Sets up:
 * - Event queue for receiving events
 * - Auto-lock timer
 * - Background task to process events
 * 
 * Returns: ESP_OK if successful, error code otherwise
 * ---------------------------------------------------------------------------- */
esp_err_t state_machine_init(void) {
    // Create queue for events (holds up to 20 events)
    sm_event_queue = xQueueCreate(20, sizeof(sm_event_data_t));
    if (sm_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create state machine event queue");
        return ESP_FAIL;
    }
    
    // Create auto-lock timer
    // Timer runs once (pdFALSE = not repeating)
    // Timeout is defined in config.h (default 30 seconds)
    auto_lock_timer = xTimerCreate(
        "AutoLockTimer",                        // Timer name
        pdMS_TO_TICKS(AUTO_LOCK_TIMEOUT_MS),    // Timeout period
        pdFALSE,                                // One-shot timer (not repeating)
        NULL,                                   // Timer ID (not used)
        auto_lock_timer_callback                // Callback function
    );
    
    if (auto_lock_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create auto-lock timer");
        vQueueDelete(sm_event_queue);
        sm_event_queue = NULL;
        return ESP_FAIL;
    }
    
    BaseType_t ret = xTaskCreate(
        state_machine_task,     // Function to run
        "state_machine",        // Task name
        4096,                   // Stack size
        NULL,                   // Parameters (none)
        10,                     // Priority high
        NULL                    // Task handle (not needed)
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create state machine task");
        xTimerDelete(auto_lock_timer, 0);
        auto_lock_timer = NULL;
        vQueueDelete(sm_event_queue);
        sm_event_queue = NULL;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "State machine initialized");
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * get_state_machine_queue
 * 
 * Returns the event queue handle
 * (Used by other modules if needed)
 * ---------------------------------------------------------------------------- */
QueueHandle_t get_state_machine_queue(void) {
    return sm_event_queue;
}

/* ----------------------------------------------------------------------------
 * send_sm_event
 * 
 * Sends an event to the state machine
 * 
 * This is how other parts of the program tell the state machine
 * that something happened (button press, timeout, etc.)
 * 
 * Parameters:
 *   event - which event occurred
 *   data - optional data (can be NULL)
 * 
 * Returns: ESP_OK if event was sent successfully
 * ---------------------------------------------------------------------------- */
esp_err_t send_sm_event(sm_event_t event, void* data) {
    if (sm_event_queue == NULL) {
        return ESP_FAIL;
    }
    
    // Create event data structure
    sm_event_data_t event_data = {
        .event = event,
        .data = data
    };
    
    // Send to queue (wait up to 100ms if queue is full)
    return xQueueSend(sm_event_queue, &event_data, pdMS_TO_TICKS(100)) == pdTRUE 
            ? ESP_OK : ESP_FAIL;
}
