#include "state_machine.h"
#include "lock_control.h"
#include "event_logger.h"
#include "buzzer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char* TAG = "STATE_MACHINE";
static QueueHandle_t sm_event_queue = NULL;
static TimerHandle_t auto_lock_timer = NULL;

extern void send_telegram_notification(const char* message);

static void auto_lock_timer_callback(TimerHandle_t xTimer) {
    ESP_LOGI(TAG, "Auto-lock timer expired");
    sm_event_data_t event = {
        .event = SM_EVENT_TIMEOUT,
        .data = NULL
    };
    xQueueSend(sm_event_queue, &event, 0);
}

static void handle_locked_state(sm_event_data_t* event) {
    switch (event->event) {
        case SM_EVENT_BUTTON_PRESS:
            ESP_LOGI(TAG, "Button pressed in LOCKED state");
            unlock_door();
            buzzer_play_unlock();
            log_event(EVENT_BUTTON_PRESS, "Physical button unlock");
            
            if (auto_lock_timer != NULL) {
                xTimerStart(auto_lock_timer, 0);
            }
            break;
            
        case SM_EVENT_REMOTE_UNLOCK:
            ESP_LOGI(TAG, "Remote unlock command received");
            unlock_door();
            buzzer_play_unlock();
            log_event(EVENT_REMOTE_UNLOCK, "Remote unlock via network");
            
            if (auto_lock_timer != NULL) {
                xTimerStart(auto_lock_timer, 0);
            }
            break;
            
        default:
            break;
    }
}

static void handle_unlocked_state(sm_event_data_t* event) {
    switch (event->event) {
        case SM_EVENT_TIMEOUT:
            ESP_LOGI(TAG, "Auto-lock timeout");
            lock_door();
            buzzer_play_lock();
            log_event(EVENT_LOCK, "Auto-lock after timeout");
            break;
            
        case SM_EVENT_BUTTON_PRESS:
            ESP_LOGI(TAG, "Button pressed in UNLOCKED state");
            lock_door();
            buzzer_play_lock();
            log_event(EVENT_LOCK, "Manual lock via button");
            
            if (auto_lock_timer != NULL) {
                xTimerStop(auto_lock_timer, 0);
            }
            break;
            
        case SM_EVENT_REMOTE_LOCK:
            ESP_LOGI(TAG, "Remote lock command received");
            lock_door();
            buzzer_play_lock();
            log_event(EVENT_LOCK, "Remote lock via network");
            
            if (auto_lock_timer != NULL) {
                xTimerStop(auto_lock_timer, 0);
            }
            break;
            
        default:
            break;
    }
}

static void state_machine_task(void* arg) {
    sm_event_data_t event;
    
    ESP_LOGI(TAG, "State machine started");
    
    while (1) {
        if (xQueueReceive(sm_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            lock_state_t current_state = get_lock_state();
            
            ESP_LOGI(TAG, "Processing event %d in state %d", event.event, current_state);
            
            switch (current_state) {
                case LOCK_STATE_LOCKED:
                    handle_locked_state(&event);
                    break;
                case LOCK_STATE_UNLOCKED:
                case LOCK_STATE_UNLOCKING:
                    handle_unlocked_state(&event);
                    break;
                case LOCK_STATE_ERROR:
                    ESP_LOGE(TAG, "System in ERROR state");
                    break;
            }
            
            update_status_led();
        }
    }
}

esp_err_t state_machine_init(void) {
    sm_event_queue = xQueueCreate(20, sizeof(sm_event_data_t));
    if (sm_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create state machine event queue");
        return ESP_FAIL;
    }
    
    auto_lock_timer = xTimerCreate(
        "AutoLockTimer",
        pdMS_TO_TICKS(AUTO_LOCK_TIMEOUT_MS),
        pdFALSE, 
        NULL,
        auto_lock_timer_callback
    );
    
    if (auto_lock_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create auto-lock timer");
        vQueueDelete(sm_event_queue);
        sm_event_queue = NULL;
        return ESP_FAIL;
    }
    
    BaseType_t ret = xTaskCreate(
        state_machine_task,
        "state_machine",
        4096,
        NULL,
        10,  
        NULL
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

QueueHandle_t get_state_machine_queue(void) {
    return sm_event_queue;
}

esp_err_t send_sm_event(sm_event_t event, void* data) {
    if (sm_event_queue == NULL) {
        return ESP_FAIL;
    }
    
    sm_event_data_t event_data = {
        .event = event,
        .data = data
    };
    
    return xQueueSend(sm_event_queue, &event_data, pdMS_TO_TICKS(100)) == pdTRUE 
            ? ESP_OK : ESP_FAIL;
}
