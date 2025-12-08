#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "config.h"
#include "lock_control.h"
#include "button_handler.h"
#include "state_machine.h"
#include "event_logger.h"
#include "network_handler.h"

static const char* TAG = "MAIN";

static QueueHandle_t button_queue = NULL;

static void button_monitor_task(void* arg) {
    button_data_t button_data;
    
    while (1) {
        if (xQueueReceive(button_queue, &button_data, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Button event received: %d", button_data.event);
            
            if (button_data.event == BUTTON_EVENT_PRESS) {
                send_sm_event(SM_EVENT_BUTTON_PRESS, NULL);
            } else if (button_data.event == BUTTON_EVENT_LONG_PRESS) {
                ESP_LOGI(TAG, "Long press detected: %lu ms", button_data.press_duration_ms);
                send_sm_event(SM_EVENT_BUTTON_PRESS, NULL);
            }
        }
    }
}

static void door_monitor_task(void* arg) {
    QueueHandle_t door_queue = get_door_event_queue();
    
    if (door_queue == NULL) {
        ESP_LOGE(TAG, "Door event queue not available!");
        vTaskDelete(NULL);
        return;
    }
    
    #define TAMPER_EVENT_BUFFER_SIZE 10
    int64_t door_event_times[TAMPER_EVENT_BUFFER_SIZE] = {0};
    uint8_t event_index = 0;
    uint8_t event_count = 0;
    
    ESP_LOGI(TAG, "Door monitor task started (ISR-driven)");
    
    while (1) {
        int door_state_value;
        
        if (xQueueReceive(door_queue, &door_state_value, portMAX_DELAY) == pdTRUE) {
            int64_t current_time = esp_timer_get_time() / 1000; // to ms
            
            door_state_t current_state = (door_state_value == 1) ? DOOR_OPEN : DOOR_CLOSED;
            
            ESP_LOGI(TAG, "Door event received (ISR): %s", 
                    current_state == DOOR_OPEN ? "OPEN" : "CLOSED");
            
            door_event_times[event_index] = current_time;
            event_index = (event_index + 1) % TAMPER_EVENT_BUFFER_SIZE;
            if (event_count < TAMPER_EVENT_BUFFER_SIZE) {
                event_count++;
            }
            
            // IF there's been TAMPER_THRESHOLD_COUNT events recorded within a TAMPER_THRESHOLD_TIME_MS window, trigger tamper event
            if (event_count >= TAMPER_THRESHOLD_COUNT) {
                // gets oldest event in buffer
                uint8_t oldest_index = (event_index + TAMPER_EVENT_BUFFER_SIZE - event_count) % TAMPER_EVENT_BUFFER_SIZE;
                int64_t oldest_time = door_event_times[oldest_index];
                int64_t time_span = current_time - oldest_time;
                
                if (time_span < TAMPER_THRESHOLD_TIME_MS) {
                    ESP_LOGE(TAG, "TAMPER DETECTED: %d door events in %lld ms", 
                            event_count, time_span);
                    send_sm_event(SM_EVENT_TAMPER_DETECTED, NULL);
                    
                    event_count = 0;
                    event_index = 0;
                    memset(door_event_times, 0, sizeof(door_event_times));
                }
            }
            
            // sm means state machine btw
            if (current_state == DOOR_OPEN) {
                send_sm_event(SM_EVENT_DOOR_OPENED, NULL);
            } else {
                send_sm_event(SM_EVENT_DOOR_CLOSED, NULL);
            }
        }
    }
}

void app_main(void) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "=======================================");
    ESP_LOGI(TAG, "    BoltLock - Smart Door Lock System");
    ESP_LOGI(TAG, "    DFRobot Firebeetle ESP32");
    ESP_LOGI(TAG, "=======================================");
    
    ESP_LOGI(TAG, "Initializing event logger and NVS...");
    ret = event_logger_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize event logger!");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing lock control hardware...");
    ret = lock_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize lock control!");
        return;
    }
    
    log_event(EVENT_LOCK, "System startup - door locked");
    
    button_queue = xQueueCreate(10, sizeof(button_data_t));
    if (button_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create button queue!");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing button handler...");
    ret = button_handler_init(button_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize button handler!");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing state machine...");
    ret = state_machine_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize state machine!");
        return;
    }
    
    if (xTaskCreate(button_monitor_task, "button_monitor", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button monitor task!");
        return;
    }

    if (xTaskCreate(door_monitor_task, "door_monitor", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create door monitor task!");
        return;
    }
    
    // REMINDER: UPDATE THE WIFI CREDENTIALS IN config.h BEFORE ENABLING NETWORK
    ESP_LOGI(TAG, "Network initialization disabled - configure WiFi credentials first");

    // Uncomment below ONLY if WiFi credentials are configured:

    /*
    ESP_LOGI(TAG, "Initializing WiFi...");
    ret = network_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi initialization failed - continuing without network");
    }
    */
    
    ESP_LOGI(TAG, "=======================================");
    ESP_LOGI(TAG, "System initialization complete!");
    ESP_LOGI(TAG, "Lock State: %s", 
            get_lock_state() == LOCK_STATE_LOCKED ? "LOCKED" : "UNLOCKED");
    ESP_LOGI(TAG, "Door State: %s", 
            get_door_state() == DOOR_CLOSED ? "CLOSED" : "OPEN");
    ESP_LOGI(TAG, "=======================================");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // check every 30 seconds
        static int counter = 0;
        if (++counter >= 30) {
            ESP_LOGI(TAG, "Status: Lock=%s, Door=%s, Network=%s",
                    get_lock_state() == LOCK_STATE_LOCKED ? "LOCKED" : "UNLOCKED",
                    get_door_state() == DOOR_CLOSED ? "CLOSED" : "OPEN",
                    is_network_connected() ? "CONNECTED" : "DISCONNECTED");
            counter = 0;
        }
    }
}