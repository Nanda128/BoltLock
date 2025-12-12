#include <stdio.h>
#include <string.h>
#include <inttypes.h>
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
                ESP_LOGI(TAG, "Long press detected: %" PRIu32 " ms", button_data.press_duration_ms);
                send_sm_event(SM_EVENT_BUTTON_PRESS, NULL);
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
    ESP_LOGI(TAG, "=======================================");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // check every 30 seconds
        static int counter = 0;
        if (++counter >= 30) {
            ESP_LOGI(TAG, "Status: Lock=%s, Network=%s",
                    get_lock_state() == LOCK_STATE_LOCKED ? "LOCKED" : "UNLOCKED",
                    is_network_connected() ? "CONNECTED" : "DISCONNECTED");
            counter = 0;
        }
    }
}