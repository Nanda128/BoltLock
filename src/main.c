/* ============================================================================
 * BoltLock - Smart Door Lock System
 * Main Application File
 * 
 * This file starts up the system and coordinates all the parts.
 * Think of it like the "brain" that turns on all the other parts.
 * ============================================================================ */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "config.h"
#include "lock_control.h"
#include "button_handler.h"
#include "state_machine.h"
#include "event_logger.h"
#include "network_handler.h"
#include "buzzer.h"
#include "thread_queues.h"
#include "rt_control_thread.h"

// Tag for debug messages from this file
static const char* TAG = "MAIN";

static QueueHandle_t button_queue = NULL;

/* ----------------------------------------------------------------------------
 * button_monitor_task
 * 
 * This function runs continuously in the background (AGENT THREAD).
 * It waits for button events and tells the state machine when a button is pressed.
 * 
 * This is the AGENT THREAD running at PRIORITY 5 (normal priority)
 * 
 * How it works:
 * 1. Wait for a button event to arrive in the queue
 * 2. When an event arrives, check what kind it is
 * 3. Tell the state machine to handle the button press
 * 4. Monitor feedback from the control thread (bi-directional communication)
 * ---------------------------------------------------------------------------- */
static void button_monitor_task(void* arg) {
    button_data_t button_data;  // Variable to hold button event info
    control_feedback_t feedback;  // Variable to hold feedback from control thread
    
    ESP_LOGI(TAG, "Agent thread monitoring button events and control feedback");
    
    // Run forever 
    while (1) {
        // Wait for button event from the queue (timeout after 100ms)
        if (xQueueReceive(button_queue, &button_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "Button event received: %d", button_data.event);
            
            // Check if it's a press
            if (button_data.event == BUTTON_EVENT_PRESS) {
                send_sm_event(SM_EVENT_BUTTON_PRESS, NULL);
            }
        }
        
        // Check for feedback from control thread (non-blocking)
        // This demonstrates bi-directional communication:
        // - Agent sends commands via send_control_command()
        // - Control thread sends feedback via post_control_feedback()
        while (receive_control_feedback(&feedback, 0) == ESP_OK) {
            switch (feedback.type) {
                case FEEDBACK_POSITION_UPDATE:
                    ESP_LOGD(TAG, "Position update: %.1f° (target: %.1f°, error: %.2f°)",
                            feedback.current_position, feedback.target_position, feedback.error);
                    break;
                    
                case FEEDBACK_COMMAND_COMPLETE:
                    ESP_LOGI(TAG, "Command completed: position=%.1f°", feedback.current_position);
                    break;
                    
                case FEEDBACK_COMMAND_FAILED:
                    ESP_LOGW(TAG, "Command failed: %s", feedback.message);
                    break;
                    
                case FEEDBACK_ERROR:
                    ESP_LOGE(TAG, "Control error: %s (code: %d)", 
                            feedback.message, feedback.error_code);
                    break;
                    
                case FEEDBACK_STALL_DETECTED:
                    ESP_LOGW(TAG, "Motor stall detected at %.1f°", feedback.current_position);
                    // Could trigger error handling or retry logic here
                    break;
                    
                case FEEDBACK_CALIBRATION_DONE:
                    ESP_LOGI(TAG, "Calibration completed");
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown feedback type: %d", feedback.type);
                    break;
            }
        }
    }
}

/* ----------------------------------------------------------------------------
 * app_main
 * 
 * This is a main function that runs when the ESP32 starts up.
 * duh
 * 
 * What it does:
 * 1. Set up flash memory for storing data
 * 2. Initialize all hardware components (lock, buzzer, button, LEDs)
 * 3. Start background tasks to monitor button presses
 * 4. Run forever, printing status updates every 30 seconds
 * ---------------------------------------------------------------------------- */
void app_main(void) {
    esp_err_t ret;  // Variable to check if things succeed or fail
    
    // Print startup banner
    ESP_LOGI(TAG, "=======================================");
    ESP_LOGI(TAG, "    BoltLock - Smart Door Lock System");
    ESP_LOGI(TAG, "    DFRobot Firebeetle ESP32");
    ESP_LOGI(TAG, "=======================================");
    
    ESP_LOGI(TAG, "Setting up NVS flash...");
    ret = nvs_flash_init();
    // If flash is corrupted or needs erasing, fix it
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());  // Erase and try again
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);  // Crash if this fails (critical error)
    
    ESP_LOGI(TAG, "Initializing event logger and NVS...");
    ret = event_logger_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize event logger!");
        return;  // Stop if this fails
    }
    
    ESP_LOGI(TAG, "Initializing lock control hardware...");
    ret = lock_control_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize lock control!");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing buzzer...");
    ret = buzzer_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buzzer!");
        return;
    }
    
    ESP_LOGI(TAG, "Testing speaker...");
    buzzer_beep(200);  // Beep for 200ms
    vTaskDelay(pdMS_TO_TICKS(300));  // Wait 300ms
    buzzer_beep(200);  // Beep again
    
    // Log that the system started
    log_event(EVENT_LOCK, "System startup - door locked");
    
    ESP_LOGI(TAG, "Initializing thread queues...");
    ret = thread_queues_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize thread queues!");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing real-time control thread...");
    ret = rt_control_thread_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RT control thread!");
        return;
    }
    ESP_LOGI(TAG, "Real-time control thread started at priority %d", RT_CONTROL_THREAD_PRIORITY);
    
    // Queue holds up to 10 button events
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
    
    // The state machine manages lock/unlock logic
    ESP_LOGI(TAG, "Initializing state machine...");
    ret = state_machine_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize state machine!");
        return;
    }
    
    // Create a background task that watches for button presses
    // This runs at PRIORITY 5 (lower than control thread priority 10)
    if (xTaskCreate(button_monitor_task, "agent_thread", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create agent thread!");
        return;
    }
    ESP_LOGI(TAG, "Agent thread started at priority 5");

    // ========================================================================
    // NETWORK SETUP INSTRUCTIONS:
    // ========================================================================
    // This system is configured to work with a LAPTOP INTERMEDIARY setup.
    // 
    // BEFORE enabling network, complete these steps:
    // 1. Set up WiFi hotspot on your laptop (see LAPTOP_SETUP_GUIDE.md)
    // 2. Install and start Mosquitto MQTT broker on laptop
    // 3. Update WiFi credentials in include/config.h:
    //    - WIFI_SSID: Your laptop hotspot name
    //    - WIFI_PASSWORD: Your laptop hotspot password
    //    - MQTT_BROKER_URI: Your laptop's IP (e.g., mqtt://192.168.137.1)
    //
    // Quick reference: See QUICK_SETUP.md
    // Full guide: See LAPTOP_SETUP_GUIDE.md
    // ========================================================================

    ESP_LOGI(TAG, "Initializing WiFi...");
    ret = network_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi initialization failed - continuing without network");
    } else {
        ESP_LOGI(TAG, "Waiting for WiFi connection...");
        // Wait up to 10 seconds for WiFi to connect
        for (int i = 0; i < 20 && !is_network_connected(); i++) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        if (is_network_connected()) {
            ESP_LOGI(TAG, "Initializing MQTT...");
            ret = mqtt_init();
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "MQTT initialization failed - continuing without MQTT");
            }
        } else {
            ESP_LOGW(TAG, "WiFi connection timeout - skipping MQTT initialization");
        }
    }
    
    ESP_LOGI(TAG, "=======================================");
    ESP_LOGI(TAG, "System initialization complete!");
    ESP_LOGI(TAG, "Threading Model:");
    ESP_LOGI(TAG, "  - Control Thread: Priority %d (real-time servo control)", RT_CONTROL_THREAD_PRIORITY);
    ESP_LOGI(TAG, "  - Agent Thread:   Priority 5 (state machine, buttons, network)");
    ESP_LOGI(TAG, "Bi-directional Queue Communication:");
    ESP_LOGI(TAG, "  - Command Queue:  Agent -> Control (sends lock/unlock commands)");
    ESP_LOGI(TAG, "  - Feedback Queue: Control -> Agent (sends position updates)");
    ESP_LOGI(TAG, "Lock State: %s", 
            get_lock_state() == LOCK_STATE_LOCKED ? "LOCKED" : "UNLOCKED");
    ESP_LOGI(TAG, "=======================================");
    
    // Main Loop - Print Status Every 30 Seconds
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second
        
        static int counter = 0;
        counter++;
        if (counter >= 30) {  // Every 30 seconds
            // Get control thread statistics
            control_stats_t stats;
            rt_control_get_stats(&stats);
            
            ESP_LOGI(TAG, "=== System Status ===");
            ESP_LOGI(TAG, "Lock: %s | WiFi: %s | MQTT: %s",
                    get_lock_state() == LOCK_STATE_LOCKED ? "LOCKED" : "UNLOCKED",
                    is_network_connected() ? "CONNECTED" : "DISCONNECTED",
                    is_mqtt_connected() ? "CONNECTED" : "DISCONNECTED");
            ESP_LOGI(TAG, "RT Control: %.1f° | Loops: %u | Avg: %.1fus",
                    rt_control_get_position(),
                    (unsigned int)stats.loop_count,
                    stats.avg_loop_time_us);
            counter = 0;  // Reset counter
        }
    }
}