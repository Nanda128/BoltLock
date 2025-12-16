/* ============================================================================
 * Event Logger - Records and Stores System Events
 * 
 * - Keeps a history of everything that happens (lock, unlock, button press)
 * - Saves events to flash memory (survives power loss)
 * - Sends notifications via MQTT
 * - Gets accurate timestamps from internet time servers
 * 
 * Key Concepts:
 * - NVS (Non-Volatile Storage): Saves data to flash memory permanently
 * - SNTP (Simple Network Time Protocol): Gets accurate time from internet
 * - Circular buffer: Stores up to 100 events, oldest get overwritten
 * - Timestamp: Records when something happened (Unix time format)
 * 
 * Storage Strategy:
 * - Stores last 100 events in a circular buffer
 * - Each event has: type, timestamp, description
 * - Uses NVS (flash memory) so data survives power cycles
 * ============================================================================ */

#include "event_logger.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "network_handler.h"
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <inttypes.h>

static const char* TAG = "EVENT_LOGGER";

// Handle for accessing NVS (Non-Volatile Storage)
static nvs_handle_t event_logger_nvs_handle;

// Flag to track if SNTP time sync is initialized
static bool sntp_initialized = false;

/* ----------------------------------------------------------------------------
 * time_sync_notification_cb
 * 
 * Callback function called when time is synchronized with internet
 * ---------------------------------------------------------------------------- */
static void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Time synchronized via SNTP");
}

/* ----------------------------------------------------------------------------
 * initialize_sntp
 * 
 * Sets up internet time synchronization
 * 
 * SNTP = Simple Network Time Protocol
 * Gets accurate time from internet servers like pool.ntp.org
 * This ensures event timestamps are correct
 * ---------------------------------------------------------------------------- */
static void initialize_sntp(void) {
    // Don't initialize twice
    if (sntp_initialized) {
        return;
    }
    
    ESP_LOGI(TAG, "Initializing SNTP");
    
    // Set SNTP to poll mode (periodically check for time updates)
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    
    // Configure time servers (these are free public NTP servers)
    esp_sntp_setservername(0, "pool.ntp.org");   // Primary server
    esp_sntp_setservername(1, "time.nist.gov");  // Backup server
    
    // Set callback for when time syncs
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    
    // Start SNTP service
    esp_sntp_init();
    sntp_initialized = true;
}

/* ----------------------------------------------------------------------------
 * get_current_time
 * 
 * Gets the current time as a Unix timestamp
 * 
 * Unix timestamp = number of seconds since January 1, 1970
 * 
 * If WiFi is connected and SNTP not initialized, it starts time sync
 * Returns 0 if time hasn't been synchronized yet (year < 2020)
 * ---------------------------------------------------------------------------- */
static time_t get_current_time(void) {
    time_t now = 0;
    struct tm timeinfo = { 0 };
    
    // If connected to internet, make sure SNTP is running
    if (is_network_connected() && !sntp_initialized) {
        initialize_sntp();
    }
    
    // Get current time
    time(&now);
    localtime_r(&now, &timeinfo);
    
    // Check if time is valid (year should be 2020 or later)
    // If not synced, ESP32 defaults to year 1970
    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGD(TAG, "Time not yet synchronized, using 0");
        return 0;  // Invalid time
    }
    
    return now;
}

/* Event type names for logging and display */
static const char* event_type_names[] = {
    "LOCK",
    "UNLOCK",
    "BUTTON_PRESS",
    "REMOTE_UNLOCK"
};

/* ----------------------------------------------------------------------------
 * event_logger_init
 * 
 * Initialize the event logging system
 * 
 * Sets up NVS (Non-Volatile Storage) to save events to flash memory
 * 
 * NVS is like a small database in flash memory that survives:
 * - Power loss
 * - Reboots
 * - Firmware updates
 * 
 * Returns: ESP_OK if successful, error code otherwise
 * ---------------------------------------------------------------------------- */
esp_err_t event_logger_init(void) {
    esp_err_t ret;
    
    // Initialize NVS flash
    ret = nvs_flash_init();
    
    // If NVS partition is corrupted or needs erasing, fix it
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Open NVS namespace for reading/writing events
    // Namespace = like a folder in the NVS database
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &event_logger_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Event logger initialized with NVS");
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * log_event
 * 
 * Records an event and saves it to flash memory
 * 
 * It:
 * 1. Creates an event record with type, timestamp, and description
 * 2. Saves it to NVS flash memory
 * 3. Sends notifications (Telegram and MQTT) if internet connected
 * 
 * Storage uses a circular buffer:
 * - Stores last 100 events
 * - When full, oldest event gets overwritten
 * - Events numbered 0-99 in a loop
 * 
 * Parameters:
 *   type - what kind of event (LOCK, UNLOCK, BUTTON_PRESS, etc.)
 *   description - human-readable description
 * 
 * Returns: ESP_OK if successful
 * ---------------------------------------------------------------------------- */
esp_err_t log_event(event_type_t type, const char* description) {
    esp_err_t ret;
    
    // Create event record
    event_log_t event;
    event.type = type;
    event.timestamp = get_current_time();  // Get current Unix timestamp
    
    // Copy description safely (prevent buffer overflow)
    strncpy(event.description, description, sizeof(event.description) - 1);
    event.description[sizeof(event.description) - 1] = '\0';  // Ensure null termination
    
    // Validate and log the event type
    const size_t event_type_names_count = sizeof(event_type_names) / sizeof(event_type_names[0]);
    const char* type_name = (type >= 0 && (size_t)type < event_type_names_count) ? event_type_names[type] : "UNKNOWN";
    
    if (strcmp(type_name, "UNKNOWN") == 0) {
        ESP_LOGW(TAG, "Invalid event type: %d", type);
    }
    ESP_LOGI(TAG, "Event: %s - %s", type_name, description);
    
    // Get current event count
    // Event count tracks how many events have been logged
    uint32_t event_count = 0;
    ret = nvs_get_u32(event_logger_nvs_handle, NVS_EVENT_COUNT_KEY, &event_count);
    
    // If not found (first time), start at 0
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        event_count = 0;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read event count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Calculate NVS key for this event
    // Use modulo 100 to create circular buffer (0-99)
    // Example: event 0 -> evt_0, event 100 -> evt_0 (overwrites old)
    char key[16];
    snprintf(key, sizeof(key), "evt_%" PRIu32, event_count % 100);
    
    // Save event to NVS
    ret = nvs_set_blob(event_logger_nvs_handle, key, &event, sizeof(event_log_t));
    
    // Handle case where NVS is full
    if (ret == ESP_ERR_NVS_NOT_ENOUGH_SPACE) {
        ESP_LOGW(TAG, "NVS storage full, erasing old event data...");
        
        // Close handle, erase partition, reinitialize
        nvs_close(event_logger_nvs_handle);
        ret = nvs_flash_erase_partition("nvs");
        if (ret == ESP_OK) {
            ret = nvs_flash_init_partition("nvs");
            if (ret == ESP_OK) {
                ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &event_logger_nvs_handle);
                if (ret == ESP_OK) {
                    event_count = 0;  // Reset count after erase
                    ret = nvs_set_blob(event_logger_nvs_handle, key, &event, sizeof(event_log_t));
                }
            }
        }
        
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to recover from storage full condition: %s", esp_err_to_name(ret));
            return ret;
        }
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to store event: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Increment event count for next event
    event_count++;
    ret = nvs_set_u32(event_logger_nvs_handle, NVS_EVENT_COUNT_KEY, event_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update event count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Commit changes to NVS
    // nvs_commit writes buffered changes to flash permanently
    ret = nvs_commit(event_logger_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Send internet notifications
    char message[256];
    format_event_message(&event, message, sizeof(message));
    
    if (is_network_connected()) {
        // Publish to MQTT if connected
        if (is_mqtt_connected()) {
            ret = mqtt_publish("boltlock/events", message);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to publish to MQTT: %s", esp_err_to_name(ret));
            } else {
                ESP_LOGD(TAG, "MQTT event published successfully");
            }
        }
    } else {
        ESP_LOGD(TAG, "Network not connected, skipping remote event transmission");
    }
    
    ESP_LOGI(TAG, "Event logged successfully (count: %" PRIu32 ")", event_count);
    
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * get_recent_events
 * 
 * Retrieves the most recent events from storage
 * Reads events from NVS and puts them in the provided array
 * Returns them in reverse order (newest first)
 * 
 * Parameters:
 *   events - array to store retrieved events
 *   count - how many events to retrieve (max 100)
 * 
 * Returns: number of events actually retrieved
 * ---------------------------------------------------------------------------- */
int get_recent_events(event_log_t* events, int count) {
    // Validate input
    if (events == NULL || count <= 0) {
        return 0;
    }
    
    // Get total event count
    uint32_t event_count = 0;
    esp_err_t ret = nvs_get_u32(event_logger_nvs_handle, NVS_EVENT_COUNT_KEY, &event_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read event count: %s", esp_err_to_name(ret));
        return 0;
    }
    
    // Calculate how many to read
    // Read requested count, but not more than what exists or max 100
    int events_to_read = (event_count < count) ? event_count : count;
    if (events_to_read > 100) events_to_read = 100;  // Max circular buffer size
    
    // Read events from NVS (newest first)
    int events_read = 0;
    for (int i = 0; i < events_to_read; i++) {
        // Calculate index (newest to oldest)
        // Example: if event_count=105, read indices 4, 3, 2, 1, 0, 99, 98...
        uint32_t idx = (event_count - 1 - i) % 100;
        
        // Generate key for this event
        char key[16];
        snprintf(key, sizeof(key), "evt_%" PRIu32, idx);
        
        // Try to read event from NVS
        size_t required_size = sizeof(event_log_t);
        ret = nvs_get_blob(event_logger_nvs_handle, key, &events[i], &required_size);
        if (ret == ESP_OK) {
            events_read++;  // Successfully read this event
        }
    }
    
    ESP_LOGI(TAG, "Retrieved %d recent events", events_read);
    return events_read;
}

/* ----------------------------------------------------------------------------
 * format_event_message
 * 
 * Formats an event into a human-readable message
 * 
 * Creates a formatted message for MQTT notifications
 * Uses emojis or text based on USE_EMOJI_FORMAT setting
 * 
 * Parameters:
 *   event - the event to format
 *   buffer - where to store the formatted message
 *   buffer_size - size of the buffer
 * ---------------------------------------------------------------------------- */
void format_event_message(event_log_t* event, char* buffer, size_t buffer_size) {
    // Validate inputs
    if (event == NULL || buffer == NULL) {
        return;
    }
    
    // Choose emoji or text icon
#if USE_EMOJI_FORMAT
    // Emoji mode
    const char* emoji = "ℹ️";  // Default info emoji
    
    switch (event->type) {
        case EVENT_LOCK:
            emoji = "🔒";  // Lock emoji
            break;
        case EVENT_UNLOCK:
            emoji = "🔓";  // Unlocked emoji
            break;
        case EVENT_BUTTON_PRESS:
            emoji = "👆";  // Pointing finger emoji
            break;
        case EVENT_REMOTE_UNLOCK:
            emoji = "📱";  // Mobile phone emoji
            break;
    }
#else
    // Text mode (for terminals that don't support emoji)
    const char* emoji = "[i]";
    
    switch (event->type) {
        case EVENT_LOCK:
            emoji = "[LOCK]";
            break;
        case EVENT_UNLOCK:
            emoji = "[UNLOCK]";
            break;
        case EVENT_BUTTON_PRESS:
            emoji = "[BUTTON]";
            break;
        case EVENT_REMOTE_UNLOCK:
            emoji = "[REMOTE]";
            break;
    }
#endif
    
    // Get event type name
    const char* event_type_str = "UNKNOWN";
    if (event->type >= 0 && event->type < (sizeof(event_type_names)/sizeof(event_type_names[0]))) {
        event_type_str = event_type_names[event->type];
    }
    
    // Format the message
    snprintf(buffer, buffer_size, 
             "%s *BoltLock Alert*\n\n"
                "*Event:* %s\n"
                "*Details:* %s\n"
                "*Time:* %lld",
                emoji,
                event_type_str,
                event->description,
                (long long)event->timestamp);
}
