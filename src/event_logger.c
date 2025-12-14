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
static nvs_handle_t event_logger_nvs_handle;
static bool sntp_initialized = false;

static void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Time synchronized via SNTP");
}

static void initialize_sntp(void) {
    if (sntp_initialized) {
        return;
    }
    
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
    sntp_initialized = true;
}

static time_t get_current_time(void) {
    time_t now = 0;
    struct tm timeinfo = { 0 };
    
    if (is_network_connected() && !sntp_initialized) {
        initialize_sntp();
    }
    
    time(&now);
    localtime_r(&now, &timeinfo);
    
    if (timeinfo.tm_year < (2020 - 1900)) {
        ESP_LOGD(TAG, "Time not yet synchronized, using 0");
        return 0;
    }
    
    return now;
}

static const char* event_type_names[] = {
    "LOCK",
    "UNLOCK",
    "BUTTON_PRESS",
    "REMOTE_UNLOCK"
};

esp_err_t event_logger_init(void) {
    esp_err_t ret;
    
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &event_logger_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Event logger initialized with NVS");
    return ESP_OK;
}

esp_err_t log_event(event_type_t type, const char* description) {
    esp_err_t ret;
    
    event_log_t event;
    event.type = type;
    event.timestamp = get_current_time();
    strncpy(event.description, description, sizeof(event.description) - 1);
    event.description[sizeof(event.description) - 1] = '\0';
    
    const size_t event_type_names_count = sizeof(event_type_names) / sizeof(event_type_names[0]);
    const char* type_name = (type >= 0 && (size_t)type < event_type_names_count) ? event_type_names[type] : "UNKNOWN";
    if (strcmp(type_name, "UNKNOWN") == 0) {
        ESP_LOGW(TAG, "Invalid event type: %d", type);
    }
    ESP_LOGI(TAG, "Event: %s - %s", type_name, description);
        
    uint32_t event_count = 0;
    ret = nvs_get_u32(event_logger_nvs_handle, NVS_EVENT_COUNT_KEY, &event_count);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        event_count = 0;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read event count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "evt_%" PRIu32, event_count % 100);
    
    ret = nvs_set_blob(event_logger_nvs_handle, key, &event, sizeof(event_log_t));
    if (ret == ESP_ERR_NVS_NOT_ENOUGH_SPACE) {
        ESP_LOGW(TAG, "NVS storage full, erasing old event data...");
        nvs_close(event_logger_nvs_handle);
        ret = nvs_flash_erase_partition("nvs");
        if (ret == ESP_OK) {
            ret = nvs_flash_init_partition("nvs");
            if (ret == ESP_OK) {
                ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &event_logger_nvs_handle);
                if (ret == ESP_OK) {
                    event_count = 0;
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
    
    event_count++;
    ret = nvs_set_u32(event_logger_nvs_handle, NVS_EVENT_COUNT_KEY, event_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update event count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_commit(event_logger_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(ret));
        return ret;
    }
    
    char message[256];
    format_event_message(&event, message, sizeof(message));
    
    // TODO: Send to network queue for transmission via MQTT/Telegram
    // NETWORK INTEGRATION HERE
    ESP_LOGI(TAG, "Event logged successfully (count: %" PRIu32 ")", event_count);
    
    return ESP_OK;
}

int get_recent_events(event_log_t* events, int count) {
    if (events == NULL || count <= 0) {
        return 0;
    }
    
    uint32_t event_count = 0;
    esp_err_t ret = nvs_get_u32(event_logger_nvs_handle, NVS_EVENT_COUNT_KEY, &event_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read event count: %s", esp_err_to_name(ret));
        return 0;
    }
    
    int events_to_read = (event_count < count) ? event_count : count;
    if (events_to_read > 100) events_to_read = 100;  // take 100 events
    
    int events_read = 0;
    for (int i = 0; i < events_to_read; i++) {
        uint32_t idx = (event_count - 1 - i) % 100;
        char key[16];
        snprintf(key, sizeof(key), "evt_%" PRIu32, idx);
        
        size_t required_size = sizeof(event_log_t);
        ret = nvs_get_blob(event_logger_nvs_handle, key, &events[i], &required_size);
        if (ret == ESP_OK) {
            events_read++;
        }
    }
    
    ESP_LOGI(TAG, "Retrieved %d recent events", events_read);
    return events_read;
}

void format_event_message(event_log_t* event, char* buffer, size_t buffer_size) {
    if (event == NULL || buffer == NULL) {
        return;
    }
    
    // emojis look better on mobiles but it can look weird on some clients
    // TODO: Make this configurable between maybe text and emoji later
    const char* emoji = "ℹ️";
    
    switch (event->type) {
        case EVENT_LOCK:
            emoji = "🔒";
            break;
        case EVENT_UNLOCK:
            emoji = "🔓";
            break;
        case EVENT_BUTTON_PRESS:
            emoji = "👆";
            break;
        case EVENT_REMOTE_UNLOCK:
            emoji = "📱";
            break;
    }
    
    const char* event_type_str = "UNKNOWN";
    if (event->type >= 0 && event->type < (sizeof(event_type_names)/sizeof(event_type_names[0]))) {
        event_type_str = event_type_names[event->type];
    }
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
