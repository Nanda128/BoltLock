#include "network_handler.h"
#include "config.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include <string.h>

static const char* TAG = "NETWORK";
static bool wifi_connected = false;

// WiFi event handler for connection management
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA started, initiating connection...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        ESP_LOGW(TAG, "WiFi disconnected, attempting to reconnect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected! IP address: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

esp_err_t network_init(void) {
    // Initialize network interface
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize network interface: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create default event loop
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create default WiFi station interface
    esp_netif_create_default_wifi_sta();
    
    // Initialize WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register event handlers
    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure WiFi with SSID and password from config.h
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    
    // Set WiFi mode to station and start
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WiFi initialization completed");
    return ESP_OK;
}

esp_err_t send_telegram_notification(const char* message) {
    // Validate message parameter
    if (message == NULL) {
        ESP_LOGE(TAG, "Invalid message: null pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check WiFi connection status
    if (!wifi_connected) {
        ESP_LOGW(TAG, "WiFi not connected. Cannot send Telegram notification.");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Construct Telegram Bot API URL
    char url[256];
    snprintf(url, sizeof(url), "https://api.telegram.org/bot%s/sendMessage", TELEGRAM_BOT_TOKEN);
    
    // Initialize HTTP client
    esp_http_client_config_t http_config = {
        .url = url,
        .method = HTTP_METHOD_POST,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&http_config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return ESP_FAIL;
    }
    
    // Format POST data as JSON
    char post_data[512];
    snprintf(post_data, sizeof(post_data),
             "{\"chat_id\":\"%s\",\"text\":\"%s\",\"parse_mode\":\"HTML\"}",
             TELEGRAM_CHAT_ID, message);
    
    // Set Content-Type header
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    
    // Send POST request and handle response
    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }
    
    // Log response status
    int status_code = esp_http_client_get_status_code(client);
    if (status_code == 200) {
        ESP_LOGI(TAG, "Telegram notification sent successfully (HTTP %d)", status_code);
    } else {
        ESP_LOGW(TAG, "Telegram notification failed with HTTP status %d", status_code);
    }
    
    // Clean up HTTP client resources
    esp_http_client_cleanup(client);
    
    return (status_code == 200) ? ESP_OK : ESP_FAIL;
}

esp_err_t mqtt_init(void) {
    // TODO: Implement MQTT client initialization
    // This would use esp_mqtt_client_init() from mqtt_client library
    // Example outline:
    // 1. Configure MQTT client with broker URI
    // 2. Set up event handlers for connected/disconnected/data events
    // 3. Start MQTT client
    // 4. Subscribe to command topic for remote control
    
    ESP_LOGW(TAG, "MQTT initialization not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t mqtt_publish(const char* topic, const char* message) {
    // TODO: Implement MQTT publish
    // This would use esp_mqtt_client_publish()
    
    ESP_LOGW(TAG, "MQTT publish not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

bool is_network_connected(void) {
    return wifi_connected;
}
