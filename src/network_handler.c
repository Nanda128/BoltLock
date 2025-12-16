/* ============================================================================
 * Network Handler - Manages WiFi and MQTT Connectivity
 * 
 * WiFi Management:
 * - Connects to WiFi network using credentials from config
 * - Automatically reconnects if connection drops
 * - Handles IP address assignment
 * - Optimized for iPhone hotspot compatibility (802.11b/g/n, 20MHz bandwidth)
 * 
 * MQTT Management:
 * - Connects to MQTT broker for remote control
 * - Subscribes to command topic to receive lock/unlock commands
 * - Publishes status updates and events
 * - Handles JSON command parsing
 * 
 * Remote Commands:
 * - Receives {"action":"lock"} or {"action":"unlock"} via MQTT
 * - Sends events to state machine for processing
 * - Publishes status updates back to broker
 * ============================================================================ */

#include "network_handler.h"
#include "config.h"
#include "state_machine.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include <string.h>

static const char* TAG = "NETWORK";

// WiFi connection state
static bool wifi_connected = false;

// MQTT client handle and connection state
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

// Automatic WiFi reconnection timer
static esp_timer_handle_t wifi_reconnect_timer = NULL;
static bool reconnect_scheduled = false;

static void wifi_reconnect_timer_callback(void* arg);

/* ----------------------------------------------------------------------------
 * wifi_reconnect_timer_callback
 * 
 * Called automatically when WiFi reconnection timer expires
 * 
 * - Clears the reconnect scheduled flag
 * - Attempts to reconnect to WiFi network
 * - Triggered after WIFI_RECONNECT_DELAY_MS milliseconds of disconnection
 * ---------------------------------------------------------------------------- */
static void wifi_reconnect_timer_callback(void* arg) {
    ESP_LOGI(TAG, "Reconnect timer expired, attempting WiFi connection...");
    reconnect_scheduled = false;
    esp_wifi_connect();
}

/* ----------------------------------------------------------------------------
 * wifi_event_handler
 * 
 * Handles WiFi and IP events from ESP32 event loop
 * 
 * Events handled:
 * - WIFI_EVENT_STA_START: WiFi station started -> initiate connection
 * - WIFI_EVENT_STA_DISCONNECTED: Lost connection -> schedule reconnect
 * - IP_EVENT_STA_GOT_IP: Got IP address -> connection successful
 * 
 * Uses a timer-based reconnection strategy to avoid rapid retry loops
 * ---------------------------------------------------------------------------- */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA started, initiating connection...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        ESP_LOGW(TAG, "WiFi disconnected, will attempt reconnect in %d seconds...", WIFI_RECONNECT_DELAY_MS / 1000);
        
        if (wifi_reconnect_timer != NULL && !reconnect_scheduled) {
            reconnect_scheduled = true;
            esp_timer_start_once(wifi_reconnect_timer, WIFI_RECONNECT_DELAY_MS * 1000);  // Convert ms to us
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected! IP address: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/* ----------------------------------------------------------------------------
 * network_init
 * 
 * Initializes WiFi subsystem and connects to network
 * 
 * 1. Creates automatic reconnection timer
 * 2. Initializes network interface (netif)
 * 3. Creates default event loop
 * 4. Configures WiFi station mode
 * 5. Sets WiFi protocol and bandwidth for iPhone compatibility
 * 6. Starts WiFi connection process
 * 
 * Returns: ESP_OK on success, error code otherwise
 * ---------------------------------------------------------------------------- */
esp_err_t network_init(void) {
    // Create reconnect timer
    const esp_timer_create_args_t timer_args = {
        .callback = &wifi_reconnect_timer_callback,
        .arg = NULL,
        .name = "wifi_reconnect"
    };
    
    esp_err_t ret = esp_timer_create(&timer_args, &wifi_reconnect_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create reconnect timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize network interface: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        return ret;
    }
    
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
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
    
    // Configure WiFi credentials and security settings
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,  // Require WPA2 for security
            .pmf_cfg = {
                .capable = true,   // Device supports Protected Management Frames
                .required = false  // But doesn't require it (for compatibility)
            },
        },
    };
    
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set WiFi protocol to support 802.11b/g/n for better iPhone compatibility
    // iPhone hotspots work best with these protocols enabled
    ret = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set WiFi protocol: %s", esp_err_to_name(ret));
        // Continue anyway - not critical
    }
    
    // Set WiFi bandwidth to 20MHz for better compatibility with iPhone hotspots
    // Some iPhone hotspots don't support 40MHz bandwidth
    ret = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set WiFi bandwidth: %s", esp_err_to_name(ret));
        // Continue anyway - not critical
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

/* ----------------------------------------------------------------------------
 * mqtt_event_handler
 * 
 * Handles all MQTT events from the MQTT client
 * 
 * Events handled:
 * - MQTT_EVENT_CONNECTED: Connected to broker -> subscribe to command topic
 * - MQTT_EVENT_DISCONNECTED: Lost connection to broker
 * - MQTT_EVENT_DATA: Received message -> parse JSON and trigger lock/unlock
 * - MQTT_EVENT_ERROR: Connection or communication errors
 * 
 * Command format: {"action":"lock"} or {"action":"unlock"}
 * Sends corresponding events to state machine for processing
 * ---------------------------------------------------------------------------- */
static void mqtt_event_handler(void* handler_args, esp_event_base_t base, 
                                int32_t event_id, void* event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected to broker");
            
            // Subscribe to command topic to receive lock/unlock commands
            int msg_id = esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_COMMAND, 0);
            ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", MQTT_TOPIC_COMMAND, msg_id);
            
            // Publish online status so server knows we're connected
            mqtt_publish(MQTT_TOPIC_STATUS, "{\"status\":\"online\"}");
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected from broker");
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT subscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT unsubscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT message published, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT data received:");
            ESP_LOGI(TAG, "  Topic: %.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG, "  Data: %.*s", event->data_len, event->data);
            
            // Check if this message is on the command topic
            if (strncmp(event->topic, MQTT_TOPIC_COMMAND, event->topic_len) == 0) {
                // Expected format: {"action":"lock"} or {"action":"unlock"}
                
                // Allocate memory for null-terminated JSON string
                char* json_str = (char*)malloc(event->data_len + 1);
                if (json_str == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate memory for JSON parsing");
                    break;
                }
                
                // Copy data and add null terminator for cJSON
                memcpy(json_str, event->data, event->data_len);
                json_str[event->data_len] = '\0';
                
                // Parse the JSON command
                cJSON* json = cJSON_Parse(json_str);
                free(json_str);  // Don't need the string anymore
                
                if (json == NULL) {
                    ESP_LOGE(TAG, "Failed to parse JSON command");
                    const char* error_ptr = cJSON_GetErrorPtr();
                    if (error_ptr != NULL) {
                        ESP_LOGE(TAG, "JSON error before: %s", error_ptr);
                    }
                    break;
                }
                
                // Extract the "action" field from JSON
                cJSON* action = cJSON_GetObjectItem(json, "action");
                if (cJSON_IsString(action) && (action->valuestring != NULL)) {
                    ESP_LOGI(TAG, "Processing command: %s", action->valuestring);
                    
                    // Handle unlock command
                    if (strcmp(action->valuestring, "unlock") == 0) {
                        ESP_LOGI(TAG, "Triggering remote unlock");
                        if (send_sm_event(SM_EVENT_REMOTE_UNLOCK, NULL) == ESP_OK) {
                            ESP_LOGI(TAG, "Remote unlock command sent successfully");
                        } else {
                            ESP_LOGE(TAG, "Failed to send remote unlock command");
                        }
                    // Handle lock command
                    } else if (strcmp(action->valuestring, "lock") == 0) {
                        ESP_LOGI(TAG, "Triggering remote lock");
                        if (send_sm_event(SM_EVENT_REMOTE_LOCK, NULL) == ESP_OK) {
                            ESP_LOGI(TAG, "Remote lock command sent successfully");
                        } else {
                            ESP_LOGE(TAG, "Failed to send remote lock command");
                        }
                    } else {
                        ESP_LOGW(TAG, "Unknown action: %s", action->valuestring);
                    }
                } else {
                    ESP_LOGE(TAG, "Invalid or missing 'action' field in JSON");
                }
                
                cJSON_Delete(json);
            }
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", 
                        event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "Last tls stack error number: 0x%x", 
                        event->error_handle->esp_tls_stack_err);
                ESP_LOGE(TAG, "Last captured errno : %d (%s)", 
                        event->error_handle->esp_transport_sock_errno,
                        strerror(event->error_handle->esp_transport_sock_errno));
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGE(TAG, "Connection refused error: 0x%x", 
                        event->error_handle->connect_return_code);
            }
            break;
            
        default:
            ESP_LOGD(TAG, "MQTT event id: %d", event->event_id);
            break;
    }
}

/* ----------------------------------------------------------------------------
 * mqtt_init
 * 
 * Initializes and starts MQTT client
 * 
 * Prerequisites: WiFi must be connected first
 * 
 * Steps:
 * 1. Verifies WiFi connection
 * 2. Creates MQTT client with broker URI from config
 * 3. Registers event handler
 * 4. Starts MQTT client (will connect asynchronously)
 * 
 * Returns: ESP_OK on success, error code otherwise
 * ---------------------------------------------------------------------------- */
esp_err_t mqtt_init(void) {
    if (!wifi_connected) {
        ESP_LOGW(TAG, "WiFi not connected. Start WiFi before MQTT.");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        // Don't set port separately when using URI - it's already in the URI or uses default
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    esp_err_t ret = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, 
                                                    mqtt_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(ret));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return ret;
    }
    
    ret = esp_mqtt_client_start(mqtt_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return ret;
    }
    
    ESP_LOGI(TAG, "MQTT client started, connecting to %s", MQTT_BROKER_URI);
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * mqtt_publish
 * 
 * Publishes a message to an MQTT topic
 * 
 * Parameters:
 *   topic - MQTT topic to publish to (e.g., "boltlock/status")
 *   message - Message payload (usually JSON string)
 * 
 * Prerequisites: MQTT client must be initialized and connected
 * 
 * Returns: ESP_OK on success, error code otherwise
 * ---------------------------------------------------------------------------- */
esp_err_t mqtt_publish(const char* topic, const char* message) {
    if (topic == NULL || message == NULL) {
        ESP_LOGE(TAG, "Invalid parameters: topic or message is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (mqtt_client == NULL) {
        ESP_LOGW(TAG, "MQTT client not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!mqtt_connected) {
        ESP_LOGW(TAG, "MQTT not connected. Message not sent.");
        return ESP_ERR_INVALID_STATE;
    }
    
    // Publish message with QoS 1 (at least once delivery)
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, message, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish message to topic %s", topic);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Published to %s: %s (msg_id=%d)", topic, message, msg_id);
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * is_network_connected
 * 
 * Checks if WiFi is currently connected
 * 
 * Returns: true if WiFi has IP address, false otherwise
 * ---------------------------------------------------------------------------- */
bool is_network_connected(void) {
    return wifi_connected;
}

/* ----------------------------------------------------------------------------
 * is_mqtt_connected
 * 
 * Checks if MQTT client is currently connected to broker
 * 
 * Returns: true if connected to MQTT broker, false otherwise
 * ---------------------------------------------------------------------------- */
bool is_mqtt_connected(void) {
    return mqtt_connected;
}
