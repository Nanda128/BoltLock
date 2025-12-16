#include "network_handler.h"
#include "config.h"
#include "state_machine.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include <string.h>

static const char* TAG = "NETWORK";
static bool wifi_connected = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

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
    esp_err_t ret = esp_netif_init();
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
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, 
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set WiFi protocol to support 802.11b/g/n for better iPhone compatibility
    ret = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set WiFi protocol: %s", esp_err_to_name(ret));
        // Continue anyway - not critical
    }
    
    // Set WiFi bandwidth to 20MHz for better compatibility with iPhone hotspots
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

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, 
                                int32_t event_id, void* event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected to broker");
            
            int msg_id = esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_COMMAND, 0);
            ESP_LOGI(TAG, "Subscribed to %s, msg_id=%d", MQTT_TOPIC_COMMAND, msg_id);
            
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
            
            if (strncmp(event->topic, MQTT_TOPIC_COMMAND, event->topic_len) == 0) {
                // Expected format: {"action":"lock"} or {"action":"unlock"}
                
                char* json_str = (char*)malloc(event->data_len + 1);
                if (json_str == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate memory for JSON parsing");
                    break;
                }
                
                memcpy(json_str, event->data, event->data_len);
                json_str[event->data_len] = '\0';
                
                cJSON* json = cJSON_Parse(json_str);
                free(json_str);
                
                if (json == NULL) {
                    ESP_LOGE(TAG, "Failed to parse JSON command");
                    const char* error_ptr = cJSON_GetErrorPtr();
                    if (error_ptr != NULL) {
                        ESP_LOGE(TAG, "JSON error before: %s", error_ptr);
                    }
                    break;
                }
                
                cJSON* action = cJSON_GetObjectItem(json, "action");
                if (cJSON_IsString(action) && (action->valuestring != NULL)) {
                    ESP_LOGI(TAG, "Processing command: %s", action->valuestring);
                    
                    if (strcmp(action->valuestring, "unlock") == 0) {
                        ESP_LOGI(TAG, "Triggering remote unlock");
                        if (send_sm_event(SM_EVENT_REMOTE_UNLOCK, NULL) == ESP_OK) {
                            ESP_LOGI(TAG, "Remote unlock command sent successfully");
                        } else {
                            ESP_LOGE(TAG, "Failed to send remote unlock command");
                        }
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
    
    int msg_id = esp_mqtt_client_publish(mqtt_client, topic, message, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish message to topic %s", topic);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Published to %s: %s (msg_id=%d)", topic, message, msg_id);
    return ESP_OK;
}

bool is_network_connected(void) {
    return wifi_connected;
}

bool is_mqtt_connected(void) {
    return mqtt_connected;
}
