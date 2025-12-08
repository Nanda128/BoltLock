#ifndef CONFIG_H
#define CONFIG_H

// I HAVEN'T LOOKED AT THE ACTUAL PIN LAYOUT YET SO THESE ARE JUST ASSUMPTIONS
#define LOCK_RELAY_PIN GPIO_NUM_25    // servo control pin
#define DOOR_SENSOR_PIN GPIO_NUM_26   // reed switch pin
#define UNLOCK_BUTTON_PIN GPIO_NUM_27 // unlock button pin
#define STATUS_LED_PIN GPIO_NUM_2     // status LED pin

#define BUTTON_DEBOUNCE_MS 50      // debounce delay for button
#define AUTO_LOCK_TIMEOUT_MS 5000  // auto-lock timeout duration (5 seconds)
#define DOOR_CHECK_INTERVAL_MS 100 // door sensor check interval
#define TAMPER_THRESHOLD_COUNT 5   // number of rapid door events to trigger tamper
#define TAMPER_THRESHOLD_TIME_MS 3000 // time window for tamper detection (3 seconds)

typedef enum
{
    LOCK_STATE_LOCKED,
    LOCK_STATE_UNLOCKING,
    LOCK_STATE_UNLOCKED,
    LOCK_STATE_ERROR
} lock_state_t;

typedef enum
{
    DOOR_CLOSED,
    DOOR_OPEN,
    DOOR_TAMPERED
} door_state_t;

typedef enum
{
    EVENT_LOCK,
    EVENT_UNLOCK,
    EVENT_DOOR_OPENED,
    EVENT_DOOR_CLOSED,
    EVENT_BUTTON_PRESS,
    EVENT_REMOTE_UNLOCK,
    EVENT_UNAUTHORIZED_ACCESS,
    EVENT_TAMPER_DETECTED
} event_type_t;

// Everything below here is placeholder, we'll flesh it out later

#define WIFI_SSID "PLACEHOLDER_SSID"
#define WIFI_PASSWORD "PLACEHOLDER_PASSWORD"

#define MQTT_BROKER_URI "mqtt://broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC_STATUS "boltlock/status"
#define MQTT_TOPIC_COMMAND "boltlock/command"
#define MQTT_TOPIC_EVENTS "boltlock/events"

#define TELEGRAM_BOT_TOKEN "PLACEHOLDER_BOT_TOKEN"
#define TELEGRAM_CHAT_ID "PLACEHOLDER_CHAT_ID"

#define NVS_NAMESPACE "boltlock"
#define NVS_LOCK_STATE_KEY "lock_state"
#define NVS_EVENT_COUNT_KEY "event_count"

#endif // CONFIG_H
