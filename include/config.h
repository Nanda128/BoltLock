#ifndef CONFIG_H
#define CONFIG_H

// ==================== HARDWARE PIN CONFIGURATION ====================

#define LOCK_SERVO_PIN GPIO_NUM_25    // Servo signal
#define DOOR_SENSOR_PIN GPIO_NUM_26   // Reed switch (DOOR SENSOR)
#define UNLOCK_BUTTON_PIN GPIO_NUM_27 // UNLOCK BUTTON IS AT THE 2ND BUTTON FROM THE TOP
#define LOCK_BUTTON_PIN GPIO_NUM_14   // LOCK BUTTON IS AT THE 1ST BUTTON ON THE VERY TOP

// Status LED pins
#define STATUS_LED_BUILTIN GPIO_NUM_2
#define STATUS_LED_RED GPIO_NUM_12      // locked LED (on the left)
#define STATUS_LED_UNLOCKED GPIO_NUM_13 // unlocked LED (on the right, physically red)

// ==================== SERVO CONFIGURATION ====================

#define USE_SERVO_MOTOR 1

#if USE_SERVO_MOTOR
#define SERVO_LOCKED_ANGLE 0    // Angle when locked (in degrees)
#define SERVO_UNLOCKED_ANGLE 90 // Angle when unlocked (in degrees)
#define SERVO_PWM_FREQ 50       // Standard servo frequency (Hz)
#define SERVO_MIN_PULSE_US 500  // Minimum pulse width (us)
#define SERVO_MAX_PULSE_US 2500 // Maximum pulse width (us)
#else
#define RELAY_ACTIVE_LOW 0
#endif

// ==================== TIMING CONFIGURATION ====================

#define BUTTON_DEBOUNCE_MS 50      // button debounce delay
#define AUTO_LOCK_TIMEOUT_MS 5000  // auto-lock after 5 seconds
#define DOOR_CHECK_INTERVAL_MS 100 // door sensor polling interval
#define SERVO_MOVE_TIME_MS 500     // time for servo to complete movement

#define TAMPER_THRESHOLD_COUNT 5      // rapid state changes to trigger tamper
#define TAMPER_THRESHOLD_TIME_MS 3000 // time window for tamper detection

// ==================== STATE DEFINITIONS ====================

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

// ==================== NETWORK CONFIGURATION ====================

// UPDATE BEFORE WIFI DEPLOYMENT
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#define MQTT_BROKER_URI "mqtt://broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC_STATUS "boltlock/status"
#define MQTT_TOPIC_COMMAND "boltlock/command"
#define MQTT_TOPIC_EVENTS "boltlock/events"

#define TELEGRAM_BOT_TOKEN "8211750808:AAHjzySteM0FmwT3G3QCZfk2xbQWTI0TLuQ"
#define TELEGRAM_CHAT_ID "7286752705"

#define NVS_NAMESPACE "boltlock"
#define NVS_LOCK_STATE_KEY "lock_state"
#define NVS_EVENT_COUNT_KEY "event_count"

#endif // CONFIG_H