#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"

#define LOCK_SERVO_PIN GPIO_NUM_25
#define TOGGLE_BUTTON_PIN GPIO_NUM_27 // TOGGLE BUTTON 

#define BUZZER_PIN GPIO_NUM_26        

#define STATUS_LED_BUILTIN GPIO_NUM_2
#define STATUS_LED_EXTERNAL GPIO_NUM_13 

#define USE_SERVO_MOTOR 1

#if USE_SERVO_MOTOR
#define SERVO_LOCKED_ANGLE 0
#define SERVO_UNLOCKED_ANGLE 90
#define SERVO_PWM_FREQ 50
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#else
#define RELAY_ACTIVE_LOW 0
#endif

#define BUTTON_DEBOUNCE_MS 50      // button debounce delay
#define AUTO_LOCK_TIMEOUT_MS 30000  // auto-lock after 30 seconds
#define SERVO_MOVE_TIME_MS 500     // time for servo to complete movement

#define USE_EMOJI_FORMAT 1         // use emojis in event messages (1=emoji, 0=text)

typedef enum
{
    LOCK_STATE_LOCKED,
    LOCK_STATE_UNLOCKING,
    LOCK_STATE_UNLOCKED,
    LOCK_STATE_ERROR
} lock_state_t;

typedef enum
{
    EVENT_LOCK,
    EVENT_UNLOCK,
    EVENT_BUTTON_PRESS,
    EVENT_REMOTE_UNLOCK
} event_type_t;

#define WIFI_SSID "Tom's Galaxy S21"
#define WIFI_PASSWORD "tiernans"

#define MQTT_BROKER_URI "mqtt://alderaan.software-engineering.ie"
#define MQTT_PORT 1883
#define MQTT_TOPIC_BASE "BoltLock"
#define MQTT_TOPIC_STATUS "BoltLock/status"
#define MQTT_TOPIC_COMMAND "BoltLock/command"
#define MQTT_TOPIC_EVENTS "BoltLock/events"

#define TELEGRAM_BOT_TOKEN "8211750808:AAHjzySteM0FmwT3G3QCZfk2xbQWTI0TLuQ"
#define TELEGRAM_CHAT_ID "7286752705"

#define NVS_NAMESPACE "boltlock"
#define NVS_LOCK_STATE_KEY "lock_state"
#define NVS_EVENT_COUNT_KEY "event_count"

#endif // CONFIG_H