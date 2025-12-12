#include "lock_control.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <inttypes.h>

static const char* TAG = "LOCK_CONTROL";
static lock_state_t current_lock_state = LOCK_STATE_LOCKED;
static SemaphoreHandle_t lock_state_mutex = NULL;

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT
#define LEDC_FREQUENCY          (SERVO_PWM_FREQ)

static uint32_t servo_angle_to_duty(uint8_t angle) {
    if (angle > 180) angle = 180;
    
    uint32_t pulse_us = SERVO_MIN_PULSE_US + 
                        (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
    
    // duty = (pulse x frequency x 2^14) / 1000000
    uint32_t duty = (pulse_us * SERVO_PWM_FREQ * (1 << LEDC_DUTY_RES)) / 1000000;
    
    return duty;
}

static esp_err_t servo_write_angle(uint8_t angle) {
#if USE_SERVO_MOTOR
    uint32_t duty = servo_angle_to_duty(angle);
    esp_err_t ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo duty: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update servo duty: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Servo moved to %d degrees (duty: %" PRIu32 ")", angle, duty);
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t lock_control_init(void) {
    esp_err_t ret;
    
    lock_state_mutex = xSemaphoreCreateMutex();
    if (lock_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create lock state mutex");
        return ESP_FAIL;
    }
    
#if USE_SERVO_MOTOR
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LOCK_SERVO_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    servo_write_angle(SERVO_LOCKED_ANGLE);
    ESP_LOGI(TAG, "Servo initialized at LOCKED position (%d degrees)", SERVO_LOCKED_ANGLE);
#else
    gpio_config_t io_conf_output = {
        .pin_bit_mask = (1ULL << LOCK_SERVO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_conf_output);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure relay GPIO");
        return ret;
    }
    
#if RELAY_ACTIVE_LOW
    gpio_set_level(LOCK_SERVO_PIN, 1); 
#else
    gpio_set_level(LOCK_SERVO_PIN, 0); 
#endif
    ESP_LOGI(TAG, "Relay initialized to LOCKED state");
#endif
    
    gpio_config_t io_conf_input = {
        .pin_bit_mask = (1ULL << DOOR_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ret = gpio_config(&io_conf_input);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure door sensor GPIO");
        return ret;
    }
    
    gpio_config_t io_conf_button = {
        .pin_bit_mask = (1ULL << UNLOCK_BUTTON_PIN) | (1ULL << LOCK_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ret = gpio_config(&io_conf_button);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure button GPIO");
        return ret;
    }
    
    gpio_config_t io_conf_leds = {
        .pin_bit_mask = (1ULL << STATUS_LED_BUILTIN) | 
                        (1ULL << STATUS_LED_RED) | 
                        (1ULL << STATUS_LED_UNLOCKED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_conf_leds);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED GPIO");
        return ret;
    }
    
    gpio_set_level(STATUS_LED_BUILTIN, 0);
    gpio_set_level(STATUS_LED_RED, 1); 
    gpio_set_level(STATUS_LED_UNLOCKED, 0); 
    
    current_lock_state = LOCK_STATE_LOCKED;
    
    ESP_LOGI(TAG, "Lock control initialized successfully (Mode: %s)", 
                USE_SERVO_MOTOR ? "SERVO" : "RELAY");
    return ESP_OK;
}

esp_err_t lock_door(void) {
    ESP_LOGI(TAG, "Locking door...");
    
#if USE_SERVO_MOTOR
    esp_err_t ret = servo_write_angle(SERVO_LOCKED_ANGLE);
    if (ret != ESP_OK) {
        return ret;
    }
#else
#if RELAY_ACTIVE_LOW
    gpio_set_level(LOCK_SERVO_PIN, 1); 
#else
    gpio_set_level(LOCK_SERVO_PIN, 0); 
#endif
    vTaskDelay(pdMS_TO_TICKS(500));
#endif
    
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        current_lock_state = LOCK_STATE_LOCKED;
        xSemaphoreGive(lock_state_mutex);
    }
    
    update_status_led();
    
    ESP_LOGI(TAG, "Door locked");
    return ESP_OK;
}

esp_err_t unlock_door(void) {
    ESP_LOGI(TAG, "Unlocking door...");
    
    lock_state_t current_state = get_lock_state();
    if (current_state == LOCK_STATE_UNLOCKED) {
        ESP_LOGW(TAG, "Lock is already unlocked");
        return ESP_OK;
    }
    
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        current_lock_state = LOCK_STATE_UNLOCKING;
        xSemaphoreGive(lock_state_mutex);
    }
    update_status_led();
    
#if USE_SERVO_MOTOR
    esp_err_t ret = servo_write_angle(SERVO_UNLOCKED_ANGLE);
    if (ret != ESP_OK) {
        if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
            current_lock_state = LOCK_STATE_ERROR;
            xSemaphoreGive(lock_state_mutex);
        }
        return ret;
    }
#else
#if RELAY_ACTIVE_LOW
    gpio_set_level(LOCK_SERVO_PIN, 0);
#else
    gpio_set_level(LOCK_SERVO_PIN, 1);
#endif
    vTaskDelay(pdMS_TO_TICKS(500));
#endif
    
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        current_lock_state = LOCK_STATE_UNLOCKED;
        xSemaphoreGive(lock_state_mutex);
    }
    update_status_led();
    
    ESP_LOGI(TAG, "Door unlocked");
    return ESP_OK;
}

lock_state_t get_lock_state(void) {
    lock_state_t state;
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        state = current_lock_state;
        xSemaphoreGive(lock_state_mutex);
    } else {
        state = LOCK_STATE_ERROR;
    }
    return state;
}

door_state_t get_door_state(void) {
    int level = gpio_get_level(DOOR_SENSOR_PIN);
    return (level == 0) ? DOOR_CLOSED : DOOR_OPEN;
}

void update_status_led(void) {
    lock_state_t state;
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        state = current_lock_state;
        xSemaphoreGive(lock_state_mutex);
    } else {
        state = LOCK_STATE_ERROR;
    }
    
    switch (state) {
        case LOCK_STATE_LOCKED:
            gpio_set_level(STATUS_LED_BUILTIN, 0);
            gpio_set_level(STATUS_LED_RED, 1); 
            gpio_set_level(STATUS_LED_UNLOCKED, 0); 
            break;
            
        case LOCK_STATE_UNLOCKING:
            gpio_set_level(STATUS_LED_BUILTIN, 1);
            gpio_set_level(STATUS_LED_RED, 1);
            gpio_set_level(STATUS_LED_UNLOCKED, 1);
            break;
            
        case LOCK_STATE_UNLOCKED:
            gpio_set_level(STATUS_LED_BUILTIN, 1);
            gpio_set_level(STATUS_LED_RED, 0); 
            gpio_set_level(STATUS_LED_UNLOCKED, 1); 
            break;
            
        case LOCK_STATE_ERROR:
            gpio_set_level(STATUS_LED_BUILTIN, 1);
            gpio_set_level(STATUS_LED_RED, 1);
            gpio_set_level(STATUS_LED_UNLOCKED, 1);
            break;
    }
}