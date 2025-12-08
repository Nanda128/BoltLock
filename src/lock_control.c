#include "lock_control.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char* TAG = "LOCK_CONTROL";
static lock_state_t current_lock_state = LOCK_STATE_LOCKED;
static SemaphoreHandle_t lock_state_mutex = NULL;

esp_err_t lock_control_init(void) {
    esp_err_t ret;
    
    // mutex to stop race conditions on lock state
    lock_state_mutex = xSemaphoreCreateMutex();
    if (lock_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create lock state mutex");
        return ESP_FAIL;
    }
    
    gpio_config_t io_conf_output = {
        .pin_bit_mask = (1ULL << LOCK_RELAY_PIN) | (1ULL << STATUS_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_conf_output);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure output GPIO");
        return ret;
    }
    
    gpio_config_t io_conf_input = {
        .pin_bit_mask = (1ULL << DOOR_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // reed switch pull up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,  
    };
    ret = gpio_config(&io_conf_input);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure input GPIO");
        return ret;
    }
    
    gpio_config_t io_conf_button = {
        .pin_bit_mask = (1ULL << UNLOCK_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // button pull up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,  // button press and release interrupt
    };
    ret = gpio_config(&io_conf_button);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure button GPIO");
        return ret;
    }
    
    gpio_set_level(LOCK_RELAY_PIN, 0);  // 0 = locked 
    current_lock_state = LOCK_STATE_LOCKED;
    
    gpio_set_level(STATUS_LED_PIN, 0);
    
    ESP_LOGI(TAG, "Lock control initialized successfully");
    return ESP_OK;
}

esp_err_t lock_door(void) {
    ESP_LOGI(TAG, "Locking door...");
    
    gpio_set_level(LOCK_RELAY_PIN, 0);
    
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
    
    // High=unlock
    gpio_set_level(LOCK_RELAY_PIN, 1);
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
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
    // LOW when closed, HIGH when open
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
            gpio_set_level(STATUS_LED_PIN, 0); // turn off LED when it's locked
            break;
            
        case LOCK_STATE_UNLOCKING:
            gpio_set_level(STATUS_LED_PIN, 1); // blink when moving between locked and unlocked
            break;
            
        case LOCK_STATE_UNLOCKED:
            gpio_set_level(STATUS_LED_PIN, 1); // turn on LED when it's unlocked
            break;
            
        case LOCK_STATE_ERROR:
            gpio_set_level(STATUS_LED_PIN, 1); // blink for error 
            break;
    }
} // UNLOCKING and ERROR states are handled by the state machine 
