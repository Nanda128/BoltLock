#include "button_handler.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <inttypes.h>

static const char* TAG = "BUTTON_HANDLER";
static QueueHandle_t button_event_queue = NULL;
static QueueHandle_t gpio_evt_queue = NULL;

typedef struct {
    int64_t press_time;
    bool is_pressed;
    portMUX_TYPE mux;
} button_state_t;

static button_state_t unlock_button = {0, false, portMUX_INITIALIZER_UNLOCKED};
static button_state_t lock_button = {0, false, portMUX_INITIALIZER_UNLOCKED};

static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static bool debounce_gpio(uint32_t gpio_num, int expected_level) {
    const int SAMPLE_COUNT = 5;
    const int SAMPLE_DELAY_MS = BUTTON_DEBOUNCE_MS / SAMPLE_COUNT;
    
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (gpio_get_level(gpio_num) != expected_level) {
            return false;
        }
        if (i < SAMPLE_COUNT - 1) {
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
        }
    }
    return true;
}

static void handle_button_event(uint32_t gpio_num, button_state_t* btn_state) {
    int64_t current_time = esp_timer_get_time() / 1000; // to ms
    int level = gpio_get_level(gpio_num);
    
    if (!debounce_gpio(gpio_num, level)) {
        return;
    }
    
    button_id_t btn_id = (gpio_num == UNLOCK_BUTTON_PIN) ? BUTTON_ID_UNLOCK : BUTTON_ID_LOCK;
    const char* btn_name = (gpio_num == UNLOCK_BUTTON_PIN) ? "UNLOCK" : "LOCK";
    
    if (level == 0) {
        portENTER_CRITICAL(&btn_state->mux);
        btn_state->press_time = current_time;
        btn_state->is_pressed = true;
        portEXIT_CRITICAL(&btn_state->mux);
        
        ESP_LOGI(TAG, "%s button pressed", btn_name);
        
    } else {
        bool is_pressed;
        int64_t press_time;
        
        portENTER_CRITICAL(&btn_state->mux);
        is_pressed = btn_state->is_pressed;
        press_time = btn_state->press_time;
        btn_state->is_pressed = false;
        portEXIT_CRITICAL(&btn_state->mux);
        
        if (is_pressed) {
            int64_t release_time = esp_timer_get_time() / 1000;
            uint32_t duration = (uint32_t)(release_time - press_time);
            
            button_data_t btn_data;
            btn_data.button_id = btn_id;
            btn_data.press_duration_ms = duration;
            
            if (duration > 2000) {
                btn_data.event = BUTTON_EVENT_LONG_PRESS;
                ESP_LOGI(TAG, "%s button long press: %" PRIu32 " ms", btn_name, duration);
            } else {
                btn_data.event = BUTTON_EVENT_PRESS;
                ESP_LOGI(TAG, "%s button released: %" PRIu32 " ms", btn_name, duration);
            }
            
            if (button_event_queue != NULL) {
                xQueueSend(button_event_queue, &btn_data, 0);
            }
        }
    }
}

static void gpio_event_task(void* arg) {
    uint32_t io_num;
    
    ESP_LOGI(TAG, "GPIO event task started");
    
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == UNLOCK_BUTTON_PIN) {
                handle_button_event(io_num, &unlock_button);
                
            } else if (io_num == LOCK_BUTTON_PIN) {
                handle_button_event(io_num, &lock_button);
            }
        }
    }
}

esp_err_t button_handler_init(QueueHandle_t button_queue) {
    button_event_queue = button_queue;
    
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue");
        return ESP_FAIL;
    }
    
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return ret;
    }
    
    ret = gpio_isr_handler_add(UNLOCK_BUTTON_PIN, button_isr_handler, 
                                (void*) UNLOCK_BUTTON_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add unlock button ISR: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return ret;
    }
    
    ret = gpio_isr_handler_add(LOCK_BUTTON_PIN, button_isr_handler, 
                                (void*) LOCK_BUTTON_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add lock button ISR: %s", esp_err_to_name(ret));
        gpio_isr_handler_remove(UNLOCK_BUTTON_PIN);
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return ret;
    }
    
    BaseType_t task_ret = xTaskCreate(
        gpio_event_task,
        "gpio_event",
        4096,
        NULL,
        10,
        NULL
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO event task");
        gpio_isr_handler_remove(LOCK_BUTTON_PIN);
        gpio_isr_handler_remove(UNLOCK_BUTTON_PIN);
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Button handler initialized (unlock + lock buttons)");
    return ESP_OK;
}