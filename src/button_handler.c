#include "button_handler.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_timer.h"

static const char* TAG = "BUTTON_HANDLER";
static QueueHandle_t button_event_queue = NULL;
static QueueHandle_t door_event_queue = NULL;
static QueueHandle_t gpio_evt_queue = NULL;

static volatile int64_t button_press_time = 0;
static volatile bool button_is_pressed = false;

static volatile int64_t last_door_change_time = 0;

void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void IRAM_ATTR door_sensor_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_event_task(void* arg) {
    uint32_t io_num;
    
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int64_t current_time = esp_timer_get_time() / 1000; // convert to milliseconds
            
            if (io_num == UNLOCK_BUTTON_PIN) {
                int level = gpio_get_level(UNLOCK_BUTTON_PIN);
                
                vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
                
                if (level == gpio_get_level(UNLOCK_BUTTON_PIN)) {
                    if (level == 0) {
                        // falling edge - button pressed
                        button_press_time = current_time;
                        button_is_pressed = true;
                        
                        button_data_t btn_data = {
                            .event = BUTTON_EVENT_PRESS,
                            .press_duration_ms = 0
                        };
                        
                        if (button_event_queue != NULL) {
                            xQueueSend(button_event_queue, &btn_data, 0);
                        }
                        
                        ESP_LOGI(TAG, "Button pressed (ISR)");
                    } else {
                        // rising edge - button released
                        if (button_is_pressed) {
                            uint32_t duration = (uint32_t)(current_time - button_press_time);
                            button_is_pressed = false;
                            
                            button_data_t btn_data;
                            
                            // long press = 2s clicking
                            if (duration > 2000) {
                                btn_data.event = BUTTON_EVENT_LONG_PRESS;
                                ESP_LOGI(TAG, "Button long press (ISR): %lu ms", duration);
                            } else {
                                btn_data.event = BUTTON_EVENT_RELEASE;
                                ESP_LOGI(TAG, "Button released (ISR): %lu ms", duration);
                            }
                            
                            btn_data.press_duration_ms = duration;
                            
                            if (button_event_queue != NULL) {
                                xQueueSend(button_event_queue, &btn_data, 0);
                            }
                        }
                    }
                }
            } else if (io_num == DOOR_SENSOR_PIN) {                
                if (current_time - last_door_change_time > 50) {
                    last_door_change_time = current_time;
                    
                    int level = gpio_get_level(DOOR_SENSOR_PIN);
                    
                    vTaskDelay(pdMS_TO_TICKS(50));
                    
                    if (level == gpio_get_level(DOOR_SENSOR_PIN)) {
                        int door_state = level; // 0=closed, 1=open
                        
                        if (level == 1) {
                            ESP_LOGI(TAG, "Door opened (ISR)");
                        } else {
                            ESP_LOGI(TAG, "Door closed (ISR)");
                        }
                        
                        if (door_event_queue != NULL) {
                            xQueueSend(door_event_queue, &door_state, 0);
                        }
                    }
                }
            }
        }
    }
}

esp_err_t button_handler_init(QueueHandle_t button_queue) {
    button_event_queue = button_queue;
    
    door_event_queue = xQueueCreate(10, sizeof(int));
    if (door_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create door event queue");
        return ESP_FAIL;
    }
    
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue");
        return ESP_FAIL;
    }
    
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = gpio_isr_handler_add(UNLOCK_BUTTON_PIN, button_isr_handler, (void*) UNLOCK_BUTTON_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add button ISR handler: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = gpio_isr_handler_add(DOOR_SENSOR_PIN, door_sensor_isr_handler, (void*) DOOR_SENSOR_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add door sensor ISR handler: %s", esp_err_to_name(ret));
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
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Button handler initialized with ISR");
    return ESP_OK;
}

QueueHandle_t get_door_event_queue(void) {
    return door_event_queue;
}
