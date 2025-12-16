/* ============================================================================
 * Button Handler - Detects and Processes Button Presses
 * 
 * - Monitors the physical button connected to the ESP32
 * - Detects when button is pressed and released
 * - Figures out if it's a short press or long press
 * - Sends button events to the main program
 * 
 * How it works:
 * 1. When button state changes (pressed/released), hardware triggers interrupt
 * 2. Interrupt puts event in a queue
 * 3. Background task reads from queue and processes the button event
 * 4. Task measures how long button was held
 * 5. Sends appropriate event (press or long press) to main program
 * ============================================================================ */

#include "button_handler.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <inttypes.h>

static const char* TAG = "BUTTON_HANDLER";

// Queue to send button events to main program
static QueueHandle_t button_event_queue = NULL;

// Queue for GPIO interrupt events (internal use)
static QueueHandle_t gpio_evt_queue = NULL;

/* Button state tracking structure
 * This keeps track of when button was pressed and if it's currently pressed */
typedef struct {
    int64_t press_time;      // When was button pressed (in milliseconds)
    bool is_pressed;         // Is button currently being held down?
    portMUX_TYPE mux;        // Lock to protect this data from race conditions
} button_state_t;

// State for the toggle button
static button_state_t toggle_button = {0, false, portMUX_INITIALIZER_UNLOCKED};

/* ----------------------------------------------------------------------------
 * button_isr_handler
 * 
 * ISR runs immediately when button changes
 * 
 * - Gets called automatically by hardware when button state changes
 * - Puts the GPIO pin number in a queue
 * ---------------------------------------------------------------------------- */
static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;  // Which pin triggered this?
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);  // Send to queue
}

/* ----------------------------------------------------------------------------
 * debounce_gpio
 * 
 * Debounces button input to avoid false triggers
 * 
 * Returns: true if button is stable in expected_level, false otherwise
 * ---------------------------------------------------------------------------- */
static bool debounce_gpio(uint32_t gpio_num, int expected_level) {
    const int SAMPLE_COUNT = 5;  // Check 5 times
    const int SAMPLE_DELAY_MS = BUTTON_DEBOUNCE_MS / SAMPLE_COUNT;
    
    // Check the button multiple times
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        if (gpio_get_level(gpio_num) != expected_level) {
            return false;  // Button not stable, it's bouncing
        }
        if (i < SAMPLE_COUNT - 1) {
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));  // Wait a bit
        }
    }
    return true;  // Button is stable!
}

/* ----------------------------------------------------------------------------
 * handle_button_event
 * 
 * Processes a button state change (press or release)
 * 
 * What it does:
 * 1. Check if button state is stable (debounce)
 * 2. If button pressed: record the time
 * 3. If button released: calculate how long it was held
 * 4. Send press or long-press event to main program
 * ---------------------------------------------------------------------------- */
static void handle_button_event(uint32_t gpio_num, button_state_t* btn_state) {
    int64_t current_time = esp_timer_get_time() / 1000;  // Get time in milliseconds
    int level = gpio_get_level(gpio_num);  // Read current button state (0=pressed, 1=released)
    
    // Make sure button is stable (not bouncing)
    if (!debounce_gpio(gpio_num, level)) {
        return;  // Button bouncing, ignore this event
    }
    
    // Button was just PRESSED 
    if (level == 0) {
        // Safely update state (protected from interrupts)
        portENTER_CRITICAL(&btn_state->mux);
        btn_state->press_time = current_time;
        btn_state->is_pressed = true;
        portEXIT_CRITICAL(&btn_state->mux);
        
        ESP_LOGI(TAG, "Toggle button pressed");
        
    } 
    // Button was just RELEASED (level = 1)
    else {
        bool is_pressed;
        int64_t press_time;
        
        // Safely read state
        portENTER_CRITICAL(&btn_state->mux);
        is_pressed = btn_state->is_pressed;
        press_time = btn_state->press_time;
        btn_state->is_pressed = false;
        portEXIT_CRITICAL(&btn_state->mux);
        
        // Only process if we recorded a press
        if (is_pressed) {
            int64_t release_time = esp_timer_get_time() / 1000;
            uint32_t duration = (uint32_t)(release_time - press_time);
            
            button_data_t btn_data;
            btn_data.press_duration_ms = duration;
            btn_data.event = BUTTON_EVENT_PRESS;
            
            ESP_LOGI(TAG, "Toggle button released: %" PRIu32 " ms", duration);
            
            // Send event to main program
            if (button_event_queue != NULL) {
                xQueueSend(button_event_queue, &btn_data, 0);
            }
        }
    }
}

/* ----------------------------------------------------------------------------
 * gpio_event_task
 * 
 * Background task that processes button events
 * 
 * This runs continuously in the background, waiting for GPIO events
 * from the interrupt handler, then processing them.
 * ---------------------------------------------------------------------------- */
static void gpio_event_task(void* arg) {
    uint32_t io_num;
    
    ESP_LOGI(TAG, "GPIO event task started");
    
    // Run forever
    while (1) {
        // Wait for GPIO event from ISR
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if (io_num == TOGGLE_BUTTON_PIN) {
                handle_button_event(io_num, &toggle_button);
            }
        }
    }
}

/* ----------------------------------------------------------------------------
 * button_handler_init
 * 
 * Initialize the button handler system
 * 
 * Sets up:
 * - Queues for passing events
 * - GPIO interrupt service
 * - Background task to process events
 * 
 * Parameters:
 *   button_queue - where to send button events for the main program
 * 
 * Returns: ESP_OK if successful, error code otherwise
 * ---------------------------------------------------------------------------- */
esp_err_t button_handler_init(QueueHandle_t button_queue) {
    button_event_queue = button_queue;  // Save queue for later use
    
    // Create queue for GPIO interrupt events (holds up to 10 events)
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create GPIO event queue");
        return ESP_FAIL;
    }
    
    // Install GPIO interrupt service (allows GPIO pins to trigger interrupts)
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return ret;
    }
    
    // Attach interrupt handler to the toggle button pin
    ret = gpio_isr_handler_add(TOGGLE_BUTTON_PIN, button_isr_handler, 
                                (void*) TOGGLE_BUTTON_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add toggle button ISR: %s", esp_err_to_name(ret));
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return ret;
    }
    
    // Create background task to process GPIO events
    BaseType_t task_ret = xTaskCreate(
        gpio_event_task,    // Function to run
        "gpio_event",       // Task name
        4096,               // Stack size (memory for the task)
        NULL,               // Parameters (none)
        10,                 // Priority high
        NULL                // Task handle (not needed)
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO event task");
        gpio_isr_handler_remove(TOGGLE_BUTTON_PIN);
        vQueueDelete(gpio_evt_queue);
        gpio_evt_queue = NULL;
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Button handler initialized (single toggle button)");
    return ESP_OK;
}