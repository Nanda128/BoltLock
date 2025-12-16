/* ============================================================================
 * Lock Control - Controls the Physical Lock Mechanism
 *
 * PWM-based servo control using ESP32 LEDC
 * - Converts angles to PWM duty cycles using 50Hz frequency
 * - Controls status LEDs to indicate lock state
 * - Manages lock state with mutex protection
 * 
 * Also an example of separate threads at diff priorities
 * Control Thread has a priority of 10 since it has real-time servo control with timing constraints
 * Agent Thread has priority 5 since it just does button monitoring, state achine, network etc
 *  
 * Controls:
 * - Status LEDs (show if door is locked or unlocked)
 * - Lock state tracking (remembers current state)
 * - 9g Servo motor to control deadbolt mechanism
 * ============================================================================ */

#include "lock_control.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <inttypes.h>

static const char* TAG = "LOCK_CONTROL";

// Current state of the lock
static lock_state_t current_lock_state = LOCK_STATE_LOCKED;

// Mutex to protect lock_state from being changed by multiple tasks at once
static SemaphoreHandle_t lock_state_mutex = NULL;

#define LEDC_TIMER              LEDC_TIMER_0        // Which timer to use
#define LEDC_MODE               LEDC_LOW_SPEED_MODE // Low speed mode is fine for servo
#define LEDC_CHANNEL            LEDC_CHANNEL_0      // PWM channel
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT   // 14-bit resolution (0-16383 duty values)
#define LEDC_FREQUENCY          (SERVO_PWM_FREQ)    // 50 Hz for standard servo

/* ----------------------------------------------------------------------------
 * servo_angle_to_duty
 * 
 * Converts servo angle (0-180 degrees) to PWM duty cycle
 * 
 * - ~0.5ms pulse = 0 degrees
 * - ~1.5ms pulse = 90 degrees
 * - ~2.5ms pulse = 180 degrees
 * 
 * Calculates the duty cycle needed for a given angle.
 * 
 * Parameters:
 *   angle - desired angle (0-180 degrees)
 * 
 * Returns: duty cycle value for the LEDC peripheral
 * ---------------------------------------------------------------------------- */
static uint32_t servo_angle_to_duty(uint8_t angle) {
    // Limit angle to maximum of 180 degrees
    if (angle > 180) angle = 180;
    
    // Calculate pulse width in microseconds for this angle
    // Linear interpolation between min and max pulse widths
    uint32_t pulse_us = SERVO_MIN_PULSE_US + 
                        (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
    
    // Convert pulse width to duty cycle value
    // duty = (pulse_width_us × frequency × 2^resolution) / 1,000,000
    uint32_t duty = (pulse_us * SERVO_PWM_FREQ * (1 << LEDC_DUTY_RES)) / 1000000;
    
    return duty;
}

/* ----------------------------------------------------------------------------
 * servo_write_angle
 * 
 * Commands the servo to move to a specific angle
 * 
 * Parameters:
 *   angle - target angle (0-180 degrees)
 * 
 * Returns: ESP_OK if successful
 * ---------------------------------------------------------------------------- */
static esp_err_t servo_write_angle(uint8_t angle) {
    // Convert angle to PWM duty cycle
    uint32_t duty = servo_angle_to_duty(angle);
    
    // Set the duty cycle
    esp_err_t ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set servo duty: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Apply the duty cycle change
    ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update servo duty: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "9g Servo moved to %d degrees (duty: %" PRIu32 ")", angle, duty);
    
    // Wait for servo to reach position
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * lock_control_init
 * 
 * Initialize all lock control hardware
 * 
 * Sets up:
 * 1. Mutex for protecting lock state
 * 2. 9g Servo motor PWM control
 * 3. Button GPIO with pull-up and interrupt
 * 4. LED GPIOs for status indication
 * 5. Tests LEDs to verify wiring
 * 
 * Returns: ESP_OK if successful, error code otherwise
 * ---------------------------------------------------------------------------- */
esp_err_t lock_control_init(void) {
    esp_err_t ret;
    
    /* ---- Step 1: Create mutex to protect lock state ---- */
    lock_state_mutex = xSemaphoreCreateMutex();
    if (lock_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create lock state mutex");
        return ESP_FAIL;
    }
    
    /* ---- Step 2: Setup 9g Servo Motor (PWM) ---- */
    
    // Configure PWM timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // 50 Hz for 9g servo
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure PWM channel (output pin)
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LOCK_SERVO_PIN,
        .duty           = 0,  // Start at 0
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Move servo to locked position
    servo_write_angle(SERVO_LOCKED_ANGLE);
    ESP_LOGI(TAG, "9g Servo initialized at LOCKED position (%d degrees)", SERVO_LOCKED_ANGLE);
    
    /* ---- Step 3: Setup Button GPIO ---- */
    gpio_config_t io_conf_button = {
        .pin_bit_mask = (1ULL << TOGGLE_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Enable pull-up (button pulls to ground)
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,    // Interrupt on both press and release
    };
    ret = gpio_config(&io_conf_button);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure button GPIO");
        return ret;
    }
    
    /* ---- Step 4: Setup LED GPIOs ---- */
    gpio_config_t io_conf_leds = {
        .pin_bit_mask = (1ULL << STATUS_LED_BUILTIN) |   // Built-in LED
                        (1ULL << STATUS_LED_EXTERNAL),   // External LED
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
    
    ESP_LOGI(TAG, "LED GPIO configured: BUILTIN=%d, EXTERNAL=%d", 
            STATUS_LED_BUILTIN, STATUS_LED_EXTERNAL);
    
    /* ---- Step 5: Test LEDs ---- */
    // LEDs can be wired two ways:
    // - Common Cathode: HIGH = ON, LOW = OFF
    // - Common Anode:  LOW = ON, HIGH = OFF
    // We test both to help you figure out your wiring
    
    ESP_LOGI(TAG, "Testing LEDs - COMMON CATHODE (HIGH=ON)...");
    gpio_set_level(STATUS_LED_BUILTIN, 1);  // Try turning on
    ESP_LOGI(TAG, "  BUILTIN=HIGH (should be ON)");
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(STATUS_LED_BUILTIN, 0);  // Turn off
    
    gpio_set_level(STATUS_LED_EXTERNAL, 1); // Try turning on
    ESP_LOGI(TAG, "  EXTERNAL=HIGH (should be ON)");
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(STATUS_LED_EXTERNAL, 0); // Turn off
    
    ESP_LOGI(TAG, "Testing LEDs - COMMON ANODE (LOW=ON)...");
    gpio_set_level(STATUS_LED_BUILTIN, 0);  // Try turning on
    ESP_LOGI(TAG, "  BUILTIN=LOW (should be ON)");
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(STATUS_LED_BUILTIN, 1);  // Turn off
    
    gpio_set_level(STATUS_LED_EXTERNAL, 0); // Try turning on
    ESP_LOGI(TAG, "  EXTERNAL=LOW (should be ON)");
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(STATUS_LED_EXTERNAL, 1); // Turn off
    
    ESP_LOGI(TAG, "LED test complete - which lights worked?");
    
    // Set initial state
    current_lock_state = LOCK_STATE_LOCKED;
    
    ESP_LOGI(TAG, "Lock control initialized successfully (Mode: 9g SERVO)");
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * lock_door
 * 
 * Locks the door by moving 9g servo to locked position
 * 
 * Returns: ESP_OK if successful
 * ---------------------------------------------------------------------------- */
esp_err_t lock_door(void) {
    ESP_LOGI(TAG, "Locking door...");
    
    // Move servo to locked angle
    esp_err_t ret = servo_write_angle(SERVO_LOCKED_ANGLE);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Update state (thread-safe)
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        current_lock_state = LOCK_STATE_LOCKED;
        xSemaphoreGive(lock_state_mutex);
    }
    
    // Update LED to show locked state
    update_status_led();
    
    ESP_LOGI(TAG, "Door locked");
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * unlock_door
 * 
 * Unlocks the door by moving 9g servo to unlocked position
 * 
 * Returns: ESP_OK if successful
 * ---------------------------------------------------------------------------- */
esp_err_t unlock_door(void) {
    ESP_LOGI(TAG, "Unlocking door...");
    
    // Check if already unlocked
    lock_state_t current_state = get_lock_state();
    if (current_state == LOCK_STATE_UNLOCKED) {
        ESP_LOGW(TAG, "Lock is already unlocked");
        return ESP_OK;
    }
    
    // Set state to "unlocking" (in progress)
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        current_lock_state = LOCK_STATE_UNLOCKING;
        xSemaphoreGive(lock_state_mutex);
    }
    update_status_led();  // Show "unlocking" status
    
    // Move servo to unlocked angle
    esp_err_t ret = servo_write_angle(SERVO_UNLOCKED_ANGLE);
    if (ret != ESP_OK) {
        // If servo fails, set state to error
        if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
            current_lock_state = LOCK_STATE_ERROR;
            xSemaphoreGive(lock_state_mutex);
        }
        return ret;
    }
    
    // Update state to fully unlocked
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        current_lock_state = LOCK_STATE_UNLOCKED;
        xSemaphoreGive(lock_state_mutex);
    }
    update_status_led();  // Show "unlocked" status
    
    ESP_LOGI(TAG, "Door unlocked");
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * get_lock_state
 * 
 * Returns the current lock state (thread-safe)
 * 
 * Returns: current lock state
 * ---------------------------------------------------------------------------- */
lock_state_t get_lock_state(void) {
    lock_state_t state;
    
    // Safely read the state
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        state = current_lock_state;
        xSemaphoreGive(lock_state_mutex);
    } else {
        state = LOCK_STATE_ERROR;  // Couldn't get mutex
    }
    
    return state;
}

/* ----------------------------------------------------------------------------
 * update_status_led
 * 
 * Updates LEDs based on current lock state
 * 
 * LED patterns:
 * - LOCKED:     External LED ON,  Built-in LED OFF
 * - UNLOCKING:  Both LEDs ON (transitioning)
 * - UNLOCKED:   Built-in LED ON,  External LED OFF
 * - ERROR:      Both LEDs ON (error indication)
 * ---------------------------------------------------------------------------- */
void update_status_led(void) {
    lock_state_t state;
    
    // Get current state safely
    if (xSemaphoreTake(lock_state_mutex, portMAX_DELAY) == pdTRUE) {
        state = current_lock_state;
        xSemaphoreGive(lock_state_mutex);
    } else {
        state = LOCK_STATE_ERROR;
    }
    
    // Set LEDs based on state
    switch (state) {
        case LOCK_STATE_LOCKED:
            gpio_set_level(STATUS_LED_BUILTIN, 0);   // Built-in OFF
            gpio_set_level(STATUS_LED_EXTERNAL, 1);  // External ON
            ESP_LOGI(TAG, "LED: EXTERNAL=ON, BUILTIN=OFF (locked)");
            break;
            
        case LOCK_STATE_UNLOCKING:
            gpio_set_level(STATUS_LED_BUILTIN, 1);   // Built-in ON
            gpio_set_level(STATUS_LED_EXTERNAL, 1);  // External ON
            ESP_LOGI(TAG, "LED: BOTH=ON (unlocking)");
            break;
            
        case LOCK_STATE_UNLOCKED:
            gpio_set_level(STATUS_LED_BUILTIN, 1);   // Built-in ON
            gpio_set_level(STATUS_LED_EXTERNAL, 0);  // External OFF
            ESP_LOGI(TAG, "LED: BUILTIN=ON, EXTERNAL=OFF (unlocked)");
            break;
            
        case LOCK_STATE_ERROR:
            gpio_set_level(STATUS_LED_BUILTIN, 1);   // Built-in ON
            gpio_set_level(STATUS_LED_EXTERNAL, 1);  // External ON
            ESP_LOGI(TAG, "LED: BOTH=ON (error)");
            break;
    }
}

/* ----------------------------------------------------------------------------
 * set_lock_position
 * 
 * Directly set 9g servo position (used by real-time control thread)
 * 
 * This function provides low-level position control without changing
 * the lock state. Used by the RT control thread for smooth control.
 * ---------------------------------------------------------------------------- */
esp_err_t set_lock_position(float angle) {
    // Executes servo movements
    return servo_write_angle((uint8_t)angle);
}
