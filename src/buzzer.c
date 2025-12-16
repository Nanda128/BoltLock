/* ============================================================================
 * Buzzer Control - Plays Sounds Through Speaker
 * 
 * This controls a small speaker (buzzer) that plays different sounds:
 * - Unlock sound: beepbeep
 * - Lock sound: beep
 * - Error sound: beepbeepbeep
 * - Simple beep: beeeeeeeep(200ms)
 * ^^^ this one is for testing
 * 
 * - Uses PWM (Pulse Width Modulation) to generate different frequencies
 * - Different frequencies = different musical notes
 * - Turning PWM on/off quickly creates sound waves
 * - Higher frequency = higher pitch sound
 * 
 * Musical notes used:
 * - C5, D5, E5, G5, A5, C6 (middle to high range notes)
 * ============================================================================ */

#include "buzzer.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "BUZZER";

#define LEDC_TIMER              LEDC_TIMER_1        // Use timer 1 (timer 0 is for servo)
#define LEDC_MODE               LEDC_LOW_SPEED_MODE // Low speed is fine for audio
#define LEDC_CHANNEL            LEDC_CHANNEL_1      // Use channel 1
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT   // 10-bit resolution (0-1023)
#define LEDC_DUTY               (768)               // 75% duty cycle (768/1024)
#define LEDC_FREQUENCY          (2000)              // Default 2000 Hz

#define NOTE_C5  523      // Middle C
#define NOTE_D5  587      // D
#define NOTE_E5  659      // E
#define NOTE_G5  784      // G
#define NOTE_A5  880      // A
#define NOTE_C6  1047     // High C
#define NOTE_SILENT 0     // No sound

typedef struct {
    uint16_t frequency;   // Note frequency in Hz (0 = silence)
    uint16_t duration;    // How long to play in milliseconds
} note_t;


// Unlock melody: beepbeep
static const note_t unlock_melody[] = {
    {NOTE_C6, 150},        // High C for 150ms
    {NOTE_SILENT, 50},     // Silence for 50ms
    {NOTE_C6, 150},        // High C again for 150ms
    {0, 0}                 // End marker (both 0)
};

// Lock melody: beep
static const note_t lock_melody[] = {
    {NOTE_E5, 200},        // E note for 200ms
    {0, 0}                 // End marker
};

// Error melody: beepbeepbeep
static const note_t error_melody[] = {
    {NOTE_A5, 100},        // A note for 100ms
    {NOTE_SILENT, 100},    // Silence
    {NOTE_A5, 100},        // A note
    {NOTE_SILENT, 100},    // Silence
    {NOTE_A5, 100},        // A note
    {0, 0}                 // End marker
};

/* ----------------------------------------------------------------------------
 * buzzer_init
 * 
 * Initialize the buzzer/speaker hardware
 * 
 * Sets up PWM to generate audio frequencies
 * 
 * Returns: ESP_OK if successful
 * ---------------------------------------------------------------------------- */
esp_err_t buzzer_init(void) {
    ESP_LOGI(TAG, "Initializing Gravity Digital Speaker Module on GPIO %d", BUZZER_PIN);
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,     // 10-bit resolution
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,    // Default frequency
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer");
        return ret;
    }
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,   // No interrupts needed
        .gpio_num       = BUZZER_PIN,          // Output pin
        .duty           = 0,                   // Start silent (0% duty)
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC channel");
        return ret;
    }
    
    ESP_LOGI(TAG, "Speaker module initialized successfully");
    return ESP_OK;
}

/* ----------------------------------------------------------------------------
 * play_tone
 * 
 * Plays a single tone at a specific frequency for a duration
 * 
 * How it works:
 * 1. Sets PWM frequency to the note's frequency
 * 2. Sets PWM duty cycle to 75% (makes sound)
 * 3. Waits for the duration
 * 4. For silence, sets duty cycle to 0%
 * 
 * Parameters:
 *   frequency - frequency in Hz (0 or NOTE_SILENT = silence)
 *   duration_ms - how long to play in milliseconds
 * ---------------------------------------------------------------------------- */
static void play_tone(uint16_t frequency, uint16_t duration_ms) {
    if (frequency == 0 || frequency == NOTE_SILENT) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    } else {
        uint16_t adjusted_freq = (frequency * 11) / 10;
        
        // Set the frequency
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, adjusted_freq);
        
        // Set duty cycle to 75% (makes the sound)
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
    
    // Wait for the duration
    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
}

/* ----------------------------------------------------------------------------
 * play_melody
 * 
 * Plays a sequence of notes (a melody)
 * 
 * Reads through an array of notes and plays each one
 * Stops when it finds a note with both frequency and duration = 0
 * 
 * Parameters:
 *   melody - array of notes to play
 * ---------------------------------------------------------------------------- */
static void play_melody(const note_t* melody) {
    int i = 0;
    
    // Play notes until we hit the end marker {0, 0}
    while (melody[i].frequency != 0 || melody[i].duration != 0) {
        play_tone(melody[i].frequency, melody[i].duration);
        i++;
    }
    
    // Turn off sound when done
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

/* ----------------------------------------------------------------------------
 * buzzer_play_unlock
 * 
 * Plays the unlock sound (two quick high beeps)
 * ---------------------------------------------------------------------------- */
void buzzer_play_unlock(void) {
    ESP_LOGI(TAG, "Playing unlock melody");
    play_melody(unlock_melody);
}

/* ----------------------------------------------------------------------------
 * buzzer_play_lock
 * 
 * Plays the lock sound (one medium beep)
 * ---------------------------------------------------------------------------- */
void buzzer_play_lock(void) {
    ESP_LOGI(TAG, "Playing lock melody");
    play_melody(lock_melody);
}

/* ----------------------------------------------------------------------------
 * buzzer_play_error
 * 
 * Plays the error sound (three rapid beeps)
 * ---------------------------------------------------------------------------- */
void buzzer_play_error(void) {
    ESP_LOGI(TAG, "Playing error melody");
    play_melody(error_melody);
}

/* ----------------------------------------------------------------------------
 * buzzer_beep
 * 
 * Plays a simple beep for any duration you specify
 * 
 * Useful for testing or generic notifications
 * 
 * Parameters:
 *   duration_ms - how long to beep in milliseconds
 * ---------------------------------------------------------------------------- */
void buzzer_beep(uint32_t duration_ms) {
    ESP_LOGI(TAG, "Playing beep for %lu ms", (unsigned long)duration_ms);
    
    // Play a high C note
    play_tone(NOTE_C6, duration_ms);
    
    // Turn off when done
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}
