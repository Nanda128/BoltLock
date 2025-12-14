#include "buzzer.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "BUZZER";

#define LEDC_TIMER              LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_1
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT
#define LEDC_DUTY               (768)
#define LEDC_FREQUENCY          (2000)

#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_C6  1047
#define NOTE_SILENT 0

typedef struct {
    uint16_t frequency;
    uint16_t duration;
} note_t;

static const note_t unlock_melody[] = {
    {NOTE_C6, 150},
    {NOTE_SILENT, 50},
    {NOTE_C6, 150},
    {0, 0}
};

static const note_t lock_melody[] = {
    {NOTE_E5, 200},
    {0, 0}
};

static const note_t error_melody[] = {
    {NOTE_A5, 100},
    {NOTE_SILENT, 100},
    {NOTE_A5, 100},
    {NOTE_SILENT, 100},
    {NOTE_A5, 100},
    {0, 0}
};

esp_err_t buzzer_init(void) {
    ESP_LOGI(TAG, "Initializing Gravity Digital Speaker Module on GPIO %d", BUZZER_PIN);
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
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
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER_PIN,
        .duty           = 0,
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

static void play_tone(uint16_t frequency, uint16_t duration_ms) {
    if (frequency == 0 || frequency == NOTE_SILENT) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    } else {
        uint16_t adjusted_freq = (frequency * 11) / 10;  // 10% higher?
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, adjusted_freq);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
    
    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
}

static void play_melody(const note_t* melody) {
    int i = 0;
    while (melody[i].frequency != 0 || melody[i].duration != 0) {
        play_tone(melody[i].frequency, melody[i].duration);
        i++;
    }
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void buzzer_play_unlock(void) {
    ESP_LOGI(TAG, "Playing unlock melody");
    play_melody(unlock_melody);
}

void buzzer_play_lock(void) {
    ESP_LOGI(TAG, "Playing lock melody");
    play_melody(lock_melody);
}

void buzzer_play_error(void) {
    ESP_LOGI(TAG, "Playing error melody");
    play_melody(error_melody);
}

void buzzer_beep(uint32_t duration_ms) {
    ESP_LOGI(TAG, "Playing beep for %lu ms", (unsigned long)duration_ms);
    play_tone(NOTE_C6, duration_ms);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}
