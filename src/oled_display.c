#include "oled_display.h"
#include "u8g2.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

static const char* TAG = "OLED_DISPLAY";

static u8g2_t u8g2;
static bool oled_initialized = false;

uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    static uint8_t buffer[32];
    static uint8_t buf_idx;
    
    switch(msg) {
        case U8X8_MSG_BYTE_SEND:
            {
                uint8_t *data = (uint8_t *)arg_ptr;
                while (arg_int > 0) {
                    buffer[buf_idx++] = *data;
                    data++;
                    arg_int--;
                }
            }
            break;
            
        case U8X8_MSG_BYTE_INIT:
            // I2C already initialized
            break;
            
        case U8X8_MSG_BYTE_SET_DC:
            // Not used in I2C mode
            break;
            
        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;
            
        case U8X8_MSG_BYTE_END_TRANSFER:
            if (buf_idx > 0) {
                i2c_master_write_to_device(I2C_NUM_0, u8x8_GetI2CAddress(u8x8) >> 1, 
                                            buffer, buf_idx, pdMS_TO_TICKS(1000));
            }
            break;
            
        default:
            return 0;
    }
    return 1;
}

uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch(msg) {
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(pdMS_TO_TICKS(arg_int));
            break;
            
        case U8X8_MSG_DELAY_10MICRO:
            esp_rom_delay_us(arg_int * 10);
            break;
            
        case U8X8_MSG_DELAY_100NANO:
            __asm__ __volatile__("nop");
            break;
            
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            // GPIO initialization already done
            break;
            
        case U8X8_MSG_GPIO_I2C_CLOCK:
        case U8X8_MSG_GPIO_I2C_DATA:
            // not needed for hardware I2C
            break;
            
        default:
            return 0;
    }
    return 1;
}

esp_err_t oled_display_init(void) {
#if !OLED_ENABLED
    ESP_LOGI(TAG, "OLED display disabled in config");
    return ESP_OK;
#endif
    
    esp_err_t ret;
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_SDA_PIN,
        .scl_io_num = OLED_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = OLED_I2C_FREQ,
    };
    
    ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, 
                                            u8g2_esp32_i2c_byte_cb, 
                                            u8g2_esp32_gpio_and_delay_cb);
    
    u8g2_SetI2CAddress(&u8g2, OLED_ADDRESS << 1);
    
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
    
    oled_initialized = true;
    ESP_LOGI(TAG, "OLED display initialized successfully with U8g2 library");
    
    return ESP_OK;
}

void oled_clear(void) {
    if (!oled_initialized) return;
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SendBuffer(&u8g2);
}

void oled_update_status(lock_state_t lock_state, door_state_t door_state, bool wifi_connected) {
    if (!oled_initialized) return;
    
    u8g2_ClearBuffer(&u8g2);
    
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    
    u8g2_DrawStr(&u8g2, 10, 10, "BoltLock v1.0");
    
    u8g2_DrawHLine(&u8g2, 0, 16, 128);
    
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
    
    const char* lock_str = "UNKNOWN";
    switch (lock_state) {
        case LOCK_STATE_LOCKED:    lock_str = "LOCKED";    break;
        case LOCK_STATE_UNLOCKED:  lock_str = "UNLOCKED";  break;
        case LOCK_STATE_UNLOCKING: lock_str = "UNLOCKING"; break;
        case LOCK_STATE_ERROR:     lock_str = "ERROR";     break;
    }
    
    char line2[32];
    snprintf(line2, sizeof(line2), "Lock: %s", lock_str);
    u8g2_DrawStr(&u8g2, 5, 28, line2);
    
    const char* door_str = (door_state == DOOR_CLOSED) ? "CLOSED" : "OPEN";
    char line3[32];
    snprintf(line3, sizeof(line3), "Door: %s", door_str);
    u8g2_DrawStr(&u8g2, 5, 41, line3);
    
    const char* wifi_str = wifi_connected ? "Connected" : "Disconn.";
    char line4[32];
    snprintf(line4, sizeof(line4), "WiFi: %s", wifi_str);
    u8g2_DrawStr(&u8g2, 5, 54, line4);
    
    if (wifi_connected) {
        u8g2_DrawBox(&u8g2, 115, 48, 8, 6);
    }
    
    u8g2_SendBuffer(&u8g2);
    
    ESP_LOGI(TAG, "Status updated: %s | Door:%s | WiFi:%s", lock_str, door_str, wifi_str);
}

void oled_show_message(const char* message, uint32_t duration_ms) {
    if (!oled_initialized) return;
    
    u8g2_ClearBuffer(&u8g2);
    
    u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
    
    char temp[128];
    strncpy(temp, message, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    
    int y = 20;
    int line_num = 0;
    char* line_start = temp;
    
    while (*line_start && line_num < 4) {
        char line[32];
        int len = 0;
        char* word_start = line_start;
        
        while (*word_start && len < 20) {
            while (*word_start == ' ') word_start++;
            if (!*word_start) break;
            
            char* word_end = word_start;
            while (*word_end && *word_end != ' ') word_end++;
            
            int word_len = word_end - word_start;
            if (len + word_len > 20 && len > 0) break;
            
            memcpy(line + len, word_start, word_len);
            len += word_len;
            if (*word_end) {
                line[len++] = ' ';
                word_start = word_end + 1;
            } else {
                word_start = word_end;
                break;
            }
        }
        
        line[len] = '\0';
        if (len > 0) {
            int text_width = u8g2_GetStrWidth(&u8g2, line);
            int x = (128 - text_width) / 2;
            u8g2_DrawStr(&u8g2, x, y, line);
            y += 13;
            line_num++;
        }
        
        line_start = word_start;
    }
    
    u8g2_SendBuffer(&u8g2);
    
    ESP_LOGI(TAG, "Message: %s (duration: %" PRIu32 " ms)", message, duration_ms);
    
    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        oled_clear();
    }
}