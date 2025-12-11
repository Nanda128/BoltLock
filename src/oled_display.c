#include "oled_display.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char* TAG = "OLED_DISPLAY";

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_PAGES 8  // 64 pixels / 8 rows per page

#define SSD1306_CMD_DISPLAY_OFF 0xAE
#define SSD1306_CMD_DISPLAY_ON 0xAF
#define SSD1306_CMD_SET_CONTRAST 0x81
#define SSD1306_CMD_SET_ADDRESSING_MODE 0x20
#define SSD1306_CMD_SET_COLUMN_ADDR 0x21
#define SSD1306_CMD_SET_PAGE_ADDR 0x22
#define SSD1306_CMD_SET_START_LINE 0x40
#define SSD1306_CMD_SET_SEGMENT_REMAP 0xA1
#define SSD1306_CMD_SET_MULTIPLEX 0xA8
#define SSD1306_CMD_SET_COM_SCAN_DEC 0xC8
#define SSD1306_CMD_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_CMD_SET_COM_PINS 0xDA
#define SSD1306_CMD_SET_PRECHARGE 0xD9
#define SSD1306_CMD_SET_VCOMH 0xDB
#define SSD1306_CMD_CHARGE_PUMP 0x8D
#define SSD1306_CMD_DEACTIVATE_SCROLL 0x2E

static bool oled_initialized = false;
static uint8_t display_buffer[SCREEN_WIDTH * SCREEN_PAGES];

static const uint8_t font5x7[][5] = { // oh my god my hands
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x56, 0x20, 0x50}, // &
    {0x00, 0x08, 0x07, 0x03, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x2A, 0x1C, 0x7F, 0x1C, 0x2A}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x80, 0x70, 0x30, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x00, 0x60, 0x60, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x72, 0x49, 0x49, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x49, 0x4D, 0x33}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x31}, // 6
    {0x41, 0x21, 0x11, 0x09, 0x07}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x46, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x00, 0x14, 0x00, 0x00}, // :
    {0x00, 0x40, 0x34, 0x00, 0x00}, // ;
    {0x00, 0x08, 0x14, 0x22, 0x41}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x59, 0x09, 0x06}, // ?
    {0x3E, 0x41, 0x5D, 0x59, 0x4E}, // @
    {0x7C, 0x12, 0x11, 0x12, 0x7C}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x41, 0x51, 0x73}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x1C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x26, 0x49, 0x49, 0x49, 0x32}, // S
    {0x03, 0x01, 0x7F, 0x01, 0x03}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // Y
    {0x61, 0x59, 0x49, 0x4D, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x41}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // backslash
    {0x00, 0x41, 0x41, 0x41, 0x7F}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x03, 0x07, 0x08, 0x00}, // `
    {0x20, 0x54, 0x54, 0x78, 0x40}, // a
    {0x7F, 0x28, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x28}, // c
    {0x38, 0x44, 0x44, 0x28, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x00, 0x08, 0x7E, 0x09, 0x02}, // f
    {0x18, 0xA4, 0xA4, 0x9C, 0x78}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x40, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x78, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0xFC, 0x18, 0x24, 0x24, 0x18}, // p
    {0x18, 0x24, 0x24, 0x18, 0xFC}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x24}, // s
    {0x04, 0x04, 0x3F, 0x44, 0x24}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x4C, 0x90, 0x90, 0x90, 0x7C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // {
    {0x00, 0x00, 0x77, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // }
    {0x02, 0x01, 0x02, 0x04, 0x02}, // ~
};

static esp_err_t ssd1306_write_command(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd}; // Control byte (Co=0, D/C=0) + command
    return i2c_master_write_to_device(I2C_NUM_0, OLED_ADDRESS, data, 2, pdMS_TO_TICKS(1000));
}

static esp_err_t ssd1306_write_data(uint8_t* data, size_t len) {
    uint8_t* buffer = malloc(len + 1);
    if (!buffer) return ESP_ERR_NO_MEM;
    
    buffer[0] = 0x40; // Control byte (Co=0, D/C=1) for data
    memcpy(buffer + 1, data, len);
    
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, OLED_ADDRESS, buffer, len + 1, pdMS_TO_TICKS(1000));
    free(buffer);
    return ret;
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
    
    ssd1306_write_command(SSD1306_CMD_DISPLAY_OFF);
    ssd1306_write_command(SSD1306_CMD_SET_MULTIPLEX);
    ssd1306_write_command(0x3F);
    ssd1306_write_command(SSD1306_CMD_SET_DISPLAY_OFFSET);
    ssd1306_write_command(0x00);
    ssd1306_write_command(SSD1306_CMD_SET_START_LINE | 0x00);
    ssd1306_write_command(SSD1306_CMD_CHARGE_PUMP);
    ssd1306_write_command(0x14);
    ssd1306_write_command(SSD1306_CMD_SET_ADDRESSING_MODE);
    ssd1306_write_command(0x00);
    ssd1306_write_command(SSD1306_CMD_SET_SEGMENT_REMAP);
    ssd1306_write_command(SSD1306_CMD_SET_COM_SCAN_DEC);
    ssd1306_write_command(SSD1306_CMD_SET_COM_PINS);
    ssd1306_write_command(0x12);
    ssd1306_write_command(SSD1306_CMD_SET_CONTRAST);
    ssd1306_write_command(0xCF);
    ssd1306_write_command(SSD1306_CMD_SET_PRECHARGE);
    ssd1306_write_command(0xF1);
    ssd1306_write_command(SSD1306_CMD_SET_VCOMH);
    ssd1306_write_command(0x40);
    ssd1306_write_command(SSD1306_CMD_DEACTIVATE_SCROLL);
    ssd1306_write_command(0xA4);
    ssd1306_write_command(0xA6);
    ssd1306_write_command(SSD1306_CMD_DISPLAY_ON);
    
    memset(display_buffer, 0, sizeof(display_buffer));
    
    oled_initialized = true;
    ESP_LOGI(TAG, "OLED display initialized successfully");
    
    oled_clear();
    
    return ESP_OK;
}

static void ssd1306_update_display(void) {
    ssd1306_write_command(SSD1306_CMD_SET_COLUMN_ADDR);
    ssd1306_write_command(0);
    ssd1306_write_command(SCREEN_WIDTH - 1);
    ssd1306_write_command(SSD1306_CMD_SET_PAGE_ADDR);
    ssd1306_write_command(0);
    ssd1306_write_command(SCREEN_PAGES - 1);
    
    for (int i = 0; i < sizeof(display_buffer); i += 16) {
        ssd1306_write_data(&display_buffer[i], 16);
    }
}

static void draw_char(int x, int y, char c) {
    if (c < 32 || c > 126) c = ' ';
    const uint8_t* glyph = font5x7[c - 32];
    
    for (int col = 0; col < 5; col++) {
        if (x + col >= SCREEN_WIDTH) break;
        
        for (int row = 0; row < 7; row++) {
            if (y + row >= SCREEN_HEIGHT) break;
            
            if (glyph[col] & (1 << row)) {
                int page = (y + row) / 8;
                int bit = (y + row) % 8;
                int idx = page * SCREEN_WIDTH + (x + col);
                if (idx < sizeof(display_buffer)) {
                    display_buffer[idx] |= (1 << bit);
                }
            }
        }
    }
}

static void draw_string(int x, int y, const char* str) {
    while (*str) {
        draw_char(x, y, *str);
        x += 6; // 5 pixels + 1 pixel spacing
        str++;
    }
}

static void draw_hline(int x, int y, int width) {
    for (int i = 0; i < width; i++) {
        if (x + i >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) break;
        
        int page = y / 8;
        int bit = y % 8;
        int idx = page * SCREEN_WIDTH + (x + i);
        if (idx < sizeof(display_buffer)) {
            display_buffer[idx] |= (1 << bit);
        }
    }
}

static void draw_filled_rect(int x, int y, int width, int height) {
    for (int dy = 0; dy < height; dy++) {
        for (int dx = 0; dx < width; dx++) {
            int px = x + dx;
            int py = y + dy;
            if (px >= SCREEN_WIDTH || py >= SCREEN_HEIGHT) continue;
            
            int page = py / 8;
            int bit = py % 8;
            int idx = page * SCREEN_WIDTH + px;
            if (idx < sizeof(display_buffer)) {
                display_buffer[idx] |= (1 << bit);
            }
        }
    }
}

void oled_clear(void) {
    if (!oled_initialized) return;
    
    memset(display_buffer, 0, sizeof(display_buffer));
    ssd1306_update_display();
}

void oled_update_status(lock_state_t lock_state, door_state_t door_state, bool wifi_connected) {
    if (!oled_initialized) return;
    
    memset(display_buffer, 0, sizeof(display_buffer));
    
    draw_string(10, 2, "BoltLock v1.0");
    
    draw_hline(0, 16, 128);
    
    const char* lock_str = "UNKNOWN";
    switch (lock_state) {
        case LOCK_STATE_LOCKED:    lock_str = "LOCKED";    break;
        case LOCK_STATE_UNLOCKED:  lock_str = "UNLOCKED";  break;
        case LOCK_STATE_UNLOCKING: lock_str = "UNLOCKING"; break;
        case LOCK_STATE_ERROR:     lock_str = "ERROR";     break;
    }
    
    char line2[32];
    snprintf(line2, sizeof(line2), "Lock: %s", lock_str);
    draw_string(5, 24, line2);
    
    const char* door_str = (door_state == DOOR_CLOSED) ? "CLOSED" : "OPEN";
    char line3[32];
    snprintf(line3, sizeof(line3), "Door: %s", door_str);
    draw_string(5, 37, line3);
    
    const char* wifi_str = wifi_connected ? "Connected" : "Disconn.";
    char line4[32];
    snprintf(line4, sizeof(line4), "WiFi: %s", wifi_str);
    draw_string(5, 50, line4);
    
    if (wifi_connected) {
        draw_filled_rect(115, 54, 8, 6);
    }
    
    ssd1306_update_display();
    
    ESP_LOGI(TAG, "Status updated: %s | Door:%s | WiFi:%s", lock_str, door_str, wifi_str);
}

void oled_show_message(const char* message, uint32_t duration_ms) {
    if (!oled_initialized) return;
    
    memset(display_buffer, 0, sizeof(display_buffer));
    
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
            int x = (SCREEN_WIDTH - len * 6) / 2;
            draw_string(x, y, line);
            y += 13;
            line_num++;
        }
        
        line_start = word_start;
    }
    
    ssd1306_update_display();
    
    ESP_LOGI(TAG, "Message: %s (duration: %lu ms)", message, duration_ms);
    
    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        oled_clear();
    }
}