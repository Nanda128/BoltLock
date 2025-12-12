#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "esp_err.h"
#include "config.h"
#include <stdbool.h>
#include <stdint.h>
#include "u8g2.h"

esp_err_t oled_display_init(void);

void oled_update_status(lock_state_t lock_state, door_state_t door_state, bool wifi_connected);

void oled_show_message(const char *message, uint32_t duration_ms);

void oled_clear(void);

#endif // OLED_DISPLAY_H