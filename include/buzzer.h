#ifndef BUZZER_H
#define BUZZER_H

#include "esp_err.h"
#include <stdint.h>

esp_err_t buzzer_init(void);

void buzzer_play_unlock(void);

void buzzer_play_lock(void);

void buzzer_play_error(void);

void buzzer_beep(uint32_t duration_ms);

#endif // BUZZER_H
