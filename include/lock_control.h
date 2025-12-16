#ifndef LOCK_CONTROL_H
#define LOCK_CONTROL_H

#include "config.h"
#include "esp_err.h"

esp_err_t lock_control_init(void);

esp_err_t lock_door(void);

esp_err_t unlock_door(void);

lock_state_t get_lock_state(void);

void update_status_led(void);

esp_err_t set_lock_position(float angle);

#endif // LOCK_CONTROL_H
