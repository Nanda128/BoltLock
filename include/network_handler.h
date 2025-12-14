#ifndef NETWORK_HANDLER_H
#define NETWORK_HANDLER_H

#include "esp_err.h"
#include <stdbool.h>

esp_err_t network_init(void);

esp_err_t send_telegram_notification(const char *message);

esp_err_t mqtt_init(void);

esp_err_t mqtt_publish(const char *topic, const char *message);

bool is_network_connected(void);

bool is_mqtt_connected(void);

#endif // NETWORK_HANDLER_H
