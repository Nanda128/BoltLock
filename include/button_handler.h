#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum
{
    BUTTON_EVENT_PRESS,
    BUTTON_EVENT_RELEASE,
    BUTTON_EVENT_LONG_PRESS
} button_event_t;

typedef enum
{
    BUTTON_ID_UNLOCK,
    BUTTON_ID_LOCK
} button_id_t;

typedef struct
{
    button_event_t event;
    button_id_t button_id;
    uint32_t press_duration_ms;
} button_data_t;

esp_err_t button_handler_init(QueueHandle_t button_queue);

#endif // BUTTON_HANDLER_H
