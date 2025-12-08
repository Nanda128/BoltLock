#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "config.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum
{
    SM_EVENT_BUTTON_PRESS,
    SM_EVENT_DOOR_OPENED,
    SM_EVENT_DOOR_CLOSED,
    SM_EVENT_TIMEOUT,
    SM_EVENT_REMOTE_UNLOCK,
    SM_EVENT_REMOTE_LOCK,
    SM_EVENT_UNAUTHORIZED,
    SM_EVENT_TAMPER_DETECTED
} sm_event_t;

typedef struct
{
    sm_event_t event;
    void *data;
} sm_event_data_t;

esp_err_t state_machine_init(void);

QueueHandle_t get_state_machine_queue(void);

esp_err_t send_sm_event(sm_event_t event, void *data);

#endif // STATE_MACHINE_H
