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

typedef struct
{
    button_event_t event;
    uint32_t press_duration_ms;
} button_data_t;

// takes queue to receive button events
esp_err_t button_handler_init(QueueHandle_t button_queue);

// Get door event queue for receiving ISR door events
QueueHandle_t get_door_event_queue(void);

void IRAM_ATTR button_isr_handler(void *arg);

void IRAM_ATTR door_sensor_isr_handler(void *arg);

#endif // BUTTON_HANDLER_H
