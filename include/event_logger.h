#ifndef EVENT_LOGGER_H
#define EVENT_LOGGER_H

#include "config.h"
#include "esp_err.h"
#include <time.h>

typedef struct
{
    event_type_t type;
    time_t timestamp;
    char description[128];
} event_log_t;

esp_err_t event_logger_init(void);

esp_err_t log_event(event_type_t type, const char *description);

int get_recent_events(event_log_t *events, int count);

void format_event_message(event_log_t *event, char *buffer, size_t buffer_size);

#endif // EVENT_LOGGER_H
