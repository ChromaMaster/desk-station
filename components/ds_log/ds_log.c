#include "include/ds_log.h"
#include <stdarg.h>
#include <stdio.h>

#include "esp_log.h"

void ds_log(ds_log_level_t log_level, const char *tag, const char *template, ...) {

    va_list arguments;
    va_start(arguments, template);

    // Get the final size of the log_message (template filled with arguments)
    char tmp_buffer[1];
    int message_size = vsnprintf(tmp_buffer, sizeof(char), template, arguments);

    // Fill the log_message and free arguments
    char log_message[message_size + 1];
    vsnprintf(log_message, message_size, template, arguments);
    va_end(arguments);

    // Use device sdk function to print the log_message.
    switch (log_level) {
        case DS_LOG_LEVEL_INFO:
            ESP_LOGI(tag, "%s", log_message);
            break;
        case DS_LOG_LEVEL_WARNING:
            ESP_LOGW(tag, "%s", log_message);
            break;
        case DS_LOG_LEVEL_ERROR:
            ESP_LOGE(tag, "%s", log_message);
            break;
        default:
            break;
    }
}