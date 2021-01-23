#ifndef DESK_STATION_DS_LOG_H
#define DESK_STATION_DS_LOG_H

typedef enum {
    DS_LOG_LEVEL_INFO,
    DS_LOG_LEVEL_WARNING,
    DS_LOG_LEVEL_ERROR
} ds_log_level_t;

#define DS_LOG_INFO( tag, format, ... ) ds_log(DS_LOG_LEVEL_INFO, tag, format, ##__VA_ARGS__)
#define DS_LOG_WARNING( tag, format, ... ) ds_log(DS_LOG_LEVEL_WARNING, tag, format, ##__VA_ARGS__)
#define DS_LOG_ERROR( tag, format, ... ) ds_log(DS_LOG_LEVEL_ERROR, tag, format, ##__VA_ARGS__)

void ds_log(ds_log_level_t log_level, const char *tag, const char *template, ...);
#endif //DESK_STATION_DS_LOG_H
