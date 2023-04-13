#pragma once

#include "esp_log.h"

#define ENABLE_LOGGING 1

#if ENABLE_LOGGING
#ifdef CORE_DEBUG_LEVEL
#undef CORE_DEBUG_LEVEL
#endif

#define CORE_DEBUG_LEVEL 5

#ifdef CONFIG_LOG_MAXIMUM_LEVEL
#undef CONFIG_LOG_MAXIMUM_LEVEL
#endif

#ifdef LOG_LOCAL_LEVEL
#undef LOG_LOCAL_LEVEL
#endif
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE

#define LOGE(format, ...) ESP_LOGE(TAG, format, ##__VA_ARGS__)
#define LOGW(format, ...) ESP_LOGW(TAG, format, ##__VA_ARGS__)
#define LOGI(format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define LOGD(format, ...) ESP_LOGD(TAG, format, ##__VA_ARGS__)
#define LOGV(format, ...) ESP_LOGV(TAG, format, ##__VA_ARGS__)

#else

#define LOGE(format, ...)
#define LOGW(format, ...)
#define LOGI(format, ...)
#define LOGD(format, ...)
#define LOGV(format, ...)

#endif