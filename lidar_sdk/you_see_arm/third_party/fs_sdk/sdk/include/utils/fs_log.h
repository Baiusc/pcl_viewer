#ifndef __FS_LOG_H__
#define __FS_LOG_H__

/*
 * The default LOG_TAG is 'fsLD', using these two lines to define newtag.
 * # undef LOG_TAG
 * #define LOG_TAG "newtag"
 */
#include <stdint.h>
#include <stddef.h>

#ifndef LOG_TAG
#define LOG_TAG "fsLD"
#endif

/*
 * Log level can be used in 'logcat <tag>[:priority]', and also
 * be used in output control while '__FS_LOG_LEVEL' is defined.
 */
typedef enum {
    FS_LOG_LEVEL_NON = 0,
    FS_LOG_LEVEL_ERR = 1,
    FS_LOG_LEVEL_WAR = 2,
    FS_LOG_LEVEL_INF = 3,
    FS_LOG_LEVEL_DBG = 4,
    FS_LOG_LEVEL_ALL = 10,
} fs_log_level_t;

typedef enum {
    FS_LOG_FEED_RAW = 0, // 仅打印 raw 日志
    FS_LOG_FEED_NON = 1, // 打印不带回车换行的日志
    FS_LOG_FEED_YES = 2, // 打印带回车换行的日志
} fs_log_feed_t;

#define LOG_MAGIC '!'

typedef struct {
    uint8_t _magic; // LOG_MAGIC charachter.
    uint8_t *buffer; // memory buffer.
    int32_t buflen; // total size.
    uint8_t *wbuf; // next write pointer.
    int32_t left; // remain size.
} fs_log_context_t;

typedef struct {
    int level;
    char tag[32];
    char buf[0];
} fs_log_chunk_t;

/*
 * __FS_LOG_LEVEL can be defined in compilation option.
 * default level is ALL(all the logs will be output).
 */
#ifndef __FS_LOG_LEVEL
#define __FS_LOG_LEVEL FS_LOG_LEVEL_ALL
#endif

#define COLOR_RED(str) "\033[31m" str "\033[0m" // color red
#define COLOR_GRN(str) "\033[32m" str "\033[0m" // color green
#define COLOR_YEL(str) "\033[33m" str "\033[0m" // color yellow

/*
 * Using the following three macros for conveniently logging.
 */
#if __FS_LOG_LEVEL >= FS_LOG_LEVEL_DBG
#define FS_F_LOGD(feed, format, args...)                                                    \
    do {                                                                                    \
        if (g_log_level >= FS_LOG_LEVEL_DBG) {                                              \
            fs_log_printf(FS_LOG_LEVEL_DBG, feed, LOG_TAG, NULL, __LINE__, format, ##args); \
        };                                                                                  \
    } while (0)
#define FS_LOGD(format, args...)  FS_F_LOGD(FS_LOG_FEED_YES, format, ##args)
#define FS_NLOGD(format, args...) FS_F_LOGD(FS_LOG_FEED_NON, format, ##args)
#define FS_RLOGD(format, args...) FS_F_LOGD(FS_LOG_FEED_RAW, format, ##args)
#else
#define FS_LOGD (format, ...)
#define FS_NLOGD(format, ...)
#define FS_RLOGD(format, ...)
#endif

#if __FS_LOG_LEVEL >= FS_LOG_LEVEL_INF
#define FS_F_LOGI(feed, format, args...)                                                    \
    do {                                                                                    \
        if (g_log_level >= FS_LOG_LEVEL_INF) {                                              \
            fs_log_printf(FS_LOG_LEVEL_INF, feed, LOG_TAG, NULL, __LINE__, format, ##args); \
        };                                                                                  \
    } while (0)
#define FS_LOGI(format, args...)  FS_F_LOGI(FS_LOG_FEED_YES, format, ##args)
#define FS_NLOGI(format, args...) FS_F_LOGI(FS_LOG_FEED_NON, format, ##args)
#define FS_RLOGI(format, args...) FS_F_LOGI(FS_LOG_FEED_RAW, format, ##args)
#else
#define FS_LOGI (format, ...)
#define FS_NLOGI(format, ...)
#define FS_RLOGI(format, ...)
#endif

#if __FS_LOG_LEVEL >= FS_LOG_LEVEL_WAR
#define FS_F_LOGW(feed, format, args...)                                                            \
    do {                                                                                            \
        if (g_log_level >= FS_LOG_LEVEL_WAR) {                                                      \
            fs_log_printf(FS_LOG_LEVEL_WAR, feed, LOG_TAG, __FUNCTION__, __LINE__, format, ##args); \
        };                                                                                          \
    } while (0)
#define FS_LOGW(format, args...)  FS_F_LOGW(FS_LOG_FEED_YES, format, ##args)
#define FS_NLOGW(format, args...) FS_F_LOGW(FS_LOG_FEED_NON, format, ##args)
#define FS_RLOGW(format, args...) FS_F_LOGW(FS_LOG_FEED_RAW, format, ##args)
#else
#define FS_LOGW (format, ...)
#define FS_NLOGW(format, ...)
#define FS_RLOGW(format, ...)
#endif

#if __FS_LOG_LEVEL >= FS_LOG_LEVEL_ERR
#define FS_F_LOGE(feed, format, args...)                   \
    do {                                                   \
        if (g_log_level >= FS_LOG_LEVEL_ERR) {             \
            fs_log_printf(FS_LOG_LEVEL_ERR, feed, LOG_TAG, \
                          __FUNCTION__, __LINE__, format,  \
                          ##args);                         \
        }                                                  \
    } while (0)
#define FS_LOGE(format, args...)  FS_F_LOGE(FS_LOG_FEED_YES, format, ##args)
#define FS_NLOGE(format, args...) FS_F_LOGE(FS_LOG_FEED_NON, format, ##args)
#define FS_RLOGE(format, args...) FS_F_LOGE(FS_LOG_FEED_RAW, format, ##args)
#else
#define FS_LOGE (format, ...)
#define FS_NLOGE(format, ...)
#define FS_RLOGE(format, ...)
#endif

/*
 * Log level can be runtime configurable.
 */
extern volatile fs_log_level_t g_log_level; /* = __FS_LOG_LEVEL */

#ifdef __cplusplus
extern "C" {
#endif

int32_t fs_log_printf(fs_log_level_t level, fs_log_feed_t feed, const char *tag, const char *func, int32_t line, const char *fmt, ...) __attribute__((format(printf, 6, 7)));
void    fs_log_deinit(void);
int32_t fs_log_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __FS_LOG_H__ */
