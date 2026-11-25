/**
 * log.c, REE side implement.
 *
 *
 *
 **/

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <syslog.h>
#include <sys/syscall.h> //Linux system call for thread id
#include <unistd.h>
#include <time.h>
#include "fs_util.h"
#include "fs_log.h"
#include "fs_err.h"
#include "fs_version.h"

#undef LOG_TAG
#define LOG_TAG "soc.log"
#define LOG_TAG_PREFIX "fslidar-"

// log buffer size
#define LOG_BUF_SIZE (64 * 1024)

// log chunk size
#define LOG_CHUNK_SIZE 1024

volatile fs_log_level_t g_log_level = __FS_LOG_LEVEL;

fs_log_context_t g_log_context = {
    0,
};

int32_t fs_log_printf(fs_log_level_t level, fs_log_feed_t feed, const char *tag, const char *func, int32_t line, const char *fmt, ...)
{
    char logbuf[LOG_CHUNK_SIZE]; // should be long enough.
    char level_tag[4] = "[E]";
    int loglen, n = 0, tag_len;
    fs_log_chunk_t *log = (fs_log_chunk_t *)logbuf;
    log->level = level;
    tag_len = strlen(tag);
    memcpy(log->tag, tag, ((int32_t)sizeof(log->tag) > tag_len) ? tag_len : (int32_t)sizeof(log->tag) - 1);
    log->tag[sizeof(log->tag) - 1] = '\0';
    log->buf[0] = '\0';
    /*
      * logbuf:
      * "[level][tag]<...>\0"
      *     4  + 32 +  5 + 1 = 42
      */
    loglen = sizeof(uint32_t) + sizeof(log->tag); // 36
    va_list ap;
    va_start(ap, fmt);
    n = vsnprintf((char *)log->buf, LOG_CHUNK_SIZE, fmt, ap);
    n = n > LOG_CHUNK_SIZE ? LOG_CHUNK_SIZE : n;
    /*
     int lev = LOG_INFO;

     switch (level) {
         case FS_LOG_LEVEL_ALL:
         case FS_LOG_LEVEL_DBG:
             lev = LOG_DEBUG;
             break;

         case FS_LOG_LEVEL_INF:
             lev = LOG_INFO;
             break;

         case FS_LOG_LEVEL_ERR:
             lev = LOG_INFO;//LOG_ERR; // write FS_LOGE to /var/log/messages
             break;
     }*/

    // vsyslog(lev,fmt,ap);  //sudo cat /var/log/syslog
    va_end(ap);
    loglen += (n + 1); // including '\0'.

    /* It took Beanpod 5~6 ms to print a log. Release version shouldn't print. */
    /*allow error log print to kernel  ----by Jone.Chen 2019.12.03*/
    if ((g_log_level >= FS_LOG_LEVEL_DBG) || (level == FS_LOG_LEVEL_ERR)) {
        if (level >= FS_LOG_LEVEL_DBG) {
            level_tag[1] = 'D';
        }
        else if (level == FS_LOG_LEVEL_INF) {
            level_tag[1] = 'I';
        }
        else if (level == FS_LOG_LEVEL_ERR) {
            level_tag[1] = 'E';
        }
        else {
            level_tag[1] = 'E';
        }

        uint64_t timestamp = fs_util_timestamp_ms();
        time_t tv_sec = (time_t)(timestamp / 1000);
        struct tm *tm_info = localtime(&tv_sec);
        char date[26];
        strftime(date, 26, "%Y-%m-%d %H:%M:%S", tm_info);

        if (feed == FS_LOG_FEED_YES) {
            syslog(LOG_INFO /*lev*/, "[%s] %s.%03u %c [%d] %s(%s%s%d): %s\n", SOC_APP_VERSION, date, (uint32_t)(timestamp % 1000),
                   level_tag[1], (int32_t)syscall(SYS_gettid), tag, func ? func : "", func ? ":" : "", line, log->buf);
            printf("[%s] %s.%03u %c [%d] %s(%s%s%d): %s\n", SOC_APP_VERSION, date, (uint32_t)(timestamp % 1000),
                   level_tag[1], (int32_t)syscall(SYS_gettid), tag, func ? func : "", func ? ":" : "", line, log->buf);
        }
        else if (feed == FS_LOG_FEED_NON) {
            syslog(LOG_INFO /*lev*/, "[%s] %s.%03u %c [%d] %s(%s%s%d): %s", SOC_APP_VERSION, date, (uint32_t)(timestamp % 1000),
                   level_tag[1], (int32_t)syscall(SYS_gettid), tag, func ? func : "", func ? ":" : "", line, log->buf);
            printf("[%s] %s.%03u %c [%d] %s(%s%s%d): %s", SOC_APP_VERSION, date, (uint32_t)(timestamp % 1000),
                   level_tag[1], (int32_t)syscall(SYS_gettid), tag, func ? func : "", func ? ":" : "", line, log->buf);
        }
        else {
            syslog(LOG_INFO /*lev*/, "%s", log->buf);
            printf("%s", log->buf);
        }

        {
            char log_buff[1024] = { 0 };
            int log_num = snprintf(log_buff, sizeof(log_buff), "[%s] %5u.%03u [%d] %s [%s]: %s\r\n",
                                   SOC_APP_VERSION, (uint32_t)(timestamp / 1000), (uint32_t)(timestamp % 1000),
                                   (int32_t)syscall(SYS_gettid), level_tag, tag, log->buf);

            if (log_num > (int)sizeof(log_buff)) {
                log_num = (int)sizeof(log_buff);
            }

            //  if (g_protocol) {
            //      g_protocol->process_log_data((uint8_t *)log_buff, log_num);
            //  }
        }
    }

    if (!g_log_context.buffer) {
        return FS_ERR_NULL_PTR;
    }

    /* Check the remain space to store new log. */
    loglen += 2; // "![log]$"

    // log_debug(LOG_TAG": logging need=%d, left=%d\n", loglen, g_log_context.left);
    if ((g_log_context.left - loglen) <= (42 + 2) || loglen > g_log_context.left) {
        loglen = loglen - n + 5; // using <...> instead.

        if (g_log_context.left >= loglen) {
            log->level = FS_LOG_LEVEL_ERR;
            strcpy(log->tag, LOG_TAG_PREFIX "lidar-log");
            strcpy(log->buf, "<...>");
        }
        else {
            // log_error("\n"LOG_TAG": ERROR, TA logging buffer is overflow!\n");
            // ut_sys_log("\n"LOG_TAG": ERROR, TA logging buffer is overflow!\n"); //lanh
            return FS_ERR_BUF_OVERFLOW;
        }
    }

    loglen -= 2;
    /*
      * Do it.
      * Note: CA extract one log by looking for the LOG_MAGIC.
      */
    g_log_context.wbuf[0] = LOG_MAGIC;
    g_log_context.wbuf++;
    g_log_context.left--;
    memcpy(g_log_context.wbuf, logbuf, loglen);
    g_log_context.wbuf += loglen;
    g_log_context.left -= loglen;
    /* CA finishes searching at '$'. */
    g_log_context.wbuf[0] = '$';
    return n;
}

void fs_log_deinit(void)
{
    if (g_log_context.buffer != NULL) {
        free(g_log_context.buffer);
    }

    memset(&g_log_context, 0x00, sizeof(g_log_context));
    closelog();
    return;
}

int32_t fs_log_init(void)
{
    // 重定向stdout和stderr到日志文件
    // FILE *log_file = fopen("/fortsense/fslidar/app_debug.log", "w");
    // if (log_file) {
    //     // 先写入初始信息
    //     fprintf(log_file, "Application starting...\n");
    //     fflush(log_file);

    //     // 重定向输出
    //     dup2(fileno(log_file), STDOUT_FILENO);
    //     dup2(fileno(log_file), STDERR_FILENO);
    //     setvbuf(stdout, NULL, _IOLBF, 0); // 行缓冲
    //     setvbuf(stderr, NULL, _IOLBF, 0);
    // }
    g_log_context.buffer = (uint8_t *)malloc(LOG_BUF_SIZE);
    CHECK_PTR(g_log_context.buffer);
    g_log_context.buflen = LOG_BUF_SIZE;
    g_log_context.wbuf   = g_log_context.buffer;
    g_log_context.left   = g_log_context.buflen;
    g_log_context._magic = LOG_MAGIC;
    openlog("fslidar", LOG_PID, LOG_USER); // sudo cat /var/log/messages |grep fslidar
    return FS_SUCCESS;
}
