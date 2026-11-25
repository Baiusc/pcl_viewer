/*
 * fs_file.c - 全局文件操作接口实现
 */

#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <errno.h>

#include "utils/fs_file.h"

char *fs_file_dirname(const char *path)
{
    char *path_copy = strdup(path);
    char *last_slash = NULL;

    if (!path_copy) {
        return NULL;
    }

    last_slash = strrchr(path_copy, '/');
    if (last_slash) {
        *last_slash = '\0';
    } else {
        free(path_copy);
        return NULL;
    }

    return path_copy;
}

/**
 * @brief 递归创建目录，类似于 mkdir -p
 * @param path 要创建的目录路径
 * @return 成功返回 0，失败返回 -1
 */
int32_t fs_file_mkdir(const char *path) {
    char *path_copy = strdup(path);
    char *p = path_copy;
    char *last_slash = NULL;
    int result = 0;

    if (!path_copy) {
        return -1;
    }

    /* 处理绝对路径 */
    if (*p == '/') {
        p++;
    }

    /* 逐个创建目录 */
    while ((last_slash = strchr(p, '/'))) {
        *last_slash = '\0';

        /* 跳过空目录名 */
        if (p[0] != '\0') {
            if (mkdir(path_copy, 0755) != 0 && errno != EEXIST) {
                result = -1;
                break;
            }
        }

        *last_slash = '/';
        p = last_slash + 1;
    }

    /* 创建最后一个目录 */
    if (result == 0 && p[0] != '\0') {
        if (mkdir(path_copy, 0755) != 0 && errno != EEXIST) {
            result = -1;
        }
    }

    free(path_copy);
    return result;
}