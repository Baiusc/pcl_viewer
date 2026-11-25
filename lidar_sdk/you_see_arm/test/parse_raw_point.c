/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2025-03-19 08:19:55
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-03-21 00:47:37
 * @FilePath     : /you_see_arm/test/parse_raw_point.c
 * @Description  :
 * Copyright (c) 2025 by Vitalchem, All Rights Reserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#define SLOT_LINE_START "start of slot"
#define SLOT_LINE_END "end of slot"
#define ANGLE_LINE "rotate mirror angle"
#define PIXEL_LINE "pixel_number"

#define DIR_PATH "./raw_data"
#define OUT_PATH "./point_cloud_data"

char **file_names;

#pragma pack(push, 1)
typedef struct echo_data_h
{
    uint8_t ft[3];
    uint16_t wd;
    uint16_t pk;
    uint8_t reserved[3];
} echo_data_t;

typedef struct piexl_data_h
{
    echo_data_t data[3];
    uint8_t reserved[2];
} piexl_data_t;

typedef struct
{
    uint16_t slot_number;     // slot序号 最大值799
    uint16_t code_disc_angle; // 码盘的编码值
    uint8_t delay[10];        // 时间戳，精确到us
    uint8_t reserved[18];     // 预留
    piexl_data_t piexl_value[96];
} org_point_cloud_packet_t;
#pragma pack(pop)

org_point_cloud_packet_t org_point_cloud_data;

int get_echo_data(char *src, int *dest)
{
    int count = 0;
    char *p = strstr(src, ": ");
    p += 2;
    char delim[] = "                ";
    char *token = NULL;

    if (p == NULL)
    {
        return 0;
    }

    token = strtok(p, delim);
    while (token != NULL)
    {
        dest[count++] = atoi(token);
        token = strtok(NULL, delim);
    }

    return count;
}

int get_slot(char *src, int srclen)
{
    char *p = src;

    src[srclen - 1] = '\0';
    p = strstr(src, "[");
    return atoi(p + 1);
}

int get_angle(char *src)
{
    char *p = NULL;
    p = strstr(src, ": ");

    if (p == NULL)
    {
        return 0;
    }

    return atoi(p + 1);
}

void wirte_data(FILE *fd)
{
    uint8_t *data = (uint8_t *)&org_point_cloud_data;
    fwrite(data, sizeof(org_point_cloud_data), 1, fd);
    memset(&org_point_cloud_data, 0, sizeof(org_point_cloud_data));
}

int compare_strings(const void *a, const void *b)
{
    return strcmp(*(const char **)a, *(const char **)b);
}

int get_raw_data_file_name()
{
    DIR *dir;
    struct dirent *entry;
    struct stat file_stat;

    // 打开目录
    if ((dir = opendir(DIR_PATH)) == NULL)
    {
        perror("无法打开目录");
        return -1;
    }

    // 计算文件数量
    int file_count = 0;
    while (readdir(dir) != NULL)
    {
        file_count++;
    }
    rewinddir(dir); // 重置目录流到开始位置

    // 分配存储文件名的数组
    file_names = (char **)malloc(file_count * sizeof(char *));
    if (file_names == NULL)
    {
        perror("内存分配失败");
        closedir(dir);
        return -1;
    }

    // 读取所有文件名
    int index = 0;
    while ((entry = readdir(dir)) != NULL)
    {
        // 跳过 "." 和 ".." 目录
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        // 分配内存并复制文件名
        file_names[index] = strdup(entry->d_name);
        if (file_names[index] == NULL)
        {
            perror("内存分配失败");
            // 释放已分配的内存
            for (int i = 0; i < index; i++)
            {
                free(file_names[i]);
            }
            free(file_names);
            closedir(dir);
            return -1;
        }
        index++;
    }

    // 关闭目录
    closedir(dir);

    // 对文件名进行排序
    qsort(file_names, index, sizeof(char *), compare_strings);

    // 按顺序处理文件
    printf("目录 '%s' 中的文件(按名称排序):\n", DIR_PATH);
    for (int i = 0; i < index; i++)
    {
        char path[1024];
        snprintf(path, sizeof(path), "%s/%s", DIR_PATH, file_names[i]);

        // 获取文件状态
        if (stat(path, &file_stat) == -1)
        {
            perror("获取文件状态失败");
            continue;
        }

        // 打印文件名和类型
        printf("%s", file_names[i]);
        if (S_ISDIR(file_stat.st_mode))
            printf(" [目录]");
        printf("\n");
    }

    return index;
}

int main()
{
    FILE *fd_w = NULL;
    fd_w = fopen(OUT_PATH, "wb+");

    if (fd_w == NULL)
    {
        printf("open data file error!\n");
        return -1;
    }

    int count = get_raw_data_file_name();

    for (int i = 0; i < count; i++)
    {
        char read_buf[512] = {0};
        FILE *fd_r = NULL;
        char file_path[64] = {0};
        snprintf(file_path, sizeof(file_path), "./%s/%s", DIR_PATH, file_names[i]);
        fd_r = fopen(file_path, "r");

        if (fd_r == NULL)
        {
            printf("open %s file error!\n", file_path);
            goto end;
        } else {
            printf("open %s file succ!\n", file_path);
        }

        int values[10]; // 假设最多有10个数字
        while (fgets(read_buf, sizeof(read_buf), fd_r) != NULL)
        {
            if (strstr(read_buf, SLOT_LINE_START))
            {
                int slot = get_slot(read_buf, strlen(read_buf));
                org_point_cloud_data.slot_number = (uint16_t)slot;
            }
            else if (strstr(read_buf, ANGLE_LINE))
            {
                int angle = get_angle(read_buf);
                org_point_cloud_data.code_disc_angle = (uint16_t)angle;
            }
            else if (strstr(read_buf, PIXEL_LINE))
            {
                int count = get_echo_data(read_buf, values);
                int piexl_index = values[0];
                for (int i = 0, j = 1; j < count; i++, j += 3)
                {
                    org_point_cloud_data.piexl_value[piexl_index].data[i].ft[0] = (uint8_t)values[j];
                    org_point_cloud_data.piexl_value[piexl_index].data[i].ft[1] = (uint8_t)(values[j] >> 8);
                    org_point_cloud_data.piexl_value[piexl_index].data[i].ft[2] = (uint8_t)(values[j] >> 16);
                    org_point_cloud_data.piexl_value[piexl_index].data[i].wd = (uint16_t)values[j + 1];
                    org_point_cloud_data.piexl_value[piexl_index].data[i].pk = (uint16_t)values[j + 2];
                }
            } else if (strstr(read_buf, SLOT_LINE_END)) {
                wirte_data(fd_w); // 读到文件结束的时候保存当前piexl
            }
        }

end:
        if (fd_r != NULL)
        {
            fclose(fd_r);
        }

        free(file_names[i]);

        sleep(1);
    }

    if (fd_w != NULL)
    {
        fclose(fd_w);
    }

    free(file_names);

    return 0;
}