/*
 * fs_file.h - 全局文件操作接口
 */

#ifndef __FS_FILE_H__
#define __FS_FILE_H__

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>

/**
 * @brief 文件定位模式常量定义
 */
#define FS_SEEK_SET 0 /* 从文件开头开始计算偏移量 */
#define FS_SEEK_CUR 1 /* 从当前位置开始计算偏移量 */
#define FS_SEEK_END 2 /* 从文件末尾开始计算偏移量 */

typedef void fs_file_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取路径中的目录部分，类似于 POSIX 的 dirname 函数
 * @param path 输入的文件路径
 * @return 成功返回目录路径字符串（需要调用者使用 free() 释放内存），失败返回 NULL
 */
char *fs_file_dirname(const char *path);

/**
 * @brief 递归创建目录，类似于 mkdir -p
 * @param path 要创建的目录路径（支持相对路径和绝对路径）
 * @return 成功返回 FS_SUCCESS (0)，失败返回相应错误码，如 FS_ERR_NOMEM (内存分配失败) 或 FS_ERR_INTERNAL (内部错误)
 */
int32_t fs_file_mkdir(const char *path);

/**
 * @brief 递归删除目录及其所有内容，类似于 rm -r 命令
 * @param pathname 要删除的目录路径
 * @return 无返回值
 * @note 该函数会递归删除指定目录下的所有文件和子目录，使用时需谨慎
 */
void fs_file_rmdir(const char *pathname);

/**
 * @brief 打开文件，自动创建所需的父目录（当需要创建文件时）
 * @param path 文件路径
 * @param mode 打开模式（支持 "r", "w", "a", "r+", "w+", "a+"）
 * @return 成功返回文件句柄指针，失败返回 NULL
 * @note 当打开模式包含创建标记(O_CREAT)时，会自动递归创建必要的父目录
 */
fs_file_t *fs_file_open(const char *path, const char *mode);

/**
 * @brief 关闭文件并释放相关资源
 * @param fp 文件句柄指针
 * @return 成功返回 FS_SUCCESS (0)，失败返回 FS_ERR_BAD_PARAMS (参数无效) 或 FS_ERR_INTERNAL (内部错误)
 */
int32_t fs_file_close(fs_file_t *fp);

/**
 * @brief 从文件中读取内容到缓冲区
 * @param ptr 数据缓冲区指针
 * @param size 每个数据元素的大小（字节）
 * @param nmemb 要读取的元素个数
 * @param fp 文件句柄指针
 * @return 成功返回实际读取的元素个数，失败返回 0
 * @note 返回值 = 实际读取字节数 / size
 */
uint32_t fs_file_read(void *ptr, uint32_t size, uint32_t nmemb, fs_file_t *fp);

/**
 * @brief 将缓冲区内容写入文件
 * @param ptr 源数据缓冲区指针
 * @param size 每个数据元素的大小（字节）
 * @param nmemb 要写入的元素个数
 * @param fp 文件句柄指针
 * @return 成功返回实际写入的元素个数，失败返回 0
 * @note 返回值 = 实际写入字节数 / size
 */
uint32_t fs_file_write(const void *ptr, uint32_t size, uint32_t nmemb, fs_file_t *fp);

/**
 * @brief 设置文件读写位置指针
 * @param fp 文件句柄指针
 * @param offset 偏移量（字节）
 * @param whence 起始位置（FS_SEEK_SET - 文件开头，FS_SEEK_CUR - 当前位置，FS_SEEK_END - 文件末尾）
 * @return 成功返回 FS_SUCCESS (0)，失败返回 FS_ERR_BAD_PARAMS (参数无效) 或 FS_ERR_INTERNAL (内部错误)
 */
int32_t fs_file_seek(fs_file_t *fp, int32_t offset, int32_t whence);

/**
 * @brief 将文件内容从内存缓冲区同步到存储设备
 * @param fp 文件句柄指针
 * @return 成功返回 FS_SUCCESS (0)，失败返回 FS_ERR_BAD_PARAMS (参数无效) 或 FS_ERR_INTERNAL (内部错误)
 */
int32_t fs_file_sync(fs_file_t *fp);

/**
 * @brief 删除文件，支持通配符删除（*和?）
 * @param path 文件路径（可以包含通配符*和?）
 * @return 成功返回 FS_SUCCESS (0)，失败返回 FS_ERR_BAD_PARAMS (参数无效) 或 FS_ERR_INTERNAL (内部错误)
 * @note 1. 当路径包含通配符时，将遍历匹配的所有文件并删除
 *       2. * 匹配任意数量的任意字符
 *       3. ? 匹配单个任意字符
 *       4. 如果没有文件匹配通配符模式，仍返回成功
 */
int32_t fs_file_delete(const char *path);

/**
 * @brief 获取文件大小，并根据参数决定是否恢复原文件位置
 * @param fp 文件句柄指针
 * @param whence 决定是否保存和恢复当前文件位置（FS_SEEK_CUR - 保存并恢复，FS_SEEK_SET - 不恢复）
 * @return 成功返回文件大小（字节），失败返回错误码，如 FS_ERR_BAD_PARAMS (参数无效) 或 FS_ERR_IO (I/O错误)
 */
int32_t fs_file_size(fs_file_t *fp, int whence);

/**
 * @brief 检查文件或目录是否存在
 * @param path 要检查的文件或目录路径
 * @return 成功返回 FS_SUCCESS (0) 表示存在，失败返回 FS_ERR_NOT_FOUND (-2) 表示不存在，或其他错误码表示检查过程出错
 * @note 1. 此函数可以检查普通文件和目录是否存在
 *       2. 当文件或目录存在时返回 FS_SUCCESS
 *       3. 当文件或目录不存在时返回 FS_ERR_NOT_FOUND
 *       4. 其他错误码表示检查过程中发生了错误（如参数无效等）
 */
int32_t fs_file_exists(const char *path);

/**
 * @brief 使用通配符路径查找出所有符合条件的文件名
 * @param pattern 通配符路径模式，支持*和?通配符
 * @param files 输出缓冲区，用于存储找到的文件名列表
 * @param count 输入时表示输出缓冲区的大小（单位：字节），输出时表示实际找到的文件名数量
 * @return 成功返回FS_SUCCESS，失败返回错误码
 */
int32_t fs_file_list(const char *pattern, char *files, uint32_t *count);

#ifdef __cplusplus
}
#endif

#endif /* __FS_FILE_H__ */