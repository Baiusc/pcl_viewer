/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-06-16 21:37:49
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-26 01:53:31
 * @FilePath     : /you_see_arm/core/ys_memory_pool.h
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#ifndef YS_MEMORY_POOL_H
#define YS_MEMORY_POOL_H

#include "ys_core_def.h"

#include "../common/common.h"

/**
 * @description  : set memory pool block size
 * @param         {int} size
 * @return        {*}
 */
void set_block_size(int size);

/**
 * @description  : set memory pool start/stop allocate memory
 * @param         {uint8_t} flag
 * @return        {*}
 */
void set_memory_pool_run_flag(uint8_t flag);

/**
 * @description  : Create memorypool
 * @param         {size_t} size
 * @return        {memory_pool_t} *memory_pool_t
 */
memory_pool_t *create_memory_pool(size_t size);

/**
 * @description  : Allocate memory from memorypool (blocking: spins until a node is free)
 * @param         {memory_pool_t} *pool
 * @return        {void *} allocate space
 */
void *allocate_memory(memory_pool_t *pool);

/**
 * @description  : Try to allocate memory from memorypool (non-blocking: returns NULL immediately if pool is full)
 * @param         {memory_pool_t} *pool
 * @return        {void *} allocated node, or NULL if no writable node available
 */
void *try_allocate_memory(memory_pool_t *pool);

/**
 * @description  : Destroy memorypool
 * @param         {memory_pool_t} *pool
 * @return        {*}
 */
void destroy_memory_pool(memory_pool_t *pool);

#endif // YS_MEMORY_POOL_H
