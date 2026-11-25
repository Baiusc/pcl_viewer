/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-06-16 21:37:36
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-05-27 00:39:40
 * @FilePath     : /you_see_arm/core/ys_memory_pool.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include "ys_base.h"
#include "ys_memory_pool.h"

#include "../common/common.h"

#include "../third_party/log.c/log.h"

#include <string.h>
#include <stdatomic.h>

#define CACHE_LINE_SIZE 64 // A53 一般使用 64 字节缓存行

static atomic_uint_fast8_t g_allocate_flag;
static int block_size = 0; // 内存池节点大小
static size_t aligned_size = 0; // 对齐后的内存池节点大小，仅用来对齐内存分配，真正使用的大小还是 block_size

void set_block_size(int size)
{
	aligned_size = (size + CACHE_LINE_SIZE - 1) & ~(CACHE_LINE_SIZE - 1);
	block_size = size;
	log_info("set block size %d", aligned_size);
	return;
}

void set_memory_pool_run_flag(uint8_t flag)
{
	atomic_store(&g_allocate_flag, flag);
}

memory_pool_t *create_memory_pool(size_t size)
{
	atomic_init(&g_allocate_flag, ON);

	memory_pool_t *pool = (memory_pool_t *)Malloc(sizeof(memory_pool_t));
	if (!pool) {
		log_error("Memory allocation failed for memory pool");
		return NULL;
	}

	// 先创建普通指针
	block_t *first_block = (block_t *)Malloc(sizeof(block_t));
	if (!first_block) {
		log_error("Memory allocation failed for Block");
		Free((void **)&pool);
		return NULL;
	}

	first_block->node = (node_t *)Malloc(sizeof(node_t));
	if (!first_block->node) {
		log_error("Memory allocation failed for Node");
		Free((void **)&(first_block));
		Free((void **)&pool);
		return NULL;
	} else {
		atomic_init(&(first_block->node->write_ok), WRITABLE);
		first_block->node->data = NULL;
	}

	first_block->node->data = (data_t *)Malloc(sizeof(data_t));
	if (!first_block->node->data) {
		log_error("Memory allocation failed for data_t");
		Free((void **)&(first_block->node));
		Free((void **)&(first_block));
		Free((void **)&pool);
		return NULL;
	}

	first_block->node->data->buf = (uint8_t *)aligned_alloc(CACHE_LINE_SIZE, aligned_size * sizeof(uint8_t));
	if (!first_block->node->data->buf) {
		log_error("Memory allocation failed for Block data");
		Free((void **)&(first_block->node->data));
		Free((void **)&(first_block->node));
		Free((void **)&(first_block));
		Free((void **)&pool);
		return NULL;
	} else {
		memset(first_block->node->data->buf, 0, aligned_size);
	}

	first_block->node->data->len = block_size;
	atomic_store(&(pool->head), first_block);
	atomic_store(&(pool->tail), first_block);
	pool->size = size;

	block_t *current = first_block;
	for (size_t i = 1; i < size; i++) {
		current->next = (block_t *)Malloc(sizeof(block_t));
		if (!current->next) {
			log_error("Memory allocation failed for Block");
			// 释放已分配的内存
			block_t *temp = atomic_load(&(pool->head));
			while (temp) {
				Free((void **)&(temp->node->data->buf));
				Free((void **)&(temp->node->data));
				Free((void **)&(temp->node));
				block_t *next = temp->next;
				Free((void **)&temp);
				temp = next;
				if (temp == atomic_load(&(pool->head)))
					break; // 避免循环链表导致无限循环
			}
			Free((void **)&pool);
			return NULL;
		}
		current = current->next;
		current->node = (node_t *)Malloc(sizeof(node_t));
		if (!current->node) {
			log_error("Memory allocation failed for node_t");
			// 释放已分配的内存
			block_t *temp = atomic_load(&(pool->head));
			block_t *stop_at = current;
			while (temp != stop_at) {
				Free((void **)&(temp->node->data->buf));
				Free((void **)&(temp->node->data));
				Free((void **)&(temp->node));
				block_t *next = temp->next;
				Free((void **)&temp);
				temp = next;
			}
			Free((void **)&current);
			Free((void **)&pool);
			return NULL;
		} else {
			atomic_init(&(current->node->write_ok), WRITABLE);
			current->node->data = NULL;
		}

		current->node->data = (data_t *)Malloc(sizeof(data_t));
		if (!current->node->data) {
			log_error("Memory allocation failed for data_t");
			// 释放已分配的内存
			block_t *temp = atomic_load(&(pool->head));
			block_t *stop_at = current;
			while (temp != stop_at) {
				Free((void **)&(temp->node->data->buf));
				Free((void **)&(temp->node->data));
				Free((void **)&(temp->node));
				block_t *next = temp->next;
				Free((void **)&temp);
				temp = next;
			}
			Free((void **)&(current->node));
			Free((void **)&current);
			Free((void **)&pool);
			return NULL;
		}

		current->node->data->buf = (uint8_t *)aligned_alloc(CACHE_LINE_SIZE, aligned_size * sizeof(uint8_t));
		if (!current->node->data->buf) {
			log_error("Memory allocation failed for uint8_t");
			// 释放已分配的内存
			block_t *temp = atomic_load(&(pool->head));
			block_t *stop_at = current;
			while (temp != stop_at) {
				Free((void **)&(temp->node->data->buf));
				Free((void **)&(temp->node->data));
				Free((void **)&(temp->node));
				block_t *next = temp->next;
				Free((void **)&temp);
				temp = next;
			}
			Free((void **)&(current->node->data));
			Free((void **)&(current->node));
			Free((void **)&current);
			Free((void **)&pool);
			return NULL;
		} else {
			memset(first_block->node->data->buf, 0, aligned_size);
		}

		current->node->data->len = block_size;
	}

	// 创建环形链表
	// 初始化完成后，链表的尾巴的下一个应该是链表的头节点
	current->next = atomic_load(&(pool->head));
	atomic_store(&(pool->tail), current);

	return pool;
}

void *allocate_memory(memory_pool_t *pool)
{
	block_t *current = atomic_load(&(pool->head));

	/*
		增加对 g_allocate_flag 的检查，确保在 ON 状态下才进行内存分配
		有一种情况是，如果消费者已经退出，但生产者还在运行，那么内存池中将没有可用的块可供分配
		此时应将 g_allocate_flag 设置为 OFF，避免死循环
	*/
	while (atomic_load(&(g_allocate_flag)) == ON) {
		if (atomic_load(&(current->node->write_ok)) == WRITABLE) {
			// 更新头指针到下一个位置
			atomic_store(&(pool->head), current->next);
			return current->node;
		}

		// 如果 head 不可用，等待
		usleep(1);

		// 重新获取 head 指针（可能已经被其他线程更新）
		current = atomic_load(&(pool->head));
	}

	return NULL;
}

void *try_allocate_memory(memory_pool_t *pool)
{
	if (atomic_load(&g_allocate_flag) != ON) {
		return NULL;
	}
	block_t *current = atomic_load(&(pool->head));
	if (atomic_load(&(current->node->write_ok)) == WRITABLE) {
		atomic_store(&(pool->head), current->next);
		return current->node;
	}
	return NULL; // pool full, return immediately without spinning
}

void destroy_memory_pool(memory_pool_t *pool)
{
	block_t *current = atomic_load(&(pool->head));
	for (size_t i = 0; i < pool->size; i++) {
		block_t *next = current->next;
		Free((void **)&(current->node->data->buf));
		Free((void **)&(current->node->data));
		Free((void **)&(current->node));
		Free((void **)&(current));
		current = next;
	}
	Free((void **)&pool);
}
