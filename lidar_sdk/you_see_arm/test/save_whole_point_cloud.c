/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-09-13 05:57:10
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2025-06-03 19:14:14
 * @FilePath     : /you_see_arm/test/save_whole_point_cloud.c
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "../driver/inc/yousee_lidar_api_public.h"

// cc/bin/aarch64-buildroot-linux-gnu-gcc -lpthread -Llib -lifp171_api test/save_whole_point_cloud.c -o save_whole_point_cloud

typedef struct {
	uint8_t *buf;
	int len;
	int index;
} data_t;

typedef struct node {
	int lock;
	int write_ok; // 0:读完成，可写 1:写完成，可读
	pthread_mutex_t mutex_lock;
	data_t *data; // 数据块
} node_t;

typedef struct block {
	node_t *node; // 数据体
	struct block *next; // 指向下一个块的指针
	bool is_free; // 标记块是否空闲
} block_t;

typedef struct {
	block_t *head; // 环形链表的头
	block_t *tail; // 环形链表的尾
	size_t size; // 链表中的块的数量
} memory_pool_t;

typedef struct {
	memory_pool_t *pool; // 每个 udp 实例应该有自己的内存池
	int (*deal_data_to_output_link)(void *arg, uint8_t *data, int datalen); // local -> network
} udp_t;

udp_t udp_cmd;

#define SLICES_POINT_CLOUD_LEN 3104 // 点云数据切片长度：3104 字节

#define RET_FAIL -1
#define RET_OK 0

#define WRITABLE 0 // 可写
#define READABLE 1 // 可读

static unsigned char signal_cnt = 0;
static const int memory_pool_size = 810;
static const int block_size = SLICES_POINT_CLOUD_LEN;
static const int org_point_cloud_slot_size = (32 + 96 * 32);
static int aval_cnt;

/******************************************************************************************/

void *Malloc(size_t size)
{
	void *p = NULL;

	if ((p = malloc(size)) == NULL) {
		printf("%s[%d]: Malloc error\n", __FILE__, __LINE__);
	}
	return p;
}

void Free(void **ptr)
{
	if (ptr == NULL || *ptr == NULL) {
		return;
	}

	free(*ptr);
	*ptr = NULL;
	return;
}

memory_pool_t *create_memory_pool(size_t size)
{
	memory_pool_t *pool = (memory_pool_t *)Malloc(sizeof(memory_pool_t));
	if (!pool) {
		printf("Memory allocation failed for memory pool\n");
		return NULL;
	}

	pool->head = (block_t *)Malloc(sizeof(block_t));
	if (!pool->head) {
		printf("Memory allocation failed for Block\n");
		Free((void **)&pool);
		return NULL;
	}

	pool->head->node = (node_t *)Malloc(sizeof(node_t));
	if (!pool->head->node) {
		printf("Memory allocation failed for Node\n");
		Free((void **)&(pool->head));
		Free((void **)&pool);
		return NULL;
	} else {
		pthread_mutex_init(&(pool->head->node->mutex_lock), NULL);
		pool->head->node->lock = 0;
		pool->head->node->data = NULL;
	}

	pool->head->node->data = (data_t *)Malloc(sizeof(data_t));
	if (!pool->head->node->data) {
		printf("Memory allocation failed for data_t\n");
		pthread_mutex_destroy(&(pool->head->node->mutex_lock));
		Free((void **)&(pool->head->node));
		Free((void **)&(pool->head));
		Free((void **)&pool);
		return NULL;
	}

	pool->head->node->data->buf = (uint8_t *)Malloc(block_size * sizeof(uint8_t));
	if (!pool->head->node->data->buf) {
		printf("Memory allocation failed for Block data\n");
		pthread_mutex_destroy(&(pool->head->node->mutex_lock));
		Free((void **)&(pool->head->node->data));
		Free((void **)&(pool->head->node));
		Free((void **)&(pool->head));
		Free((void **)&pool);
		return NULL;
	}

	pool->head->node->data->len = block_size;
	pool->head->node->data->index = 0;
	pool->head->is_free = true;
	pool->tail = pool->head;
	pool->size = size;

	block_t *current = pool->head;
	for (size_t i = 1; i < size; i++) {
		current->next = (block_t *)Malloc(sizeof(block_t));
		if (!current->next) {
			printf("Memory allocation failed for Block\n");
			// 释放已分配的内存
			block_t *temp = pool->head;
			while (temp) {
				pthread_mutex_destroy(&(temp->node->mutex_lock));
				Free((void **)&(temp->node->data->buf));
				Free((void **)&(temp->node->data));
				Free((void **)&(temp->node));
				block_t *next = temp->next;
				Free((void **)&temp);
				temp = next;
			}
			Free((void **)&pool);
			return NULL;
		}
		current = current->next;
		current->node = (node_t *)Malloc(sizeof(node_t));
		if (!current->node) {
			printf("Memory allocation failed for node_t\n");
			// 释放已分配的内存
			block_t *temp = pool->head;
			while (temp != current) {
				pthread_mutex_destroy(&(temp->node->mutex_lock));
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
			pthread_mutex_init(&(current->node->mutex_lock), NULL);
			current->node->lock = 0;
			current->node->data = NULL;
		}

		current->node->data = (data_t *)Malloc(sizeof(data_t));
		if (!current->node->data) {
			printf("Memory allocation failed for data_t\n");
			// 释放已分配的内存
			block_t *temp = pool->head;
			while (temp != current) {
				pthread_mutex_destroy(&(temp->node->mutex_lock));
				Free((void **)&(temp->node->data->buf));
				Free((void **)&(temp->node->data));
				Free((void **)&(temp->node));
				block_t *next = temp->next;
				Free((void **)&temp);
				temp = next;
			}
			pthread_mutex_destroy(&(current->node->mutex_lock));
			Free((void **)&(current->node));
			Free((void **)&current);
			Free((void **)&pool);
			return NULL;
		}

		current->node->data->buf = (uint8_t *)Malloc(block_size * sizeof(uint8_t));
		if (!current->node->data->buf) {
			printf("Memory allocation failed for uint8_t\n");
			// 释放已分配的内存
			block_t *temp = pool->head;
			while (temp != current) {
				pthread_mutex_destroy(&(temp->node->mutex_lock));
				Free((void **)&(temp->node->data->buf));
				Free((void **)&(temp->node->data));
				Free((void **)&(temp->node));
				block_t *next = temp->next;
				Free((void **)&temp);
				temp = next;
			}
			pthread_mutex_destroy(&(current->node->mutex_lock));
			Free((void **)&(current->node->data));
			Free((void **)&(current->node));
			Free((void **)&current);
			Free((void **)&pool);
			return NULL;
		}

		current->node->data->len = block_size;
		current->node->data->index = i;
		current->is_free = true;
	}

	// 创建环形链表
	// 初始化完成后，链表的尾巴的下一个应该是链表的头节点
	current->next = pool->head;
	pool->tail = current;

	return pool;
}

void *allocate_memory(memory_pool_t *pool)
{
	block_t *current = pool->head;
	do {
		pthread_mutex_lock(&(current->node->mutex_lock));
		// if (current->is_free) {
		current->is_free = false;
		current->node->write_ok = WRITABLE;
		memset(current->node->data->buf, 0, block_size * sizeof(uint8_t));
		pool->head = current->next; // 更新头指针
		pthread_mutex_unlock(&(current->node->mutex_lock));
		return current->node;
		// }
		pthread_mutex_unlock(&(current->node->mutex_lock));
		usleep(100);
	} while (1);

	printf("Memory pool exhausted\n");
	return NULL;
}

void destroy_memory_pool(memory_pool_t *pool)
{
	block_t *current = pool->head;
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

static void ifp171_signal_handler(int signal)
{
	printf("catch signal:%d signal_cnt:%d\n", signal, ++signal_cnt);
	if (signal_cnt == 2) {
		lidar_api_exit();
		sleep(2);
		exit(0);
	}
}

/**
 * @description  : combine two uint8 to one uint16
 * @param         {uint8_t} high_byte
 * @param         {uint8_t} low_byte
 * @return        {uint16_t} result
 */
uint16_t combine_uint8_to_uint16(uint8_t high_byte, uint8_t low_byte)
{
	return (uint16_t)(((uint16_t)high_byte << 8) | (uint16_t)low_byte);
}

static int transform_data_to_new_node(udp_t *udp_instance, uint8_t *data, int datalen)
{
	node_t *p_node = NULL;
	p_node = (node_t *)allocate_memory(udp_instance->pool);
	pthread_mutex_lock(&(p_node->mutex_lock));
	if (p_node == NULL) {
		printf("Allocate memory fail\n");
		goto fail;
	}

	if (p_node->data == NULL) {
		printf("p_node->data is NULL\n");
		goto fail;
	}

	if (p_node->data->buf == NULL) {
		printf("p_node->data->buf is NULL\n");
		goto fail;
	}

	p_node->data->len = datalen;
	memcpy(p_node->data->buf, data, datalen);
	p_node->write_ok = READABLE;
	pthread_mutex_unlock(&(p_node->mutex_lock));

	return RET_OK;

fail:
	if (p_node != NULL) {
		// release_memory(udp_instance->pool, p_node);
	}
	return RET_FAIL;
}

static int default_deal_data_to_output_link(void *arg, uint8_t *data, int datalen)
{
	if (arg == NULL) {
		printf("parameter is NULL\n");
		return RET_FAIL;
	}

	udp_t *udp_instance = (udp_t *)arg;
	int ret = RET_FAIL;

	ret = transform_data_to_new_node(udp_instance, data, datalen);
	if (ret != RET_OK) {
		printf("transform data to a new node fail\n");
		return RET_FAIL;
	}

	return RET_OK;
}

int main(int argc, char *argv[])
{
	signal(SIGINT, ifp171_signal_handler);

	if (lidar_api_init()) {
		printf("lidar_api_init() failed!\n");
		goto end;
	}

	aval_cnt = memory_pool_size;
	udp_cmd.pool = create_memory_pool(memory_pool_size);
	udp_cmd.deal_data_to_output_link = default_deal_data_to_output_link;

	// 生产者
	printf("start producer\n");
	void *node = NULL;
	while (aval_cnt > 0) {
		node = lidar_get_bufaddr();
		if (!node) {
			usleep(100);
			continue;
		}

		for (int i = 0; i < 10; i++) {
			uint8_t *point_cloud_head = (uint8_t *)((uint8_t *)node + i * org_point_cloud_slot_size);
			uint16_t slot_number = combine_uint8_to_uint16(point_cloud_head[1], point_cloud_head[0]);

			printf("p slot_number=%d\n", slot_number);

			// 直接放到链表里
			(*udp_cmd.deal_data_to_output_link)(&udp_cmd, point_cloud_head, org_point_cloud_slot_size);

			aval_cnt--;
			usleep(1);
		}
	}

	// 消费者
	printf("start customer\n");
	block_t *current = udp_cmd.pool->head;
	node_t *current_node = current->node;
	int start_flag = 0;
	FILE *fd = fopen("whole_point_file", "wb");

	if (fd == NULL) {
		printf("fopen fail\n");
		lidar_api_exit();
		goto end;
	}

	// 800 * (32 + 96 * 32) = 2483200 字节
	while (aval_cnt < memory_pool_size - 1) {
		uint16_t slot_number = combine_uint8_to_uint16(current_node->data->buf[1], current_node->data->buf[0]);

		if (slot_number == 0) {
			start_flag = 1;
		}

		if (start_flag == 1) {
			printf("save index = %d slot_number=%d\n", current_node->data->index, slot_number);
			fwrite(current_node->data->buf, current_node->data->len, 1, fd);
		}

		current = current->next;
		current_node = current->node;
		usleep(1);
		aval_cnt++;
	}

	fclose(fd);
	lidar_api_exit();
	sleep(3);

end:
	return 0;
}
