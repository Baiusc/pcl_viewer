
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include "concurrent_queue.h"

// 创建队列
ConcurrentQueue* queue_create(int capacity)
{
    ConcurrentQueue* queue = (ConcurrentQueue*)malloc(sizeof(ConcurrentQueue));
    if (!queue) return NULL;

    queue->head = NULL;
    queue->tail = NULL;
    queue->size = 0;
    queue->capacity = capacity;

    // 初始化互斥锁和条件变量
    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);

    return queue;
}

// 销毁队列
void queue_destroy(ConcurrentQueue* queue)
{
    if (!queue) return;

    // 清空所有节点
    QueueNode* current = queue->head;
    while (current) {
        QueueNode* next = current->next;
        free(current);
        current = next;
    }

    pthread_mutex_destroy(&queue->mutex);
    pthread_cond_destroy(&queue->not_empty);
    pthread_cond_destroy(&queue->not_full);

    free(queue);
}

// 入队操作
int queue_push(ConcurrentQueue* queue, void* data)
{
    if (!queue || !data) return -1;

    pthread_mutex_lock(&queue->mutex);

    // 等待队列非满
    while (queue->size >= queue->capacity) {
        pthread_cond_wait(&queue->not_full, &queue->mutex);
    }

    // 创建新节点
    QueueNode* new_node = (QueueNode*)malloc(sizeof(QueueNode));
    if (!new_node) {
        pthread_mutex_unlock(&queue->mutex);
        return -1;
    }

    new_node->data = data;
    new_node->next = NULL;

    if (queue->tail) {
        queue->tail->next = new_node;
    } else {
        queue->head = new_node;
    }
    queue->tail = new_node;
    queue->size++;

    // 通知消费者队列非空
    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);

    return 0;
}

// 出队操作
void* queue_pop(ConcurrentQueue* queue)
{
    if (!queue) return NULL;

    pthread_mutex_lock(&queue->mutex);

    // 等待队列非空
    while (queue->size == 0) {
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }

    QueueNode* node = queue->head;
    void* data = node->data;

    queue->head = node->next;
    if (!queue->head) {
        queue->tail = NULL;
    }

    free(node);
    queue->size--;

    // 通知生产者队列非满
    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);

    return data;
}

// 带超时的入队操作
int queue_push_timeout(ConcurrentQueue* queue, void* data, int timeout_ms)
{
    if (!queue || !data) return -1;

    pthread_mutex_lock(&queue->mutex);

    // 如果 timeout_ms == 0，不等待，立即检查
    if (timeout_ms == 0) {
        if (queue->size >= queue->capacity) {
            pthread_mutex_unlock(&queue->mutex);
            return -2; // 队列满
        }
    }
    // 如果 timeout_ms < 0，永久等待
    else if (timeout_ms < 0) {
        while (queue->size >= queue->capacity) {
            pthread_cond_wait(&queue->not_full, &queue->mutex);
        }
    }
    // 否则，带超时等待
    else {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        while (queue->size >= queue->capacity) {
            int ret = pthread_cond_timedwait(&queue->not_full, &queue->mutex, &ts);
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&queue->mutex);
                return -3; // 超时
            } else if (ret != 0) {
                pthread_mutex_unlock(&queue->mutex);
                return -1; // 其他错误
            }
        }
    }

    // 创建新节点
    QueueNode* new_node = (QueueNode*)malloc(sizeof(QueueNode));
    if (!new_node) {
        pthread_mutex_unlock(&queue->mutex);
        return -1;
    }

    new_node->data = data;
    new_node->next = NULL;

    if (queue->tail) {
        queue->tail->next = new_node;
    } else {
        queue->head = new_node;
    }
    queue->tail = new_node;
    queue->size++;

    // 通知消费者队列非空
    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);

    return 0;
}

// 带超时的出队操作
void* queue_pop_timeout(ConcurrentQueue* queue, int timeout_ms)
{
    if (!queue) return NULL;

    pthread_mutex_lock(&queue->mutex);

    // 如果 timeout_ms == 0，不等待，立即检查
    if (timeout_ms == 0) {
        if (queue->size == 0) {
            pthread_mutex_unlock(&queue->mutex);
            return NULL;
        }
    }
    // 如果 timeout_ms < 0，永久等待
    else if (timeout_ms < 0) {
        while (queue->size == 0) {
            pthread_cond_wait(&queue->not_empty, &queue->mutex);
        }
    }
    // 否则，带超时等待
    else {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_sec++;
            ts.tv_nsec -= 1000000000;
        }

        while (queue->size == 0) {
            int ret = pthread_cond_timedwait(&queue->not_empty, &queue->mutex, &ts);
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&queue->mutex);
                return NULL; // 超时返回 NULL
            } else if (ret != 0) {
                pthread_mutex_unlock(&queue->mutex);
                return NULL; // 其他错误
            }
        }
    }

    QueueNode* node = queue->head;
    void* data = node->data;

    queue->head = node->next;
    if (!queue->head) {
        queue->tail = NULL;
    }

    free(node);
    queue->size--;

    // 通知生产者队列非满
    pthread_cond_signal(&queue->not_full);
    pthread_mutex_unlock(&queue->mutex);

    return data;
}

// 检查队列是否为空
int queue_is_empty(ConcurrentQueue* queue)
{
    if (!queue) return 1;

    pthread_mutex_lock(&queue->mutex);
    int empty = (queue->size == 0);
    pthread_mutex_unlock(&queue->mutex);

    return empty;
}

// 检查队列是否已满
int queue_is_full(ConcurrentQueue* queue)
{
    if (!queue) return 0;

    pthread_mutex_lock(&queue->mutex);
    int full = (queue->size >= queue->capacity);
    pthread_mutex_unlock(&queue->mutex);

    return full;
}

// 获取队列当前大小
int queue_size(ConcurrentQueue* queue)
{
    if (!queue) return 0;

    pthread_mutex_lock(&queue->mutex);
    int size = queue->size;
    pthread_mutex_unlock(&queue->mutex);

    return size;
}
