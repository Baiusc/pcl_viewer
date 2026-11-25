#ifndef __CONCURRENT_QUEUE_H__
#define __CONCURRENT_QUEUE_H__

#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

// 队列节点结构
typedef struct QueueNode {
    void* data;
    struct QueueNode* next;
} QueueNode;

// 并发队列结构
typedef struct {
    QueueNode* head;               // 队列头
    QueueNode* tail;               // 队列尾
    int size;                      // 当前队列大小
    int capacity;                  // 队列容量
    pthread_mutex_t mutex;         // 互斥锁
    pthread_cond_t not_empty;      // 非空条件变量
    pthread_cond_t not_full;       // 非满条件变量
} ConcurrentQueue;

// 函数声明
ConcurrentQueue* queue_create(int capacity);
void queue_destroy(ConcurrentQueue* queue);
int queue_push(ConcurrentQueue* queue, void* data);
void* queue_pop(ConcurrentQueue* queue);

// 带超时的操作（timeout_ms: 超时时间，单位毫秒；0表示立即返回；-1表示永久等待）
int queue_push_timeout(ConcurrentQueue* queue, void* data, int timeout_ms);
void* queue_pop_timeout(ConcurrentQueue* queue, int timeout_ms);

int queue_is_empty(ConcurrentQueue* queue);
int queue_is_full(ConcurrentQueue* queue);
int queue_size(ConcurrentQueue* queue);

#ifdef __cplusplus
}
#endif

#endif // __CONCURRENT_QUEUE_H__
