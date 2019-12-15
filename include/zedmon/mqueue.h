#ifndef _ZEDMON_MQUEUE_H
#define _ZEDMON_MQUEUE_H

#define MAX_QUEUE_SIZE 1000
#define QUEUE_ERR_NULLPTR -1
#define QUEUE_ERR_FULL -2
#define QUEUE_ERR_EMPTY -3
#define QUEUE_ERR_OK 0

typedef struct MessageQueue {
    //fifo
    unsigned int rdPtr;
    unsigned int wrPtr;
    void* data[MAX_QUEUE_SIZE];
} MessageQueue;

int mQueueFullness(MessageQueue* mq);
int mQueueInit(MessageQueue* mq);
int mQueuePush(MessageQueue* mq, void* data);
int mQueuePop(MessageQueue* mq, void** dest);

#endif
