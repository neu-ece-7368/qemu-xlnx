#include "zedmon/mqueue.h"

int mQueueFullness(MessageQueue* mq)
{
    if (!mq)
    {
        return QUEUE_ERR_NULLPTR;
    }

    if (mq->rdPtr == mq->wrPtr)
    {
        //empty
        return 0;
    }

    if (mq->wrPtr > mq->rdPtr)
    {
        //easiest case
        return mq->wrPtr - mq->rdPtr + 1;
    }
    else
    {
        // has wrapped
        return mq->rdPtr - mq->wrPtr;
    }

}

int mQueueInit(MessageQueue* mq)
{
    if (!mq)
    {
        return QUEUE_ERR_NULLPTR;
    }

    mq->rdPtr = 0;
    mq->wrPtr = 0;

    return QUEUE_ERR_OK;
}

int mQueuePush(MessageQueue* mq, void* data)
{
    if (!mq)
    {
        return QUEUE_ERR_NULLPTR;
    }

    if (mQueueFullness(mq) == MAX_QUEUE_SIZE)
    {
        //full
        return QUEUE_ERR_FULL;
    }

    //write
    mq->data[mq->wrPtr] = data;
    if (mq->wrPtr < (MAX_QUEUE_SIZE-1))
    {
        mq->wrPtr++;
    }
    else
    {
        //wrap around
        mq->wrPtr = 0;
    }

    return QUEUE_ERR_OK;
}

int mQueuePop(MessageQueue* mq, void** dest)
{
    void* popped;
    if (!mq || !dest)
    {
        return QUEUE_ERR_NULLPTR;
    }

    if (mQueueFullness(mq) == 0)
    {
        //empty
        return QUEUE_ERR_EMPTY;
    }

    //read
    popped = mq->data[mq->rdPtr];
    if (mq->rdPtr < (MAX_QUEUE_SIZE-1))
    {
        mq->rdPtr++;
    }
    else
    {
        //wrap around
        mq->rdPtr = 0;
    }

    *dest = popped;
    return QUEUE_ERR_OK;
}
