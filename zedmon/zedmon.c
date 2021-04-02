
#include <pthread.h>
#include <zmq.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "zedmon/mqueue.h"
#include "zedmon/zedmon.h"

#define PERIPHERAL_LABEL_SIZE 32

static const char* EVENT_CLASS_NAMES[] = {"GPIO", "TIMER"};

typedef struct ZPeripheral {
    unsigned char pType;
    char *pLabel;
    ZedmonReadCallback read;
    ZedmonWriteCallback write;
} ZPeripheral;

#define MAX_PERIPHERALS 64
typedef struct ExtCommData {
    // external comm stuff
    pthread_t ipc_thread;
    pthread_t pub_thread;

    char loop_stop;
    pthread_mutex_t gpio_mutex;

    //message queues
    MessageQueue boardEvents; // push board HW change notification

    unsigned int flags;

    unsigned int pCount;
    ZPeripheral peripherals[MAX_PERIPHERALS];
} ExtCommData;

typedef struct evtWrapper {
    unsigned char evtType;
    void* evtData;
    unsigned int evtFlags;
} evtWrapper;

static ExtCommData externalComm;

static int list_peripherals(char* destBuf, unsigned int bufSize)
{
    unsigned int strSize = 0;
    unsigned int pIdx = 0;
    const char* pType;
    const char* pLabel;

    if (!destBuf)
    {
        return -EFAULT;
    }

    for (pIdx = 0; pIdx < externalComm.pCount; pIdx++)
    {
        //peripheral preamble
        pType = EVENT_CLASS_NAMES[externalComm.peripherals[pIdx].pType];
        pLabel = externalComm.peripherals[pIdx].pLabel;
        strSize += sprintf(destBuf, "%u %s %s", pIdx, pType, pLabel);

        //type-specific information
        if (externalComm.peripherals[pIdx].pType == ZEDMON_EVENT_CLASS_GPIO)
        {
            //
        }

        //finalize peripheral
        strSize += sprintf(destBuf+strSize, "\n");
    }

    return strSize;
}

static int put_hex_value(char* destBuf, unsigned int bufSize, unsigned int value)
{
    if (!destBuf)
    {
        return -EFAULT;
    }

    return snprintf(destBuf, bufSize, "0x%08x\n", value);
}

static void parse_zedmon_cmd(char* cmdstr, void* responder)
{
    const char delimiters[] = " ";
    char* argument = strsep(&cmdstr, delimiters);
    char respBuffer[1000];
    int ret = 0;
    int chip = 0;
    unsigned int regVal = 0, value = 0;

    if (!argument)
    {
        return;
    }

    if (!strcmp(argument, "PERIPHERALS"))
    {
        ret = list_peripherals(respBuffer, 1000);
        if (ret > 0)
        {
            //send response
            zmq_send(responder, respBuffer, ret, 0);
        }
        return;
    }
    else if (!strcmp(argument, "GPIO"))
    {
        argument = strsep(&cmdstr, delimiters);
        if (!argument)
        {
            //ERROR!
            return;
        }

        if (!strcmp(argument, "GET"))
        {
            //get state
            argument = strsep(&cmdstr, delimiters);
            if (!argument)
            {
                //error
                return;
            }

            if (!strcmp(argument, "DIRECTION"))
            {
                //get direction
                argument = strsep(&cmdstr, delimiters);
                if (!argument)
                {
                    //error
                    return;
                }

                //gpiochip
                chip = strtol(argument, NULL, 10) - 1;

                //perform read
                //if (chip > externalComm.pCount || chip < 0)
                //{
                //    return;
                //}
                (externalComm.peripherals[0].read)(chip, 4, &regVal);
                ret = put_hex_value(respBuffer, 1000, regVal);
                if (ret < 0)
                {
                    return;
                }
                //send
                zmq_send(responder, respBuffer, ret, 0);
            }
            else if (!strcmp(argument, "VALUE"))
            {
                //get value
                argument = strsep(&cmdstr, delimiters);
                if (!argument)
                {
                    //error
                    return;
                }

                //gpiochip
                chip = strtol(argument, NULL, 10) - 1;

                (externalComm.peripherals[0].read)(chip, 0, &regVal);
                ret = put_hex_value(respBuffer, 1000, regVal);
                if (ret < 0)
                {
                    return;
                }
                //send
                zmq_send(responder, respBuffer, ret, 0);
            }
            else
            {
                //error
                return;
            }
        }
        else if (!strcmp(argument, "SET"))
        {
            //set state
            argument = strsep(&cmdstr, delimiters);
            if (!argument)
            {
                //error
                return;
            }

            //gpiochip
            chip = strtol(argument, NULL, 10) - 1;

            //value
            argument = strsep(&cmdstr, delimiters);
            if (!argument)
            {
                //error
                return;
            }

            value = strtol(argument, NULL, 10);

            //read
            //(externalComm.peripherals[0].read)(chip, 0, &regVal);
            (externalComm.peripherals[0].write)(chip, 0, &value);

            zmq_send(responder, "OK\n", 3, 0);
        }
        else
        {
            printf("parse_zedmon_cmd in error\n");
            //error
            return;
        }
    }
}

static void* zedmon_comm_loop(void* tArg)
{
    void *context = zmq_ctx_new();
    void *responder = zmq_socket(context, ZMQ_REP);
    int rc = zmq_bind(responder, "tcp://127.0.0.1:20001");
    int count = 0;

    if (rc)
    {
        //error! TODO:print out to QEMU console
        return (void*)1;
    }

    while (!externalComm.loop_stop)
    {
        char buffer[200];
        count = zmq_recv(responder, buffer, 200, 0);
        buffer[count] = '\0';
        parse_zedmon_cmd(buffer, responder);
        //do stuff here
        usleep(1000);
        //zmq_send(responder, "World", 5, 0);
    }

    return (void*)0;
}

//standardized size
#define PUBLISHER_STRING_SIZE 100
static int evt_to_publisher_string(evtWrapper *evt, char *destBuf)
{
    if (!evt || ! destBuf)
    {
        return -1;
    }

    if (evt->evtType == ZEDMON_EVENT_CLASS_GPIO)
    {
        GPIOEvent* gEvt = (GPIOEvent*)evt->evtData;
        return snprintf(destBuf, PUBLISHER_STRING_SIZE, "GPIO %d CHANNEL %d EVENT %s DATA 0x%08x\n",
                        gEvt->gpio_dev,
                        gEvt->channel,
                        (gEvt->type == GPIO_EVT_VALUE) ? "VALUE" :
                        (gEvt->type == GPIO_EVT_DIRECTION) ? "DIR" :
                        "UNKNOWN",
                        (unsigned int)((long int)gEvt->data));
    }
    if (evt->evtType == ZEDMON_EVENT_CLASS_TIMER)
    {
        printf("identified timer event\n");
        TimerEvent* gEvt = (TimerEvent*)evt->evtData;
        int numChars = snprintf(destBuf, PUBLISHER_STRING_SIZE, "EVENT %s DATA 0x%08x\n",
                        (gEvt->type == TIMER_EVT_DUTY) ? "DUTY" : "UNKNOWN",
                        (unsigned int)((long int)gEvt->data));
        printf("numChars: %d\n", numChars);
        for(int i = 0; i < 27; i++) {
            printf("%c", destBuf[i]);
        }
        return numChars;
    }

    //unknown event type
    return -1;
}

static void* zedmon_publish_loop(void* tArg)
{
    void *context = zmq_ctx_new();
    void *publisher = zmq_socket(context, ZMQ_PUB);
    int rc = zmq_bind(publisher, "tcp://127.0.0.1:20000");
    evtWrapper* evt = NULL;
    char msgBuffer[PUBLISHER_STRING_SIZE];
    volatile int ret = 0;

    if (rc)
    {
        return (void*)1;
    }

    ret = mQueueFullness(&externalComm.boardEvents);

    while (!externalComm.loop_stop)
    {
        //check for pending events
        if (mQueueFullness(&externalComm.boardEvents) > 0)
        {
            //pop
            if (!mQueuePop(&externalComm.boardEvents, (void**)(&evt)))
            {
                //publish
                ret = evt_to_publisher_string(evt, msgBuffer);
                if (ret < 0)
                {
                    //error
                    continue;
                }
                int check = zmq_send(publisher, msgBuffer, ret, ZMQ_DONTWAIT);
                printf("check: %d\n", check);

                //check flags
                if (evt->evtFlags & ZEDMON_EVENT_FLAG_DESTROY)
                {
                    //free event data
                    free(evt->evtData);
                }

                //free event wrapper memory
                free(evt);
            }
        }
        usleep(1000);
    }

    return (void*)0;
}

int zedmon_notify_event(unsigned char evtType, void* evtData, unsigned int evtFlags)
{
    evtWrapper* w;

    if (!(externalComm.flags & ZEDMON_FLAG_INIT))
    {
        return ZEDMON_ERR_NOT_INIT;
    }

    w = (evtWrapper*)malloc(sizeof(evtWrapper));

    if (!w)
    {
        return ZEDMON_ERR_UNKNOWN;
    }

    //populate event wrapper
    w->evtType = evtType;
    w->evtData = evtData;
    w->evtFlags = evtFlags;
    printf("calling zedmon notify event\n");

    return mQueuePush(&externalComm.boardEvents, (void*)w);
}

int zedmon_init(void)
{
    memset(&externalComm, 0, sizeof(ExtCommData));
    mQueueInit(&externalComm.boardEvents);

    // launch external interface thread
    if (pthread_create(&(externalComm.ipc_thread), NULL, zedmon_comm_loop, NULL))
    {
        //error creating thread
        return ZEDMON_ERR_INIT_FAIL;
    }

    if (pthread_create(&(externalComm.pub_thread), NULL, zedmon_publish_loop, NULL))
    {
        //error, do something
        externalComm.loop_stop = 1;
        pthread_join(externalComm.ipc_thread, 0);
        return ZEDMON_ERR_INIT_FAIL;
    }

    //flag as initialized
    externalComm.flags |= ZEDMON_FLAG_INIT;

    return 0;
}

int zedmon_register_peripheral(unsigned char evtType, const char* label,
                               ZedmonReadCallback read, ZedmonWriteCallback write)
{

    if (!(externalComm.flags & ZEDMON_FLAG_INIT))
    {
        return ZEDMON_ERR_NOT_INIT;
    }
    if (externalComm.pCount > (MAX_PERIPHERALS - 1))
    {
        return ZEDMON_ERR_PERIPH_LIMIT;
    }

    //register peripheral
    externalComm.peripherals[externalComm.pCount].pType = evtType;
    externalComm.peripherals[externalComm.pCount].read = read;
    externalComm.peripherals[externalComm.pCount].write = write;
    externalComm.peripherals[externalComm.pCount].pLabel = strndup(label, PERIPHERAL_LABEL_SIZE);
    externalComm.pCount++;

    return 0;
}
