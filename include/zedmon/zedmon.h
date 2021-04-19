#ifndef _ZEDMON_H_INCLUDED
#define _ZEDMON_H_INCLUDED

//event classes
#define ZEDMON_EVENT_CLASS_GPIO 0x00
#define ZEDMON_EVENT_CLASS_TIMER 0x01
#define ZEDMON_EVENT_CLASS_WAV 0x02

//errors
#define ZEDMON_ERR_PERIPH_LIMIT -13
#define ZEDMON_ERR_NOT_INIT -14
#define ZEDMON_ERR_INIT_FAIL -15
#define ZEDMON_ERR_UNKNOWN -16

//flags
#define ZEDMON_FLAG_INIT 0x01

//event flags
#define ZEDMON_EVENT_FLAG_DESTROY 0x01

//events
#define GPIO_EVT_DIRECTION 0
#define GPIO_EVT_VALUE 1
typedef struct GPIOEvent {
    unsigned char type;
    unsigned char gpio_dev;
    unsigned char channel;
    unsigned int value;
    void* data;
} GPIOEvent;

#define TIMER_EVT_DUTY 0
typedef struct TimerEvent {
    unsigned char type;
    void* data;
} TimerEvent;

typedef struct WAVEvent {
    char filename[95];
} WAVEvent;

//callback types
typedef int (*ZedmonReadCallback)(unsigned int pIdx, unsigned int dIdx, void* data);
typedef int (*ZedmonWriteCallback)(unsigned int pIdx, unsigned int dIdx, void* data);

//function prototypes
int zedmon_init(void);
int zedmon_notify_event(unsigned char evtType, void* evtData, unsigned int evtFlags);
int zedmon_register_peripheral(unsigned char evtType, const char* label,
                               ZedmonReadCallback read, ZedmonWriteCallback);

#endif
