#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "BluetoothSerial.h"
#include "openhaldex_defs.h"

#define CAN_DEBUG
//#define BT_SERIAL_DEBUG
//#define BT_SERIAL_STACK_DEBUG
#define STATE_DEBUG

#define CAN_STACK 2176
#define BT_STACK 2304

#define QUEUE_SEND(q, p, t) (uxQueueSpacesAvailable((q))) ? xQueueSendToBack((q), (p), (t)) : 0
#define ARRAY_SIZE(array) (sizeof((array)) / sizeof((array)[0]))

typedef struct can_s{
    byte status;
    SPIClass *spi_interface;
    MCP_CAN *can_interface;
    TaskHandle_t rx_task;
    TaskHandle_t tx_task;
    QueueHandle_t tx_q;
    SemaphoreHandle_t sem;
    const char* name;
}can_s;

typedef union {
    uint64_t uint64;
    uint32_t uint32[2]; 
    uint16_t uint16[4];
    uint8_t  uint8[8];
    int64_t int64;
    int32_t int32[2]; 
    int16_t int16[4];
    int8_t  int8[8];

    //deprecated names used by older code
    uint64_t value;
    struct {
        uint32_t low;
        uint32_t high;
    };
    struct {
        uint16_t s0;
        uint16_t s1;
        uint16_t s2;
        uint16_t s3;
    };
    uint8_t bytes[8];
    uint8_t byte[8]; //alternate name so you can omit the s if you feel it makes more sense
} BytesUnion;

typedef struct can_frame{
    uint32_t id;
    byte len;
    BytesUnion data;
}can_frame;

typedef enum openhaldex_mode{
    MODE_STOCK,
    MODE_FWD,
    MODE_5050,
    MODE_CUSTOM
}openhaldex_mode;

typedef struct lockpoint{
    byte speed;
    byte lock;
    byte intensity;
}lockpoint;

typedef struct openhaldex_state {
    openhaldex_mode mode;
    float ped_threshold;
    byte target_lock;
    byte vehicle_speed;
    byte haldex_state;
    byte haldex_engagement;
    lockpoint lockpoints[NUM_LOCK_POINTS];
}openhaldex_state;

typedef struct openhaldex_bt {
    BluetoothSerial *SerialBT;
    QueueHandle_t bt_q;
    bool connected;
}openhaldex_bt;

typedef struct bt_packet {
    byte len;
    BytesUnion data;
}bt_packet;

extern openhaldex_state state;

extern void get_lock_data(can_frame *frame);