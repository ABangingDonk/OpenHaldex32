#define HALDEX_ID                 0x2C0
#define BRAKES1_ID                0x1A0
#define BRAKES3_ID                0x4A0
#define MOTOR1_ID                 0x280
#define MOTOR2_ID                 0x288
#define MOTOR3_ID                 0x380
#define MOTOR6_ID                 0x488

#define NUM_LOCK_POINTS 10

#define DATA_CTRL_CHECK_LOCKPOINTS  0
#define DATA_CTRL_CLEAR             1
#define DATA_CTRL_CHECK_MODE        2

#define APP_MSG_MODE        0
#define APP_MSG_STATUS      1
#define APP_MSG_CUSTOM_DATA 2
#define APP_MSG_CUSTOM_CTRL 3

#define SERIAL_PACKET_END   0xff

#define CAN_STACK 2176
#define BT_STACK 2304

#define CAN_TX_RELIEF_MS 10
#define QUEUE_SEND_WAIT (20 / portTICK_PERIOD_MS)
#define QUEUE_SEND(q, p) (uxQueueSpacesAvailable((q))) ? xQueueSendToBack((q), (p), QUEUE_SEND_WAIT) : 0
#define ARRAY_SIZE(array) (sizeof((array)) / sizeof((array)[0]))

typedef struct can_s{
    byte status;
    SPIClass *spi_interface;
    MCP_CAN *can_interface;
    TaskHandle_t rx_task;
    TaskHandle_t tx_task;
    QueueHandle_t tx_q;
    SemaphoreHandle_t rx_sem;
    SemaphoreHandle_t tx_sem;
    const char* name;
}can_s;

typedef enum openhaldex_mode_id{
    MODE_STOCK,
    MODE_FWD,
    MODE_5050,
    MODE_CUSTOM
}openhaldex_mode_id;

typedef struct lockpoint{
    byte speed;
    byte lock;
    byte intensity;
}lockpoint;

typedef struct openhaldex_custom_mode{
    lockpoint lockpoints[NUM_LOCK_POINTS];
    uint16_t lockpoint_rx;
    byte lockpoint_count;
}openhaldex_custom_mode;

typedef struct openhaldex_state {
    openhaldex_mode_id mode;
    openhaldex_custom_mode custom_mode;
    float ped_threshold;
}openhaldex_state;

typedef struct openhaldex_bt {
    BluetoothSerial *SerialBT;
    SemaphoreHandle_t sem;
    QueueHandle_t bt_process_q;
    QueueHandle_t bt_tx_q;
    TaskHandle_t bt_rx_task;
    TaskHandle_t bt_tx_task;
    TaskHandle_t bt_process_task;
}openhaldex_bt;

typedef struct bt_packet {
    byte len;
    byte data[6];
}bt_packet;