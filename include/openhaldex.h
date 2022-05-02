#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Preferences.h>
#include "BluetoothSerial.h"
#include "openhaldex_defs.h"
#include "openhaldex_can.h"

/* Defines */
#define CAN_DEBUG
#define CAN_TEST_DATA
//#define BT_SERIAL_DEBUG_RX
//#define BT_SERIAL_DEBUG_TX
//#define STATE_DEBUG
//#define STACK_DEBUG

/* Globals */
extern openhaldex_bt bt;
extern openhaldex_state state;
extern byte vehicle_speed;
extern byte haldex_engagement;
extern byte haldex_state;
extern float lock_target;
extern float ped_value;
extern can_s body_can;
extern can_s haldex_can;
extern Preferences preferences;
extern TaskHandle_t process_task;

/* Functions */
extern void bt_send_status(void *params);
extern void bt_process(void *params);
extern void bt_tx(void *params);
extern void bt_rx(void *params);
extern void save_prefs(void *params);
#ifdef CAN_TEST_DATA
extern void send_test_data(void *params);
#endif
extern void body_can_comms(void *params);
extern void haldex_can_comms(void *params);
extern void can_process(void *params);
extern bool bt_send_msg_buf(bt_packet *packet);
extern bool bt_read_msg_buf(byte terminator, bt_packet *packet);
extern uint8_t get_lock_target_adjusted_value(uint8_t value);
extern float get_lock_target_adjustment(void);