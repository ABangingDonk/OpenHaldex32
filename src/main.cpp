#include "openhaldex.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

openhaldex_bt bt;
openhaldex_state state;
Preferences preferences;
byte haldex_state;
byte haldex_engagement;
byte vehicle_speed;

can_s body_can = {
    0,                          // CAN status
    NULL,                       // SPI interface
    new MCP_CAN(GPIO_NUM_15),   // CAN interface
    NULL,                       // RX task
    NULL,                       // TX task
    NULL,                       // TX queue
    NULL,                       // Semaphore
    "Body"                      // Name
};

can_s haldex_can = {
    0,                          // CAN status
    NULL,                       // SPI interface
    new MCP_CAN(GPIO_NUM_5),    // CAN interface
    NULL,                       // RX task
    NULL,                       // TX task
    NULL,                       // TX queue
    NULL,                       // Semaphore
    "Haldex"                    // Name
};

bool can_send_msg_buf(can_s *can, can_frame *frame)
{
    if (xSemaphoreTake(can->sem, 100 / portTICK_PERIOD_MS))
    {
        can->can_interface->sendMsgBuf(frame->id, frame->len, frame->data.bytes);
        xSemaphoreGive(can->sem);
        return true;
    }
    return false;
}

bool can_read_msg_buf(can_s *can, can_frame *frame)
{
    if (xSemaphoreTake(can->sem, 100 / portTICK_PERIOD_MS))
    {
        can->can_interface->readMsgBuf(&frame->id, &frame->len, frame->data.bytes);
        xSemaphoreGive(can->sem);
        return true;
    }
    return false;
}

bool bt_send_msg_buf(bt_packet *packet)
{
    if (xSemaphoreTake(bt.sem, 100 / portTICK_PERIOD_MS))
    {
        bt.SerialBT->write(packet->data, packet->len);
        xSemaphoreGive(bt.sem);
        return true;
    }
    return false;
}

bool bt_read_msg_buf(byte terminator, bt_packet *packet)
{
    if (xSemaphoreTake(bt.sem, 100 / portTICK_PERIOD_MS))
    {
        packet->len = bt.SerialBT->readBytesUntil(terminator, packet->data, ARRAY_SIZE(packet->data));
        xSemaphoreGive(bt.sem);
        return true;
    }
    return false;
}

#ifdef CAN_TEST_DATA
void send_test_data(void *params)
{
    while (1)
    {
        can_frame body_frame = {0};
        body_frame.id = HALDEX_ID;
        body_frame.len = 2;
        body_frame.data.bytes[0] = 0x00;
        body_frame.data.bytes[1] = 0x7f;
        QUEUE_SEND(body_can.tx_q, &body_frame, 100 / portTICK_PERIOD_MS);

        can_frame haldex_frame = {0};
        haldex_frame.id = MOTOR1_ID;
        haldex_frame.len = 8;
        haldex_frame.data.bytes[0] = 0;
        haldex_frame.data.bytes[1] = 0xf0;
        haldex_frame.data.bytes[2] = 0x20;
        haldex_frame.data.bytes[3] = 0x4e;
        haldex_frame.data.bytes[4] = 0xf0;
        haldex_frame.data.bytes[5] = 0xf0;
        haldex_frame.data.bytes[6] = 0x20;
        haldex_frame.data.bytes[7] = 0xf0;
        QUEUE_SEND(haldex_can.tx_q, &haldex_frame, 100 / portTICK_PERIOD_MS);

        vTaskDelete(NULL);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#endif

void body_can_rx(void *params)
{
    while (1)
    {
        can_frame frame = {0};
        if (can_read_msg_buf(&body_can, &frame))
        {
#ifdef CAN_DEBUG
            Serial.printf("Body CAN RX 0x%03x: [%d]\n", frame.id, frame.len);
#endif
            switch(frame.id)
            {
                case MOTOR1_ID:
                    ped_value = frame.data.bytes[5] * 0.4;
                    if (state.mode == MODE_FWD)
                    {
                        memset(&frame.data, 0, frame.len);
                    }
                    break;
                case MOTOR2_ID:
                    vehicle_speed = (byte)((frame.data.bytes[3] * 100 * 128) / 10000);
                    break;
            }
            
            if (state.mode == MODE_5050 || state.mode == MODE_CUSTOM)
            {
                get_lock_data(&frame);
            }

            QUEUE_SEND(haldex_can.tx_q, &frame, 100 / portTICK_PERIOD_MS);
        }
        else
        {
            Serial.println("Body RX cannot read");
        }
        vTaskSuspend(NULL);
    }
}

void body_can_tx(void *params)
{
    while (1)
    {
        can_frame frame = {0};
        if (xQueueReceive(body_can.tx_q, &frame, 10 / portTICK_PERIOD_MS))
        {
            if (!can_send_msg_buf(&body_can, &frame))
            {
                Serial.println("Body TX busy");
            }
        }
        vTaskDelay(CAN_TX_RASTER_MS / portTICK_PERIOD_MS);
    }
}

void haldex_can_rx(void *params)
{
    while (1)
    {
        can_frame frame = {0};
        if (can_read_msg_buf(&haldex_can, &frame))
        {
#ifdef CAN_DEBUG
            Serial.printf("Haldex CAN RX 0x%03x: [%d]\n", frame.id, frame.len);
#endif
            QUEUE_SEND(body_can.tx_q, &frame, 100 / portTICK_PERIOD_MS);
            haldex_state = frame.data.bytes[0];
            haldex_engagement = frame.data.bytes[1] == SERIAL_PACKET_END ? 0xfe
                                                                         : frame.data.bytes[1];
        }
        else
        {
            Serial.println("Haldex RX cannot read");
        }
        vTaskSuspend(NULL);
    }
}

void haldex_can_tx(void *params)
{
    while (1)
    {
        can_frame frame = {0};
        if (xQueueReceive(haldex_can.tx_q, &frame, 10 / portTICK_PERIOD_MS))
        {
            if (!can_send_msg_buf(&haldex_can, &frame))
            {
                Serial.println("Haldex TX busy");
            }
        }
        vTaskDelay(CAN_TX_RASTER_MS / portTICK_PERIOD_MS);
    }
}

void body_can_isr(void)
{
    if (xTaskResumeFromISR(body_can.rx_task))
    {
        portYIELD_FROM_ISR();
    }
}

void haldex_can_isr(void)
{
    if (xTaskResumeFromISR(haldex_can.rx_task))
    {
        portYIELD_FROM_ISR();
    }
}

#ifdef STACK_DEBUG
static void print_stack_hwm(void)
{
    Serial.printf("%s stack HWM %u\n", pcTaskGetTaskName(body_can.rx_task), uxTaskGetStackHighWaterMark(body_can.rx_task));
    Serial.printf("%s stack HWM %u\n", pcTaskGetTaskName(body_can.tx_task), uxTaskGetStackHighWaterMark(body_can.tx_task));
    Serial.printf("%s stack HWM %u\n", pcTaskGetTaskName(haldex_can.rx_task), uxTaskGetStackHighWaterMark(haldex_can.rx_task));
    Serial.printf("%s stack HWM %u\n", pcTaskGetTaskName(haldex_can.tx_task), uxTaskGetStackHighWaterMark(haldex_can.tx_task));
    Serial.printf("%s stack HWM %u\n", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
}
#endif

void bt_send_status(void *params)
{
    while (1)
    {
        bt_packet packet;
        packet.data[0] = APP_MSG_STATUS;
        packet.data[1] = haldex_state;
        packet.data[2] = haldex_engagement;
        packet.data[3] = lock_target;
        packet.data[4] = vehicle_speed;
        packet.data[5] = SERIAL_PACKET_END;
        packet.len = 6;
        QUEUE_SEND(bt.bt_tx_q, &packet, 100 / portTICK_PERIOD_MS);

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void bt_process(void *params)
{
    while (1)
    {
        bt_packet packet;
        if (xQueueReceive(bt.bt_process_q, &packet, 10 / portTICK_PERIOD_MS))
        {
            byte lockpoint_index;
            bt_packet tx_packet;
            switch (packet.data[0])
            {
                case APP_MSG_MODE:
                    state.mode = packet.data[1] <= MODE_CUSTOM ? (openhaldex_mode_id)packet.data[1]
                                                               : MODE_STOCK;
                    state.ped_threshold = packet.data[2];
                    if (state.mode == MODE_5050)
                    {
                        if (ped_value >= state.ped_threshold || state.ped_threshold == 0)
                        {
                            lock_target = 100;
                        }
                        else
                        {
                            lock_target = 0;
                        }
                    }
                    else if (state.mode == MODE_FWD)
                    {
                        lock_target = 0;
                    }
                    break;
                case APP_MSG_CUSTOM_DATA:
                    lockpoint_index = packet.data[1];
                    if (lockpoint_index < NUM_LOCK_POINTS)
                    {
                        state.custom_mode.lockpoints[lockpoint_index].speed = packet.data[2];
                        state.custom_mode.lockpoints[lockpoint_index].lock = packet.data[3];
                        state.custom_mode.lockpoints[lockpoint_index].intensity = packet.data[4];

                        state.custom_mode.lockpoint_rx |= (1 << lockpoint_index);
                        state.custom_mode.lockpoint_count++;
                    }
                    break;
                case APP_MSG_CUSTOM_CTRL:
                    switch (packet.data[1])
                    {
                        case DATA_CTRL_CHECK_LOCKPOINTS:
                            tx_packet.data[0] = APP_MSG_CUSTOM_CTRL;
                            tx_packet.data[1] = DATA_CTRL_CHECK_LOCKPOINTS;
                            tx_packet.data[2] = state.custom_mode.lockpoint_rx & 0xff;
                            tx_packet.data[3] = (state.custom_mode.lockpoint_rx << 8) & 0xff;
                            tx_packet.data[4] = SERIAL_PACKET_END;
                            tx_packet.len = 5;

                            QUEUE_SEND(bt.bt_tx_q, &tx_packet, 100 / portTICK_PERIOD_MS);
                            break;
                        case DATA_CTRL_CLEAR:
                            state.custom_mode.lockpoint_rx = 0;
                            state.custom_mode.lockpoint_count = 0;
                            memset(state.custom_mode.lockpoints, 0, sizeof(state.custom_mode.lockpoints));
                            break;
                        case DATA_CTRL_CHECK_MODE:
                            tx_packet.data[0] = APP_MSG_CUSTOM_CTRL;
                            tx_packet.data[1] = DATA_CTRL_CHECK_MODE;
                            tx_packet.data[2] = state.mode;
                            tx_packet.data[3] = state.ped_threshold;
                            tx_packet.data[4] = SERIAL_PACKET_END;
                            tx_packet.len = 5;

                            QUEUE_SEND(bt.bt_tx_q, &tx_packet, 100 / portTICK_PERIOD_MS);
                            break;
                    }
                    break;
            }
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void bt_tx(void *params)
{
    while (1)
    {
        bt_packet packet;
        xQueueReceive(bt.bt_tx_q, &packet, 10 / portTICK_PERIOD_MS);

        if (bt.SerialBT->connected())
        {
            bt_send_msg_buf(&packet);
#ifdef BT_SERIAL_DEBUG_TX
            Serial.write(packet.data, packet.len);
#endif
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void bt_rx(void *params)
{
    while (1)
    {        
        if (bt.SerialBT->connected())
        {
            if (bt.SerialBT->available())
            {
                bt_packet packet;
                bt_read_msg_buf(SERIAL_PACKET_END, &packet);
#ifdef BT_SERIAL_DEBUG_RX
                Serial.write(packet.data, packet.len);
#endif
                QUEUE_SEND(bt.bt_process_q, &packet, 100 / portTICK_PERIOD_MS);
            }

        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void save_prefs(void *params)
{
    while (1)
    {
        openhaldex_state saved_state;
        if (preferences.getBytes("state", &saved_state, sizeof(state)) != 0)
        {
            if (memcmp(&saved_state, &state, sizeof(state)) != 0)
            {
                preferences.putBytes("state", &state, sizeof(state));
            }
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\nOpenHaldex32 init");

    preferences.begin("openhaldex", false);

    if (preferences.getBytes("state", &state, sizeof(state)) == 0)
    {
        memset(&state, 0, sizeof(state));
        Serial.println("Previous state not found, defaults loaded");
        preferences.putBytes("state", &state, sizeof(state));
    }
    else
    {
        Serial.println("Loaded previous state");
    }

    body_can.spi_interface = new SPIClass(HSPI);
    haldex_can.spi_interface = new SPIClass(VSPI);

    body_can.tx_q = xQueueCreate(16, sizeof(can_frame));
    haldex_can.tx_q = xQueueCreate(16, sizeof(can_frame));
    bt.bt_process_q = xQueueCreate(32, sizeof(bt_packet));
    bt.bt_tx_q = xQueueCreate(32, sizeof(bt_packet));

    body_can.sem = xSemaphoreCreateMutex();
    xSemaphoreGive(body_can.sem);
    haldex_can.sem = xSemaphoreCreateMutex();
    xSemaphoreGive(haldex_can.sem);
    bt.sem = xSemaphoreCreateMutex();
    xSemaphoreGive(bt.sem);

    body_can.spi_interface->setFrequency(10000000);
    haldex_can.spi_interface->setFrequency(10000000);

    body_can.spi_interface->begin();
    haldex_can.spi_interface->begin();

    body_can.can_interface->setSpiInstance(body_can.spi_interface);
    haldex_can.can_interface->setSpiInstance(haldex_can.spi_interface);

    bt.SerialBT = new BluetoothSerial;
    bt.SerialBT->begin("OpenHaldex32");
    Serial.println("BT initialised");

    // init CAN0 bus, baudrate: 500k@16MHz
    body_can.status = body_can.can_interface->begin(MCP_STD, CAN_500KBPS, MCP_20MHZ);
    if(body_can.status == CAN_OK)
    {
        body_can.can_interface->init_Filt(0, 0, 0x080);
        body_can.can_interface->init_Filt(1, 0, 0x080);
        body_can.can_interface->init_Mask(0, 0, 0x0E0);

        body_can.can_interface->init_Filt(2, 0, 0x0A0);
        body_can.can_interface->init_Filt(3, 0, 0x0A0);
        body_can.can_interface->init_Filt(4, 0, 0x0A0);
        body_can.can_interface->init_Filt(5, 0, 0x0A0);
        body_can.can_interface->init_Mask(1, 0, 0x0E0);
        body_can.can_interface->setMode(MCP_NORMAL);
        Serial.println("Body CAN: Init OK!");
    }
    else
    {
        Serial.printf("Body CAN: Init Fail!!! (%d)\n", body_can.status);
    }
    
    // init CAN1 bus, baudrate: 500k@16MHz
    haldex_can.status = haldex_can.can_interface->begin(MCP_STD, CAN_500KBPS, MCP_20MHZ);
    if(haldex_can.status == CAN_OK)
    {
        haldex_can.can_interface->init_Filt(0, 0, 0x2C0);
        haldex_can.can_interface->init_Filt(1, 0, 0x2C0);
        haldex_can.can_interface->init_Mask(0, 0, 0x7FF);

        haldex_can.can_interface->init_Filt(2, 0, 0x2C0);
        haldex_can.can_interface->init_Filt(3, 0, 0x2C0);
        haldex_can.can_interface->init_Filt(4, 0, 0x2C0);
        haldex_can.can_interface->init_Filt(5, 0, 0x2C0);
        haldex_can.can_interface->init_Mask(1, 0, 0x7FF);
        haldex_can.can_interface->setMode(MCP_NORMAL);
        Serial.println("Haldex CAN: Init OK!");
    }
    else
    {
        Serial.printf("Haldex CAN: Init Fail!!! (%d)\n", haldex_can.status);
    }

    pinMode(GPIO_NUM_16, INPUT_PULLUP);
    attachInterrupt(GPIO_NUM_16, body_can_isr, FALLING);
    pinMode(GPIO_NUM_17, INPUT_PULLUP);
    attachInterrupt(GPIO_NUM_17, haldex_can_isr, FALLING);

    xTaskCreate(
        bt_rx,
        "bt_rx",
        BT_STACK,
        NULL,
        1,
        &bt.bt_rx_task
    );

    xTaskCreate(
        bt_tx,
        "bt_tx",
        BT_STACK,
        NULL,
        1,
        &bt.bt_tx_task
    );

    xTaskCreate(
        bt_process,
        "bt_process",
        1024,
        NULL,
        1,
        &bt.bt_process_task
    );

    xTaskCreate(
        bt_send_status,
        "bt_send_status",
        1024,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        save_prefs,
        "save_prefs",
        2048,
        NULL,
        1,
        NULL
    );

    xTaskCreate(
        body_can_rx,
        "body_can_rx",
        CAN_STACK,
        NULL,
        3,
        &body_can.rx_task
    );

    xTaskCreate(
        haldex_can_rx,
        "haldex_can_rx",
        CAN_STACK,
        NULL,
        3,
        &haldex_can.rx_task
    );

    xTaskCreate(
        body_can_tx,
        "body_can_tx",
        CAN_STACK,
        NULL,
        2,
        &body_can.tx_task
    );

    xTaskCreate(
        haldex_can_tx,
        "haldex_can_tx",
        CAN_STACK,
        NULL,
        2,
        &haldex_can.tx_task
    );
#ifdef CAN_TEST_DATA
    xTaskCreate(
        send_test_data,
        "send_test_data",
        CAN_STACK,
        NULL,
        1,
        NULL
    );
#endif
}

void loop()
{
    /* Task driven */
}