#include "openhaldex.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

openhaldex_bt bt;
openhaldex_state state;

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

void send_test_data(void *params)
{
    while (1)
    {
#if 1
        can_frame body_frame = {0};
        body_frame.id = HALDEX_ID;
        body_frame.len = 2;
        body_frame.data.bytes[0] = 0x00;
        body_frame.data.bytes[1] = 0x7f;
        QUEUE_SEND(body_can.tx_q, &body_frame, 100 / portTICK_PERIOD_MS);
#endif        
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

#ifdef BT_SERIAL_DEBUG
        bt_packet bt_tx;
        bt_tx.len = 8;
        bt_tx.data.bytes[0] = 0xff;
        bt_tx.data.bytes[1] = 0xff;
        bt_tx.data.bytes[2] = 0xff;
        bt_tx.data.bytes[3] = 0xff;
        bt_tx.data.bytes[4] = 0x00;
        bt_tx.data.bytes[5] = 0x00;
        bt_tx.data.bytes[6] = 0x00;
        bt_tx.data.bytes[7] = 0x00;
        xQueueSendToBack(bt.bt_q, &bt_tx, 100 / portTICK_PERIOD_MS);
#endif
        vTaskDelete(NULL);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

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
            get_lock_data(&frame);
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
        if (xQueueReceive(body_can.tx_q, &frame, 100 / portTICK_PERIOD_MS))
        {
            if (!can_send_msg_buf(&body_can, &frame))
            {
                Serial.println("Body TX busy");
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
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
            state.haldex_state = frame.data.bytes[0];
            state.haldex_engagement = frame.data.bytes[1] == 0xff ? 0xfe
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
        if (xQueueReceive(haldex_can.tx_q, &frame, 100 / portTICK_PERIOD_MS))
        {
            if (!can_send_msg_buf(&haldex_can, &frame))
            {
                Serial.println("Haldex TX busy");
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
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

void bt_comms(void *params)
{
    while (1)
    {
        bt.connected = bt.SerialBT->connected();
        
        if (bt.SerialBT->available())
        {
            byte bt_rx_buf[257];
            size_t rxLen = bt.SerialBT->readBytesUntil('\n', bt_rx_buf, 256);
            bt_rx_buf[rxLen++] = '\n';
            Serial.write(bt_rx_buf, rxLen);
        }

/* unused */
#if 0
        bt_packet bt_tx;
        if (xQueueReceive(bt.bt_q, &bt_tx, 100 / portTICK_PERIOD_MS))
        {
            for (int i = 0; i < bt_tx.len; i++)
            {
                bt.SerialBT->write(bt_tx.data.bytes[i]);
            }
        }
#endif

        if (bt.connected)
        {
            bt.SerialBT->write(0xff);
            bt.SerialBT->write(state.haldex_state);
            bt.SerialBT->write(state.haldex_engagement);
            bt.SerialBT->write(state.target_lock);
            bt.SerialBT->write(state.vehicle_speed);
        }

#ifdef BT_SERIAL_STACK_DEBUG
        bt.SerialBT->printf("%s stack HWM %u\n", pcTaskGetTaskName(body_can.rx_task), uxTaskGetStackHighWaterMark(body_can.rx_task));
        bt.SerialBT->printf("%s stack HWM %u\n", pcTaskGetTaskName(body_can.tx_task), uxTaskGetStackHighWaterMark(body_can.tx_task));
        bt.SerialBT->printf("%s stack HWM %u\n", pcTaskGetTaskName(haldex_can.rx_task), uxTaskGetStackHighWaterMark(haldex_can.rx_task));
        bt.SerialBT->printf("%s stack HWM %u\n", pcTaskGetTaskName(haldex_can.tx_task), uxTaskGetStackHighWaterMark(haldex_can.tx_task));
        bt.SerialBT->printf("%s stack HWM %u\n", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
#endif
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\nOpenHaldex32 init");

    state.mode = MODE_STOCK;
    state.ped_threshold = 0;
    state.vehicle_speed = 0;
    state.haldex_state = 0;
    state.haldex_engagement = 0;
    memset(state.lockpoints, 0, ARRAY_SIZE(state.lockpoints));

    body_can.spi_interface = new SPIClass(HSPI);
    haldex_can.spi_interface = new SPIClass(VSPI);

    body_can.tx_q = xQueueCreate(16, sizeof(can_frame));
    haldex_can.tx_q = xQueueCreate(16, sizeof(can_frame));
    bt.bt_q = xQueueCreate(32, sizeof(bt_packet));

    body_can.sem = xSemaphoreCreateMutex();
    xSemaphoreGive(body_can.sem);
    haldex_can.sem = xSemaphoreCreateMutex();
    xSemaphoreGive(haldex_can.sem);

    body_can.spi_interface->setFrequency(10000000);
    haldex_can.spi_interface->setFrequency(10000000);

    body_can.spi_interface->begin();
    haldex_can.spi_interface->begin();

    body_can.can_interface->setSpiInstance(body_can.spi_interface);
    haldex_can.can_interface->setSpiInstance(haldex_can.spi_interface);

    bt.SerialBT = new BluetoothSerial;
    bt.connected = false;
    bt.SerialBT->begin("OpenHaldex");
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
        bt_comms,
        "bt_comms",
        BT_STACK,
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

    xTaskCreate(
        send_test_data,
        "send_test_data",
        CAN_STACK,
        NULL,
        1,
        NULL
    );
}

void loop()
{
    /* Task driven */
}