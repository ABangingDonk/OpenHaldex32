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
    NULL,                       // RX semaphore
    NULL,                       // TX semaphore
    "Body"                      // Name
};

can_s haldex_can = {
    0,                          // CAN status
    NULL,                       // SPI interface
    new MCP_CAN(GPIO_NUM_5),    // CAN interface
    NULL,                       // RX task
    NULL,                       // TX task
    NULL,                       // TX queue
    NULL,                       // RX semaphore
    NULL,                       // TX semaphore
    "Haldex"                    // Name
};

bool can_send_msg_buf(can_s *can, can_frame *frame)
{
    if (xSemaphoreTake(can->tx_sem, 100 / portTICK_PERIOD_MS))
    {
        can->can_interface->sendMsgBuf(frame->id, frame->len, frame->data.bytes);
        xSemaphoreGive(can->tx_sem);
        return true;
    }
    return false;
}

bool can_read_msg_buf(can_s *can, can_frame *frame)
{
    if (xSemaphoreTake(can->tx_sem, 100 / portTICK_PERIOD_MS))
    {
        can->can_interface->readMsgBuf(&frame->id, &frame->len, frame->data.bytes);
        xSemaphoreGive(can->tx_sem);
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

IRAM_ATTR
void body_can_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(body_can.rx_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

IRAM_ATTR
void haldex_can_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(haldex_can.rx_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
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


    body_can.rx_sem = xSemaphoreCreateBinary();
    body_can.tx_sem = xSemaphoreCreateMutex();
    xSemaphoreGive(body_can.tx_sem);

    haldex_can.rx_sem = xSemaphoreCreateBinary();
    haldex_can.tx_sem = xSemaphoreCreateMutex();
    xSemaphoreGive(haldex_can.tx_sem);

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
    body_can.status = body_can.can_interface->begin(MCP_STD, CAN_500KBPS, MCP_8MHZ);
    //body_can.can_interface->enOneShotTX();
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
    haldex_can.status = haldex_can.can_interface->begin(MCP_STD, CAN_500KBPS, MCP_8MHZ);
    //haldex_can.can_interface->enOneShotTX();
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