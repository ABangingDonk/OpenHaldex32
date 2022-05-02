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
TaskHandle_t process_task;

can_s body_can = {
    0,                          // CAN status
    NULL,                       // SPI interface
    new MCP_CAN(GPIO_NUM_15),   // CAN interface
    NULL,                       // Comms task
    NULL,                       // Outbox
    NULL,                       // CAN SPI semaphore
    false,                      // Inited flag
    "Body"                      // Name
};

can_s haldex_can = {
    0,                          // CAN status
    NULL,                       // SPI interface
    new MCP_CAN(GPIO_NUM_5),    // CAN interface
    NULL,                       // Comms task
    NULL,                       // Outbox
    NULL,                       // CAN SPI semaphore
    false,                      // Inited flag
    "Haldex"                    // Name
};

bool bt_send_msg_buf(bt_packet *packet)
{
    if (xSemaphoreTake(bt.sem, portMAX_DELAY))
    {
        bt.SerialBT->write(packet->data, packet->len);
        xSemaphoreGive(bt.sem);
        return true;
    }
    return false;
}

bool bt_read_msg_buf(byte terminator, bt_packet *packet)
{
    if (xSemaphoreTake(bt.sem, portMAX_DELAY))
    {
        packet->len = bt.SerialBT->readBytesUntil(terminator, packet->data, ARRAY_SIZE(packet->data));
        xSemaphoreGive(bt.sem);
        return true;
    }
    return false;
}

#ifdef STACK_DEBUG
static void print_stack_hwm(void *params)
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

    body_can.outbox = xQueueCreate(32, sizeof(can_frame*));
    haldex_can.outbox = xQueueCreate(32, sizeof(can_frame*));
    bt.bt_process_q = xQueueCreate(32, sizeof(bt_packet));
    bt.bt_tx_q = xQueueCreate(32, sizeof(bt_packet));


    body_can.spi_sem = xSemaphoreCreateMutex();
    xSemaphoreGive(body_can.spi_sem);

    haldex_can.spi_sem = xSemaphoreCreateMutex();
    xSemaphoreGive(haldex_can.spi_sem);

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
    //body_can.can_interface->enOneShotTX();
    if(body_can.status == CAN_OK)
    {
#if 1
        body_can.can_interface->init_Filt(0, 0, 0x080);
        body_can.can_interface->init_Filt(1, 0, 0x080);
        body_can.can_interface->init_Mask(0, 0, 0x0E0);

        body_can.can_interface->init_Filt(2, 0, 0x0A0);
        body_can.can_interface->init_Filt(3, 0, 0x0A0);
        body_can.can_interface->init_Filt(4, 0, 0x0A0);
        body_can.can_interface->init_Filt(5, 0, 0x0A0);
        body_can.can_interface->init_Mask(1, 0, 0x0E0);
#endif
        body_can.can_interface->setMode(MCP_NORMAL);
        Serial.println("Body CAN: Init OK!");
    }
    else
    {
        Serial.printf("Body CAN: Init Fail!!! (%d)\n", body_can.status);
    }
    
    // init CAN1 bus, baudrate: 500k@16MHz
    haldex_can.status = haldex_can.can_interface->begin(MCP_STD, CAN_500KBPS, MCP_20MHZ);
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
    pinMode(GPIO_NUM_17, INPUT_PULLUP);

    xTaskCreate(
        bt_rx,
        "bt_rx",
        BT_STACK,
        NULL,
        3,
        &bt.bt_rx_task
    );

    xTaskCreate(
        bt_tx,
        "bt_tx",
        BT_STACK,
        NULL,
        3,
        &bt.bt_tx_task
    );

    xTaskCreate(
        bt_process,
        "bt_process",
        2048,
        NULL,
        2,
        &bt.bt_process_task
    );

    xTaskCreate(
        bt_send_status,
        "bt_send_status",
        1024,
        NULL,
        2,
        NULL
    );

    xTaskCreate(
        save_prefs,
        "save_prefs",
        2048,
        NULL,
        2,
        NULL
    );

    if (haldex_can.status == CAN_OK)
    {
        xTaskCreatePinnedToCore(
            haldex_can_comms,
            "haldex_can_comms",
            CAN_STACK,
            NULL,
            4,
            &haldex_can.comms_task,
            1
        );
    }

    if (body_can.status == CAN_OK)
    {
        xTaskCreatePinnedToCore(
            body_can_comms,
            "body_can_comms",
            CAN_STACK,
            NULL,
            4,
            &body_can.comms_task,
            0
        );
    }

    xTaskCreate(
        can_process,
        "can_process",
        CAN_STACK,
        NULL,
        5,
        &process_task
    );

#ifdef CAN_TEST_DATA
    //vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (body_can.status == CAN_OK && haldex_can.status == CAN_OK)
    {
        xTaskCreate(
            send_test_data,
            "send_test_data",
            CAN_STACK,
            NULL,
            6,
            NULL
        );
    }
#endif

    vTaskDelete(NULL);
}

void loop()
{
    /* Task driven */
}