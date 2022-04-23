#include "openhaldex.h"

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
                Serial.println("Updated preferences");
            }
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
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