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
        QUEUE_SEND(bt.bt_tx_q, &packet);

        //haldex_state = 0;

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void bt_process(void *params)
{
    while (1)
    {
        bt_packet packet;
        if (xQueueReceive(bt.bt_process_q, &packet, portMAX_DELAY))
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

                        if (lockpoint_index > 6)
                        {
                            state.custom_mode.lockpoint_rx_h |= (1 << (lockpoint_index - 7));
                        }
                        else
                        {
                            state.custom_mode.lockpoint_rx_l |= (1 << lockpoint_index);
                        }
                        state.custom_mode.lockpoint_count++;
#ifdef STATE_DEBUG
                        Serial.printf("lockpoint[%d] low 0x%x high 0x%x (count %d)\n",
                                      lockpoint_index,
                                      state.custom_mode.lockpoint_rx_l,
                                      state.custom_mode.lockpoint_rx_h,
                                      state.custom_mode.lockpoint_count);
#endif
                    }
                    break;
                case APP_MSG_CUSTOM_CTRL:
                    switch (packet.data[1])
                    {
                        case DATA_CTRL_CHECK_LOCKPOINTS:
                            tx_packet.data[0] = APP_MSG_CUSTOM_CTRL;
                            tx_packet.data[1] = DATA_CTRL_CHECK_LOCKPOINTS;
                            tx_packet.data[2] = state.custom_mode.lockpoint_rx_l;
                            tx_packet.data[3] = state.custom_mode.lockpoint_rx_h;
                            tx_packet.data[4] = SERIAL_PACKET_END;
                            tx_packet.len = 5;

                            QUEUE_SEND(bt.bt_tx_q, &tx_packet);
                            break;
                        case DATA_CTRL_CLEAR:
                            state.custom_mode.lockpoint_rx_l = 0;
                            state.custom_mode.lockpoint_rx_h = 0;
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

                            QUEUE_SEND(bt.bt_tx_q, &tx_packet);
                            break;
                    }
                    break;
            }
        }
    }
}

void bt_tx(void *params)
{
    while (1)
    {
        bt_packet packet;
        xQueueReceive(bt.bt_tx_q, &packet, portMAX_DELAY);

        if (bt.SerialBT->connected())
        {
            bt_send_msg_buf(&packet);
#ifdef BT_SERIAL_DEBUG_TX
            Serial.write(packet.data, packet.len);
#endif
        }
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
                QUEUE_SEND(bt.bt_process_q, &packet);
            }

        }
        vTaskDelay(125 / portTICK_PERIOD_MS);
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
        int q_result = QUEUE_SEND(body_can.tx_q, &body_frame);
        if (q_result != pdTRUE)
        {
            Serial.printf("Body CAN cannot forward to haldex (%d)\n", q_result);
        }

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
        q_result = QUEUE_SEND(haldex_can.tx_q, &haldex_frame);
        if (q_result != pdTRUE)
        {
            Serial.printf("Haldex CAN cannot forward to body (%d)\n", q_result);
        }

        vTaskDelete(NULL);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#endif

void body_can_rx(void *params)
{
    attachInterrupt(GPIO_NUM_16, body_can_isr, FALLING);
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#ifdef CAN_DEBUG
        Serial.println("body_can_rx() unblocked");
#endif
        can_frame frame = {0};
        byte result = can_read_msg_buf(&body_can, &frame);
        if (result == CAN_OK && frame.id != 0)
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
#ifdef CAN_TEST_DATA
                    vehicle_speed += 2;
#endif
                    break;
                case MOTOR2_ID:
                    vehicle_speed = (byte)((frame.data.bytes[3] * 100 * 128) / 10000);
                    break;
            }
            
            if (state.mode == MODE_5050 || state.mode == MODE_CUSTOM)
            {
                get_lock_data(&frame);
            }

            int q_result = QUEUE_SEND(haldex_can.tx_q, &frame);
            if (q_result != pdTRUE)
            {
                Serial.printf("Body CAN cannot forward to haldex (%d)\n", q_result);
            }
#ifdef CAN_DEBUG
            else
            {
                Serial.printf("Haldex CAN TX queue %d\n", uxQueueMessagesWaiting(haldex_can.tx_q));
            }
#endif
        }
        
        if (result != 0)
        {
            Serial.printf("Body RX cannot read (%d)\n", result);
        }
    }
}

void body_can_tx(void *params)
{
    while (1)
    {
        can_frame frame = {0};
        if (xQueueReceive(body_can.tx_q, &frame, portMAX_DELAY))
        {
            byte result = can_send_msg_buf(&body_can, &frame);
            if (result)
            {
                Serial.printf("Body TX busy (%d)\n", result);
            }
        }
        //vTaskDelay(CAN_TX_RELIEF_MS / portTICK_PERIOD_MS);
    }
}

void haldex_can_rx(void *params)
{
    attachInterrupt(GPIO_NUM_17, haldex_can_isr, FALLING);
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#ifdef CAN_DEBUG
        Serial.println("haldex_can_rx() unblocked");
#endif
        can_frame frame = {0};
        byte result = can_read_msg_buf(&haldex_can, &frame);
        if (result == CAN_OK && frame.id != 0)
        {
#ifdef CAN_DEBUG
            Serial.printf("Haldex CAN RX 0x%03x: [%d]\n", frame.id, frame.len);
#endif
            int q_result = QUEUE_SEND(body_can.tx_q, &frame);
            if (q_result != pdTRUE)
            {
                Serial.printf("Haldex CAN cannot forward to body (%d)\n", q_result);
            }
#ifdef CAN_DEBUG
            else
            {
                Serial.printf("Body CAN TX queue %d\n", uxQueueMessagesWaiting(body_can.tx_q));
            }
#endif
            haldex_state = frame.data.bytes[0];
            haldex_engagement = frame.data.bytes[1] == SERIAL_PACKET_END ? 0xfe
                                                                         : frame.data.bytes[1];
        }
        
        if (result != 0)
        {
            Serial.printf("Haldex RX cannot read (%d)\n", result);
        }
    }
}

void haldex_can_tx(void *params)
{
    while (1)
    {
        can_frame frame = {0};
        if (xQueueReceive(haldex_can.tx_q, &frame, portMAX_DELAY))
        {
            byte result = can_send_msg_buf(&haldex_can, &frame);
            if (result)
            {
                Serial.printf("Haldex TX busy (%d)\n", result);
            }
        }
        //vTaskDelay(CAN_TX_RELIEF_MS / portTICK_PERIOD_MS);
    }
}