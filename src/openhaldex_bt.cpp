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

        vTaskDelay(100 / portTICK_PERIOD_MS);
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
                    if (state.mode == MODE_STOCK)
                    {
                        vTaskSuspend(process_task);
                    }
                    else
                    {
                        if (eTaskGetState(process_task) == eSuspended)
                        {
                            vTaskResume(process_task);
                        }
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
        vTaskDelay(100 / portTICK_PERIOD_MS);
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

        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }
}