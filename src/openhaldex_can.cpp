#include "openhaldex.h"

static can_frame haldex_framebuf = {0};

static can_frame brakes1_framebuf1 = {0};
static can_frame brakes3_framebuf1 = {0};
static can_frame motor1_framebuf1 = {0};
static can_frame motor2_framebuf1 = {0};
static can_frame motor3_framebuf1 = {0};
static can_frame motor6_framebuf1 = {0};

static can_frame brakes1_framebuf2 = {0};
static can_frame brakes3_framebuf2 = {0};
static can_frame motor1_framebuf2 = {0};
static can_frame motor2_framebuf2 = {0};
static can_frame motor3_framebuf2 = {0};
static can_frame motor6_framebuf2 = {0};

static can_frame *brakes1_processed = &brakes1_framebuf1;
static can_frame *brakes3_processed = &brakes3_framebuf1;
static can_frame *motor1_processed = &motor1_framebuf1; 
static can_frame *motor2_processed = &motor2_framebuf1; 
static can_frame *motor3_processed = &motor3_framebuf1; 
static can_frame *motor6_processed = &motor6_framebuf1; 

static can_frame *brakes1_raw = &brakes1_framebuf2;
static can_frame *brakes3_raw = &brakes3_framebuf2;
static can_frame *motor1_raw = &motor1_framebuf2;
static can_frame *motor2_raw = &motor2_framebuf2;
static can_frame *motor3_raw = &motor3_framebuf2;
static can_frame *motor6_raw = &motor6_framebuf2;

static IRAM_ATTR can_frame* lookup_processed_from_id(uint32_t id)
{
    can_frame* framebuf = NULL;

    switch(id)
    {
        case MOTOR1_ID:
            framebuf = motor1_processed;
            break;
        case MOTOR2_ID:
            framebuf = motor2_processed;
            break;
        case MOTOR3_ID:
            framebuf = motor3_processed;
            break;
        case MOTOR6_ID:
            framebuf = motor6_processed;
            break;
        case BRAKES1_ID:
            framebuf = brakes1_processed;
            break;
        case BRAKES3_ID:
            framebuf = brakes3_processed;
            break;
        case HALDEX_ID:
            framebuf = &haldex_framebuf;
            break;
        default:
            Serial.printf("Unknown CAN ID 0x%X\n", id);
    }

    return framebuf;
}

static IRAM_ATTR can_frame* lookup_raw_from_id(uint32_t id)
{
    can_frame* framebuf = NULL;

    switch(id)
    {
        case MOTOR1_ID:
            framebuf = motor1_raw;
            break;
        case MOTOR2_ID:
            framebuf = motor2_raw;
            break;
        case MOTOR3_ID:
            framebuf = motor3_raw;
            break;
        case MOTOR6_ID:
            framebuf = motor6_raw;
            break;
        case BRAKES1_ID:
            framebuf = brakes1_raw;
            break;
        case BRAKES3_ID:
            framebuf = brakes3_raw;
            break;
        default:
            Serial.printf("Unknown CAN ID 0x%X\n", id);
    }

    return framebuf;
}

#ifdef CAN_TEST_DATA
void send_test_data(void *params)
{
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        uint32_t frame_id = 0;

        motor1_processed->id = frame_id = MOTOR1_ID;
        motor1_processed->len = 8;
        motor1_processed->data.bytes[0] = 0;
        motor1_processed->data.bytes[1] = 0xf0;
        motor1_processed->data.bytes[2] = 0x20;
        motor1_processed->data.bytes[3] = 0x4e;
        motor1_processed->data.bytes[4] = 0xf0;
        motor1_processed->data.bytes[5] = 0xf0;
        motor1_processed->data.bytes[6] = 0x20;
        motor1_processed->data.bytes[7] = 0xf0;
        xQueueSendToBack(haldex_can.outbox, &frame_id, portMAX_DELAY);

        haldex_framebuf.id = frame_id = HALDEX_ID;
        haldex_framebuf.len = 2;
        haldex_framebuf.data.bytes[0] = 0x00;
        haldex_framebuf.data.bytes[1] = 0x7f;
        xQueueSendToBack(body_can.outbox, &frame_id, portMAX_DELAY);
        
        vTaskDelete(NULL);
    }
}
#endif

void can_process(void *params)
{
    while (1)
    {
        long start_time = micros();
        can_frame frame = {0};
        can_frame *framebuf = NULL;
        
        // Copy from raw to processed
        memcpy(brakes1_processed, brakes1_raw, sizeof(can_frame));
        memcpy(brakes3_processed, brakes3_raw, sizeof(can_frame));
        memcpy(motor1_processed, motor1_raw, sizeof(can_frame));
        memcpy(motor2_processed, motor2_raw, sizeof(can_frame));
        memcpy(motor3_processed, motor3_raw, sizeof(can_frame));
        memcpy(motor6_processed, motor6_raw, sizeof(can_frame));

        // First, extract all the things we need from CAN frames
        haldex_state = haldex_framebuf.data.bytes[0];
        haldex_engagement = haldex_framebuf.data.bytes[1] == SERIAL_PACKET_END ? 0xfe
                                                                               : haldex_framebuf.data.bytes[1];
        ped_value = motor1_processed->data.bytes[5] * 0.4;
        int calc_speed = (motor2_processed->data.bytes[3] * 100 * 128) / 10000;
        vehicle_speed = (byte)(calc_speed > 254 ? 254 : calc_speed);
#ifdef CAN_TEST_DATA
        motor2_raw->data.bytes[3] += 1;
#endif
        
        // Then, apply modifications
        lock_target = get_lock_target_adjustment();
        if (state.mode == MODE_FWD)
        {
            memset(&motor1_processed->data, 0, motor1_processed->len);
        }
        else if (state.mode == MODE_5050 || state.mode == MODE_CUSTOM)
        {
            motor1_processed->data.bytes[0] = 0;
            motor1_processed->data.bytes[1] = get_lock_target_adjusted_value(0xf0);
            motor1_processed->data.bytes[2] = 0x20;
            motor1_processed->data.bytes[3] = get_lock_target_adjusted_value(0x4e);
            motor1_processed->data.bytes[4] = get_lock_target_adjusted_value(0xf0);
            motor1_processed->data.bytes[5] = get_lock_target_adjusted_value(0xf0);
            motor1_processed->data.bytes[6] = 0x20;
            motor1_processed->data.bytes[7] = get_lock_target_adjusted_value(0xf0);

            motor3_processed->data.bytes[2] = get_lock_target_adjusted_value(0xfa);
            motor3_processed->data.bytes[7] = get_lock_target_adjusted_value(0xfe);

            motor6_processed->data.bytes[1] = get_lock_target_adjusted_value(0xfe);
            motor6_processed->data.bytes[2] = get_lock_target_adjusted_value(0xfe);

            uint8_t adjusted_slip = get_lock_target_adjusted_value(0xff);
            brakes3_processed->data.high = (0xa << 24) + (0xa << 8);
            brakes3_processed->data.low = brakes3_processed->data.high + (adjusted_slip << 16) + adjusted_slip;

            //brakes3_processed->data.bytes[1] &= ~0x8;
            brakes3_processed->data.bytes[2] = 0x0;
            brakes3_processed->data.bytes[3] = 0xa;
        }
        
        Serial.printf("can_process() took %uus\n", micros() - start_time);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void body_can_comms(void *params)
{
    while (1)
    {
        can_frame frame = {0};
        can_frame *framebuf = NULL;
        byte result;

        //xSemaphoreTake(body_can.spi_sem, portMAX_DELAY);
        result = body_can.can_interface->readMsgBuf(&frame.id, &frame.len, frame.data.bytes);
        if (result == CAN_OK)
        {
            // We've received a CAN message..
#ifdef CAN_DEBUG
            Serial.printf("Body CAN RX 0x%03x: [%d]\n", frame.id, frame.len);
#endif
            // Get a pointer to the buffer for this message
            framebuf = lookup_raw_from_id(frame.id);
            // Copy the new message into the buffer
            memcpy(framebuf, &frame, sizeof(frame));
            // Queue sending of this message on the Haldex interface
            xQueueSendToBack(haldex_can.outbox, &frame.id, portMAX_DELAY);
        }

        // Now check if we have any messages to send over the Body interface
        uint32_t out_id = 0;
        if (xQueueReceive(body_can.outbox, &out_id, 0) == pdTRUE)
        {
            // We had one.. so try send it
            framebuf = lookup_processed_from_id(out_id);
            result = body_can.can_interface->sendMsgBuf(framebuf->id, framebuf->len, framebuf->data.bytes);

            // Sending failed, return it to the front of the queue
            if (result != CAN_OK)
            {
                Serial.printf("[%x]: body TX busy (%d) - requeue\n", out_id, result);
                xQueueSend(body_can.outbox, &out_id, portMAX_DELAY);
            }
        }
        //xSemaphoreGive(body_can.spi_sem);

        vTaskDelay(CAN_COMMS_RELIEF_MS / portTICK_PERIOD_MS);
    }
}

void haldex_can_comms(void *params)
{
    while (1)
    {
        can_frame *framebuf = NULL;
        byte result;

        //xSemaphoreTake(haldex_can.spi_sem, portMAX_DELAY);
        result = haldex_can.can_interface->readMsgBuf(&haldex_framebuf.id, &haldex_framebuf.len, haldex_framebuf.data.bytes);
        if (result == CAN_OK)
        {
            // We've received a CAN message and copied it directly into the buffer.
#ifdef CAN_DEBUG
            Serial.printf("Haldex CAN RX 0x%03x: [%d]\n", haldex_framebuf.id, haldex_framebuf.len);
#endif
            // Queue sending of this message on the Body interface
            xQueueSendToBack(body_can.outbox, &haldex_framebuf.id, portMAX_DELAY);
        }

        // Now check if we have any message to send over the Haldex interface
        uint32_t out_id = 0;
        if (xQueueReceive(haldex_can.outbox, &out_id, 0) == pdTRUE)
        {
            // We had one.. so try send it
            framebuf = lookup_processed_from_id(out_id);
            result = haldex_can.can_interface->sendMsgBuf(framebuf->id, framebuf->len, framebuf->data.bytes);

            // Sending failed, return it to the front of the queue
            if (result != CAN_OK)
            {
                Serial.printf("[%x]: haldex TX busy (%d) - requeue\n", out_id, result);
                xQueueSend(haldex_can.outbox, &out_id, portMAX_DELAY);
            }
        }
        //xSemaphoreGive(haldex_can.spi_sem);
        
        vTaskDelay(CAN_COMMS_RELIEF_MS / portTICK_PERIOD_MS);
    }
}
