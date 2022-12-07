//
// Created by Michał on 30.10.2022.
//

#include "commands.h"
#include "controller.h"
#include "ring_buffer.h"

#define IBUS_CH_NUM 14
#define COMMAND_CODE 0x40
#define IBUS_FRAME_LEN (IBUS_CH_NUM*2 + 4)
#define ALT_SCALING 1.0f

static QueueHandle_t output_queue_local;
static QueueHandle_t input_queue;
static command_hover_mode_t command;

typedef struct __attribute__((packed)) {
    uint8_t length;
    uint8_t command_code;
    uint16_t payload[IBUS_CH_NUM];
    uint16_t checksum;
} iBus_frame_structure_t;

union ibus_rec{
    uint8_t bytes[IBUS_FRAME_LEN];
    iBus_frame_structure_t fields;
};

typedef enum {
    LEN,
    CODE,
    PAYLOAD,
    CHECKSUM
}ibus_rx_state_t;

void commands_task(void* params);
bool check_checksum(union ibus_rec* frame);

bool commands_init(QueueHandle_t output_queue)
{
    BaseType_t returned;
    output_queue_local = output_queue;
    input_queue = xQueueCreate(100, sizeof(union ibus_rec));
    returned = xTaskCreate(commands_task,
                           "commands_task",
                           128,
                           NULL,
                           3,
                           NULL);
    return returned == pdPASS;
}

union ibus_rec received_frame;
void commands_task(void* params)
{
    uint8_t data_buffer[FRAME_BUFF_LEN];
    uint8_t index = 0;
    ring_buffer frame_buffer;
    BaseType_t ret;

    ring_buffer_init(&frame_buffer, data_buffer, FRAME_BUFF_LEN, sizeof(uint8_t));

    command_hover_mode_t command;
    BaseType_t rslt;
    while(1)
    {
        // pull received frame
        rslt = xQueueReceive(input_queue, &received_frame, 200);
        if(rslt == pdTRUE)
        {
            if (check_checksum(&received_frame)) {
                command.alt = (float) (received_frame.fields.payload[2] - 1000) / 2.0f;
                command.timeout = false;
                xQueueSendToFront(output_queue_local, &command, 100);
            }
        }else{
            // comm timeout handling
            command.timeout = true;  // controller decides what to do with comm timeout
            xQueueSendToFront(output_queue_local, &command, 100);
        }
    }
}

// callback called in ISR!
ibus_rx_state_t rx_state = LEN;
union ibus_rec rx_buf;
uint8_t payload_cnt = 0;
bool checksum_lsb = true;
bool commands_callback(uint8_t received_byte)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    switch(rx_state)
    {
        case LEN:
            if(received_byte == IBUS_FRAME_LEN)
            {
                rx_buf.bytes[0] = received_byte;
                rx_state = CODE;
            }else{
                rx_state = LEN;
            }
            break;
        case CODE:
            if(received_byte == COMMAND_CODE)
            {
                rx_buf.bytes[1] = received_byte;
                rx_state = PAYLOAD;
                payload_cnt = 0;
            }else{
                rx_state = LEN;
            }
            break;
        case PAYLOAD:
            if(payload_cnt <= IBUS_CH_NUM*2-2)
            {
                rx_buf.bytes[payload_cnt+2] = received_byte;
                ++payload_cnt;
                rx_state = PAYLOAD;
            }else{
                rx_buf.bytes[payload_cnt+2] = received_byte;
                checksum_lsb = true;
                rx_state = CHECKSUM;
            }
            break;
        case CHECKSUM:
            if(checksum_lsb)
            {
                rx_buf.bytes[IBUS_FRAME_LEN-2] = received_byte;
                checksum_lsb = false;
                rx_state = CHECKSUM;
            }else{
                rx_buf.bytes[IBUS_FRAME_LEN-1] = received_byte;
                xQueueSendToFrontFromISR(input_queue, &rx_buf.fields,
                                         &higher_priority_task_woken);
                rx_state = LEN;
            }
            break;
        default:
            rx_state = LEN;
            break;
    }
    return higher_priority_task_woken == pdTRUE;
}

bool check_checksum(union ibus_rec* frame)
{
    uint16_t checksum_calculated = 0xffff;
    for(uint8_t i = 0; i < IBUS_FRAME_LEN-2; i++)
    {
        checksum_calculated -= frame->bytes[i];
    }
    return (checksum_calculated == frame->fields.checksum);
}