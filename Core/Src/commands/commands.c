//
// Created by Michał on 30.10.2022.
//

#include "commands.h"
#include "controller.h"
#include "ring_buffer.h"

#define IBUS_CH_NUM 14
#define COMMAND_CODE 0x40
#define IBUS_FRAME_LEN (IBUS_CH_NUM*2 + 6)
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

void commands_task(void* params);
void check_frame_and_exe(iBus_frame_structure_t *frame);
bool check_validness(iBus_frame_structure_t* frame);
bool check_checksum(iBus_frame_structure_t* frame);

static uint8_t received_buffer[IBUS_FRAME_LEN];
static uint8_t received_count = 0;
static bool opening_byte_received = false;

bool commands_init(QueueHandle_t output_queue)
{
    BaseType_t returned;
    output_queue_local = output_queue;
    input_queue = xQueueCreate(100, sizeof(iBus_frame_structure_t));
    returned = xTaskCreate(commands_task,
                           "commands_task",
                           128,
                           NULL,
                           3,
                           NULL);
    return returned == pdPASS;
}

void commands_task(void* params)
{
    uint8_t data_buffer[FRAME_BUFF_LEN];
    uint8_t index = 0;
    ring_buffer frame_buffer;
    BaseType_t ret;

    ring_buffer_init(&frame_buffer, data_buffer, FRAME_BUFF_LEN, sizeof(uint8_t));

    while(1)
    {
        // pull received frame
        xQueueReceive(input_queue, &data_buffer[index], 0);

    }
}

// callback called in ISR!
bool commands_callback(uint8_t received_byte)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    if(!opening_byte_received && received_byte == IBUS_FRAME_LEN)
    {
        received_buffer[0] = received_byte;
        opening_byte_received = true;
        received_count = 1;
    }else if(opening_byte_received) {
        received_buffer[received_count] = received_byte;
        received_count++;
        if (received_count == IBUS_FRAME_LEN) {
            opening_byte_received = false;
            xQueueSendToFrontFromISR(input_queue, (iBus_frame_structure_t *) received_buffer,
                                     &higher_priority_task_woken);
        }
    }
    return higher_priority_task_woken == pdTRUE;
}

void check_frame_and_exe(iBus_frame_structure_t *frame)
{
    if(check_validness(frame))
    {
        command_hover_mode_t command;
        command.alt = (float) frame->payload[0] * ALT_SCALING;
        xQueueSendToFront(output_queue_local, &command, 100);
    }
}

bool check_validness(iBus_frame_structure_t* frame)
{
    return (frame->command_code == COMMAND_CODE) && check_checksum(frame);
}

bool check_checksum(iBus_frame_structure_t* frame)
{
    uint16_t checksum_calculated = 0xffff;

    checksum_calculated -= frame->length;
    checksum_calculated -= frame->command_code;
    for(int i = 0; i < IBUS_CH_NUM; i++)
    {
        checksum_calculated -= frame->payload[i] & 0x00ff;  // subtract lower byte
        checksum_calculated -= frame->payload[i] >> 8;  // subtract higher byte
    }

    //TODO check byte ordering, it might be incorrect
    return (checksum_calculated == frame->checksum);
}