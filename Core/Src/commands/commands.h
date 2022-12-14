//
// Created by Michał on 30.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_COMMANDS_H
#define DRONE_CONTROLLER_FW_COMMANDS_H

#include "FreeRTOS.h"
#include "queue.h"
#include "usart.h"
#include <stdbool.h>

#define FRAME_BUFF_LEN 64

typedef enum{
    COMMANDS_OK = 00U,
    COMMANDS_TIMEOUT = 01U,
    COMMANDS_ERROR = 02U
}commands_status;

//struct used to control drone in hover mode, which means that it can move only up and down
typedef struct{
    float alt;
    commands_status status;
}command_hover_mode_t;

bool commands_init(QueueHandle_t output_queue);

// callback called in ISR!
bool commands_callback(uint8_t received_byte);

#endif //DRONE_CONTROLLER_FW_COMMANDS_H
