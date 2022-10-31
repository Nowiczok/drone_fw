//
// Created by Michał on 30.10.2022.
//

#ifndef DRONE_CONTROLLER_FW_COMMANDS_H
#define DRONE_CONTROLLER_FW_COMMANDS_H

//struct used to control drone in hover mode, which means that it can move only up and down
typedef struct{
    float speed_z;
}command_hover_mode_t;

#endif //DRONE_CONTROLLER_FW_COMMANDS_H
