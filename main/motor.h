// motor.h
#pragma once

#include <stdint.h>

#define DIR_CW  1
#define DIR_CCW 0

void stepper_init(void);
void stepper_move(float rpm, int direction);
