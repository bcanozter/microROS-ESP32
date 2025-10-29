#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

void joystick_task(void *arg);

typedef struct{
    double x;
    double y;
    double z;
} joystick_input_t;

extern joystick_input_t *joystick_controller;
#endif