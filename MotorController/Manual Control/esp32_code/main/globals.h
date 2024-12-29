
#ifndef GLOBALS_H
#define GLOBALS_H

typedef enum {
    STATE_STARTUP,
    STATE_MOV,
    STATE_MOVGPS,
    STATE_MOVPWM,
} BotState;

extern const char *TAG_STATE;
extern const char *TAG_DIRECTION;

extern BotState state;

// Global Variables
extern int leftPWM;
extern int rightPWM;

extern int *leftptr;
extern int *rightptr;

extern int latEX;
extern int longEX;

#endif 