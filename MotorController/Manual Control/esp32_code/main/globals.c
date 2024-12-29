
#include "globals.h"

const char *TAG_STATE = "STATE";
const char *TAG_DIRECTION = "DIRECTION";

BotState state = STATE_STARTUP;

// Global Variables
// Default Robot Speeds
int leftPWM = 1500;
int rightPWM = 1500;

int latEX;
int longEX;