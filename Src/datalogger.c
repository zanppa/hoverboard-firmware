#include "config.h"

#if defined(DATALOGGER_ENABLE)

volatile DATALOGGER_TYPE datalogger[DATALOGGER_MAX+1][DATALOGGER_CHANNELS] = {0};
volatile DATALOGGER_COUNT_TYPE datalogger_write_offset = DATALOGGER_MAX;
volatile uint8_t datalogger_trigger = 0;
volatile uint8_t datalogger_full = 0;
void *datalogger_var[DATALOGGER_CHANNELS] = {NULL};
volatile uint8_t datalogger_period = DATALOGGER_DIVIDER;

#endif
