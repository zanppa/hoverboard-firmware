#include "config.h"

#if defined(DATALOGGER_ENABLE)

volatile DATALOGGER_TYPE datalogger[DATALOGGER_MAX][4] = {0};
volatile DATALOGGER_COUNT_TYPE datalogger_write_offset = DATALOGGER_MAX;
volatile uint8_t datalogger_trigger = 0;
void *datalogger_var0 = NULL;
void *datalogger_var1 = NULL;
void *datalogger_var2 = NULL;
void *datalogger_var3 = NULL;
volatile uint8_t datalogger_period = DATALOGGER_DIVIDER;

#endif
