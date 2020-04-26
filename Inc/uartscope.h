#ifndef __UARTSCOPE_H__
#define __UARTSCOPE_H__


#define SCOPE_CHANNELS 8
#define SCOPE_DATATYPE uint16_t


void scope_set_data(uint8_t ch, uint16_t data);
uint8_t scope_send();


#endif // __UARTSCOPE_H__
