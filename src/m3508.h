#ifndef __M3508_H__
#define __M3508_H__

#include <Arduino.h>
#include <CAN.h>

bool can_init(int rx, int tx, int speed);
void m3508_make_data(int16_t data_in[4], uint8_t data_out1[8]);
bool m3508_send_data(uint8_t data_out1[8]);
void m3508_read_data(int id, int16_t mangle[4], int16_t mrpm[4], int16_t mtorque[4]);

#endif // __M3508_H__