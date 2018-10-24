#ifndef __X10_library_H
#define __X10_library_H

#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "cmsis_os.h"

#define SIZE_TRAM 68
#define A1_8_ADDR 0x60
#define A1_ON 0x00
#define A1_OFF 0x20
#define A2_ON 0x10
#define A2_OFF 0x30

void InitX10(void);
void InitTimer(void);
void Build_x10_tick_array(uint8_t adrress,uint8_t data,unsigned char array_out[]);
void Send_single_x10_frame(uint8_t addres_x10,uint8_t data_x10);
void Send_complete_x10_frame(uint8_t addres_x10,uint8_t data_x10);

#endif
