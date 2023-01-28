/*
 * tm1637.h
 *
 *  Created on: 29 сент. 2019 г.
 *      Author: dima
 */

#ifndef TM1637_H_
#define TM1637_H_

#include "main.h"

#define CLK_HIGH	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8)
#define CLK_LOW		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8)

#define DIO_HIGH	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9)
#define DIO_LOW		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9)

#define DIO_READ	LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_9)

#define ADDR_AUTO  0x40
#define ADDR_FIXED 0x44

#define _A 0x77
#define _B 0x7f
#define _C 0x39
#define _D 0x3f
#define _E 0x79
#define _F 0x71
#define _G 0x3d
#define _H 0x76
#define _J 0x1e
#define _L 0x38
#define _N 0x37
#define _O 0x3f
#define _P 0x73
#define _S 0x6d
#define _U 0x3e
#define _Y 0x6e
#define _a 0x5f
#define _b 0x7c
#define _c 0x58
#define _d 0x5e
#define _e 0x7b
#define _f 0x71
#define _h 0x74
#define _i 0x10
#define _j 0x0e
#define _l 0x06
#define _n 0x54
#define _o 0x5c
#define _q 0x67
#define _r 0x50
#define _t 0x78
#define _u 0x1c
#define _y 0x6e
#define __ 0x08
#define _empty 0x00
#define _0 0x3f
#define _1 0x06
#define _2 0x5b
#define _3 0x4f
#define _4 0x66
#define _5 0x6d
#define _6 0x7d
#define _7 0x07
#define _8 0x7f
#define _9 0x6f


void tm1637_init(int clk, int data);
void writeByte(int8_t wr_data);//write 8bit data to tm1637
void start(void);//send start bits
void stop(void); //send stop bits
void display_mass(int8_t DispData[]);
void display(uint8_t BitAddr,int8_t DispData);
void clearDisplay(void);
void set_brightness(uint8_t brightness); //To take effect the next time it displays.
void point(uint8_t cmd); //whether to light the clock point ":".To take effect the next time it displays.
void coding_mass(int8_t DispData[]);
int8_t coding(int8_t DispData);

#endif /* TM1637_H_ */
