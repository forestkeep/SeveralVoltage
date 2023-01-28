/*
 * tm1637.c
 *
 *  Created on: 29 сент. 2019 г.
 *      Author: dima
 */

#include "delay_micros.h"
#include "tm1637.h"

uint8_t Cmd_DispCtrl = 0;
uint8_t point_flag = 0;
//                          0  1  2  3  4  5  6  7  8  9  10 11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  
static int8_t TubeTab[] = {_0,_1,_2,_3,_4,_5,_6,_7,_8,_9,_A ,_B, _C ,_D, _E, _F, _G, _H, _J, _L, _N, _O, _P, _S, _U, _Y, _a, _b, _c,
//      29  30  31  32  33  34  35  36  37  38  39  40  41  42  43      44(.) 45(=) 46(-) 47(*)     48     49
        _d, _e, _f, _h, _i, _j, _l, _n, _o, _q, __, _t, _u, _y, _empty, 0x80, 0x48, 0x40, 0x63,_empty,_empty,
//       50(0.)    51(1.)    52(2.)    53(3.)    54(4.)    55(5.)    56(6.)    57(7.)    58(8.)    59(9.)     
          0xbf,     0x86,     0xdb,     0xCF  ,   0xe6,     0xed,     0xfd,     0x87,    0xff ,    0xef };

 
void writeByte(int8_t wr_data)
{
  uint8_t count = 0;

  for(uint8_t i = 0; i < 8; i++)
  {
    CLK_LOW;
    if(wr_data & 0x01) DIO_HIGH;
    else DIO_LOW;

    delay_micros(6);
    wr_data >>= 1;
    delay_micros(6);
    CLK_HIGH;
    delay_micros(8);
  }

  CLK_LOW;
  delay_micros(6);
  DIO_HIGH;
  delay_micros(6);
  CLK_HIGH;
  delay_micros(8);

  while(DIO_READ)
  {
    count += 1;

    if(count == 200)
    {
    	DIO_LOW;
    	count = 0;
    }
  }
}

void start(void)
{
	CLK_HIGH;
	delay_micros(6);
	DIO_HIGH;
	delay_micros(6);
	DIO_LOW;
	delay_micros(6);
	CLK_LOW;
}

void stop(void)
{
	CLK_LOW;
	delay_micros(6);
	DIO_LOW;
	delay_micros(6);
	CLK_HIGH;
	delay_micros(6);
	DIO_HIGH;
}

void display_mass(int8_t DispData[])
{
	int8_t SegData[4];

	for(uint8_t i = 0; i < 4; i++)
	{
		SegData[i] = DispData[i];
	}

	coding_mass(SegData);
	start();
	writeByte(ADDR_AUTO);
	stop();
	start();
	writeByte(0xc0);

	for(uint8_t i = 0; i < 4; i++)
	{
		writeByte(SegData[i]);
	}

	stop();
	start();
	writeByte(Cmd_DispCtrl);
	stop();
}

void display(uint8_t BitAddr, int8_t DispData)
{
	int8_t SegData;

	SegData = coding(DispData);
	start();
	writeByte(ADDR_FIXED);

	stop();
	start();
	writeByte(BitAddr | 0xc0);

	writeByte(SegData);
	stop();
	start();

	writeByte(Cmd_DispCtrl);
	stop();
}

void clearDisplay(void)
{
	display(0x00, 0x7f);
	display(0x01, 0x7f);
	display(0x02, 0x7f);
	display(0x03, 0x7f);
}

void set_brightness(uint8_t brightness)
{
	Cmd_DispCtrl = 0x88 + brightness;
}

void point(uint8_t cmd)
{
	if(cmd == 0) point_flag = (~point_flag) & 0x01;
	else point_flag = 1;
}

void coding_mass(int8_t DispData[])
{
	uint8_t PointData;

	if(point_flag == 1) PointData = 0x80;
	else PointData = 0;

	for(uint8_t i = 0; i < 4; i++)
	{
		if(DispData[i] == 0x7f) DispData[i] = 0x00;
		else DispData[i] = TubeTab[DispData[i]] + PointData;
	}
}

int8_t coding(int8_t DispData)
{
	uint8_t PointData;

	if(point_flag == 1) PointData = 0x80;
	else PointData = 0;

	if(DispData == 0x7f) DispData = 0x00 + PointData;
	else DispData = TubeTab[DispData] + PointData;

	return DispData;
}
