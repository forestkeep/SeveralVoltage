#ifndef button_H
#define button_H

#include "main.h"
#define Buttondelay 2000//задержка для подавления дребезга
#define ButtonClick 3000 //граница короткого нажатия
#define ButtonClickLong 60000//граница длинного нажатия
#define S00 LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
#define S10 LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
#define S20 LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
#define S01 LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
#define S11 LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
#define S21 LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);

//S0|S1|S2								000		100		010		001		011		111		101	 110
//but№                     1     2     3     4     5     6     7    8
//uint8_t ButtonState[8] ={  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  , 0 };//состояние кнопок после опроса. если 1 - то было короткое нажатие, если 2, то было длинное нажатие
//uint8_t alarm_Button = 0;//флаг для сигнализирования о том, что с кнопками взаимодействовали и надо бы проверить их состояние.

uint8_t ButtonRead(uint8_t* alarm_button, uint8_t* ButtonState);
//ButtonState - указатель на массив из 8 элементов, функция туда запишет состояния кнопок не нажата(0) короткое(1) длинное(2)
//allarm_button - указатель на переменную флаг, флаг показывает, что была вызвана функция ButtonRead и состояния кнопок были обновлены

#endif
