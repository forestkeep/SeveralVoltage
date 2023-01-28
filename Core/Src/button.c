#include "button.h"
uint32_t Buttonwait=0;//Пауза для подавления дребезга
uint8_t ButtonFlag;//Флаг состояния кнопок, если все кнопки определили свое состояние, флаг опущен(=0)
uint32_t btn_cnt[8];//Счетчик кнопок
static uint8_t b;
static uint8_t i;
uint8_t button_short_flag;//флаг для однократкного звучания
uint8_t button_long_flag;
void S_state(int number)//функция предназначена для выставления бинарного кода на мультиплексоре S0 S1 S2
{
	switch (number)
  {
  	case 0:
			S00;S10;S20;
  		break;
  	case 1:
			S01;S10;S20;
  		break;
		case 2:
			S00;S11;S20;
  		break;
  	case 3:
			S00;S10;S21;
  		break;
		case 4:
			S00;S11;S21;
  		break;
  	case 5:
			S01;S11;S21;
  		break;
		case 6:
			S01;S10;S21;
  		break;
  	case 7:
			S01;S11;S20;
  		break;
  	default:
  		break;
  }
	for(b=0;b<10;b++)//цикл необходим для задания небольшой задержи, чтобы мультиплексор успел поменять канал
	{}
}


uint8_t ButtonRead(uint8_t* alarm_button, uint8_t* ButtonState)
{
	if(!(*alarm_button)){
	button_short_flag = 1;
	ButtonState[0]=0; ButtonState[1]=0; ButtonState[2]=0; ButtonState[3]=0; ButtonState[4]=0; ButtonState[5]=0; ButtonState[6]=0; ButtonState[7]=0;
	while(Buttonwait<Buttondelay) //задержка для подавления дребезга кнопок, подбирается экспериментально
		{
			for(i=0;i<8;i++)
      {
				S_state(i);//выставляем порт для считывания кнопки
				if(!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_11)) btn_cnt[i]++;//если кнопка нажата(нулевой уровень на порте), то счетчик этой кнопки увеличивается на единицу, в противном случае он обнуляется.
				else btn_cnt[i]=0;
      }
			Buttonwait++;
		}
	Buttonwait=0;
	ButtonFlag=1;

	while (ButtonFlag)
  {
		ButtonFlag=0;
		for(i=0;i<8;i++)
		{
			S_state(i);
			if(btn_cnt[i]>0)
			{
				if(btn_cnt[i]>ButtonClick)
				{
					if(btn_cnt[i]>ButtonClickLong)
					{
						ButtonState[i]=2;
						Long_sound();
						*alarm_button=1;
						btn_cnt[i]=0;
						continue;
					}
					else
					{
						ButtonState[i]=1;
						if(!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_11))
						{
							ButtonFlag=1;
							btn_cnt[i]++;
						}
						else{
							btn_cnt[i]=0;
							if(button_short_flag){
							*alarm_button=1;
							Short_sound();
							button_short_flag = 0;}
							continue;
						}
					}
				}
				else
				{
					if(!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_11))
						{
							ButtonFlag=1;
							btn_cnt[i]++;
						}
					else{
						btn_cnt[i]=0;
						continue;
					}
				}
			}
		}
  }
	
	//*alarm_button=1;
	}
	return 0;
}
