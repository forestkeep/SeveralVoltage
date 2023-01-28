/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
	//--------------------------------------------------------------------------------

	
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//------------------------------------------------------NOT USED IN SEVERAL--------------------------
#define CS_set LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
#define CS_Reset LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint8_t flagtemp = 1;
//====================================USART================================================
char logs[10]="";
char logs1[10]="";
char str0[]="12345";//пример строки из 5 символов '\0' НУЛЕВОЙ С�?МВОЛ НЕ УКАЗЫВАЕТСЯ
char str1[6]={'1','2','3','4','5','\0'};//"тоже самое". '\0'- символ оканчания строки
char str2[6]={49,50,51,52,53,0};//"аналог"строки из 6 элементов типа char только уже в числовом представлении.
char rx_buf[10];//буфер для записи пришедших данных по usart
uint8_t flag_usart=0;//флаг для сигнализирования о том, что в юсарт что-то пришло
char rx_buf1[256];
volatile uint8_t rx_pos;
volatile uint8_t  send_temp_flag=0;
volatile uint8_t  show_channel_flag=0;
uint16_t adc_data;
//====================================END USART=============================================

uint32_t res_addr;//переменная, куда запишется последний свободный адрес в памяти. туда будет происходить запись новых элементов
int i;
int y;
float t=0;
int16_t temp;
uint8_t Maincondition = 0; //переменная флаг. определяет, какой блок в главном цикле будет выполняться
uint8_t Line_channel_current = 0;
uint8_t flagwhichfunctionisactive;//переменная флаг, которая показывает, какая из функций продолжительности звука сейчас в работе. 1 - long, 0 - short
uint8_t counterforsoundshort=Shortsoundtime;
uint8_t counterforsoundlong=Longsoundtime;
int8_t TimeDisp[4] = {0,};//переменная для дисплея
//                      {mos_1|mos_2|mos_3|mos_4|mos_5|mos_6|mos_7|rel_1|rel_2|rel_3|rel_4|rel_5|rel_6|rel_7| Fan |not used}
uint8_t Register595[16]={  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,    0   };//массив, отправляемый на регистр 595 и включающий каналы и реле
//-------------------------------------------------------------------------------------
//                               Front Panel
//                    but1   but2   but3  but4   but5  but6   but7
//      but8
//                    led1   led2   led3  led4   led5  led6   led7
//-------------------------------------------------------------------------------------
//led№                     1     2     3     4     5     6     7   not
uint8_t LedState[8] =   {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  , 0 };//состояние светодиодов передней панели
//-------------------------------------------------------------------------------------
//S0|S1|S2								000		100		010		001		011		111		101	 110
//but№                     1     2     3     4     5     6     7    8
uint8_t ButtonState[8] ={  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  , 0 };//состояние кнопок после опроса. если 1 - то было короткое нажатие, если 2, то было длинное нажатие
uint8_t alarm_Button = 0;//флаг для сигнализирования о том, что с кнопками взаимодействовали и надо бы проверить их состояние.
//-------------------------------------------------------------------------------------
uint8_t button_8_mos_state_flag = 0;//флаг состояния кнопки 8 для управления всеми каналами одновременно. 0 - мосфеты выключены, 1 мосфеты включены
uint8_t button_8_rel_state_flag = 0;//флаг состояния кнопки 8 для управления всеми реле одновременно. 0 - реле выключеныб 1 - реле включены
//--------------------------------components with calculate current--------------------------
uint16_t adc_value[7];
double voltage[7];//переменная для накопления значений из ацп, в ней так же хранятся данные о значении тока на соответсвующем канале
//         												 (Xmin1 Xmin2 Xmin3 Xmin4 Xmin5 Xmin6 Xmin7) (Xmax1    Xmax2    Xmax3    Xmax4    Xmax5    Xmax6    Xmax7    zero)
myBuf_t   X_components[BUFFSIZE] = {  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  0   ,  4095  ,  4095  ,  4095  ,  4095  ,  4095  ,  4095  ,  4095  ,  0  };//массив для хранения компонент расчета тока, сохраняется во флеш памяти после калибровки
//                                 (Ymin1 Ymin2 Ymin3 Ymin4 Ymin5 Ymin6 Ymin7) (Ymax1 Ymax2  Ymax3  Ymax4  Ymax5  Ymax6  Ymax7  zero)
double    Y_components[BUFFSIZE] = { 0   ,  0  ,  0  ,  0  ,  0  ,  0  ,  0  ,  3.30 ,3.30 , 3.30 , 3.30 , 3.30 , 3.30 , 3.30 ,  0  };//эти компоненты измеряются мультиметром и записываются в прошивку
double K_coeff[7];//коэффициента к уравнениея прямой
double B_coeff[7];// смещение прямой, компонент В

//    X*Ymax_component - Xmin_component*Ymax_component
// Y=--------------------------------------------------
//             Xmax_component - Xmin_component
//
//    - Xmin_component*Ymax_component
// B=---------------------------------
//     Xmax_component - Xmin_component
//
//           Ymax_component
// K=--------------------------------
//    Xmax_component - Xmin_component
//-------------------------------------------------------------------------------------------
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void channels_on_off(uint8_t a[]);
void LedFrontPanelUpdate(uint8_t a[]);
void fill_array_for_show_integer(int16_t number, int8_t* adress);
void fill_array_for_show_number_of_channel(int8_t chanel, int8_t* adress);
void fill_array_for_show_double(double number, int8_t* adress);
uint8_t Calibration(void);//функция калибровки, возвращает статус калибровки
void Check_Button(void); //функция проверяет состояние флага alarm batton и производит действия со сдвиговыми регистрами, если эта необходимо, включает\отключает каналы и зажигает светодиоды.
void fill_array_for_show_temp(float temp, int8_t * adress);
//===================================usart===========================
void usart_send_byte (char chr);
void usart_send_string(const char* str);
void clear_rx_buf(void);
void check_uart(void);
uint8_t Check_condition(void);
void condition_0(void);
void condition_1(void);
void condition_2(void);
void Are_all_lights_on_cheking(void);
//==================================end usart=========================
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
	Are_all_lights_on_cheking();
  /* USER CODE BEGIN 2 */
	DWT_Init();        // инициализация DWT
  set_brightness(7); // 0 - 7 яркость
  clearDisplay();    // очистка дисплея
	HAL_ADCEx_Calibration_Start(&hadc1);
	LL_GPIO_SetOutputPin(MasterReset595_GPIO_Port, MasterReset595_Pin);//master reset high
	//============================================TIM3============================================================================//	
	LL_TIM_EnableUpdateEvent(TIM3);//TIM3->CR1&=~TIM_CR1_UDIS
	LL_TIM_EnableIT_UPDATE(TIM3); //TIM3->DIER |= TIM_DIER_UIE; прерывания 
	LL_TIM_EnableCounter(TIM3); //TIM3->CR1 |= TIM_CR1_CEN; // Запуск таймера
	//============================================TIM4============================================================================//	
	LL_TIM_EnableUpdateEvent(TIM4);//TIM4->CR1&=~TIM_CR1_UDIS
	LL_TIM_EnableIT_UPDATE(TIM4); //TIM4->DIER |= TIM_DIER_UIE; прерывания 
	//LL_TIM_EnableCounter(TIM4); //TIM4->CR1 |= TIM_CR1_CEN; // Запуск таймера
	//============================================================================================================================//
	//================================================usart===================================
	LL_USART_EnableIT_RXNE(USART1);
	//========================================================================================
	//erase_flash();//активируется одиз раз, затем комментариуется
	res_addr=flash_search_adress(STARTADDR, BUFFSIZE * DATAWIDTH);//ищем последний свободный адрес во влеш памяти
	//write_to_flash(X_components); // запись данных во флеш
	read_last_data_in_flash(X_components);//чтение последних данных из флеша
	Calibration();//калибровка
	for(i=0;i<7;i++){
			K_coeff[i] = Y_components[i+7]/(X_components[i+7] - X_components[i]);
		  B_coeff[i] = (-1)*(X_components[i]*Y_components[i+7])/(X_components[i+7] - X_components[i]);
	}
	
	Init_ds18b20(DS18B20_Resolution_12_bit);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	y=0;
	int flag_change_fan_condition = 1;
  while (1)
  {
		check_uart();
		Check_Button();
		Maincondition = Check_condition();
		switch(Maincondition){
			case 0:
				condition_0();
				break;
			case 1:
				condition_1();
				break;
			case 2:
				condition_2();
				break;
		}
		if(t > 50 && flag_change_fan_condition){// переменная t хранит текущую температуру, если температура выше 50 градусов, то включается вентилятор
			Register595[14]=1;
			flag_change_fan_condition = 0;
			channels_on_off(Register595);}
		
		if(t < 45 && !flag_change_fan_condition){
			Register595[14]=0;
			flag_change_fan_condition = 1;
			channels_on_off(Register595);}
		
			
		if(t > 75) {
			for(i = 0; i<14;i++)Register595[i]=0; // выключаем все каналы и реле
			Register595[14] = 1;//включаем вентилятор 
			for(i = 0;i<8;i++) LedState[i] = 0;//выключаем светодиоды передней панели
			LedFrontPanelUpdate(LedState);
			channels_on_off(Register595);
			Short_sound();
			TimeDisp[0]= 10;TimeDisp[1]= 19;TimeDisp[2]= 19;TimeDisp[3]= 43;// Вывести надпись ALL на дисплей
			display_mass(TimeDisp);
			usart_send_string("Allarm! High T!");
			while(t > 65){ // в цикле постоянно мониторим температуру, если она опустится ниже порога, то цикл прерывается. Программа при это переходит в состояние 0
					ds18b20_getTemperature();//запуск нового измерения, результат будет через 750 мс.
					LL_mDelay(1000);
					temp = resTemperature(); // зависаем на ~3мс
					t = temp/16;
			}
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_8);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7600;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 7199;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 9999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 7199;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, ClockPin595_Pin|latchPin595_Pin|DataPin595_Pin|dataled_Pin
                          |clockled_Pin|S2_Pin|S1_Pin|S0_Pin
                          |Datatemp_Pin|CLK_Pin|DIO_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, Sound_Pin|CS_Pin);

  /**/
  LL_GPIO_SetOutputPin(MasterReset595_GPIO_Port, MasterReset595_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = MasterReset595_Pin|Sound_Pin|CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ClockPin595_Pin|latchPin595_Pin|DataPin595_Pin|dataled_Pin
                          |clockled_Pin|S2_Pin|S1_Pin|S0_Pin
                          |CLK_Pin|DIO_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ReadButton_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(ReadButton_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Datatemp_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(Datatemp_GPIO_Port, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE7);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinMode(interrupt_GPIO_Port, interrupt_Pin, LL_GPIO_MODE_FLOATING);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

//=================================usart1_send_byte==============================//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
void usart_send_byte (char chr){
	       LL_USART_TransmitData8(USART1,chr);//USART1->DR = chr;
	       while (!LL_USART_IsActiveFlag_TC(USART1));
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//=================================usart1_send_string============================//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//void usart_send_string(const char* str) {//по началу использовал этот вариант!!!
//  while(*str){	
//		usart_send_byte(*str++);
//	}
//}
void usart_send_string(const char str[]) {//этот вариант проще для понимания
	uint16_t i = 0;
  while(str[i]){	
		usart_send_byte(str[i++]);
	}
}
//=========================================================================//
//-----------------------------clear_rx_buf--------------------------------//
//=========================================================================//
void clear_rx_buf(void){
	for(;rx_pos!=0;rx_pos--) rx_buf[rx_pos]=0xFF;
	rx_buf[0]=0xFF;
}
//----------------------------------------------------------------------------------------------------------------------------
uint8_t Calibration(void){//функция калибровки, возвращает ноль в случае успеха или код ошибки, в случае неудачи
	//===================================calibration start=========================================================
	uint8_t Calibration_Status=0;
	ButtonRead(&alarm_Button, &ButtonState[0]);//читаем кнопки и записываем 
	for(i=0;i<8;i++){//в цикле проверяем все кнопки, если хотя бы одна нажата, то выполняем код калибровки и дальше выходим из цикла в обычную загрузку
			uint8_t global_flag=1;//флаг для каскадного выхода из циклов
			uint8_t flag;//флаг для выхода из цикла
			uint16_t count=0;//счетчик для мигания светодиодом канала в цикле
			if( ButtonState[i] ){	
				Short_sound();
				TimeDisp[0] = 12;TimeDisp[1] = 10;TimeDisp[2] = 19;TimeDisp[3] = 19;
				display_mass(TimeDisp);//Выводим надпись CALL
				LL_mDelay(1000);
				for(int j=0;j<7;j++)   Register595[j+7]=1; //заполняем ячейки реле единицами

				channels_on_off(Register595);// включаем все реле
				LL_mDelay(2000);
				for(int j=0;j<Sample_ADC;j++){
					HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_value,7);
					for(i=0;i<7;i++) voltage[i]+=adc_value[i];
				}
				LL_mDelay(3000);
				for(int j=0;j<Sample_ADC;j++){
					HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_value,7);
					for(i=0;i<7;i++) voltage[i]+=adc_value[i];
				}
				for(int j=0;j<7;j++) {
					X_components[j]=voltage[j]/(Sample_ADC);//запись минимальной компоненты, такое значение счетчика при нулевом токе через канал
					voltage[j]=0;
				}
				
				while(1){
					if(!global_flag) break;
					for(int j=0;j<7;j++){
						if(LedState[0]&&LedState[1]&&LedState[2]&&LedState[3]&&LedState[4]&&LedState[5]&&LedState[6]) global_flag=0;
						if(!global_flag) break;
						if(LedState[j]) continue;
						ButtonState[j]=0;//обнуляем кнопку канала на тот случай, если нажали ее когда-то и в переменной сохранилось значение, теперь мы считаем что нажатий не было
						flag=1;
						Register595[j]=1;
						channels_on_off(Register595);//включаем канал
						fill_array_for_show_number_of_channel(j+1,TimeDisp);
						display_mass(TimeDisp);//отображаем калибруемый канал на дисплее
						count=0;//счетчик для мигания светодиодом канала в цикле
						while(flag){
							LL_GPIO_TogglePin(GPIOC,LL_GPIO_PIN_13);
							LL_mDelay(50);
							if(count==20){
								count=0;
								if(LedState[j]){
									LedState[j]=0;
									LedFrontPanelUpdate(LedState);
								}
								else{
									LedState[j]=1;
									LedFrontPanelUpdate(LedState);
								}
							}
						
							if(ButtonState[j]==1){//если кнопка калибруемого канала нажата коротко, то переходим на след. канал
								LedState[j]=0; ButtonState[j]=0; Register595[j]=0; flag=0; Short_sound();
							}
							if(ButtonState[j]==2){//если кнопка канала нажата "длинно", это значит подтверждение, что мы с помощью внешнего прибора выставили максимальный ток на канале. Записываем значения максимальной компоненты и переходим на след канал
								for(int y=0;y<Sample_ADC;y++){
									HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_value,7);
									for(i=0;i<7;i++) voltage[i]+=adc_value[i];
								}
								X_components[j+7]=voltage[j]/(Sample_ADC);
								for(i=0;i<7;i++) voltage[i]=0;
								LedState[j]=1;//если калибровка канала прошла, то светодиод канала включен всегда и канал больше не участвует в калибровке
								ButtonState[j]=0; Register595[j]=0; flag=0; Long_sound();
							}
							if(ButtonState[7]){//если нажата общая кнопка, то мы выходим из режима калибровки путем опускания всех флагов
								ButtonState[7]=0; flag=0; global_flag=0;
							}
							count++;
						}
					}
				}
				//РАСКОМЕНТ�?ТЬ ПОСЛЕ ОТЛАДК�?!!!write_to_flash(X_components);//записываем данные в память после калибровки. После каждого входа в режим калибровки, данные в памяти будут обновлены, даже если они не изменились
				break;//выход из режима калибровки
			}
	}
	alarm_Button=0;//обнуляем флаг кнопок после выхода из калибровки
	for(int j=0;j<14;j++)   Register595[j]=0; //заполняем ячейки сдвиг регистра нулями после выхода из калибровки
	for(int j=0;j<8;j++){
		LedState[j]=0;
		ButtonState[j]=0;
	}
	channels_on_off(Register595);//выключаем все каналы и реле после калибровки
	LedFrontPanelUpdate(LedState);//выключаем все светодиоды на передней панели после выхода из калибровки
	return Calibration_Status;//возвращает код статуса калибровки
//=========================================END Calibration=================================================================
}
//================================================================================================================
void channels_on_off(uint8_t a[])//отправка на сдвиговый регистр входного массива для управления реле и мосфетами
{
	//                      {rel_4|mos_5|rel_5|mos_6|rel_6|mos_7|rel_7|fan|mos_1|rel_1|mos_2|rel_2|mos_3|rel_3|mos_4|not used} порядок при загрузке в сдвиговый регистр
	static int8_t buf[] = {10,4,11,5,12,6,13,14,0,7,1,8,2,9,3,15};//массив правильного порядка считывания
	LL_GPIO_ResetOutputPin(latchPin595_GPIO_Port, latchPin595_Pin);
	for(uint8_t i=0;i<16;i++)
	{
		if(a[buf[i]]) LL_GPIO_SetOutputPin(DataPin595_GPIO_Port, DataPin595_Pin);
		else LL_GPIO_ResetOutputPin(DataPin595_GPIO_Port, DataPin595_Pin);
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);//ClockPin595 low to high 
	}
	LL_GPIO_SetOutputPin(latchPin595_GPIO_Port, latchPin595_Pin);
}
//----------------------------------------------------------------------------------------------------------------------------
void LedFrontPanelUpdate(uint8_t a[])//обновление состояний светодиодов на передней панели
{
  //led№                   not    3     2     1     7     6     5    4
  static int8_t b[8] =   {  7  ,  2  ,  1  ,  0  ,  6  ,  5  ,  4  , 3 };//массив верного порядка считывания и загрузки в регистр 164.
	for(uint8_t i = 0; i < 8 ; i++)
	{
		if(a[b[i]]) LL_GPIO_SetOutputPin(dataled_GPIO_Port, dataled_Pin);
		else LL_GPIO_ResetOutputPin(dataled_GPIO_Port, dataled_Pin);
		LL_GPIO_SetOutputPin(clockled_GPIO_Port, clockled_Pin); LL_GPIO_ResetOutputPin(clockled_GPIO_Port, clockled_Pin);
	}
}
//-------------------------------------------------------------------------------------------------------------------------------
void fill_array_for_show_integer(int16_t number, int8_t* b) //функция разбивки числа по разрядам с записью разрядов в виде отдельных чисел по указателю
{
  int16_t s;
	int16_t buf=number;//zxcv
	s=buf/1000;*b = s;//z
	buf=buf-s*1000;//xcv
	s=buf/100;*(b+1)=s;//=x
	buf=buf-s*100;//cv
	s=buf/10;*(b+2)=s;//c
	s=buf-s*10;*(b+3)=s;//v
	
	if(*b == 0){        *b=43; //проверка старшего разряда, если он равен 0, то меняем его на пустое место и дальше методом рекурсии проходим по всем разрядам, кроме младшего, младший всегда горит.
		if(*(b+1) == 0){  *(b+1) = 43;
		if(*(b+2) == 0)   *(b+2) = 43;
	  }
  }
}

//==================================================================================================================
void fill_array_for_show_number_of_channel(int8_t chanel,int8_t* adress)
{
	*(adress)   = 12;
	*(adress+1) = 32;
	*(adress+2) = 46;
	*(adress+3) = chanel;
}
//==================================================================================================================
void fill_array_for_show_double(double number, int8_t* adress)//функция разбивает пришедшее число на цифры и ставит точку после первого, записываются цифры в массив по указателю
{
	
	//z.xcv
	int16_t s;
	*(adress) = (int8_t)number+50;//z.
	int16_t buf = (number-(*(adress)-50))*1000;//xcv
	
	s=buf/100;*(adress+1)=s;//=x
	buf=buf-s*100;//cv
	s=buf/10;*(adress+2)=s;//c
	s=buf-s*10;*(adress+3)=s;//v
}

//==================================================================================================================
void fill_array_for_show_temp(float temp, int8_t * adress){
		
	//zx.cv
	int16_t s = temp*100;//zxcv
	*(adress) = s/1000;//z
	int16_t buf = s-(*(adress)*1000);//xcv
	
	s=buf/100;*(adress+1)=(s+50);//=x.
	buf=buf-s*100;//cv
	s=buf/10;*(adress+2)=s;//c
	s=buf-s*10;
	*(adress+3)=s;//v
}
//==================================================================================================================
//==================================================================================================================
void check_uart(void){
	if(flag_usart){
		flag_usart=0;
		int flg =0;
		if (strstr(rx_buf,"C1N")){ Register595[0] = 1;flg = 1;}
		if (strstr(rx_buf,"C2N")){ Register595[1] = 1;flg = 1;}
		if (strstr(rx_buf,"C3N")){ Register595[2] = 1;flg = 1;}
		if (strstr(rx_buf,"C4N")){ Register595[3] = 1;flg = 1;}
		if (strstr(rx_buf,"C5N")){ Register595[4] = 1;flg = 1;}
		if (strstr(rx_buf,"C6N")){ Register595[5] = 1;flg = 1;}
		if (strstr(rx_buf,"C7N")){ Register595[6] = 1;flg = 1;}
		if (strstr(rx_buf,"CN")){
			for(int i = 0; i < 7; i++) Register595[i] = 1;
			flg = 1;
		}
		if (strstr(rx_buf,"C1F")){ Register595[0] = 0;flg = 1;}
		if (strstr(rx_buf,"C2F")){ Register595[1] = 0;flg = 1;}
		if (strstr(rx_buf,"C3F")){ Register595[2] = 0;flg = 1;}
		if (strstr(rx_buf,"C4F")){ Register595[3] = 0;flg = 1;}
		if (strstr(rx_buf,"C5F")){ Register595[4] = 0;flg = 1;}
		if (strstr(rx_buf,"C6F")){ Register595[5] = 0;flg = 1;}
		if (strstr(rx_buf,"C7F")){ Register595[6] = 0;flg = 1;}
		if (strstr(rx_buf,"CF")){
			for(int i = 0; i < 7; i++) Register595[i] = 0;
			flg = 1;
		}
		if (strstr(rx_buf,"R1N")){ Register595[7] = 1;LedState[0]=1;flg = 1;}
		if (strstr(rx_buf,"R2N")){ Register595[8] = 1;LedState[1]=1;flg = 1;}
		if (strstr(rx_buf,"R3N")){ Register595[9] = 1;LedState[2]=1;flg = 1;}
		if (strstr(rx_buf,"R4N")){ Register595[10] = 1;LedState[3]=1;flg = 1;}
		if (strstr(rx_buf,"R5N")){ Register595[11] = 1;LedState[4]=1;flg = 1;}
		if (strstr(rx_buf,"R6N")){ Register595[12] = 1;LedState[5]=1;flg = 1;}
		if (strstr(rx_buf,"R7N")){ Register595[13] = 1;LedState[6]=1;flg = 1;}
		if (strstr(rx_buf,"RN")){
			for(int i = 7; i < 14; i++){ Register595[i] = 1;LedState[i-7]=1;}
			flg = 1;
		}
		if (strstr(rx_buf,"R1F")){ Register595[7] = 0;LedState[0]=0;flg = 1;}
		if (strstr(rx_buf,"R2F")){ Register595[8] = 0;LedState[1]=0;flg = 1;}
		if (strstr(rx_buf,"R3F")){ Register595[9] = 0;LedState[2]=0;flg = 1;}
		if (strstr(rx_buf,"R4F")){ Register595[10] = 0;LedState[3]=0;flg = 1;}
		if (strstr(rx_buf,"R5F")){ Register595[11] = 0;LedState[4]=0;flg = 1;}
		if (strstr(rx_buf,"R6F")){ Register595[12] = 0;LedState[5]=0;flg = 1;}
		if (strstr(rx_buf,"R7F")){ Register595[13] = 0;LedState[6]=0;flg = 1;}
		if (strstr(rx_buf,"RF")){
			for(int i = 7; i < 14; i++) {Register595[i] = 0;LedState[i-7]=0;}
			flg = 1;
		}
		if (strstr(rx_buf,"FAN")){ Register595[14] = 1;flg = 1;}
		if (strstr(rx_buf,"FAF")){ Register595[14] = 0;flg = 1;}
		if(flg){
			flg = 0;
			channels_on_off(Register595);
			LedFrontPanelUpdate(LedState);
			clear_rx_buf();
			sprintf(logs1,"ok");
			usart_send_string(logs1);
		}
	}
}
	

//==================================================================================================================
void Short_sound(void){
	flagwhichfunctionisactive=0;
	if(!counterforsoundshort){
		LL_GPIO_ResetOutputPin(Sound_GPIO_Port,Sound_Pin);
		LL_TIM_DisableCounter(TIM4);
		counterforsoundshort=Shortsoundtime;
	}
	else{
		LL_GPIO_SetOutputPin(Sound_GPIO_Port,Sound_Pin);
		LL_TIM_EnableCounter(TIM4);		//TIM4->CR1 |= TIM_CR1_CEN; // Запуск таймера
		counterforsoundshort--;
	}
}
//==================================================================================================================
void Long_sound(void){
	flagwhichfunctionisactive=1;
	if(!counterforsoundlong){
		LL_GPIO_ResetOutputPin(Sound_GPIO_Port,Sound_Pin);
		LL_TIM_DisableCounter(TIM4);
		counterforsoundlong=Longsoundtime;
	}
	else{
		LL_GPIO_SetOutputPin(Sound_GPIO_Port,Sound_Pin);
		LL_TIM_EnableCounter(TIM4);		//TIM4->CR1 |= TIM_CR1_CEN; // Запуск таймера
		counterforsoundlong--;
	}
}
//==================================================================================================================
void Check_Button(void){//проверяет флаг, если поднят, то сканирует состояние кнопок и состояние регистра каналов, если нужно, изменяет состояние светодиодов, обновляет информацию и меняет состояние Maincondition
			if(alarm_Button){
				alarm_Button=0;
				for(i=0;i<7;i++){
					if(ButtonState[i]){
						if(ButtonState[i]==1){
							if(Register595[i]) Register595[i]=0;
							else               Register595[i]=1;}
						else{
							if(Register595[i+7]){ 
								Register595[i+7]=0;
								LedState[i]=0;}
							else {
								Register595[i+7]=1;
								LedState[i]=1;}}}}
				
				if(ButtonState[7]){
					if(ButtonState[7]==1){
						if(button_8_mos_state_flag){
							button_8_mos_state_flag=0;
							for(i=0;i<7;i++){
								Register595[i]=0;	}}
						else {
							button_8_mos_state_flag=1;
							for(i=0;i<7;i++){
								Register595[i]=1;}}}
					else{
						if(button_8_rel_state_flag){
							button_8_rel_state_flag=0;
							for(i=0;i<7;i++){
								Register595[i+7]=0;
								LedState[i]=0;}}
						else {
							button_8_rel_state_flag=1;
							for(i=0;i<7;i++){
								Register595[i+7]=1;
								LedState[i]=1;}}}}
				channels_on_off(Register595);
				LedFrontPanelUpdate(LedState);
			}
}
//==================================================================================================================
uint8_t Check_condition(void){
	uint8_t condition = 0;
	for(int i = 0;i<7;i++){
		if(Register595[i]){
			condition = 1;
			break;}}
	
	if(condition){
		for(int i = 0;i<7;i++){
			if(Register595[i] && Register595[i+7]){
				condition = 2;
				break;}}}
	return condition;
}
//==================================================================================================================
void condition_0(void){
		if(send_temp_flag>=10){
			send_temp_flag=0;
			ds18b20_getTemperature();}//запуск нового измерения, результат будет через 750 мс.
	  if(flagtemp == 2){ // когда таймер отсчитает 750мс, он установит flag == 2
		  flagtemp = 0;
		  temp = resTemperature(); // зависаем на ~3мс
			t = temp/16;
			sprintf(logs,"T = %.2f \n\r",t);
			usart_send_string(logs);}
		if(show_channel_flag>=1){
			show_channel_flag = 0;
			if(!Line_channel_current){
				Line_channel_current = 1;
				fill_array_for_show_temp(t,TimeDisp);
				display_mass(TimeDisp);}
			else{
				Line_channel_current = 0;
				fill_array_for_show_temp(t,TimeDisp);
				TimeDisp[2]= 47;TimeDisp[3]= 12;//TimeDisp[2]= 43;TimeDisp[3]= 43;
				display_mass(TimeDisp);
			}
		
		
		}
		
}
void condition_1(void){
		if(send_temp_flag>=10){
			send_temp_flag=0;
			ds18b20_getTemperature();}//запуск нового измерения, результат будет через 750 мс.
	  if(flagtemp == 2){ // когда таймер отсчитает 750мс, он установит flag == 2
		  flagtemp = 0;
		  temp = resTemperature(); // зависаем на ~3мс
			t = temp/16;
			sprintf(logs,"T = %.2f \n\r",t);
			usart_send_string(logs);
		}
		if(show_channel_flag){
			show_channel_flag = 0;
			if(y>=7) y = 0;
			while(!Register595[y]){
				y++;
				if(y>6) y = 0;
			}
			y+=1;
			fill_array_for_show_number_of_channel(y,TimeDisp);
			display_mass(TimeDisp);
			usart_send_string("Ch on: ");
			for(int i = 0;i<7;i++){
				if(Register595[i]){
					sprintf(logs,"%d ",(i+1));
					usart_send_string(logs);}}
			usart_send_string("\n\r");
		}
}
void condition_2(void){
		if(send_temp_flag>=10){
			send_temp_flag=0;
			ds18b20_getTemperature();}//запуск нового измерения, результат будет через 750 мс.
	  if(flagtemp == 2){ // когда таймер отсчитает 750мс, он установит flag == 2
		  flagtemp = 0;
		  temp = resTemperature(); // зависаем на ~3мс
			t = temp/16;
			sprintf(logs,"T = %.2f \n\r",t);
			usart_send_string(logs);
		}
		
		if(show_channel_flag>=1){
			show_channel_flag = 0;
			
			for(i=0;i<7;i++)	voltage[i]=0;
			for(int j=0;j<Sample_ADC;j++){
				HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_value,7);
				for(i=0;i<7;i++) voltage[i]+=adc_value[i];
			}
			for(i=0;i<7;i++)voltage[i]=(voltage[i]/Sample_ADC);
			for(i=0;i<7;i++)voltage[i]=(voltage[i]* K_coeff[i])+B_coeff[i];
			if(!Line_channel_current){
				Line_channel_current = 1;
				if(y>=7) y = 0;
				while(!Register595[y] || !Register595[y+7]){
					y++;
					if(y>6) y = 0;
				}
				y+=1;
				fill_array_for_show_number_of_channel(y,TimeDisp);
				display_mass(TimeDisp);}
			else{
				Line_channel_current = 0;
				fill_array_for_show_double(voltage[y-1],TimeDisp);
				usart_send_string("Ch on:");
			for(int i = 0;i<7;i++){
				if(Register595[i]){
					sprintf(logs," %d",(i+1));
					usart_send_string(logs);}
				if(Register595[i+7]){
					sprintf(logs,"(%.3f)",voltage[i]);
					usart_send_string(logs);
				}
			}
			usart_send_string("\n\r");
				display_mass(TimeDisp);
			}
		}
}
//=======================================
void Are_all_lights_on_cheking(void){
	Register595[14] = 1;
	channels_on_off(Register595);
	TimeDisp[0] = 58;TimeDisp[1] = 58;TimeDisp[2] = 58;TimeDisp[3] = 58;
	for(i = 0;i<8;i++){
		set_brightness(i);
		display_mass(TimeDisp);
		LL_mDelay(100);
	}
	for(i = 8;i>0;i--){
		set_brightness(i-1);
		display_mass(TimeDisp);
		LL_mDelay(100);
	}
	int local_count = 0;
	while(local_count !=3){
		local_count++;
		for(i = 0;i<8;i++){
			LedState[i] = 1;
			LedFrontPanelUpdate(LedState);
			LL_mDelay(50);
			LedState[i] = 0;
		}
		for(i = 7;i>0;i--){
			LedState[i] = 1;
			LedFrontPanelUpdate(LedState);
			LL_mDelay(50);
			LedState[i] = 0;
		}
	}
	Register595[14] = 0;
	channels_on_off(Register595);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
