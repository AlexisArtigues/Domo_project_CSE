#include "x10_library.h"

static void Error_Handler(void);
void Cp_array(unsigned char arrayIn[], unsigned char arrayOut[],unsigned char dim);

extern volatile unsigned char ptr_data_x10;
extern volatile unsigned char bInterrupt;
extern unsigned char x10_frame_array_send[SIZE_TRAM];
extern volatile signed char interrupt_count;

/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle3;
/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

void InitX10(void){

bInterrupt = 0;

  // D2 is on Port G - Bit 6 
  GPIO_InitTypeDef  gpio_init_structure;

  /* Enable the GPIO_G clock */
    __HAL_RCC_GPIOG_CLK_ENABLE();

  /* Configure the GPIOG_6 pin */
    gpio_init_structure.Pin = GPIO_PIN_6;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;

   HAL_GPIO_Init(GPIOG, &gpio_init_structure);
    
    /* By default, turn off output */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

    /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
    //uwPrescalerValue = (uint32_t)(200-1);
    uwPrescalerValue = (uint32_t)((SystemCoreClock / 4) / 1000000) - 1;
    /* Set TIMx instance */
    TimHandle3.Instance = TIM3;

    /* Initialize TIMx peripheral as follows:
        + Period = 10000 - 1
        + Prescaler = ((SystemCoreClock / 2)/10000) - 1
        + ClockDivision = 0
        + Counter direction = Up
    */
    TimHandle3.Init.Period            = (1125/4) - 1;
    TimHandle3.Init.Prescaler         = uwPrescalerValue;
    TimHandle3.Init.ClockDivision     = 0;
    TimHandle3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    TimHandle3.Init.RepetitionCounter = 0;
    TimHandle3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&TimHandle3) != HAL_OK)
    {
        /* Initialization Error */
        Error_Handler();
    }

    /*##-2- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */
    if (HAL_TIM_Base_Start_IT(&TimHandle3) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }

}

static void Cp_array(unsigned char arrayIn[], unsigned char arrayOut[],unsigned char dim){
  unsigned char i=0;
  for(i=0;i<dim;i++){
    arrayOut[i] = arrayIn[i];
  }
}

void Build_x10_tick_array(uint8_t adress,uint8_t data,unsigned char array_out[]){
  unsigned char temp_array_tick[] = {1,64,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5};
  uint32_t temp_array_addr_data;
  char i;
  unsigned char ptr_array_tick = 3;
  uint8_t adress_ = ~adress;
  uint8_t data_ = ~data;
  temp_array_addr_data = (adress << 24) | (adress_ << 16) | (data << 8) | data_;
  i=32;
  do{
		i--;
    if(temp_array_addr_data &(1<<i)){
      temp_array_tick[ptr_array_tick] = 3;
      ptr_array_tick++;
      temp_array_tick[ptr_array_tick] = 11;
      ptr_array_tick++;
    }else{
      temp_array_tick[ptr_array_tick] = 3;
      ptr_array_tick++;
      temp_array_tick[ptr_array_tick] = 3;
      ptr_array_tick++;
    }
  }while(i>0);
  Cp_array(temp_array_tick,array_out,SIZE_TRAM);
}

void Send_single_x10_frame(uint8_t addres_x10,uint8_t data_x10){
  unsigned char array_tick_x10[SIZE_TRAM];
  Build_x10_tick_array(addres_x10,data_x10,array_tick_x10);
  Cp_array(array_tick_x10,x10_frame_array_send,SIZE_TRAM);
  ptr_data_x10=0;
	interrupt_count=0;
	bInterrupt = 1;
}

void Send_complete_x10_frame(uint8_t addres_x10,uint8_t data_x10){
    unsigned char i = 0;
    for(i=0;i<4;i++){
     Send_single_x10_frame(addres_x10,data_x10);
      osDelay(105);
    }
}

static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}
