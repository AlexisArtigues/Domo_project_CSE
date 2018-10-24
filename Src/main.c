/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "app_ethernet.h"
#include "httpserver-netconn.h"
#include "lcd_log.h"

/* Private define*/
#define NB_SEND 5
#define SIZE_TRAM 68
#define A1_8_ADDR 0x60
#define A1_ON 0x00
#define A1_OFF 0x20
#define A2_ON 0x10
#define A2_OFF 0x30
/* Private typedef -----------------------------------------------------------*/
/* Struct */
/* Private variables ---------------------------------------------------------*/
static TS_StateTypeDef  TS_State;
/* TIM handle declaration */
TIM_HandleTypeDef    TimHandle3;
/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

/* Private variables ---------------------------------------------------------*/
//TEtatX10 EtatX10;
//TEtatSendX10 SendX10;
volatile signed char interrupt_count = 0;
volatile unsigned char bit_one=0;
struct netif gnetif; /* network interface structure */
osThreadId thread2_Id;
osThreadId touchscreen_thread;
volatile unsigned char ptr_data_x10;
volatile unsigned char bInterrupt;
unsigned char x10_frame_array_send[SIZE_TRAM];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void StartThread(void const * argument);
static void touchscreen_demo(void const *argument);
static void BSP_Config(void);
static void Netif_Config(void);
static void MPU_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
static void InitX10(void);
static void InitTimer(void);
void ResetEtatX10(void);
void Build_x10_tick_array(uint8_t adrress,uint8_t data,unsigned char array_out[]);
void Cp_array(unsigned char arrayIn[], unsigned char arrayOut[],unsigned char dim);
void Send_single_x10_frame(uint8_t addres_x10,uint8_t data_x10);
void Send_complete_x10_frame(uint8_t addres_x10,uint8_t data_x10);

void Cp_array(unsigned char arrayIn[], unsigned char arrayOut[],unsigned char dim){
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

static void InitX10(void){
  // D2 is on Port G - Bit 6 
  GPIO_InitTypeDef  gpio_init_structure;

  /* Enable the GPIO_LED clock */
    __HAL_RCC_GPIOG_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
    gpio_init_structure.Pin = GPIO_PIN_6;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_PULLUP;
    gpio_init_structure.Speed = GPIO_SPEED_HIGH;

   HAL_GPIO_Init(GPIOG, &gpio_init_structure);
    
    /* By default, turn off LED */
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
}


static void thread2(void const *argument)
{
  (void) argument;
  InitTimer();
	bInterrupt = 0;
  for(;;)
  {
    Send_complete_x10_frame(A1_8_ADDR,A2_OFF);
    osDelay(5000);
    Send_complete_x10_frame(A1_8_ADDR,A2_ON);
    osDelay(5000);
  }
}



static void touchscreen_demo(void const *argument)
{
 uint8_t  status = 0;
  uint16_t x, y;
  uint8_t  text[30];

  status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());

  while (1)
  {
    if (status == TS_OK)
    {
      /* Check in polling mode in touch   screen the touch status and coordinates */
      /* if touch occurred                                                      */
      BSP_TS_GetState(&TS_State);
      if(TS_State.touchDetected)
      {
        /* Get X and Y position of the touch post calibrated */
        x = TS_State.touchX[0];
        y = TS_State.touchY[0];
        /* Display 1st touch detected coordinates */
        sprintf((char*)text, "TS detected : 1[%d,%d] \n", x, y);
        LCD_UsrLog ((char *)&text);

      } /* of if(TS_State.touchDetected) */
    }
    osDelay(100);
  }
}

static void InitTimer(void){
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

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance==TIM6){
			HAL_IncTick();
	}else{
		if(bInterrupt==1){
    if(ptr_data_x10<SIZE_TRAM){
      if(x10_frame_array_send[ptr_data_x10]==interrupt_count){
        ptr_data_x10++;
        HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
        interrupt_count = 0;
      }else{
        interrupt_count++;
      }
    }else{
      bInterrupt = 0;
      HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
    }
    }
  }
	}

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

int main(void)
{
  /* Configure the MPU attributes as Device memory for ETH DMA descriptors */
  MPU_Config();

  /* Enable the CPU Cache */
   CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();  
  
  /* Configure the system clock to 200 MHz */
  SystemClock_Config(); 

  InitX10();

  /* Init thread */
#if defined(__GNUC__)
  osThreadDef(Start, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 5);
#else
  osThreadDef(Start, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
#endif
  osThreadCreate (osThread(Start), NULL);

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}

/**
  * @brief  Start Thread 
  * @param  argument not used
  * @retval None
  */
static void StartThread(void const * argument)
{ 
  /* Initialize LCD */
  BSP_Config();
  
  /* Create tcp_ip stack thread */
  tcpip_init(NULL, NULL);
  
  /* Initialize the LwIP stack */
  Netif_Config();
  
  /* Initialize webserver demo */
  http_server_netconn_init();
  
  /* Notify user about the network interface config */
  User_notification(&gnetif);
  
#ifdef USE_DHCP
  /* Start DHCPClient */
  osThreadDef(DHCP, DHCP_thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
  osThreadCreate (osThread(DHCP), &gnetif);
#endif
  osThreadDef(LED2,thread2,osPriorityNormal,0, configMINIMAL_STACK_SIZE*2);
  osThreadDef(touchscreen,touchscreen_demo,osPriorityBelowNormal,0, configMINIMAL_STACK_SIZE*2);
  thread2_Id = osThreadCreate(osThread(LED2),NULL);
  touchscreen_thread = osThreadCreate(osThread(touchscreen),NULL);
  for( ;; )
  {
    /* Delete the Init Thread */ 
    osThreadTerminate(NULL);
  }
}

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
static void Netif_Config(void)
{ 
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
 
#ifdef USE_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else
  IP_ADDR4(&ipaddr,IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
  IP_ADDR4(&netmask,NETMASK_ADDR0,NETMASK_ADDR1,NETMASK_ADDR2,NETMASK_ADDR3);
  IP_ADDR4(&gw,GW_ADDR0,GW_ADDR1,GW_ADDR2,GW_ADDR3);
#endif /* USE_DHCP */
  
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
  
  /*  Registers the default network interface. */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }
}

/**
  * @brief  Initializes the STM327546G-Discovery's LCD  resources.
  * @param  None
  * @retval None
  */
static void BSP_Config(void)
{
  /* Initialize the LCD */
  BSP_LCD_Init();
  
  /* Initialize the LCD Layers */
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS);
  
  /* Set LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);
  
  BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
  
  /* Initialize LCD Log module */
  LCD_LOG_Init();
  
  /* Show Header and Footer texts */
  LCD_LOG_SetHeader((uint8_t *)"Projet Domotique CSE");
  LCD_LOG_SetFooter((uint8_t *)"CPE Engineering School");
  
  LCD_UsrLog ((char *)"  State: Ethernet Initialization ...\n");
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* activate the OverDrive */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes as Device for  Ethernet Descriptors in the SRAM1.
  * @note   The Base Address is 0x20010000 since this memory interface is the AXI.
  *         The Configured Region Size is 256B (size of Rx and Tx ETH descriptors) 
  *       
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();
  
  /* Configure the MPU attributes as Device for Ethernet Descriptors in the SRAM */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
