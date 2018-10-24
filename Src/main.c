/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "app_ethernet.h"
#include "httpserver-netconn.h"
#include "lcd_log.h"
#include "x10_library.h"

/* Private define*/
#define NB_SEND 5
#define SIZE_TRAM 68

/* Private variables ---------------------------------------------------------*/
static TS_StateTypeDef  TS_State;
struct netif gnetif; /* network interface structure */
osThreadId x10_thread_Id;
osThreadId touchscreen_thread;
volatile unsigned char ptr_data_x10;
volatile unsigned char bInterrupt;
volatile signed char interrupt_count = 0;
unsigned char x10_frame_array_send[SIZE_TRAM];
unsigned char bToogle = 0;

/* OS message declaration*/
osMessageQDef(x10_send_message, 1, uint16_t); 
osMessageQId (x10_send_message);           

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void StartThread(void const * argument);
static void touchscreen_demo(void const *argument);
static void BSP_Config(void);
static void Netif_Config(void);
static void MPU_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);

static void x10_thread(void const *argument)
{
  (void) argument;
  InitX10();
  osEvent  osEvent_X10;
  for(;;)
  {
    osEvent_X10 = osMessageGet(x10_send_message,osWaitForever);
    if (osEvent_X10.status == osEventMessage){
      uint16_t x10_data = osEvent_X10.value.v;
      uint16_t address = (x10_data & 0xFF00)>>8;
      uint16_t data = (x10_data & 0xFF);
      Send_complete_x10_frame(address,data);
    }
  }
}



static void touchscreen_demo(void const *argument)
{
 uint8_t  status = 0;
  uint16_t x, y;
  uint16_t data_x10_send_ts;
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

        if(bToogle==1){
          data_x10_send_ts = (A1_8_ADDR<<8) | A1_ON;
          bToogle = 0;
          LCD_UsrLog ((char *)" A1 : OFF\n");
        }else{
          data_x10_send_ts = (A1_8_ADDR<<8) | A1_OFF;
          LCD_UsrLog ((char *)" A1 : ON\n");
          bToogle = 1;
        }
        osMessagePut(x10_send_message, data_x10_send_ts, osWaitForever );
      } /* of if(TS_State.touchDetected) */
    }
    osDelay(100);
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
  osThreadDef(x10,x10_thread,osPriorityNormal,0, configMINIMAL_STACK_SIZE*2);
  osThreadDef(touchscreen,touchscreen_demo,osPriorityBelowNormal,0, configMINIMAL_STACK_SIZE*2);
  x10_thread_Id = osThreadCreate(osThread(x10),NULL);
  touchscreen_thread = osThreadCreate(osThread(touchscreen),NULL);

  // Create message queu

   x10_send_message = osMessageCreate(osMessageQ(x10_send_message), NULL);

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
