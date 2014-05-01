/**
  ******************************************************************************
  * @file    ADC_Reading X and Y axis voltage value/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    7-April-2014
  * @brief   Main program body
  ******************************************************************************

* Includes ------------------------------------------------------------------**/
#include "stm32f30x.h"

/*Variables para Controlador*/
float SetpointX=3.3f;
float SetpointY=6.5f;
uint8_t ServomotorX=90;
uint8_t ServomotorY=90;
float PosX=0.1f;
float PosY=0.1f;

/*Variables para HMI*/
uint8_t SetX;
uint8_t SetY;
uint8_t LocX;
uint8_t LocY;
uint8_t ServoX;
uint8_t ServoY;

/*Variables para USB*/
extern __IO uint8_t Receive_Buffer[6];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
uint8_t Send_Buffer[6];
uint32_t packet_sent=1;
uint32_t packet_receive=1;

__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedValue2 = 0, axisValueX = 0, axisValueY = 0, calibration_value = 0, calibration_value2 = 0;
__IO uint32_t TimingDelay = 0;

void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void convParamSend(void){
	SetX=(uint8_t)(SetpointX/0.048046875f);
	SetY=(uint8_t)(SetpointY/0.064453125f);
	ServoX=(uint8_t)(ServomotorX);
	ServoY=(uint8_t)(ServomotorY);
	LocX=(uint8_t)(PosX/0.048046875f);
	LocY=(uint8_t)(PosY/0.064453125f);
}
	
static void setupGpioAnalogPC4(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	/*GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
		
	/* Configure ADC2 Channel1 as analog input-->PA4 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void setupGpioAnalogPF4(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	/*GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	
  /* Configure ADC1 Channel7 as analog input-->PC1 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

}

static void setupGpioHZPD8(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	/*GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
		
	/* Configure ADC2 Channel1 as analog input-->PA4 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOD->PUPDR &= 0xFFFFFFF3;
	GPIOD->MODER &= 0xFFFFFFF3;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static void setupGpioHZPB0(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	/*GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
		
	/* Configure ADC2 Channel1 as analog input-->PA4 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOB->PUPDR &= 0xFFFFFFF3;
	GPIOB->MODER &= 0xFFFFFFF3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void setupGpioPC4Output(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	/*GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	
  /* Configure PA4 in output pushpull mode */

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

static void setupGpioPF4Output(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	/*GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	
	/* Configure PA4 in output pushpull mode */

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}

static void setupGpioPD8Output(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	/*GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
	
  /* Configure PA4 in output pushpull mode */

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static void setupGpioPB0Output(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	/*GPIOA Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
  /* Configure PA4 in output pushpull mode */

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void calibrateADC1(void){

  /* Calibration procedure */  
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  
  /* Insert delay equal to 10 µs */
  Delay(10);
  
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
	
	while(ADC_GetCalibrationStatus(ADC1) != RESET ){}
  calibration_value = ADC_GetCalibrationValue(ADC1); 
}

static void calibrateADC2(void){
	
  /* Calibration procedure */  
  ADC_VoltageRegulatorCmd(ADC2, ENABLE);
  
  /* Insert delay equal to 10 µs */
  Delay(10);
 	
	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC2);
	
	while(ADC_GetCalibrationStatus(ADC2) != RESET ){}
  calibration_value2 = ADC_GetCalibrationValue(ADC2);
}

static void setupAdcPF4(void){
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	/* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
	  
  /* Enable ADC1 and ADC2 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	
	/* Calling functions */
	setupGpioAnalogPF4();
	calibrateADC1();
  

  ADC_StructInit(&ADC_InitStructure);

	/* ADC1 configuration */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
	
	/* ADC1 configuration */
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 1; 
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* ADC1 regular channel7 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_7Cycles5);
   
  /* Enable ADC1 and ADC2 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* wait for ADRDY */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  
  /* Start ADC1 and ADC2 Conversion */ 
  ADC_StartConversion(ADC1);   
	
}

static void setupAdcPC4(void){
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	/* Configure the ADC clock */
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
	  
  /* Enable ADC1 and ADC2 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	
	/* Calling functions */
	setupGpioAnalogPC4();
	calibrateADC2();
  

  ADC_StructInit(&ADC_InitStructure);

	/* ADC2 configuration */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC2, &ADC_CommonInitStructure);
	
	/* ADC2 configuration */
  ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);
  
  /* ADC2 regular channel1 configuration */ 
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_7Cycles5);
   
  /* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);
  
  /* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));
  
  /* Start ADC2 Conversion */   
	ADC_StartConversion(ADC2);   

}

static void readXValue(void){

	setupGpioHZPB0();
	setupGpioPC4Output();
	setupGpioPD8Output();
	/* Set PE14 and PE15 */
  GPIOD->BSRR = 0x0100;
  /* Reset PE14 and PE15 */
  GPIOC->BRR = 0x0010;
	setupAdcPF4();

}

static void readYValue(void){

	setupGpioHZPD8();
	setupGpioPF4Output();
	setupGpioPB0Output();
	/* Set PE14 and PE15 */
  GPIOB->BSRR = 0x0001 ;
  /* Reset PE14 and PE15 */
  GPIOF->BRR = 0x0010 ;
	setupAdcPC4();

}

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f30x.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f30x.c file
     */ 
  /* Setup SysTick Timer for 1 µsec interrupts  */
 if (SysTick_Config(SystemCoreClock / 1000000))
  { 
    /* Capture error */ 
    while (1)
    {}
  }

  /* Infinite loop */
  while (1)
  {
		
		/*Reading X Axis */
		readXValue();
    /* Test EOC flag */
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		
    /* Get ADC1 converted data */
    ADC1ConvertedValue =ADC_GetConversionValue(ADC1);
    
    /* Compute the voltage */
    axisValueX = (ADC1ConvertedValue *3300)/0xFFF;
		
 		ADC_DeInit (ADC1);
		ADC_DeInit (ADC2);
		GPIO_DeInit (GPIOA);
		GPIO_DeInit (GPIOB);
		GPIO_DeInit (GPIOE);
		GPIO_DeInit (GPIOC);
		
		/*Reading Y Axis */
		readYValue();
		/* Test EOC flag */
		while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
		
    /* Get ADC2 converted data */
		ADC1ConvertedValue2 =ADC_GetConversionValue(ADC2);
    
    /* Compute the voltage */
		axisValueY = (ADC1ConvertedValue2 *3300)/0xFFF;

		ADC_DeInit (ADC1);
		ADC_DeInit (ADC2);
		GPIO_DeInit (GPIOA);
		GPIO_DeInit (GPIOB);
		GPIO_DeInit (GPIOE);
		GPIO_DeInit (GPIOC);
		
		
  }
}

int main(void)
{
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();

	
  while (1)
  {
    if (bDeviceState == CONFIGURED)
    {
			
			CDC_Receive_DATA();

			SetpointX=(float)Receive_Buffer[0];
			SetpointX=SetpointX*0.048046875f;
			SetpointY=(float)Receive_Buffer[1];
			SetpointY=SetpointY*0.064453125f;

			PosX=PosX+0.1f;
			PosY=PosY+0.1f;
			
			if (PosX==12.0f) PosX=0.1f;
			if (PosY==16.0f) PosY=0.1f;
			
			convParamSend();
			
			Send_Buffer[0]=SetX;
			Send_Buffer[1]=SetY;
			Send_Buffer[2]=LocX;
			Send_Buffer[3]=LocY;
			Send_Buffer[4]=ServoX;
			Send_Buffer[5]=ServoY;
      /*Check to see if we have data yet */
      if(Receive_length!=0){
      if (packet_sent == 1)
      CDC_Send_DATA ((unsigned char*)Send_Buffer,6);
      Receive_length = 0;
		}
      
    }
  }
} 
