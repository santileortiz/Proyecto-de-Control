/**
  ******************************************************************************
  * @file   Proyecto de Control: Ball and Beam
  * @author  Grisel Morales, Santiago León, Alejandro Juárez, Gabriela Ramos, Caroline Siordia
  * @version V1.1.0
  * @date    09 de Mayo de 2014
  * @brief   Main program body
  ******************************************************************************

* Includes ------------------------------------------------------------------**/
#include "stm32f30x.h"
#include "main.h"

/*Variables para enviar y recibir de Procedimientos del Controlador*/
float SetpointX=0.0f;
float SetpointY=0.0f;
float ServomotorX=90;
float ServomotorY=-90;
float PosX=0.1f;
float PosY=0.1f;

/*Variables para ser enviadas al HMI*/
int8_t SetX;
int8_t SetY;
int8_t LocX;
int8_t LocY;
int8_t ServoX;
int8_t ServoY;

/*Variables para USB*/
extern __IO int8_t Receive_Buffer[6];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
int8_t Send_Buffer[6];
uint32_t packet_sent=1;
uint32_t packet_receive=1;

__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedValue2 = 0, axisValueX = 0, axisValueY = 0, calibration_value = 0, calibration_value2 = 0;
float axisValueYm = 0.0f, axisValueXm = 0.0f;
__IO uint32_t TimingDelay = 0;

/*Variables para el control*/
float theta1 = 0;
float theta2 = 0;
float Xerr[] = {0,0,0};
float Yerr[] = {0,0,0};
float xCoords[] = {0, 0};
float yCoords[] = {0, 0};
float theta1arr[] = {0,0};
float theta2arr[] = {0,0};

static void Servos_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t TimerPeriod = 0;
  uint16_t Channel1Pulse = 0, Channel2Pulse = 0;

  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOB, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_2);

  TimerPeriod = 60000-1;
  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
  Channel1Pulse = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
  /* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 and 2N */
  Channel2Pulse = (uint16_t) (((uint32_t) 375 * (TimerPeriod - 1)) / 1000);

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  
  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 24-1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void set_motor1(float theta1){ // theta1 -> grados
	float pulse_width;
	if(theta1<-1.5708f) pulse_width = 1516.070f;
	else if (theta1>1.5708f) pulse_width = 7369.5499f;
	else pulse_width =	3*(621.0734637f*(theta1+1.5708f) + 505.356813f); // == 10.8398*thetagrad + 505.357 (el 3 convierte los us a ciclos del CPU)
	TIM1->CCR2 = (int)pulse_width;
}

void set_motor2(float theta2){ // theta1 -> grados
	float pulse_width;
	if(theta2>1.5708f) pulse_width = 1544.032258f;
	else if (theta2<-1.5708f) pulse_width = 7350.48387f;
	else pulse_width =	3*(2428.279568f - 616.08365f*(1.5708f+theta2)); // == 10.8398*thetagrad + 505.357 (el 3 convierte los us a ciclos del CPU)
	TIM1->CCR1 = (int)pulse_width;
}

void delaybyms(unsigned int j){
	unsigned int k;
	while(j--)
    for(k=10283;k!=0;k--); //10283 7198
}

void delaybyus(unsigned int j){
	unsigned char k;
	while(j--)
    for(k=8;k!=0;k--); //8
}

/*Procedimiento que prepara salida de datos a HMI*/
void convParamSend(void){
	//Se escalan los datos a 8bits para su envío
	SetX=(int8_t)(SetpointX/0.064960f);
	SetY=(int8_t)(SetpointY/0.0484252f);
	ServoX=(int8_t)(ServomotorX*57.2958f)*4.2333f;
	ServoY=(int8_t)(ServomotorY*57.2958f)*4.2333f;
	LocX=(int8_t)(PosX/0.064960f);
	LocY=(int8_t)(PosY/0.0484252f);

	//Se ingresan al buffer de envío.
	Send_Buffer[0]=SetX;
	Send_Buffer[1]=SetY;
	Send_Buffer[2]=LocX;
	Send_Buffer[3]=LocY;
	Send_Buffer[4]=ServoX;
	Send_Buffer[5]=ServoY;

}

/* Configuración pin analógico */
static void setupGpioAnalogPC4(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ; // Pin 4 del puerto C
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; // Analógico
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* Configuración pin analógico */
static void setupGpioAnalogPF4(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 ; // Pin 4 del puerto F
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; // Analógico
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

}

/* Configuración pin alta impedancia */
static void setupGpioHZPD8(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // Pin 8 del puerto D
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // Entrada digital
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOD->PUPDR &= 0xFFFFFFF3;
	GPIOD->MODER &= 0xFFFFFFF3;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* Configuración pin alta impedancia */
static void setupGpioHZPB0(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // Pin 0 del puerto B
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //Entrada digital
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOB->PUPDR &= 0xFFFFFFF3;
	GPIOB->MODER &= 0xFFFFFFF3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* Configuración pin salida digital */
static void setupGpioPC4Output(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // Pin 4 del puerto C
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Salida digital
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/* Configuración pin salida digital */
static void setupGpioPF4Output(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // Pin 4 del puerto F
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Salida digital
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/* Configuración pin salida digital */
static void setupGpioPD8Output(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //Pin 8 del puerto D
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  //Salida digital
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* Configuración pin salida digital */
static void setupGpioPB0Output(void){
	
	GPIO_InitTypeDef      GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //Pin 0 del puerto B
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Salida digital
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* Procedimiento de calibración del ADC1 */
static void calibrateADC1(void){

  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  delaybyus(10);
  
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
	
	while(ADC_GetCalibrationStatus(ADC1) != RESET ){} //Espera a que se calibre
  calibration_value = ADC_GetCalibrationValue(ADC1); 
}
/* Procedimiento de calibración del ADC2 */
static void calibrateADC2(void){
	
  ADC_VoltageRegulatorCmd(ADC2, ENABLE);
  delaybyus(10);
 	
	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single); 
  ADC_StartCalibration(ADC2);
	
	while(ADC_GetCalibrationStatus(ADC2) != RESET ){} //Espera a que se calibre
  calibration_value2 = ADC_GetCalibrationValue(ADC2);
}

/*Configuración de ADC para el PF4 donde se leerá un electródod del panel*/
static void setupAdcPF4(void){
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);

/* Llamando funciones que configuran el pin como analógico */
	setupGpioAnalogPF4();
	calibrateADC1();  

  ADC_StructInit(&ADC_InitStructure);

	/*Configuración del ADC*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
  ADC_InitStructure.ADC_NbrOfRegChannel = 1; 
  ADC_Init(ADC1, &ADC_InitStructure);
  
/*Se lee el canal 5*/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_7Cycles5);
   
/* Se habilita el ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
/* espera por ADRDY */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
  
/* Comienza la conversión del ADC*/
  ADC_StartConversion(ADC1);   
	
}

/*Configuración de ADC para el PC4 donde se leerá un electródod del panel*/

static void setupAdcPC4(void){
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	
	/* Llamando funciones que configuran el pin como analógico */
	setupGpioAnalogPC4();
	calibrateADC2();

  ADC_StructInit(&ADC_InitStructure);

	/*Configuración del ADC*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
  ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
  ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          
  ADC_CommonInit(ADC2, &ADC_CommonInitStructure);
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
  ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
  ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
  ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_Init(ADC2, &ADC_InitStructure);
  
/*Se lee el canal 5*/
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_7Cycles5);
   
/* Se habilita el ADC2 */
	ADC_Cmd(ADC2, ENABLE);
  
/* espera por ADRDY */
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));
  
/* Comienza la conversión del ADC*/
	ADC_StartConversion(ADC2);   

}

/* Esta función se encarga de configurar los pines de la STM como VCC, GND, Alta impedancia y un analógico en que 
se leerá la posición en el eje X */
static void readXValue(void){

	setupGpioHZPB0(); //Llama Función que configura PB0 como alta impedancia
	setupGpioPC4Output(); //Llama Función que configura PC4 como VCC
	setupGpioPD8Output(); //Llama Función que configura PD8 como GND
  GPIOD->BSRR = 0x0100;
  GPIOC->BRR = 0x0010;
	setupAdcPF4(); //Llama Función que configura PF4 como pin analógico

}

/* Esta función se encarga de configurar los pines de la STM como VCC, GND, Alta impedancia y un analógico en que 
se leerá la posición en el eje Y */
static void readYValue(void){

	setupGpioHZPD8();  //Llama Función que configura PB0 como alta impedancia
	setupGpioPF4Output(); //Llama Función que configura PC4 como VCC
	setupGpioPB0Output(); //Llama Función que configura PD8 como GND
  GPIOB->BSRR = 0x0001 ;
  GPIOF->BRR = 0x0010 ;
	setupAdcPC4(); //Llama Función que configura PF4 como pin analógico

}

/*Función que rota un arreglo*/
void rotArray(float *p_arr, int arr_len){
	int i;
	for(i=arr_len-1; i>0; i--){
		p_arr[i] = p_arr[i-1];
	}
}

/*Para implementar el control diseñado en el microcontrolador se utilizaron arreglos 
que almacenan la versión actual de la posición de la bola y las mediciones anteriores, 
cada iteración del bucle principal el arreglo se recorre en una posición descartando la 
última y poniendo la nueva a la cabeza por medio de la función rotArray() que recibe un 
pointer al inicio del arreglo que se quiere rotar así como el tamaño del mismo. 
Las mediciones del panel se limitan a los valores que son físicamente posibles pues 
cuando no hay ninguna bola sobre él dan mediciones fuera de las distancias caracterizadas, 
posteriormente se rota el arreglo y se almacena la última medición a la cabeza para proceder 
a calcular el valor de la ley de control con los valores del arreglo y las ganancias calculadas 
del diseño de control, esta es directamente enviada a los servos por medio de la función set_motor1() 
el factor de resta que se ve dentro de la llamada a la función es una compensación a la inclinación 
original del sistema. Finalmente es necesario que haya unos casos que generen saturación cuando el 
ángulo enviado a los motores sea más grande que un valor máximo. El mismo procedimiento aplica para 
las leyes de control con las 2 coordenadas del panel como entradas.*/

void leyDeControlX(float xCoord){
	float ley, prom;
	if(xCoord >= 0.087) // Se limita lo que se lee el panel para no tomar en cuenta el ruido
		xCoord = xCoords[0];
	rotArray(xCoords, 2); //se rota el arreglo de las coordenadas, actual y anterior
	xCoords[0] = xCoord; //se almacena la coordenada actual
	prom = (xCoord + xCoords[1])/2; // se hace un promedio de las coordenadas para evitar picos en la lectura del panel
	rotArray(Xerr, 2); //se rota el arreglo del error, actual y anterior
	Xerr[0] = (SetpointX/100.0f)-prom; //Se calcula el error actual de X
	ley = 7.7652*Xerr[0]-6.5125*Xerr[1]; //se calcula la ley de control
	if (ley > 0.4) //se limita el ángulo enviado a los motores
		ley = 0.4;
	else if (ley <-0.4)
		ley = -0.4;
	ServomotorX=ley;
	set_motor2(-ley); //se envía el ángulo a los motores
}

void leyDeControlY(float yCoord){
	float ley, prom;
	if(yCoord >= 0.087) // Se limita lo que se lee el panel para no tomar en cuenta el ruido
		yCoord = yCoords[0];
	rotArray(yCoords, 2); //se rota el arreglo de las coordenadas, actual y anterior
	yCoords[0] = yCoord; //se almacena la coordenada actual
	prom = (yCoord + yCoords[1])/2; // se hace un promedio de las coordenadas para evitar picos en la lectura del panel
	rotArray(Yerr, 2); //se rota el arreglo del error, actual y anterior
	Yerr[0] = (SetpointY/100.0f)-prom; //Se calcula el error actual de Y
	ley = 7.7652*Yerr[0]-6.5125*Yerr[1]; //se calcula la ley de control
	if (ley > 0.4) //se limita el ángulo enviado a los motores
		ley = 0.4;
	else if (ley <-0.4)
		ley = -0.4;
	ServomotorY=ley;
	set_motor1(-ley+0.013); //se envía el ángulo a los motores
}

int main(void)
{
	//Se configuran los Timers e interrupciones necesarias
	//para el funcionamiento del puerto USB
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init(); //Se inicializa la comunicación USB
	Servos_Config();	//Se inicializan los movimientos de los Servomotores
	set_motor1(0.0f); //0.08726646f
	set_motor2(0.0f);

 if (SysTick_Config(SystemCoreClock / 1000000))
  { 
    /* Capturar error en STM32 */ 
    while (1)
    {}
  }

  /* Loop repetido infinitamente */
  while (1)
  {
		
		/*Leyendo Eje X de Panel */
		readXValue();
    /* Esperar hasta final de conversion */
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		
    /* Guardando valor leido de Eje X */
    ADC1ConvertedValue =ADC_GetConversionValue(ADC1);
    
    /* Calculando la posición correspondiente según caracterización */
		axisValueXm = ((ADC1ConvertedValue-191.87f)/22248.0f)-0.083f;
		
		//Se configuran ADC para siguiente lectura
		ADC_DeInit (ADC1);
		ADC_DeInit (ADC2);
		
		//Se calcula el Control del Eje X según la posicion de la pelota
		delaybyus(500);
		leyDeControlX(axisValueXm);

		/*Leyendo Eje Y del Panel */
		readYValue();
		/* Esperar hasta el final de conversión */
		while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
		
    /* Guardando valor ledio de Eje Y */
		ADC1ConvertedValue2 =ADC_GetConversionValue(ADC2);
    
    /* Calculando la posición correspondiente según caracterización */
		axisValueYm = ((ADC1ConvertedValue2-385.7f)/27229.0f)-0.0625f;

		//Se configuran ADC para siguiente lectura
		ADC_DeInit (ADC1);
		ADC_DeInit (ADC2);
		
		//Se calcula el Control del Eje Y según la posision de la pelota
    leyDeControlY(axisValueYm);
		delaybyus(500);
	
		//Se verifica estado del USB
    if (bDeviceState == CONFIGURED)
    {
			//Se reciben datos de nuevos SetPoint
			CDC_Receive_DATA();

			/*Se convierten los datos de 8 bits a las 
			coordenadas correspondientes de cada eje*/
			SetpointX=(int8_t)Receive_Buffer[0];
			SetpointX=(float)Receive_Buffer[0];
			SetpointX=(float)SetpointX*0.064960f;
			SetpointY=(int8_t)Receive_Buffer[1];
			SetpointY=(float)Receive_Buffer[1];
			SetpointY=(float)SetpointY*0.0484252f;
			
			
			//Se escalan datos de M a CM para posición de la pelota
			PosX=(axisValueXm)*100.0f;
			PosY=(axisValueYm)*100.0f;
			
			//Se preparan datos para ser enviados al HMI
			convParamSend();
			
      /*Se verifica si la recepción de datos fue correcta */
      if(Receive_length!=0){
      if (packet_sent == 1)
			//Se envían los datos al HMI
      CDC_Send_DATA ((unsigned char*)Send_Buffer,6);
      Receive_length = 0;
		  }    
  }
 }
}