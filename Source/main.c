#include "main.h"
#include "stm32f30x.h"

extern __IO uint8_t Receive_Buffer[6];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
uint8_t Send_Buffer[6];
uint32_t packet_sent=1;
uint32_t packet_receive=1;

__IO uint16_t  ADC1ConvertedValue = 0, ADC1ConvertedValue2 = 0, axisValueX = 0, axisValueY = 0, calibration_value = 0, calibration_value2 = 0;
__IO uint32_t TimingDelay = 0;

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
	
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

	void convParamSend(void){
	SetX=(uint8_t)(SetpointX/0.048046875f);
	SetY=(uint8_t)(SetpointY/0.064453125f);
	ServoX=(uint8_t)(ServomotorX);
	ServoY=(uint8_t)(ServomotorY);
	LocX=(uint8_t)(PosX/0.048046875f);
	LocY=(uint8_t)(PosY/0.064453125f);
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
