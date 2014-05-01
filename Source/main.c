#include "main.h"
#include "stm32f30x.h"

extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
uint8_t Send_Buffer[64];
uint32_t packet_sent=1;
uint32_t packet_receive=1;

static __IO uint32_t TimingDelay = 0;
	
int main( void )
{
	float arreglo[3] = {1.25,0.5,0.25};
	
	Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();

  while (1)
  {
		
    if (bDeviceState == CONFIGURED)
    {
      CDC_Receive_DATA();
      if (Receive_length  != 0)
      {
        if (packet_sent == 1)
				CDC_Send_DATA ((unsigned char*)arreglo,12);
        Receive_length = 0;
      }
    }
  }
}

void Delay(__IO uint32_t nTicks)
{
	
	TimingDelay = nTicks;
	while(TimingDelay != 0);
}
