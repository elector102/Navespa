//=================================================================================================
// Navespa_rx_test
//
// Emder, Fabricio
// Mas, German Emilio
//
// Testeo a ver que recibe el wixel receptor mediante el puerto Serie.
//
// dato(0) = Cabecera (0xF0)
// dato(1) = Mouse X
// dato(2) = Mouse Y
// dato(3) = Botones
//
//=================================================================================================

#include <wixel.h>
#include <usb.h>
#include <usb_com.h>
#include <usb_hid.h>
#include <radio_queue.h>
#include <stdio.h>


//-------------------------------------------------------------------------------------------------

uint8 dato[5];
uint16 oldtime = 0;
uint8 dataBytesLeft = 0;
uint8 dataBytesReceived;
uint8 XDATA response[32];

//-------------------------------------------------------------------------------------------------

void updateLeds()
{
    usbShowStatusWithGreenLed();
}

//-------------------------------------------------------------------------------------------------

void rxMouseState(void)
{
    uint8 XDATA * rxBuf;
    static uint8 indice_inicio = 0;
    static uint8 indice_orden = 0;
    static uint8 i = 0;
	
    if (rxBuf = radioQueueRxCurrentPacket())
    {
		// De los cuatro datos leidos, busca la cabecera
        for(i=0; i<5; i++)
            if(rxBuf[i]==0x80) {indice_inicio = i;}
        
		// En base al indice de la cabecera, ordena el resto de los datos
        for(i=0; i<5; i++)
        {
            indice_orden = indice_inicio+i;
            if(indice_orden >= 5) {indice_orden = 0;}
            dato[i]=rxBuf[indice_orden];
        }
		
        //usbHidMouseInput.x = dato[1];
        //usbHidMouseInput.y = dato[2];
        //usbHidMouseInput.buttons = dato[3];
		
        //usbHidMouseInputUpdated = 1;
        radioQueueRxDoneWithPacket();
    }
}

void sendByte()
{
    static uint32 time_old = 0;
    uint32 time;
    uint8 responseLength;
    
    time = getMs();
    
    if(time - time_old > 10)
    {
        time_old = time;
        responseLength = sprintf(response, "%X\t%d\t%d\t%d\tT=0x%04x%04x\r\n",
            dato[0], dato[1], dato[2], dato[3], (uint16)(time >> 16), (uint16)time);
    
        usbComTxSend(response, responseLength);
    }
    
}

void processBytesFromUsb()
{
    uint8 bytesLeft = 1;
    while(bytesLeft && usbComTxAvailable() >= sizeof(response))
    {
        sendByte();
        bytesLeft--;
    }
}

//-------------------------------------------------------------------------------------------------

void main() 
{
    // Inicializaciones
    systemInit();
    usbInit();
    radioQueueInit();
    
    // Lazo de Servicios
    while(1)
    {
        updateLeds();
        boardService();
        usbComService();
        rxMouseState();
        processBytesFromUsb();
        //usbHidService();
    }
}
