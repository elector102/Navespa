//=================================================================================================
// navespa_rx.c
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en:
// - wireless_tilt_mouse_receiver app
//
// Ultimo update: 16/11/2015 - 10:40
//
//=================================================================================================
// Aplicacion Principal del Wixel Receptor.
// Recibe la señal inalambrica del Wixel Transmisor.
// Envía al mouse la velocidad en x e y, ademas de los botones presionados.
//
//=================================================================================================
// dato(0) = Cabecera (0x80)
// dato(1) = Mouse X
// dato(2) = Mouse Y
// dato(3) = Botones
//
//=================================================================================================

//=================================================================================================
// DEPENDENCIAS Y DEFINICIONES
//=================================================================================================

#include <wixel.h>
#include <usb.h>
#include <usb_hid.h>
#include <radio_queue.h>

// Para debug
#define DEBUG 0

// Variables Globales
uint8 dato[5];
uint16 timer_anterior = 0;
uint8 dataBytesLeft = 0;
uint8 dataBytesReceived;
uint8 XDATA response[32];

//=================================================================================================
// FUNCIONES AUXILIARES
//=================================================================================================

// Control de LEDs del Wixel
void updateLeds()
{
    usbShowStatusWithGreenLed();
}

// Rutina de Recepcion
void rxMouseState(void)
{
    static uint32 timer_anterior = 0;
    uint32 timer;
    
    uint8 XDATA * rxBuf;
    static uint8 indice_inicio = 0;
    static uint8 indice_orden = 0;
    static uint8 i = 0;
    
    if (rxBuf = radioQueueRxCurrentPacket())
    {
	// De los cuatro datos leidos, busca la cabecera
        for(i=0; i<5; i++)
        {
            if(rxBuf[i]==0x80) {indice_inicio = i;}
        }
        
	// En base al indice de la cabecera, ordena el resto de los datos
        for(i=0; i<5; i++)
        {
            indice_orden = indice_inicio+i;
            if(indice_orden >= 5) {indice_orden = 0;}
            dato[i]=rxBuf[indice_orden];
        }
        
        radioQueueRxDoneWithPacket();
        
        // Modificacion del estado del Mouse
        timer = getMs();
        if(timer - timer_anterior > 20)
        {
            timer_anterior = timer;
            usbHidMouseInput.x = dato[1];
            usbHidMouseInput.y = dato[2];
            if(dato[3]==0x01)
                usbHidMouseInput.buttons = MOUSE_BUTTON_LEFT;
            if(dato[3]==0x02)
                usbHidMouseInput.buttons = MOUSE_BUTTON_RIGHT;
            
            usbHidMouseInputUpdated = 1;
        }
    }
}

//=================================================================================================
// MAIN
//=================================================================================================

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
        usbHidService();
        rxMouseState();
    }
}
