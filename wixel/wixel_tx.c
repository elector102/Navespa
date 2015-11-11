//=================================================================================================
// Navespa_TX
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en wireless_tilt_mouse app
//
// Ultimo update: 29/09/2015 - 20:30
//
//=================================================================================================
// Aplicacion Principal del Wixel Transmisor.
//
//=================================================================================================

#include <wixel.h>
#include <usb.h>
#include <usb_com.h>
#include <usb_hid_constants.h>
#include <radio_queue.h>
#include <uart1.h>

#define TX_INTERVAL 10 // Tiempo entre Transmisiones [ms]

//-------------------------------------------------------------------------------------------------

int32 CODE param_invertir_x = 0;
int32 CODE param_invertir_y = 0;
int32 CODE param_uart_baud_rate = 19200;

uint8 dato_uart[4]; // Dato que recibe por serie
uint8 dato[4]; // Dato ordenado para enviar

//-------------------------------------------------------------------------------------------------

void updateLeds()
{
    usbShowStatusWithGreenLed();
    LED_YELLOW(vinPowerPresent());
}

//-------------------------------------------------------------------------------------------------

void txMouseState()
{
    static uint8 indice_inicio = 0;
    static uint8 indice_orden = 0;
    static uint8 i = 0;
    static uint8 lastTx = 0;

    uint8 XDATA *txBuf = radioQueueTxCurrentPacket();
    
    if (txBuf != 0)
    //if ((uint8)(getMs() - lastTx) > TX_INTERVAL && (txBuf != 0))
    {
        while(uart1RxAvailable())
        {
            dato_uart[0] = uart1RxReceiveByte();
            dato_uart[1] = uart1RxReceiveByte();
            dato_uart[2] = uart1RxReceiveByte();
            dato_uart[3] = uart1RxReceiveByte();
        }
        
	// De los cuatro datos leidos, busca la cabecera
        for(i=0; i<4; i++)
            if(dato_uart[i] == 0x80) {indice_inicio = i;}
        
        // En base al indice de la cabecera, ordena el resto de los datos
        for(i=0; i<4; i+=1)
        {
            indice_orden = indice_inicio+i;
            if(indice_orden >= 4) {indice_orden = 0;}
            dato[i]=dato_uart[indice_orden];
        }
		
	// Una vez ordenado, dato posee lo siguiente:
	// dato(0) = Cabecera (0xF0)
	// dato(1) = Mouse X
	// dato(2) = Mouse Y
	// dato(3) = Botones
        //dato[0]=-128;
        //dato[1]=1;
        //dato[2]=1;
        //dato[3]=3;
	// Construye el paquete a enviar por radio.
        txBuf[0] = 4; // Cantidad de bytes del paquete.
        txBuf[1] = 0x80;
        txBuf[2] = 0xD1;
        txBuf[3] = 0xD2;
	txBuf[4] = 0xD3; // Las constantes HID son MOUSE_BUTTON_LEFT y MOUSE_BUTTON_RIGHT
        //txBuf[1] = dato[0];
        //txBuf[2] = dato[1];
        //txBuf[3] = dato[2];
	//txBuf[4] = dato[3]; // Las constantes HID son MOUSE_BUTTON_LEFT y MOUSE_BUTTON_RIGHT
        radioQueueTxSendPacket();
        
        for(i=0; i<4; i+=1)
        {
            dato_uart[i] = 0;
            dato[i] = 0;
	}
        //lastTx = getMs(); // Comentar si no se usa el lazo con tiempo
    }
}

//-------------------------------------------------------------------------------------------------

void main()
{
    // Inicializaciones
    systemInit();
    usbInit();
    radioQueueInit();
    uart1Init();
    uart1SetBaudRate(param_uart_baud_rate);

    while(1)
    {
	// Lazo de Servicios
        updateLeds();
        boardService();
        usbComService();
        txMouseState();
    }
}
