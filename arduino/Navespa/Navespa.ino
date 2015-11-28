//=================================================================================================
// Navespa
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en:
// - https://github.com/pololu/minimu-9-ahrs-arduino
// - http://starlino.com/imu_guide.html
//
// 07:31 PM 28/11/2015
//
//=================================================================================================
// Control de Mouse mediante la inclinación de un IMU.
// Se utiliza un filtro complementario para combinar la señal del acelerometro y giroscopio.
//
// Los ejes del IMU deben apuntar a las siguientes direcciones:
// - Eje x: Hacia adelante.
// - Eje y: Hacia abajo.
// - Eje z: Hacia la izquierda.
// La inclinacion de los siguientes ejes corresponde a la velocidad del mouse:
// - Eje x: Eje Vertical.
// - Eje z: Eje Horizontal.
//
//=================================================================================================

//-------------------------------------------------------------------------------------------------
// DEPENDENCIAS Y DEFINICIONES
//-------------------------------------------------------------------------------------------------

#include <Wire.h>

//Constantes
#define PI 3.14159265358979f
#define LED_LOW HIGH //El led se apaga con LOW
#define LED_HIGH LOW //El led se prende con HIGH

//Pines
#define STATUS_LED 16
#define BOTON_UP 2 //Boton UP para incrementar parametros
#define BOTON_DOWN 4 //Boton DOWN para decrementar parametros
#define BOTON_MODE 3 //Boton MODE para Configuracion
#define BOTON_CLICK A3 //Boton Sensor Presion (Click)

//Para Debug
#define SERIAL_PRINT 0 //Para usar Serial.print
#define SERIAL_SEND 1 //Para enviar al Wixel

#define PRINT_GYRO 0 //Valores Giroscopio
#define PRINT_ACC 0 //Valores Acelerometro
#define PRINT_RGYRO 0 //Vector RwGyro
#define PRINT_REST 0 //Vector RwEst
#define PRINT_MOUSE 0 //Datos del Mouse
#define PRINT_LIMITES 0 //Limites de Calibracion

//-------------------------------------------------------------------------------------------------
// IMU
//-------------------------------------------------------------------------------------------------

//Variables Globales
const int SENSOR_SIGN[6] = {1,1,1,-1,-1,-1}; // (x,y,z) de (Giro, Accel)
const int GRAVITY = 256; // Equivalente a 1G en los datos del acelerometro

//Sensibilidad Acelerometro [g/LSB] Ver datasheet
const float sen_ax = 0.004;
const float sen_ay = 0.004;
const float sen_az = 0.004;

//Sensibilidad Giroscopio [dps/bit] Ver datasheet
const float sen_gx = 0.07;
const float sen_gy = 0.07;
const float sen_gz = 0.07;

//Metodos de conversion
#define ToRad(x) ((x)*0.01745329252) // (x)*pi/180
#define ToDeg(x) ((x)*57.2957795131) // (x)*180/pi

//Valores de Lectura
int AN[6]; //Lecturas previas de acelerometro y giroscopio
int AN_OFFSET[6] = {0,0,0,0,0,0}; //

int ax; //Acelerometro
int ay;
int az;
float vAcc[3] = {0,0,0}; //Aceleracion (x,y,z) [G]

int gx; //Giroscopio
int gy;
int gz;
float vGyro[3] = {0,0,0}; //Giroscopio (x,y,z) [deg/s]

//-------------------------------------------------------------------------------------------------
// PARAMETROS Y VARIABLES DE CONTROL
//-------------------------------------------------------------------------------------------------

//Vectores utilizados en el control
float RwAcc[3] = {0,0,0}; //Vector R obtenido por Acc.
float RwGyro[3] = {0,0,0}; //Vector R estimado del Gyro.
float RwEst[3] = {0,0,0}; //Vector R combinando RwAcc y RwGyro. Salida.
int RwEstSign[3] = {1,1,-1}; //Para corregir el signo de RwEst al usarlo.
float Awz[2] = {0,0}; //Angulo de la Proyeccion de R sobre los planos <XZ> y <YZ> con el eje Z. [deg]

//Configuracion Filtro Complementario
boolean firstSample = true;
int gyroWeight = 2; //Peso del Gyro respecto al Acc. Valores entre 5 y 20. (Pero uso 2!)

//Calibracion de Datos
float lim[4] = {0,0,0,0}; //Limites de Calibracion. Izquierda, Derecha, Atras, Adelante.
float zm[4] = {0,0,0,0}; //Zonas muertas. Izquierda, Derecha, Atras, Adelante.
float zmPorcentaje = 0.3; //Del limite, que porcentaje es zona muerta.
int velocidad = 20;
int vMax = 127; //Limite de Velocidad

//Flags de Calibracion
boolean configLim = true;
boolean configZM = false;
boolean configVel = false;

//Contadores
//Para evitar rebotes en las llaves se usan contadores que decrecen en cada loop (a DT_S segundos)
//Cuando el contador es cero el boton es leido
int contConfig = 0; //Delay entre MODE.
int contBoton = 0; //Delay entre UP y DOWN.

//Variables de Control y Tiempo
float DT_S = 0.02; //Tiempo de integracion en Segundos. Un calculo cada 20 ms.
unsigned long timer = 0;
unsigned long timerAnterior = 0;

//-------------------------------------------------------------------------------------------------
// COMUNICACION CON WIXEL
//-------------------------------------------------------------------------------------------------

byte datoHeader = 0x80;
char datoX = 0;
char datoY = 0;
byte datoBotones = 0x00; //Bit 0 es el boton izdo. Bit 1 el derecho.

//=================================================================================================
// SETUP
//=================================================================================================

void setup()
{
  //-----------------------------------------------------------------------------------------------
  // INICIO DE PINES Y COMUNICACION
  //-----------------------------------------------------------------------------------------------
  
  Serial.begin(19200);
  I2C_Init();
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BOTON_UP, INPUT);
  pinMode(BOTON_DOWN, INPUT);
  pinMode(BOTON_MODE, INPUT);
  
  //-----------------------------------------------------------------------------------------------
  // CALIBRACION
  //-----------------------------------------------------------------------------------------------
  
  digitalWrite(STATUS_LED,LED_HIGH);
  IMU_Init();
  IMU_Calibration(128);
  delay(20);
  digitalWrite(STATUS_LED,LED_LOW);
  
  #if SERIAL_PRINT == 1
  delay(1000);
  Serial.println("- Mouse3D -");
  Serial.println("Offsets Calculados:");
  Serial.print("Acc: (");
  for(int i=3; i<5; i++)
  {
    Serial.print(AN_OFFSET[i]);
    Serial.print(", ");
  }
  Serial.print(AN_OFFSET[5]);
  Serial.print(")\tGyro: (");
  for(int i=0; i<2; i++)
  {
    Serial.print(AN_OFFSET[i]);
    Serial.print(", ");
  }
  Serial.print(AN_OFFSET[2]);
  Serial.println(")");
  #endif
  
  delay(2000);
  
  digitalWrite(STATUS_LED,LED_HIGH);
  firstSample = true;
  configLim = true;
  configZM = false;
  configVel = false;
}

//=================================================================================================
// LOOP
//=================================================================================================

void loop()
{
  timer = millis();
  if((timer-timerAnterior)>=(DT_S*1000)) //Corre cada DT_S [segundos]
  {
    //Actualizacion de Variables
    timerAnterior = timer;
    datoBotones = 0x00;
    
    //Decrementa los contadores
    if(contConfig>0) {contConfig--;}
    if(contBoton>0) {contBoton--;}
    
    if((contConfig==0)&&(contBoton==0))
    {
      //Si se esta configurando algo, el led esta prendido.
      if(configLim || configZM || configVel)
      {
        digitalWrite(STATUS_LED, LED_HIGH);
      }
      else
      {
        digitalWrite(STATUS_LED, LED_LOW);
      }
    }

    getRwEst(); //Lectura del IMU y aplicacion del Filtro
    
    //-----------------------------------------------------------------------------------------------
    // CALIBRACION
    //-----------------------------------------------------------------------------------------------
    
    //Calibracion de Limites
    if(configLim)
    {
      setLimites();
      
      //Termina con la calibracion de Limites
      if(digitalRead(BOTON_MODE)==HIGH)
      {
        configLim = false;
        setZonaMuerta();
        contConfig = 10;
        digitalWrite(STATUS_LED, LED_LOW);
      }
    }
    else
    {
      //Limites ya calibrados - Chequea el boton
      if((contConfig==0)&&(digitalRead(BOTON_MODE)==HIGH))
      {
        //Verifica que configurar
        if(!configZM)
        {
          //Configurar Zona Muerta
          configZM = true;
          contConfig = 10;
          digitalWrite(STATUS_LED, LED_LOW);
        }
        else
        {
          if(!configVel)
          {
            //Configurar Velocidad
            configVel = true;
            contConfig = 10;
            digitalWrite(STATUS_LED, LED_LOW);
          }
          else
          {
            //Quita la configuracion
            configZM = false;
            configVel = false;
            contConfig = 10;
            digitalWrite(STATUS_LED, LED_LOW);
          }
        } //Fin configZM o configVel
      } //Fin verificacion
    } //Fin configLim
    
    //Chequeo de Botones UP/DOWN para calibracion
    if((digitalRead(BOTON_UP)==HIGH)||(digitalRead(BOTON_DOWN)==HIGH))
    {
      if((configZM)&&(contBoton==0))
      {
        if((configVel)&&(contBoton==0))
        {
          if(digitalRead(BOTON_UP)==HIGH)
          {
            velocidad +=10;
            velocidad = constrain(velocidad, 0, vMax);
            contBoton = 10;
            digitalWrite(STATUS_LED, LED_LOW);
          }
          else
          {
            velocidad -= 10;
            velocidad = constrain(velocidad, 0, vMax);
            contBoton = 10;
            digitalWrite(STATUS_LED, LED_LOW);
          }
        }
        else
        {
          if(digitalRead(BOTON_UP)==HIGH)
          {
            zmPorcentaje += 0.05;
            zmPorcentaje = constrain(zmPorcentaje, 0, 1);
            contBoton = 10;
            digitalWrite(STATUS_LED, LED_LOW);
          }
          else
          {
            zmPorcentaje -= 0.05;
            zmPorcentaje = constrain(zmPorcentaje, 0, 1);
            contBoton = 10;
            digitalWrite(STATUS_LED, LED_LOW);
          }
        }
      }
    }
    
    //Calcula datos del mouse solo despues de haber configurado los limites
    if(!configLim)
    {
      setMouseData();
    }

    //-----------------------------------------------------------------------------------------------
    // COMUNICACION
    //-----------------------------------------------------------------------------------------------
    
    #if SERIAL_PRINT == 1
    data_print();
    #endif
    
    #if SERIAL_SEND == 1
    data_send();
    #endif
  }
}
