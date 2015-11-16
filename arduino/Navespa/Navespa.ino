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
// 12/11/2015 - 01:47
//
//=================================================================================================
// Control de Mouse mediante el Roll y el Pitch medidos por un IMU.
// Se utiliza un filtro complementario para combinar la señal del acelerometro y giroscopio.
//
//=================================================================================================
// TODO LIST
// - Checksum para comunicacion?
// - Datos a enviar
// - data_send() en Output.ino
//
//=================================================================================================
// Terminologia:
// - IMU: Unidad de Medicion Inercial
//
//=================================================================================================

//-------------------------------------------------------------------------------------------------
// DEPENDENCIAS Y DEFINICIONES
//-------------------------------------------------------------------------------------------------

#include <Wire.h>

// Pines
#define STATUS_LED 13
#define BOTON_1 6 // Boton Izquierdo
#define BOTON_2 5 // Boton Derecho
#define BOTON_CONFIG 4 // Boton de Configuracion

// Para Debug
#define SERIAL_PRINT 1 // Para visualizar los datos por el puerto serie
#define SERIAL_SEND 0 // Si envía los datos por el puerto serie al Wixel

#define PRINT_GYRO 1 // Imprime los valores del gyro
#define PRINT_ACC 1 // Imprime los valores del acc
#define PRINT_RGYRO 0 // Imprime los valores del vector RwGyro
#define PRINT_REST 0 // Imprime los valores del vector RwEst

// Constantes
#define PI 3.14159265358979f

//-------------------------------------------------------------------------------------------------
// IMU
//-------------------------------------------------------------------------------------------------

// Variables Globales
const int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; // (x,y,z) de (Giro, Accel y Mag)
const int GRAVITY = 256; // Equivalente a 1G en los datos del acelerometro

// Sensibilidad Acelerometro [g/LSB] Ver datasheet
const float sen_ax = 0.004;
const float sen_ay = 0.004;
const float sen_az = 0.004;

// Sensibilidad Giroscopio [dps/bit] Ver datasheet
const float sen_gx = 0.07;
const float sen_gy = 0.07;
const float sen_gz = 0.07;

// Metodos de conversion
#define ToRad(x) ((x)*0.01745329252) // (x)*pi/180
#define ToDeg(x) ((x)*57.2957795131) // (x)*180/pi

// Valores de Lectura
int AN[6]; // Lecturas previas de acelerometro y giroscopio
int AN_OFFSET[6] = {0,0,0,0,0,0}; // Offsets de Lecturas

int ax; // Acelerometro
int ay;
int az;

int gx; // Giroscopio
int gy;
int gz;

//-------------------------------------------------------------------------------------------------
// PARAMETROS Y VARIABLES DE CONTROL
//-------------------------------------------------------------------------------------------------

// Vectores utilizados en el control
float RwEst[3] = {0,0,0}; // Vector R combinando RwAcc y RwGyro. Salida.
float RwAcc[3] = {0,0,0}; // Vector R obtenido por Acc.
float RwGyro[3] = {0,0,0}; // Vector R estimado del Gyro.

float Awz[2] = {0,0}; // Angulo de la Proyeccion de R sobre los planos <XZ> y <YZ> con el eje Z. [deg]

float v_acc[3] = {0,0,0}; // Aceleracion (x,y,z) [G]
float v_gyro[3] = {0,0,0}; // Giroscopio (x,y,z) [deg/s]

// Variables de Control y Tiempo
float DT_S = 0.01; //Tiempo de integracion en Segundos. Un calculo cada 10 ms.
unsigned long timer = 0;
unsigned long timer_anterior = 0;

boolean firstSample = true;

int weight_gyro = 10; // Peso del Gyro respecto al Acc. Valores entre 5 y 20.

//-------------------------------------------------------------------------------------------------
// COMUNICACION CON WIXEL
//-------------------------------------------------------------------------------------------------

byte dato_header = 0x80;
int dato_x = 0;
int dato_y = 0;
byte dato_botones;
//TODO Checksum?

//=================================================================================================
// SETUP
//=================================================================================================

void setup()
{ 
  //-------------------------------------------------------------------------------------------------
  // INICIO PINES Y COMUNICACION
  //-------------------------------------------------------------------------------------------------
  
  Serial.begin(19200);
  I2C_Init();
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BOTON_1, INPUT);
  pinMode(BOTON_2, INPUT);
  pinMode(BOTON_CONFIG, INPUT);
  
  //-------------------------------------------------------------------------------------------------
  // CALIBRACION
  //-------------------------------------------------------------------------------------------------
  
  digitalWrite(STATUS_LED,HIGH);
  IMU_Init();
  IMU_Calibration(128);
  delay(20);
  digitalWrite(STATUS_LED,LOW);
  
  #if SERIAL_PRINT == 1
  Serial.println("- Mouse3D -");
  Serial.println("Offsets Calculados:");
  for(int i=0; i<3; i++)
  {
    Serial.print("\t(");
    for(int j=0; j<3; j++)
	{
	  Serial.print(AN_OFFSET[3*i+j]);
	  Serial.print(", ");
	}
    Serial.print(")");
  }
  #endif
  
  delay(2000);
  digitalWrite(STATUS_LED,HIGH);
  
  firstSample = true;
  timer = millis();
}

//=================================================================================================
// LOOP
//=================================================================================================

void loop()
{
  if((millis()-timer)>=(DT_S*1000)) // Corre cada DT_S [segundos]
  {
    timer_anterior = timer;
    timer = millis();
	
    if(timer>timer_anterior)
    {
      DT_S = (timer-timer_anterior)/1000.0; // Tiempo de lazo real. Integracion del giroscopio.
    }
    else
    {
      DT_S = 0;
    }
    
    //-------------------------------------------------------------------------------------------------
    // FILTRO COMPLEMENTARIO
    //-------------------------------------------------------------------------------------------------
	
	getRwEst();
    
    //-------------------------------------------------------------------------------------------------
    // COMUNICACION
    //-------------------------------------------------------------------------------------------------
    
    #if SERIAL_PRINT == 1
    data_print();
    #endif
	
	// TODO datos
    #if SERIAL_SEND == 1
    //dato_x = -pitch_rate;
    //dato_y = yaw_rate;
    //dato_botones = 0x00;
    /*
    if(digitalRead(BOTON_1))
      dato_botones = 0x01;
    if(digitalRead(BOTON_2))
      dato_botones = 0x02;
    */
    data_send();
    #endif
  }
}
