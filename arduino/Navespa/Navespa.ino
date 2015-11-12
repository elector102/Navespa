//=================================================================================================
// Navespa
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en https://github.com/pololu/minimu-9-ahrs-arduino
//
// Ultimo update: 29/09/2015 - 17:40
//
//=================================================================================================
// Control de Mouse mediante el Roll y el Pitch medidos por un IMU.
//
//=================================================================================================
// Terminologia:
// - IMU: Unidad de Medicion Inercial
// - DCM: Matriz de Cosenos Directores
// - Angulos de Euler:
//   - Yaw: Angulo en el plano <xy> (Relacionado con el eje Z del giroscopio)
//   - Pitch: Angulo en el plano <xz> (Relacionado con el eje Y del giroscopio)
//   - Roll: Angulo en el plano <yz> (Relacionado con el eje X del giroscopio)
//
//=================================================================================================
// TODO
// * Comunicacion Serie (x, y, botones)
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
#define PRINT_EULER 0 // Si imprime o no los angulos de Euler
#define PRINT_GYRO 1 // Si imprime o no los valores del gyro
#define PRINT_ACC 1
#define PRINT_RATE 0 // Si imprime o no el cambio en los angulos de Euler
#define PRINT_DCM 0 // Si imprime o no la matriz DCM

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

#define M_X_MIN -195 // Para calibracion del Magnetometro
#define M_Y_MIN -261
#define M_Z_MIN -171
#define M_Y_MAX 226
#define M_X_MAX 203
#define M_Z_MAX 239

// Metodos de conversion
#define ToRad(x) ((x)*0.01745329252) // (x)*pi/180
#define ToDeg(x) ((x)*57.2957795131) // (x)*180/pi

#define scaled_gx(x) ((x)*ToRad(sen_gx)) // De bits a [rad/s]
#define scaled_gy(x) ((x)*ToRad(sen_gy)) // De bits a [rad/s]
#define scaled_gz(x) ((x)*ToRad(sen_gz)) // De bits a [rad/s]

// Valores de Lectura
int AN[6]; // Lecturas previas de acelerometro y giroscopio
int AN_OFFSET[6] = {0,0,0,0,0,0}; // Offsets de Lecturas

int ax; // Acelerometro
int ay;
int az;

int gx; // Giroscopio
int gy;
int gz;

int mx; // Magnetometro
int my;
int mz;

float c_mx; // Magnetometro con correciones
float c_my;
float c_mz;

float m_heading; // Direccion que indica el Magnetometro

//-------------------------------------------------------------------------------------------------
// PARAMETROS Y VARIABLES DE CONTROL
//-------------------------------------------------------------------------------------------------

// Vectores utilizados en el control
float v_acc[3] = {0,0,0}; // Aceleracion (x,y,z) [G]
float v_gyro[3] = {0,0,0}; // Giroscopio (x,y,z) [rad/s]

float omega_gyro[3] = {0,0,0}; // v_gyro corregido
float omega_p[3] = {0,0,0}; // Correccion proporcional
float omega_i[3] = {0,0,0}; // Correccion integral
float omega[3] = {0,0,0};

float errorRollPitch[3] = {0,0,0};
float errorYaw[3] = {0,0,0};

// Variables de Control
float DT_S = 0.02; //Tiempo de integracion en Segundos para el algoritmo DCM.

long timer = 0;
long timer_anterior;

unsigned int counter = 0; // Para calcular Heading

// Constantes Proporcional e Integral
const float KP = 0.05; // Original: 0.02
const float KI = 0.0; // Original: 0.00002

const float KP_YAW = 0.0; // Original: 1.2
const float KI_YAW = 0.0; // Original: 0.00002

// Matrices
float DCM_Matrix[3][3] = {
  {1,0,0},
  {0,1,0},
  {0,0,1}
};

float Update_Matrix[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};

float Temporary_Matrix[3][3] = {
  {0,0,0},
  {0,0,0},
  {0,0,0}
};

//-------------------------------------------------------------------------------------------------
// ANGULOS DE EULER
//-------------------------------------------------------------------------------------------------

float roll; // Angulo de giro sobre el eje X
float roll_anterior;
float roll_rate;

float pitch; // Angulo de giro sobre el eje Y
float pitch_anterior;
float pitch_rate;

float yaw; // Angulo de giro sobre el eje Z
float yaw_anterior;
float yaw_rate;

int sen_rate = 100; // Sensibilidad

//-------------------------------------------------------------------------------------------------
// COMUNICACION CON WIXEL
//-------------------------------------------------------------------------------------------------

byte dato_header = 0x80;
int dato_x = 0;
int dato_y = 0;
byte dato_botones;

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
    
  timer=millis();  
  counter=0;
}

//=================================================================================================
// LOOP
//=================================================================================================

void loop()
{
  if((millis()-timer)>=(DT_S*1000)) // Corre cada DT_S [segundos]
  {
    counter++;
    timer_anterior = timer;
    timer=millis();
	
    if (timer>timer_anterior)
      DT_S = (timer-timer_anterior)/1000.0; // Tiempo de lazo real. Usado en el DCM como integracion del giroscopio.
    else
      DT_S = 0;
    
    //-------------------------------------------------------------------------------------------------
    // PRIMERA PARTE - ALGORITMO DCM
    //-------------------------------------------------------------------------------------------------
	
    // Adquisicion de Datos
    Read_Gyro();
    Read_Accel();

    if (counter > 5)
    {
      counter = 0;
      Read_Compass();
      Compass_Heading();
    }
    
    // Calculos
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    
    //-------------------------------------------------------------------------------------------------
    // SEGUNDA PARTE - COMUNICACION
    //-------------------------------------------------------------------------------------------------
    
    #if SERIAL_PRINT == 1
    data_print();
    #endif
	
    #if SERIAL_SEND == 1
    //dato_x = (byte)(pitch_rate);
    //dato_y = (byte)(-yaw_rate);
    dato_x = -pitch_rate;
    dato_y = yaw_rate;
    dato_botones = 0x00;
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
