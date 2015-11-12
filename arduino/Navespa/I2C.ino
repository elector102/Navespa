//=================================================================================================
// Navespa
// - I2C.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en https://github.com/pololu/minimu-9-ahrs-arduino
//
//=================================================================================================
// Metodos relevantes al I2C, inicializacion, calibracion y lectura del IMU.
//
//=================================================================================================
// Funciones presentes:
// void I2C_Init();
// void IMU_Init();
// void Read_Gyro();
// void Read_Accel();
// void Read_Compass();
// void IMU_Calibration(int n);
//
//=================================================================================================

#include <L3G.h>
#include <LSM303.h>

L3G gyro;
LSM303 compass;

// Inicializacion del I2C
void I2C_Init()
{
  Wire.begin();
}

// Inicializacion del IMU
void IMU_Init()
{
  // Giroscopio
  gyro.init();
  gyro.enableDefault();
  gyro.writeReg(L3G::CTRL_REG4, 0x20); // FS=10 (+/-2000 dps full scale)
  gyro.writeReg(L3G::CTRL_REG1, 0x0F); // Normal power mode; XYZ enabled; ODR 95 Hz; Cut-Off 12.5
  
  // Acelerometro
  compass.init();
  compass.enableDefault();
  compass.writeReg(LSM303::CTRL_REG1_A, 0x57); // ODR: 200Hz; XYZ enabled.
  compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // FS = 10 (+/-8G full scale); High Res Output Mode
}

// Lectura del Giroscopio
void Read_Gyro()
{
  gyro.read();
  
  AN[0] = gyro.g.x;
  AN[1] = gyro.g.y;
  AN[2] = gyro.g.z;
  gx = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
  gy = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
  gz = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

// Lectura del Acelerometro
void Read_Accel()
{
  compass.readAcc();
  
  // Muevo a la derecha 4 bits asi usamos representacion de 12 bits.
  // Con 12 bits en complemento a 2, tengo un campo de representacion de [-2048,2047]
  // Si 1G = 256, en el campo de representacion tengo [-8G, +8G]
  
  AN[3] = compass.a.x >> 4;
  AN[4] = compass.a.y >> 4;
  AN[5] = compass.a.z >> 4;
  ax = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
  ay = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
  az = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

// Lectura Magnetometro
void Read_Compass()
{
  compass.readMag();
  
  mx = SENSOR_SIGN[6] * compass.m.x;
  my = SENSOR_SIGN[7] * compass.m.y;
  mz = SENSOR_SIGN[8] * compass.m.z;
}

// Calibracion del IMU
void IMU_Calibration(int n)
{
  // Acelerometro y Giroscopio
  for(int i=0; i<n; i++)
  {
    Read_Gyro();
    Read_Accel();
    for(int j=0; j<6; j++)
      AN_OFFSET[j] += AN[j];
    delay(20);
  }
  for(int j=0; j<6; j++)
    AN_OFFSET[j] = AN_OFFSET[j]/n;
  
  // Resta 1g al Acelerometro en Z.
  // No tiene SENSOR_SIGN[5], asi que hay que tener cuidado con los signos.
  AN_OFFSET[5]-=GRAVITY;
}
