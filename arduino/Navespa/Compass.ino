//=================================================================================================
// Navespa
// - Compass.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en https://github.com/pololu/minimu-9-ahrs-arduino
//
//=================================================================================================
// Metodos relevantes al Magnetometro.
// Calculo de Heading corregido.
//
//=================================================================================================
// Funciones presentes:
// void Compass_Heading();
//
//=================================================================================================

// Direccion del Magnetometro
void Compass_Heading()
{
  float MAG_X;
  float MAG_Y;
  
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Ajusta los valores del Magnetometro
  // Los escala al medio entre el maximo y minimo calibrado
  c_mx = (float)(mx - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
  c_my = (float)(my - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
  c_mz = (float)(mz - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5;

  // Magnetometro X e Y compensados en inclinacion
  MAG_X = c_mx*cos_pitch + c_my*sin_roll*sin_pitch + c_mz*cos_roll*sin_pitch;
  MAG_Y = c_my*cos_roll - c_mz*sin_roll;
  
  // Direccion Magnetica
  m_heading = atan2(-MAG_Y, MAG_X);
}
