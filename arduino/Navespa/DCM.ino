//=================================================================================================
// Navespa
// - DCM.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en https://github.com/pololu/minimu-9-ahrs-arduino
//
//=================================================================================================
// Algoritmo DCM (Director Cosine Matrix)
//
//=================================================================================================
// Funciones presentes:
// void Normalize();
// void Drift_correction(void)
// void Matrix_update(void)
// void Euler_angles(void)
//
//=================================================================================================

// Normaliza la matriz DCM ya que debe ser siempre ortogonal
void Normalize()
{
  float error = 0;
  float temporary[3][3];
  float renorm = 0;
  
  error = -Vector_Dot(&DCM_Matrix[0][0], &DCM_Matrix[1][0])*0.5; // X.Y = error
  
  Vector_Scale(&DCM_Matrix[1][0], &temporary[0][0], error); // Y*(-error/2)
  Vector_Scale(&DCM_Matrix[0][0], &temporary[1][0], error); // X*(-error/2)
  
  Vector_Add(&temporary[0][0], &DCM_Matrix[0][0], &temporary[0][0]); // X=X-Y(error/2)
  Vector_Add(&temporary[1][0], &DCM_Matrix[1][0], &temporary[1][0]); // Y=Y-X(error/2)
  
  Vector_Cross(&temporary[0][0], &temporary[1][0], &temporary[2][0]); //Z=XxY
    
  renorm = 0.5*(3-Vector_Dot(&temporary[0][0],&temporary[0][0])); //Factor de escalamiento de X
  Vector_Scale(&temporary[0][0], &DCM_Matrix[0][0], renorm); // Normaliza el X
  
  renorm = 0.5*(3-Vector_Dot(&temporary[1][0],&temporary[1][0])); //Factor de escalamiento de Y
  Vector_Scale(&temporary[1][0], &DCM_Matrix[1][0], renorm); // Normaliza el Y
  
  renorm = 0.5*(3-Vector_Dot(&temporary[2][0],&temporary[2][0])); //Factor de escalamiento de Z
  Vector_Scale(&temporary[2][0], &DCM_Matrix[2][0], renorm); // Normaliza el Z
}

// Correccion de deriva de los giroscopios
void Drift_correction()
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
 
  static float scaled_omega_p[3];
  static float scaled_omega_i[3];
  float a_mag; // Modulo del v_acc
  float a_weight; // Peso del acelerometro
  
  //-------------------------------------------------------------------------------------------------
  // PRIMERA PARTE - DERIVA EN ROLL Y PITCH
  //-------------------------------------------------------------------------------------------------
  
  // Calcula el modulo del vector del acelerometro
  a_mag = sqrt(v_acc[0]*v_acc[0] + v_acc[1]*v_acc[1] + v_acc[2]*v_acc[2]);
  
  // Pesado dinamico de la informacion del acelerometro (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  a_weight = constrain(1-2*abs(1-a_mag),0,1);
  
  Vector_Cross(&v_acc[0], &DCM_Matrix[2][0], &errorRollPitch[0]); // Ajusta la referencia al suelo
  Vector_Scale(&errorRollPitch[0], &omega_p[0], KP*a_weight);
  Vector_Scale(&errorRollPitch[0], &scaled_omega_i[0], KI*a_weight);
  Vector_Add(scaled_omega_i, omega_i, omega_i);
  
  //-------------------------------------------------------------------------------------------------
  // SEGUNDA PARTE - DERIVA EN YAW
  //-------------------------------------------------------------------------------------------------
  
  // Corregimos la deriva del giroscopio con los valores del magnetometro 
  mag_heading_x = cos(m_heading);
  mag_heading_y = sin(m_heading);
  errorCourse = (DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x); // Error en el Yaw
  
  Vector_Scale(&DCM_Matrix[2][0], errorYaw, errorCourse);
  
  Vector_Scale(&errorYaw[0], &scaled_omega_p[0], KP_YAW);
  Vector_Add(omega_p, scaled_omega_p, omega_p);
  
  Vector_Scale(&errorYaw[0], &scaled_omega_i[0], KI_YAW);
  Vector_Add(omega_i, scaled_omega_i, omega_i);
}

// Actualizacion de la Matriz DCM
void Matrix_update()
{
  v_gyro[0] = scaled_gx(gx); // Valores de Giroscopio en [rad/s]
  v_gyro[1] = scaled_gy(gy);
  v_gyro[2] = scaled_gz(gz);
  
  v_acc[0] = (float)ax*sen_ax; // Valores de Acelerometro en [G]
  v_acc[1] = (float)ay*sen_ay;
  v_acc[2] = (float)az*sen_az;

  Vector_Add(&v_gyro[0], &omega_i[0], &omega[0]); // Omega=VGyro+OmegaI
  Vector_Add(&omega[0], &omega_p[0], &omega_gyro[0]); // OmegaGyro=Omega+OmegaP
   
  Update_Matrix[0][0] = 0; //Diagonal principal en 0.
  Update_Matrix[0][1] = -DT_S*omega_gyro[2]; // -z
  Update_Matrix[0][2] = DT_S*omega_gyro[1]; //y
  Update_Matrix[1][0] = DT_S*omega_gyro[2]; //z
  Update_Matrix[1][1] = 0;
  Update_Matrix[1][2] = -DT_S*omega_gyro[0]; //-x
  Update_Matrix[2][0] = -DT_S*omega_gyro[1]; //-y
  Update_Matrix[2][1] = DT_S*omega_gyro[0]; //x
  Update_Matrix[2][2] = 0;
 
  // Update Matrix queda entonces antisimetrica
  // [ 0, -z,  y]
  // [ z,  0, -x]
  // [-y,  x,  0]

  Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix);
  Matrix_Add(DCM_Matrix, Temporary_Matrix, DCM_Matrix); // Actualizacion de DCM
}


// Calculo de Angulos de Euler
void Euler_angles()
{
  // Sensibilidad
  sen_rate = analogRead(A1);
  // Guarda los angulos del calculo anterior
  pitch_anterior = pitch;
  roll_anterior = roll;
  yaw_anterior = yaw;
  
  // Calcula los nuevos angulos
  pitch = -asin(DCM_Matrix[2][0]);
  roll = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
  yaw = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
  
  // Calcula el grado de cambio
  pitch_rate = constrain((pitch_anterior - pitch)*(-sen_rate), -128, 127);
  roll_rate = constrain((roll_anterior - roll)*sen_rate, -128, 127);
  yaw_rate = constrain((yaw_anterior - yaw)*sen_rate, -128, 127);
}
