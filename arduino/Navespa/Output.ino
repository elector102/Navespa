//=================================================================================================
// Navespa
// - Output.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en https://github.com/pololu/minimu-9-ahrs-arduino
//
//=================================================================================================
// Imprime los datos segun la variable de Debug indicada.
//
//=================================================================================================
// Funciones presentes:
// void data_print();
// void data_send();
//
//=================================================================================================

void data_print()
{    
  // Giroscopio
  #if PRINT_GYRO == 1
  Serial.print(v_gyro[0]);
  Serial.print(",");
  Serial.print(v_gyro[1]);
  Serial.print(",");
  Serial.print(v_gyro[2]);
  #endif
  
  // Acelerometro
  #if PRINT_ACC == 1
  Serial.print(v_acc[0]);
  Serial.print(",");
  Serial.print(v_acc[1]);
  Serial.print(",");
  Serial.print(v_acc[2]);
  #endif
  
  // RwGyro
  #if PRINT_RGYRO == 1
  Serial.print(RwGyro[0]);
  Serial.print(",");
  Serial.print(RwGyro[1]);
  Serial.print(",");
  Serial.print(RwGyro[2]);
  #endif
  
  // RwEst
  #if PRINT_REST == 1
  Serial.print(RwEst[0]);
  Serial.print(",");
  Serial.print(RwEst[1]);
  Serial.print(",");
  Serial.print(RwEst[2]);
  #endif
  
  Serial.println();
}

void data_send()
{
  //TODO mejorar
  
  //byte paquete[] = {dato_header, dato_x, dato_y, dato_botones};
  //Serial.write(paquete, sizeof(paquete));
  
  //Serial.print(dato_x); 
  //Serial.print(",");
  //Serial.print(dato_y);
  //Serial.print(",");
  Serial.print(ToDeg(roll));
  Serial.print(",");
  Serial.print(ToDeg(pitch));
  Serial.print(",");
  Serial.println(ToDeg(yaw));
  
}
