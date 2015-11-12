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
  // Angulos de Euler
  #if PRINT_EULER == 1
  Serial.print(ToDeg(roll));
  Serial.print(",");
  Serial.print(ToDeg(pitch));
  Serial.print(",");
  Serial.print(ToDeg(yaw));
  #endif
  
  //Giroscopio
  #if PRINT_GYRO == 1
  Serial.print(ToDeg(scaled_gx(gx)));
  Serial.print(",");
  Serial.print(ToDeg(scaled_gy(gy)));
  Serial.print(",");
  Serial.print(ToDeg(scaled_gz(gz)));
  #endif
  
    //Giroscopio
  #if PRINT_ACC == 1
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  #endif
  
  // Cambios en los Angulos de Euler
  #if PRINT_RATE == 1
  //Serial.print(ToDeg(roll_rate));
  Serial.print(roll_rate);
  Serial.print(",");
  //Serial.print(ToDeg(pitch_rate));
  Serial.print(pitch_rate);
  Serial.print(",");
  //Serial.print(ToDeg(yaw_rate));
  Serial.print(yaw_rate);
  #endif
  
  // Matriz DCM
  #if PRINT_DCM==1
  for(int i=0; i<3; i++)
  {
    Serial.print("[");
    for(int j=0; j<3; j++)
    {
      Serial.print(DCM_Matrix[i][j]);
      Serial.print("  ");
    }
    Serial.print("]");
  }
  #endif
  
  Serial.println();
}

void data_send()
{
  
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
