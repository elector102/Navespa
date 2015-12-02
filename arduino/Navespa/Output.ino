//=================================================================================================
// Navespa
// - Output.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en https://github.com/pololu/minimu-9-ahrs-arduino
//
// mié 02 dic 2015 08:32:34 ART 
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
  Serial.print(vGyro[0]);
  Serial.print(",");
  Serial.print(vGyro[1]);
  Serial.print(",");
  Serial.print(vGyro[2]);
  #endif
  
  // Acelerometro
  #if PRINT_ACC == 1
  Serial.print(vAcc[0]);
  Serial.print(",");
  Serial.print(vAcc[1]);
  Serial.print(",");
  Serial.print(vAcc[2]);
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
  Serial.print(RwEstSign[0]*RwEst[0]);
  Serial.print(",");
  Serial.print(RwEstSign[1]*RwEst[1]);
  Serial.print(",");
  Serial.print(RwEstSign[2]*RwEst[2]);
  #endif
  
  // Print Mouse
  #if PRINT_MOUSE == 1
  Serial.print(datoHeader, HEX);
  Serial.print(",");
  Serial.print(datoX, DEC);
  Serial.print(",");
  Serial.print(datoY, DEC);
  Serial.print(",");
  Serial.print(datoBotones, HEX);
  #endif
  
  // Print Limites
  #if PRINT_LIMITES == 1
  Serial.print(lim[0]);
  Serial.print(",");
  Serial.print(lim[1]);
  Serial.print(",");
  Serial.print(lim[2]);
  Serial.print(",");
  Serial.print(lim[3]);
  #endif
  
  Serial.println();
}

void data_send()
{
  byte paquete[] = {datoHeader, datoX, datoY, datoBotones};
  Serial.write(paquete, sizeof(paquete));
}
