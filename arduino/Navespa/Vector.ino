//=================================================================================================
// Navespa
// - Vector.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en https://github.com/pololu/minimu-9-ahrs-arduino
//
//=================================================================================================
// Operaciones entre Vectores.
// Suma, producto escalar, producto vectorial y escalamiento.
//
//=================================================================================================
// Funciones presentes:
// void Vector_Add(float vectorA[3], float vectorB[3], float output[3]);
// float Vector_Dot(float vectorA[3], float vectorB[3]);
// void Vector_Cross(float vectorA[3], float vectorB[3], float output[3]);
// void Vector_Scale(float input[3], float output[3], float alpha);
//
//=================================================================================================

// Suma de Vectores
void Vector_Add(float vectorA[3], float vectorB[3], float output[3])
{
  // vectorA + vectorB = output
  for(int i=0; i<3; i++)
    output[i]=vectorA[i]+vectorB[i];
}

// Producto Escalar
float Vector_Dot(float vectorA[3], float vectorB[3])
{
  // vectorA . vectorB = valor (escalar)
  float valor = 0;
  for(int i=0; i<3; i++)
    valor += vectorA[i]*vectorB[i];
  return valor;
}

// Producto Vectorial
void Vector_Cross(float vectorA[3], float vectorB[3], float output[3])
{
  // vectorA x vectorB = output (vector)
  output[0] = (vectorA[1]*vectorB[2]) - (vectorA[2]*vectorB[1]);
  output[1] = (vectorA[2]*vectorB[0]) - (vectorA[0]*vectorB[2]);
  output[2] = (vectorA[0]*vectorB[1]) - (vectorA[1]*vectorB[0]);
}

// Escalamiento de Vectores
void Vector_Scale(float input[3], float output[3], float alpha)
{
  // input*alpha = output
  for(int i=0; i<3; i++)
    output[i]=input[i]*alpha;
}
