//=================================================================================================
// Navespa
// - Auxiliar.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en https://github.com/pololu/minimu-9-ahrs-arduino
//
//=================================================================================================
// Contiene funciones matematicas auxiliares.
//
//=================================================================================================
// Funciones presentes:
// void vector_Add(float vectorA[3], float vectorB[3], float output[3]);
// float vector_Dot(float vectorA[3], float vectorB[3]);
// void vector_Cross(float vectorA[3], float vectorB[3], float output[3]);
// void vector_Scale(float input[3], float output[3], float alpha);
// void vector_Normalize(float* vector);
// float squared(float x);
//
//=================================================================================================

// Suma de Vectores
void vector_Add(float vectorA[3], float vectorB[3], float output[3])
{
  // vectorA + vectorB = output
  for(int i=0; i<3; i++)
    output[i]=vectorA[i]+vectorB[i];
}

// Producto Escalar
float vector_Dot(float vectorA[3], float vectorB[3])
{
  // vectorA . vectorB = valor (escalar)
  float valor = 0;
  for(int i=0; i<3; i++)
    valor += vectorA[i]*vectorB[i];
  return valor;
}

// Producto Vectorial
void vector_Cross(float vectorA[3], float vectorB[3], float output[3])
{
  // vectorA x vectorB = output (vector)
  output[0] = (vectorA[1]*vectorB[2]) - (vectorA[2]*vectorB[1]);
  output[1] = (vectorA[2]*vectorB[0]) - (vectorA[0]*vectorB[2]);
  output[2] = (vectorA[0]*vectorB[1]) - (vectorA[1]*vectorB[0]);
}

// Escalamiento de Vectores
void vector_Scale(float input[3], float output[3], float alpha)
{
  // input*alpha = output
  for(int i=0; i<3; i++)
    output[i]=input[i]*alpha;
}

// Normalizacion
void vector_Normalize(float* vector)
{
  static float R;  
  R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
  vector[0] /= R;
  vector[1] /= R;  
  vector[2] /= R;  
}

// Cuadrado de un Numero
float squared(float x)
{
  return x*x;
}
