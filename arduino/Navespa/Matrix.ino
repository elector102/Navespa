//=================================================================================================
// Navespa
// - Matrix.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en https://github.com/pololu/minimu-9-ahrs-arduino
//
//=================================================================================================
// Operaciones entre matrices. Solo utilizamos matrices de 3x3.
// Suma, multiplicacion y escalamiento.
//
//=================================================================================================
// Funciones presentes:
// void Matrix_Add(float A[3][3], float B[3][3], float output[3][3]);
// void Matrix_Multiply(float A[3][3], float B[3][3], float output[3][3]);
// void Matrix_Scale(float input[3][3], float output[3][3], float alpha);
//
//=================================================================================================

// Suma de Matrices de 3x3
void Matrix_Add(float A[3][3], float B[3][3], float output[3][3])
{
  // A+B = output
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
	  output[i][j] = A[i][j] + B[i][j];
}

// Multiplicacion de Matrices de 3x3
void Matrix_Multiply(float A[3][3], float B[3][3], float output[3][3])
{
  // AB = output
  float aux[3];
  for(int x=0; x<3; x++)
    for(int y=0; y<3; y++)
    {
      for(int w=0; w<3; w++)
        aux[w] = A[x][w]*B[w][y];  
      output[x][y] = aux[0]+aux[1]+aux[2];      
    }
}

// Escalado de Matrices 3x3
void Matrix_Scale(float input[3][3], float output[3][3], float alpha)
{
  // alpha*input = output
  for(int i=0; i<3; i++)
    for(int j=0; j<3; j++)
	  output[i][j] = input[i][j]*alpha;
}
