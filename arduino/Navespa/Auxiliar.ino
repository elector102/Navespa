//=================================================================================================
// Navespa
// - Auxiliar.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en:
// - https://github.com/pololu/minimu-9-ahrs-arduino
// - http://starlino.com/imu_guide.html
//
// 07:32 PM 28/11/2015
//
//=================================================================================================
// Contiene la funcion del filtro complementario y demas funciones auxiliares.
//
//=================================================================================================
// Funciones presentes:
// void getRwEst();
// void vectorNormalize(float* vector);
// float squared(float x);
// void setLimites();
// void setZonaMuerta();
// void setMouseData();
//
//=================================================================================================

//Estimacion de Inclinacion
void getRwEst()
{
  //Variables
  static int signoRzGyro = 0;
  static int i = 0;
  
  //Lee el acelerometro y lo guarda en RwAcc [G]
  readAcc();
  for(int i=0; i<3; i++)
  {
    RwAcc[i] = vAcc[i];
  }
  vectorNormalize(RwAcc);
  
  if(firstSample)
  {
    for(i=0; i<3; i++)
	{
	  RwEst[i] = RwAcc[i];
	}
	firstSample = false;
  }
  else
  {
    //Se evalua RwGyro
	if(abs(RwEst[2]) < 0.1)
	{
	  //Si Rz es muy chico, puede causar errores.
	  //En este caso, no se calcula.
	  for(i=0; i<3; i++)
	  {
	    RwGyro[i] = RwEst[i];
	  }
	}
	else
	{
	  //Obtiene los angulos en base a RwEst[n-1]
	  readGyro();
	  for(i=0; i<2; i++)
	  {
	    Awz[i] = ToDeg(atan2(RwEst[i],RwEst[2]));
		Awz[i] += vGyro[i]*DT_S; // [deg]
	  }
	  
	  //Estimacion del signo de RzGyro.
	  //RzGyro es positivo si Axz esta entre +/- 90. cos(Awz)>=0
	  if(cos(ToRad(Awz[0]))>=0)
	  {
	    signoRzGyro = 1;
	  }
	  else
	  {
	    signoRzGyro = -1;
	  }
	  
	  //Calculo de RwGyro desde Awz
	  for(i=0; i<2; i++)
	  {
	    RwGyro[i] = sin(ToRad(Awz[i]));
		RwGyro[i] /= sqrt(1 + squared(cos(ToRad(Awz[i]))) * squared(tan(ToRad(Awz[1-i]))));
	  }
	  RwGyro[2] = signoRzGyro*sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
	}
	
	//Combina RwAcc con RwGyro
	for(i=0; i<3; i++)
	{
	  RwEst[i] = (RwAcc[i] + gyroWeight*RwGyro[i]) / (1+gyroWeight);
	}
	vectorNormalize(RwEst);
  }
}

//Normalizacion
void vectorNormalize(float* vector)
{
  static float R;  
  R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
  vector[0] /= R;
  vector[1] /= R;  
  vector[2] /= R;  
}

//Cuadrado de un Numero
float squared(float x)
{
  return x*x;
}

//Establece los Limites
void setLimites()
{
  //Calculo de Limites Horizontales
  if(RwEstSign[2]*RwEst[2]<lim[0])
  {
    lim[0] = RwEstSign[2]*RwEst[2];
  }
  if(RwEstSign[2]*RwEst[2]>lim[1])
  {
    lim[1] = RwEstSign[2]*RwEst[2];
  }
  
  //Calculo de Limites Verticales
  if(RwEst[0]<lim[2])
  {
    lim[2] = RwEst[0];
  }
  if(RwEst[0]>lim[3])
  {
    lim[3] = RwEst[0];
  }
}

//Establece las Zonas Muertas
void setZonaMuerta()
{
  static int i;
  for(i=0; i<4; i++)
  {
    zm[i] = zmPorcentaje*lim[i];
  }
}

//Prepara los datos a enviar al Mouse
void setMouseData()
{
  float x, y, aux;
  
  x = constrain(-RwEst[2], lim[0], lim[1]);
  y = constrain(RwEst[0], lim[2], lim[3]);
  
  setZonaMuerta();
  
  //Horizontal
  if(x>=0)
  {
    //Derecha
    if(x>=zm[1])
    {
      aux = (x-zm[1])*velocidad;
      aux /= (lim[1]-zm[1]);
    }
    else
    {
      aux = 0;
    }
  }
  else
  {
    //Atras
    if(x<=zm[0])
    {
      aux = (x-zm[0])*velocidad;
      aux /= (zm[0]-lim[0]);
    }
    else
    {
      aux = 0;
    }
  }
  datoX = (char)aux;
  
  //Vertical
  if(y>=0)
  {
    //Adelante
    if(y>=zm[3])
    {
      aux = (y-zm[3])*velocidad;
      aux /= (lim[3]-zm[3]);
    }
    else
    {
      aux = 0;
    }
  }
  else
  {
    //Atras
    if(y<=zm[2])
    {
      aux = (y-zm[2])*velocidad;
      aux /= (zm[2]-lim[2]);
    }
    else
    {
      aux = 0;
    }
  }
  datoY = (char)aux;
  
  if(analogRead(BOTON_CLICK)>=700)
  {
    datoBotones = 0x01;
  }
  else
  {
    datoBotones = 0x00;
  }
}
