//=================================================================================================
// Navespa
// - Complemetary.ino
//
// Emder, Fabricio
// Mas, German Emilio
//
// Basado en:
// - https://github.com/pololu/minimu-9-ahrs-arduino
// - http://starlino.com/imu_guide.html
//
// 12/11/2015 - 01:47
//
//=================================================================================================
// Metodos relevantes al Filtro Complementario.
//
//=================================================================================================
// Funciones presentes:
// void getRwEst();
//
//=================================================================================================

// Estimacion de Inclinacion
void getRwEst()
{
	// Variables
	static int signoRzGyro = 0;
	static int i = 0;
	
	// Lee el acelerometro y lo guarda en RwAcc [G]
	Read_Acc();
	for(int i=0; i<3; i++)
	{
		RwAcc[i] = v_acc[i];
	}
	vector_Normalize(RwAcc);
	
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
		// Se evalua RwGyro
		if(abs(RwEst[2]) < 0.1)
		{
			// Si Rz es muy chico, puede causar errores.
			// En este caso, no se calcula.
			for(i=0; i<3; i++)
			{
				RwGyro[i] = RwEst[i];
			}
		}
		else
		{
			// Obtiene los angulos en base a RwEst[n-1]
			Read_Gyro();
			for(i=0; i<2; i++)
			{
				Awz[i] = ToDeg(atan2(RwEst[i],RwEst[2]));
				Awz[i] += v_gyro[i]*DT_S; // [deg]
			}
			
			// Estimacion del signo de RzGyro.
			// RzGyro es positivo si Axz esta entre +/- 90. cos(Awz)>=0
			if(cos(ToRad(Awz[0]))>=0)
			{
				signoRzGyro = 1;
			}
			else
			{
				signoRzGyro = -1;
			}
			
			// Calculo de RwGyro desde Awz
			for(i=0; i<2; i++)
			{
				RwGyro[i] = sin(ToRad(Awz[i]));
				RwGyro[i] /= sqrt(1 + squared(cos(ToRad(Awz[i]))) * squared(tan(ToRad(Awz[1-i]))));
			}
			RwGyro[2] = signRzGyro*sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
		}
		
		// Combina RwAcc con RwGyro
		for(i=0; i<3; i++)
		{
			RwEst[i] = (RwAcc[i] + gyro_weight*RwGyro[i]) / (1+gyro_weight);
		}
		vector_Normalize(RwEst);
    }
}

