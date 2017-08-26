/* 
 * (c) 2012 Rafael Paz <rpaz@atc.us.es>
 *
 *************
 *
 *	This file is part of IKARUS_OSD.
 *
 *  IKARUS_OSD is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  IKARUS_OSD is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with IKARUS_OSD.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Math.h>

#include "libMath.h"
#include "DCM.h"
#include "Motion_Sensors.h"

#ifdef GPS
	#include "ParserNMEA.h"
	extern GPSInfo gpsinfo;
#endif

extern  Accel_Data acel_data;
extern  Gyro_Data gyro_data;
extern  Mag_Data mag_data;
extern uint16_t elapsedTime;

float G_Dt=0.01;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

float Kp_ROLLPITCH;
float Ki_ROLLPITCH;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
//float roll;
//float pitch;
//float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

float DCM_Matrix[3][3]= {{1,0,0},{0,1,0},{0,0,1}}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float Temporary_Matrix[3][3]={{0,0,0},{0,0,0},{0,0,0}};

char problem= 0;

void DCM_Init(DCM_Config* dcm ){
	uint16_t i,j;
	Kp_ROLLPITCH=dcm->kp;
	Ki_ROLLPITCH=dcm->ki;
	for (i=0;i<3;i++){
		for(j=0;j<3;j++){
			if(i==j){
				DCM_Matrix[i][j]=1;
			}else{
				DCM_Matrix[i][j]=0;
			}
		}
		Omega_Vector[i]=0;
		Omega_P[i]=0;
		Omega_I[i]=0;
		Omega[i]=0;
		errorRollPitch[i]=0;
		errorYaw[i]=0;
	}
}


float Renorm(float r_in)
{
	float r_out;
#if 0
	r_out= .5f *(3 - r_in);
#else
	if (r_in < 1.5625f && r_in > 0.64f) 
	{
    r_out= .5f * (3-r_in);                                                 //eq.21
  } 
	else if (r_in < 100.0f && r_in > 0.01f) 
	{
    r_out= 1. / sqrt(r_in);  
  } 
	else 
	{
    problem = 1;
  }
#endif
	return r_out;
}

/**************************************************/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;
  
  error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5f; //eq.19

  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
  
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
  
	renorm = Renorm(Vector_Dot_Product(&temporary[0][0],&temporary[0][0]));
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
	renorm = Renorm(Vector_Dot_Product(&temporary[1][0],&temporary[1][0]));
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
	renorm = Renorm(Vector_Dot_Product(&temporary[2][0],&temporary[2][0]));
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);


	if (problem) 
	{                // Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down!
      DCM_Matrix[0][0]= 1.0f;
      DCM_Matrix[0][1]= 0.0f;
      DCM_Matrix[0][2]= 0.0f;
      DCM_Matrix[1][0]= 0.0f;
      DCM_Matrix[1][1]= 1.0f;
      DCM_Matrix[1][2]= 0.0f;
      DCM_Matrix[2][0]= 0.0f;
      DCM_Matrix[2][1]= 0.0f;
      DCM_Matrix[2][2]= 1.0f;
      problem = 0;  
  }
}

/**************************************************/
void Drift_correction(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  float Integrator_magnitude;
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  
	// Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*fabs(1 - Accel_magnitude),0,1);  //  

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = mag_data.x;//cos(infoFCU.heading);
  mag_heading_y = mag_data.y;//sin(infoFCU.heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01 proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I

	//  Here we will place a limit on the integrator so that the integrator	 cannot ever exceed half the saturation limit of the gyros
  Integrator_magnitude = sqrt(Vector_Dot_Product(Omega_I,Omega_I));
  if (Integrator_magnitude > ToRad(300))
    Vector_Scale(Omega_I,Omega_I,0.5f*ToRad(300)/Integrator_magnitude);
}
/**************************************************/

#define Accel_Scale(x) x*(9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

void Accel_adjust(void)
{
#ifdef GPS
 float speed3d=gpsinfo.knots*0.514444f;
 Accel_Vector[1] += Accel_Scale(speed3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY 
#endif
}

/**************************************************/

void Matrix_update(void)
{
  int x,y;

  Gyro_Vector[0]=ToRad(gyro_data.x); //gyro x roll
  Gyro_Vector[1]=ToRad(gyro_data.y); //gyro y pitch
  Gyro_Vector[2]=ToRad(gyro_data.z); //gyro Z yaw
  
  Accel_Vector[0]=acel_data.x;
  Accel_Vector[1]=acel_data.y;
  Accel_Vector[2]=acel_data.z;
    
	G_Dt = 0.0000005f*elapsedTime;
	
  Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
  Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

  Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
        
  Update_Matrix[0][0]=0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
  Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
  Update_Matrix[1][1]=0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
  Update_Matrix[2][2]=0;

  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

  for(x=0; x<3; x++) //Matrix Addition (update)
  {
    for(y=0; y<3; y++)
    {
      DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    } 
  }
}

extern float ypr_dcm[3];

void Euler_angles(void)
{
    ypr_dcm[1] = ToDeg(-asin(DCM_Matrix[2][0]));
    ypr_dcm[2] = ToDeg(atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]));
    ypr_dcm[0] = ToDeg(atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]));
}



void Calculate_DCM()
{
	// Calculations...
	Matrix_update(); 
	Normalize();
	Drift_correction();
	Euler_angles();
}

