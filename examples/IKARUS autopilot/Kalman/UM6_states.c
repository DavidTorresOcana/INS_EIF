/* ------------------------------------------------------------------------------
  File: UM6_states.c
  Author: CH Robotics
  Version: 1.0
  
  Description: Function definitions for CHR-6dm state estimation.
------------------------------------------------------------------------------ */ 

#include <math.h>
#include "CHR_matrix.h"
#include "CHR_quat.h"
#include "stm32f10x.h"
#include "Motion_Sensors.h"
#include "UM6_states.h"
// Data structures for holding sensor data and estimated states.

AHRS_state_data gStateData;


/*******************************************************************************
* Function Name  : EKF_Init
* Input          : None
* Output         : Pre-initialized state estimate structure
* Return         : None
* Description    : Fills an AHRS_states structure with zero initial values.
*******************************************************************************/

extern uint16_t elapsedTime;

void EKF_Init(EKF_Config * ekf_cfg){
		gStateData.phi = 0;
		gStateData.theta = 0;
		gStateData.psi = 0;

		gStateData.phi_dot = 0;
		gStateData.theta_dot = 0;
		gStateData.psi_dot = 0;
	
	 	 // Process variance
		gStateData.process_var = ekf_cfg->process_var;//0.1;
			
	 	// accelerometer measurement variance
 	 gStateData.accel_var=ekf_cfg->accel_var;// 2.0f;	//5 parece bien
 	 
 	 // Magnetometer variance
 	  gStateData.mag_var = ekf_cfg->mag_var;//2.0f;
			
		mat_zero( &gStateData.R, 3, 3);
		mat_zero( &gStateData.Sigma, 3,3 );
	
		// Process variance is scaled here so that the performance in Euler Angle mode and Quaternion mode is comparable
		gStateData.Sigma.data[0][0] = gStateData.process_var;
		gStateData.Sigma.data[1][1] = gStateData.process_var;
		gStateData.Sigma.data[2][2] = gStateData.process_var;
			
		gStateData.R.data[0][0] = gStateData.process_var;
		gStateData.R.data[1][1] = gStateData.process_var;
		gStateData.R.data[2][2] = gStateData.process_var;


		gStateData.qib.a = 1;
		gStateData.qib.b = 0;
		gStateData.qib.c = 0;
		gStateData.qib.d = 0;

//Fijos desde RESET FACTORY
 	 // Magnetic field reference vector
	//ORIGINAL
 	 gStateData.mag_ref_x = ekf_cfg->mag_ref_x;//0.136;
 	 gStateData.mag_ref_y= ekf_cfg->mag_ref_y;//0.4065;
 	 gStateData.mag_ref_z= ekf_cfg->mag_ref_z;//0.894;
 	 
	 
 	 // Accelerometer	reference vector
 	 //ORIGINAL

 	 gStateData.accel_ref_x= ekf_cfg->accel_ref_x;//0;
  	 gStateData.accel_ref_y= ekf_cfg->accel_ref_y;//0;
  	 gStateData.accel_ref_z= ekf_cfg->accel_ref_z;//-1.0;

 	  
}


/*******************************************************************************
* Function Name  : EKF_EstimateStates
* Input          : AHRS_states* estimated_states, RawSensorData* sensor_data
* Output         : None
* Return         : None
* Description    : 
*******************************************************************************/
void EKF_EstimateStates(void)// AHRS_state_data* state_data)//, RawSensorData* sensor_data )
{
	// static int initialized = 0;
	
	/* if( !initialized )
	 {
		  EKF_Init();
		  initialized = 1;
	 }*/
	  	
	 // First, convert raw sensor data to actual data (acceleration to gravities, gyro data
	 // to angular rates, magnetometer to unit-norm data
	 //ConvertRawSensorData( state_data, sensor_data );	 
	 
	 // Run EKF prediction step
	 EKF_Predict();
	 	 
	 // Run EKF update step
	 EKF_Update();
	 
	 // Copy the new states into the communication interface structures
//	 CopyStatesToDataArray();
}


/*******************************************************************************
* Function Name  : EKF_Predict
* Input          : AHRS_states* estimated_states, RawSensorData* sensor_data
* Output         : None
* Return         : None
* Description    : EKF prediction step.  Uses rate gyros to make new orientation
						 estimate.
*******************************************************************************/
void EKF_Predict( void )
{
	 float T,p,q,r;
	 fMatrix A,Atranspose,temp1,temp2;
	 
	 // Get elapsed time since last prediction 

		T = 0.0000005f*elapsedTime;
	 
	 // Copy body frame angular rates to local variables for convenience
	 p = gStateData.gyro_x;
	 q = gStateData.gyro_y;
	 r = gStateData.gyro_z;
		  
	 // Euler Angle Estimation
	 {
		  float cos_phi,sin_phi,cos_theta,tan_theta,sin_theta;
		  
		  A.rows = 3;
		  A.columns = 3;
		  Atranspose.rows = 3;
		  Atranspose.columns = 3;
		  temp1.rows = 3;
		  temp1.columns = 3;
		  temp2.rows = 3;
		  temp2.columns = 3;
		  
		  // Precompute trigonometric functions - these will be used more than once
		  cos_phi = cos(gStateData.phi*.01745329f);
		  sin_phi = sin(gStateData.phi*.01745329f);
		  cos_theta = cos(gStateData.theta*.01745329f);
		  sin_theta = sin(gStateData.theta*.01745329f);	 
		  tan_theta = tan(gStateData.theta*.01745329f);
		  
		  // Compute rotation rates based on body frame angular rates measured by the rate gyros
		  /*
		  phi_dot = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta)
		  theta_dot = q*cos(phi) - r*sin(phi)
		  psi_dot = (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta)
		  */
		  gStateData.phi_dot = p + r*cos_phi*tan_theta + q*sin_phi*tan_theta;
		  gStateData.theta_dot = q*cos_phi - r*sin_phi;
		  gStateData.psi_dot = (r*cos_phi)/cos_theta + (q*sin_phi)/cos_theta;
		  
		  // Use measured rotation rates in the body frame to compute new angle estimates
		  gStateData.phi += T*gStateData.phi_dot;
		  gStateData.theta += T*gStateData.theta_dot;
		  gStateData.psi += T*gStateData.psi_dot;
		  
		  // DISCRETE STATE TRANSITION
	 //	 [ T*q*cos(phi)*tan(theta) - T*r*sin(phi)*tan(theta) + 1,               T*r*cos(phi)*(tan(theta)^2 + 1) + T*q*sin(phi)*(tan(theta)^2 + 1), 0]
	 //	 [                         - T*q*sin(phi) - T*r*cos(phi),                                                                               1, 0]
	 //	 [ (T*q*cos(phi))/cos(theta) - (T*r*sin(phi))/cos(theta), (T*r*cos(phi)*sin(theta))/cos(theta)^2 + (T*q*sin(phi)*sin(theta))/cos(theta)^2, 1]
		  A.data[0][0] = T*(q*cos_phi*tan_theta - r*sin_phi*tan_theta) + 1;
		  A.data[0][1] = T*(r*cos_phi*(tan_theta*tan_theta + 1) + q*sin_phi*(tan_theta*tan_theta + 1));
		  A.data[0][2] = 0;
		  A.data[1][0] = T*(-r*cos_phi - q*sin_phi);
		  A.data[1][1] = 1;
		  A.data[1][2] = 0;
		  A.data[2][0] = T*((q*cos_phi)/cos_theta - (r*sin_phi)/cos_theta);
		  A.data[2][1] = T*((r*cos_phi*sin_theta)/(cos_theta*cos_theta) + (q*sin_phi*sin_theta)/(cos_theta*cos_theta)); 
		  A.data[2][2] = 1;
		  
		  // Compute the new covariance estimate (discrete update: Sigma = A*Sigma*Atranspose + R
		  mat_transpose( &A, &Atranspose );
		  mat_mult( &A, &gStateData.Sigma, &temp1 );
		  mat_mult( &temp1, &Atranspose, &temp2 );
		  mat_add( &temp2, &gStateData.R, &gStateData.Sigma );
		  
		  // Finally, "unroll" states so that they range from -360 to 360 degrees
		  unroll_states( &gStateData );
	 }

}

/*******************************************************************************
* Function Name  : EKF_Update
* Input          : AHRS_states* estimated_states, RawSensorData* sensor_data
* Output         : None
* Return         : None
* Description    : EKF update step.  Uses accels to correct pitch and roll errors,
						 and magnetic sensors to correct yaw errors.  Compensation is
						 only applied when new data is available, as specified by the
						 new_mag_data and new_accel_data flags in the sensor_data structure.
*******************************************************************************/


void EKF_Update(void )
{

	 {
		  
		  float cos_phi, cos_theta, cos_psi, sin_phi, sin_theta, sin_psi;
		  
				float ax_hat, ay_hat, az_hat;
				float ax_ref, ay_ref, az_ref;
				fMatrix C;
		
				
				C.rows = 1;
				C.columns = 3;
				
				// Precompute trigonometric functions - these will be used more than once
				cos_phi = cos(gStateData.phi*.01745329f);
				cos_theta = cos(gStateData.theta*.01745329f);
				cos_psi = cos(gStateData.psi*.01745329f);
				
				sin_phi = sin(gStateData.phi*.01745329f);
				sin_theta = sin(gStateData.theta*.01745329f);
				sin_psi = sin(gStateData.psi*.01745329f);

				// Copy data into local variables for convenience		  
				ax_ref = gStateData.accel_ref_x;
				ay_ref = gStateData.accel_ref_y;
				az_ref = gStateData.accel_ref_z;
				
				// Compute expected accelerometer output based on current state estimates (the expected output is the accelerometer reference vector
				// rotated into the body-frame of the sensor
				ax_hat = ax_ref*cos_psi*cos_theta - az_ref*sin_theta + ay_ref*cos_theta*sin_psi;
				ay_hat = ay_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - ax_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) + az_ref*cos_theta*sin_phi;
				az_hat = ax_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - ay_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + az_ref*cos_phi*cos_theta;
				
				// Compute linearized state transition matrix for each axis independently
	 //		  [                                                                                                                                                  0,                          - az_ref*cos(theta) - ax_ref*cos(psi)*sin(theta) - ay_ref*sin(psi)*sin(theta),                                                                 ay_ref*cos(psi)*cos(theta) - ax_ref*cos(theta)*sin(psi)]
	 //		  [ ax_ref*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - ay_ref*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + az_ref*cos(phi)*cos(theta), ax_ref*cos(psi)*cos(theta)*sin(phi) - az_ref*sin(phi)*sin(theta) + ay_ref*cos(theta)*sin(phi)*sin(psi), - ax_ref*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - ay_ref*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))]
	 //		  [ ax_ref*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - ay_ref*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - az_ref*cos(theta)*sin(phi), ax_ref*cos(phi)*cos(psi)*cos(theta) - az_ref*cos(phi)*sin(theta) + ay_ref*cos(phi)*cos(theta)*sin(psi),   ax_ref*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + ay_ref*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))]
				
				// x-axis
				C.data[0][0] = 0;
				C.data[0][1] = -az_ref*cos_theta - ax_ref*cos_psi*sin_theta - ay_ref*sin_psi*sin_theta;
				C.data[0][2] = ay_ref*cos_psi*cos_theta - ax_ref*cos_theta*sin_psi;
				
				
				// Do correction
				EKF_Correction( &C, gStateData.accel_x, ax_hat, gStateData.accel_var, &gStateData, ACCEL_UPDATE );

				// y-axis
				C.data[0][0] = ax_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta) - ay_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + az_ref*cos_phi*cos_theta;
				C.data[0][1] = ax_ref*cos_psi*cos_theta*sin_phi - az_ref*sin_phi*sin_theta + ay_ref*cos_theta*sin_phi*sin_psi;
				C.data[0][2] = -ax_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - ay_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta);
				
				// Do correction
				EKF_Correction( &C, gStateData.accel_y, ay_hat, gStateData.accel_var, &gStateData, ACCEL_UPDATE );
				
				// z-axis
				C.data[0][0] = ax_ref*(cos_phi*sin_psi - cos_psi*sin_phi*sin_theta) - ay_ref*(cos_phi*cos_psi + sin_phi*sin_psi*sin_theta) - az_ref*cos_theta*sin_phi;
				C.data[0][1] = ax_ref*cos_phi*cos_psi*cos_theta - az_ref*cos_phi*sin_theta + ay_ref*cos_phi*cos_theta*sin_psi;
				C.data[0][2] = ax_ref*(cos_psi*sin_phi - cos_phi*sin_psi*sin_theta) + ay_ref*(sin_phi*sin_psi + cos_phi*cos_psi*sin_theta);
				
				// Do correction
				EKF_Correction( &C, gStateData.accel_z, az_hat, gStateData.accel_var, &gStateData, ACCEL_UPDATE );
				
				// Finally, "unroll" states so that they range from -360 to 360 degrees
				unroll_states( &gStateData );

		  // Do magnetometer update if enabled and if there is new magnetic sensor data available
		  if( 0 )
		  {
				float mx_hat, my_hat		, mz_hat;
				float mx_ref, my_ref , mz_ref;
				float mx_v1, my_v1, mz_v1;
				
				fMatrix C;
				
				C.rows = 1;
				C.columns = 3;

				// Precompute trigonometric functions - these will be used more than once
				cos_phi = cos(gStateData.phi*.01745329f);
				cos_theta = cos(gStateData.theta*.01745329f);
				cos_psi = cos(gStateData.psi*.01745329f);
				
				sin_phi = sin(gStateData.phi*.01745329f);
				sin_theta = sin(gStateData.theta*.01745329f);
				sin_psi = sin(gStateData.psi*.01745329f);
				
				// Copy data into local variables for convenience		  
				mx_ref = gStateData.mag_ref_x;
				my_ref = gStateData.mag_ref_y;
				mz_ref = gStateData.mag_ref_z;
				
				// Rotate the sensor measurement into the vehicle-1 frame (undo pitch and roll)
				mx_v1 = gStateData.mag_x*cos_theta + gStateData.mag_z*cos_phi*sin_theta + gStateData.mag_y*sin_phi*sin_theta;
				my_v1 = gStateData.mag_y*cos_phi - gStateData.mag_z*sin_phi;
				mz_v1 =  gStateData.mag_z*cos_phi*cos_theta - gStateData.mag_x*sin_theta + gStateData.mag_y*cos_theta*sin_phi;
				
				// Compute expected magnetometer output based on current state estimates
				// (rotates the reference vector into the vehicle-1 frame)
				mx_hat = mx_ref*cos_psi + my_ref*sin_psi;
				my_hat = my_ref*cos_psi - mx_ref*sin_psi;
				mz_hat = mz_ref;
				
				// Compute linearized state transition matrix for each axis independently	  
				// x-axis
				C.data[0][0] = 0;
				C.data[0][1] = 0;
				C.data[0][2] = my_ref*cos_psi - mx_ref*sin_psi;
				
				// Do correction
				EKF_Correction( &C, mx_v1, mx_hat, gStateData.mag_var, &gStateData, MAG_UPDATE );
				
				// y-axis
				C.data[0][0] = 0;
				C.data[0][1] = 0;
				C.data[0][2] = -mx_ref*cos_psi - my_ref*sin_psi;
				
				// Do correction
				EKF_Correction( &C, my_v1, my_hat, gStateData.mag_var, &gStateData, MAG_UPDATE );
				//-----------------ANGEL
				
// //				 z-axis doesn't do anything in the vehicle-1 frame
// 				// z-axis
// 				C.data[0][0] = 0;
// 				C.data[0][1] = 0;
// 				C.data[0][2] = 0;
// 				
// 				// Do correction
// 				EKF_Correction( &C, mz_v1, mz_hat, gStateData.mag_var, &gStateData, MAG_UPDATE );
				
				//-----------------ANGEL
				
				// Finally, "unroll" states so that they range from -360 to 360 degrees
				unroll_states( &gStateData );
		  }
	 }

}



float v_error[3];	

void EKF_Correction( fMatrix* C, float sensor_data, float sensor_hat, float sensor_covariance, AHRS_state_data* estimated_states, int sensor_type )
{

	fMatrix L;
	fMatrix Ctranspose,temp1,temp2;
	float gain_scale, error;
	

  mat_zero( &temp1, 3, 1 );
  mat_zero( &temp2, 3, 3 );
	
	 mat_transpose( C, &Ctranspose );
	 mat_zero( &L, 3, 1 );
	 // Compute Kalman Gain (L = Sigma*C'*(C*Sigma*C' + Q)^-1 )
	 mat_mult(&estimated_states->Sigma,&Ctranspose,&temp1);
	 mat_mult(C,&estimated_states->Sigma,&temp2);
	 mat_mult(&temp2,&Ctranspose,&temp2);
	 gain_scale = 1/(temp2.data[0][0] + sensor_covariance);
	 mat_scalar_mult(gain_scale,&temp1,&L);
	 
	 // Update state estimates
	 error = sensor_data - sensor_hat;
	
 
		v_error[0]=L.data[0][0]*error;
		v_error[1]=L.data[1][0]*error;
		v_error[2]=L.data[2][0]*error;
	
	estimated_states->phi += v_error[0];
	estimated_states->theta += v_error[1];
	estimated_states->psi += v_error[2];
	
// 	estimated_states->phi += L.data[0][0]*error;
// 	estimated_states->theta += L.data[1][0]*error;
// 	estimated_states->psi += L.data[2][0]*error;
	

		  mat_create_identity( &temp1, 3, 3 );
	
	 
	 // Now update the covariance estimate (Sigma = (I - L*C)*Sigma
	 mat_mult(&L,C,&temp2);
	 mat_scalar_mult(-1,&temp2,&temp2);
	 mat_add(&temp1,&temp2,&temp1);
	 mat_mult(&temp1,&estimated_states->Sigma, &estimated_states->Sigma);
}

/*******************************************************************************
* Function Name  : compute_euler_angles
* Input          : AHRS_states* states
* Output         : None
* Return         : None
* Description    : Converts quaternion attitude estimate to euler angles (yaw, pitch, roll)
*******************************************************************************/
#define _RAD2DEG 57.295779513082320876798154814105f
void compute_euler_angles( AHRS_state_data* estimated_states )
{
	 float q0, q1, q2, q3;
	 
	 q0 = estimated_states->qib.a;
	 q1 = estimated_states->qib.b;
	 q2 = estimated_states->qib.c;
	 q3 = estimated_states->qib.d;	 

	 estimated_states->phi = atan2(2*(q0*q1 + q2*q3),q3*q3 - q2*q2 - q1*q1 + q0*q0)*_RAD2DEG;
	 estimated_states->theta = -asin(2*(q1*q3 - q0*q2))*_RAD2DEG;
	 estimated_states->psi = atan2(2*(q0*q3+q1*q2),q1*q1 + q0*q0 - q3*q3 - q2*q2)*_RAD2DEG;
}

/*******************************************************************************
* Function Name  : unroll_states
* Input          : AHRS_states* states
* Output         : None
* Return         : None
* Description    : Keeps all angle estimates in the range of -360 to 360 degrees
*******************************************************************************/
void unroll_states( AHRS_state_data* states )
{
	 while( states->phi > 360 )
	 {
		  states->phi -= 360;
	 }
	 while( states->phi < -360 )
	 {
		  states->phi += 360;
	 }
	 
	 while( states->theta > 360 )
	 {
		  states->theta -= 360;
	 }
	 while( states->theta < -360 )
	 {
		  states->theta += 360;
	 }
	 
	 while( states->psi > 360 )
	 {
		  states->psi -= 360;
	 }
	 while( states->psi < -360 )
	 {
		  states->psi += 360;
	 }
	 
}
