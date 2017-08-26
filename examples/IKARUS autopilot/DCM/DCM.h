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

#define ToRad(x) (x*0.01745329252f)  // *pi/180
#define ToDeg(x) (x*57.2957795131f)  // *180/pi

//#define Kp_ROLLPITCH 0.8f
//#define Ki_ROLLPITCH 0.00002f

/*
#define Kp_ROLLPITCH 1.2f
#define Ki_ROLLPITCH 0.00002f
*/
typedef struct{
	float kp;
	float ki;
} DCM_Config;

#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f


void DCM_Init(DCM_Config* dcm);
void Normalize(void);
void Drift_correction(void);
void Accel_adjust(void);
void Matrix_update(void);
void Euler_angles(void);
void Calculate_DCM(void);
void Euler_angles(void);
