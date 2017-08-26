# INS_EIF
Extended Information Filter for Inertial Navigation: UAV Attitude determination with multisensor/multirate systems

## Synopsis
Matlab development and Autogenerated code for an Inertial navigation system using Extended Information Filter for multi IMU/GPS and multi sampling rates architecture.

Code implemented and run in SMT32F103:
![INS board](https://user-images.githubusercontent.com/10338030/29743720-319ca840-8a8f-11e7-9134-071a7a2c7d27.jpg)

## Requirements

* Requires Matlab 2015a or above


## Installation

## Usage
See *examples/IKARUS autopilot/main.c* for reference.
### Declarations and initialization 
* Add libraries
```
#include "EIF_n_dof.h"
#include "EIF_init.h" 
```
* Initialize filter variables:
```
 //EIF  variables
float ypr_eif[2];	
real_T eul[3] ={0,0,0};
int32_T number_sensors = 2; // Number of sensor being used
char_T sensors[8] ={ 'g', 'y', 'r', 'o' , 'a', 'c', 'c', 'e' };// Add here which ever number and type of sensors

real_T X[7],X_out[7];
real_T P[49],P_out[49];
real_T Q[36];
real_T sigmas[2];

real_T AT;// loop delta time
real_T measures[6];
 ```
 * Initialize EIF:
 ```
 EIF_init( eul, X, P, Q,sigmas);
 ```
 Notes: See *EIF_init/EIF_init.c* for initialization tuning
 ### Main loop at AT rate:
 * Compute AT in seconds: Elapsed delta time since last execution. See RTOS funtionalities for loop rate time
 * Execute filter:
 ```
EIF_n_dof( (const real_T*) X,(const real_T*) P,(const real_T*)Q,(const real_T*) sigmas,(const real_T*) measures , (const real_T) AT,  eul,X_out, P_out);	   // AT has to be defined correctly!!!!

// Update X and P
memcpy(X,X_out,7*sizeof(real_T));
memcpy(P,P_out,49*sizeof(real_T));	
  
ypr_eif[0] = eul[1]*(180.0/PI);	 // Pitch in degs
ypr_eif[1] = eul[0]*(180.0/PI);	 // Roll in degs
eul[2]*(180.0/PI);                // Yaw/heading in degs.

//EIF angles
pitch = ypr_eif[0];
roll = ypr_eif[1];
 ```

## Tests and Demo Video
[![INF with EIF](https://user-images.githubusercontent.com/10338030/29743693-c57caff2-8a8e-11e7-8b1c-0b19fa42d64c.png)](https://www.dropbox.com/s/rxmxjgcjtwu1rox/MOV_1099.mp4?dl=0)

## Contributors

@David Torres Ocaña
    david.torres.ocana@gmail.com
