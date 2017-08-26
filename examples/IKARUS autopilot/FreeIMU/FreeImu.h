
#define M_PI 3.14159265359f

//ORIGINALES
// #define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
// #define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

//  #define twoKpDef  (2.0f * 0.5f) // 2 * proportional gain
//  #define twoKiDef  (2.0f * 0.1f) // 2 * integral gain

typedef struct{
	float kp;
	float ki;
} FreeImu_Config;

void FreeImu_Init(FreeImu_Config * fimu_cfg);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void getQ(float * q);
void getEuler(float * angles) ;
void getYawPitchRoll(float * ypr);
float invSqrt(float number);
void getValues(float * values);
