
struct GGA
{
	float hora;
	float lon;
	float lat;
	char fix;
	char nsats;
	float hdop;
	float alt;
};

struct RMC
{
	float hora;
	char active;
	float lon;
	float lat;
	float knots;
	float rumbo;
};

typedef struct {
	float lat;
	float lon;
}GPScoord;

typedef struct 
{
	float lon;
	float lat;
	float alt;
	float knots;
	float rumbo;
//	float mag_desv;
	char fix;
	char act;
	float hdop;
	int numsats;
	float hora;
	long fecha;

	// estado
	char conected;
	char nmea_ok;
	char pos_valid;

	char RMC_received;
	char GGA_received;
	unsigned char bad_crc;
	unsigned char uart_timeout;
	unsigned char nmea_timeout;

	// filtradas
	float alt_filter;
	float knots_filtered;

	// calculadas	
	float knotsMAX;
	float altitudeMAX;
	char en_movimiento;
}GPSInfo;

void nmea_fsm(char uart_data);

void USART2_Init(void);
