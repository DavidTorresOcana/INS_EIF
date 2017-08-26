
#define HMC5883_ADDR		0x1E

//#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define ModeRegister         0x02
#define DataOutputXMSB       0x03
#define DataOutputXLSB       0x04
#define DataOutputZMSB       0x05
#define DataOutputZLSB       0x06
#define DataOutputYMSB       0x07
#define DataOutputYLSB       0x08
#define StatusRegister       0x09
#define IDRegisterA          0x0A
#define IDRegisterB          0x0B
#define IDRegisterC          0x0C

// default gain value
#define magGain              0x20

// ModeRegister valid modes
#define ContinuousConversion 0x00
#define SingleConversion     0x01
#define IDLEConversion		 0x10

// ConfigRegA valid sample averaging
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x20
#define SampleAveraging_4    0x40
#define SampleAveraging_8    0x60

// ConfigRegA valid data output rates
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x04
#define DataOutputRate_3HZ    0x08
#define DataOutputRate_7_5HZ  0x0C
#define DataOutputRate_15HZ   0x10
#define DataOutputRate_30HZ   0x14
#define DataOutputRate_75HZ   0x18

// ConfigRegA valid measurement configuration bits
#define NormalOperation      0x00	 // 0x10
#define PositiveBiasConfig   0x01 	// 0x11
#define NegativeBiasConfig   0x02	// 0x12





void HMC5883_Init(void);

Mag_Data HMC5883_Read(void);
