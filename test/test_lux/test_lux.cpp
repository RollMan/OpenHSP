extern void I2C_Init( void );
extern void I2C_Close( int ch );
extern void I2C_Term( void );
extern int I2C_Open( int ch, int adr );
extern int I2C_ReadByte( int ch );
extern int I2C_ReadWord( int ch );
extern int I2C_WriteByte( int ch, int value, int length );

#include <iostream>
#include <cstdlib>
#include <unistd.h>

using std::cout;
using std::cerr;
using std::endl;

void init_tsl2572_with_gain(int gain);
void config_tsl2572(int lux_idx, unsigned char *value, int len);
void convert_int_lux(const int c0data, const int c1data, const int ATIME, const int AGAIN, const int GA, const int CPL);

int main(){
	unsigned char value[16];
	const char COM = 1 << 7;
	const char AUTO_INC = 1 << 5;

	const int lux_idx = 0;

	init_tsl2572_with_gain(1);

	value[0] = COM | AUTO_INC | 0x14;
	config_tsl2572(lux_idx, value, 1);

	for(int loop = 0; loop < 20; loop++){
		usleep(50000);
		int lux = I2C_ReadWord(lux_idx);
		std::cout << std::hex << lux << endl;
	}
}

void init_tsl2572_with_gain(int gain){
	if(!(0 <= gain && gain <= 3)){
		std::cerr << "gain range must be 0 <= gain <= 3 but " << gain << std::endl;
		exit(2);
	}

	const int lux_slaveaddr = 0x39;
	const int lux_idx = 0;
	unsigned char value[16];
	I2C_Init();
	if(I2C_Open(lux_idx, lux_slaveaddr) != 0){
		std::cerr << "Failed to open TSL25721 with I2C" << std::endl;
		exit(1);
	}

	{
		const char COM = 1 << 7;
		const char AUTO_INC = 1 << 5;
		const char ID = 0x12;
		value[0] = COM | ID;
		config_tsl2572(lux_idx, value, 1);
		int id = I2C_ReadByte(lux_idx);
		std::cout << "id: " << id << std::endl;
	}

	const int POWERON = 1 << 0;
	const int AEN = 1 << 1;

	{
		const int EN_REG = 0x00;
		value[0] = 0x80 | EN_REG;
		value[1] = POWERON;
		config_tsl2572(lux_idx, value, 2);

		usleep(30000);

		value[0] = 0x80 | EN_REG;
		value[1] = POWERON | AEN;
		config_tsl2572(lux_idx, value, 2);
	}

	{
		const int ATIME_REG = 0x01;
		value[0] = 0x80 | ATIME_REG;
		value[1] = 0xED; // 50ms
		// value[1] = 0xFF; // 1ms
		config_tsl2572(lux_idx, value, 2);
	}


	/*
	const int AGL = 1 << 2;

	{
		const int CONF_REG = 0x0D;
		value[0] = 0x80 | CONF_REG;
		value[1] = AGL;
		config_tsl2572(lux_idx, value, 2);
	}
	*/


	{
		const int CTRL_REG = 0x0f;
		value[0] = 0x80 | CTRL_REG;
		value[1] = gain;
		config_tsl2572(lux_idx, value, 2);
	}
}

void config_tsl2572(int lux_idx, unsigned char *value, int len){
	int data = 0;
	for(int i = 0; i < len; i++){
		data |= (value[i] << (8*i));
	}
	int stat = I2C_WriteByte(lux_idx, data, len);
	if(stat != 0){
		std::cerr << "Failed to write by I2C: " << stat << std::endl;
		exit(1);
	}
}

void convert_int_lux(const int c0data, const int c1data, const int ATIME, const int AGAIN, const int GA, const int CPL);
