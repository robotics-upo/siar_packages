#ifndef __ARDUIMU_V3_HPP__
#define __ARDUIMU_V3_HPP__

#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdio.h>

//!Arduimu raw data
struct ArduImuRawData
{
	uint32_t 	time;
	bool		accgyro_update;
	bool		mag_update;
	double		acc_x;			// Accel in g
	double		acc_y;
	double		acc_z;
	double		gyr_x; 			// Gyro in º/s
	double		gyr_y;
	double		gyr_z;
	double 		mag_x;			// Mag in mG
	double 		mag_y;
	double 		mag_z;
};

class ArduimuDev
{
public:

	//!Intialize the serial port to the given values
	ArduimuDev(const char *pDev)
	{
		struct termios my_termios;

		// Open the port in read-write mode 
		m_portHandler = open(pDev, O_RDWR | O_NOCTTY);
		if(m_portHandler < 0)
			return;

		// Get the port attributes and flush all data on queues
		tcgetattr(m_portHandler, &my_termios);
		tcflush(m_portHandler, TCIOFLUSH);
	
		// Setup the communication 
		my_termios.c_iflag &= ~(BRKINT | IGNPAR | PARMRK | INPCK |ISTRIP | IXON | INLCR | IGNCR | ICRNL);
		my_termios.c_iflag |= IGNBRK | IXOFF;
		my_termios.c_oflag &= ~(OPOST);
		my_termios.c_cflag |= CLOCAL | CREAD;
		my_termios.c_cflag &= ~PARENB;
		my_termios.c_cflag |= CS8;
		my_termios.c_cflag &= ~CSTOPB;
		my_termios.c_cflag &= ~CRTSCTS;    
		my_termios.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ICANON | NOFLSH | TOSTOP | ISIG | IEXTEN);
		my_termios.c_cc[VMIN]=1;	//Each simple read call will be blocked until recive at least one byte
		my_termios.c_cc[VTIME]=0;	//No timeout for reading
		cfsetispeed(&my_termios, B115200);
		cfsetospeed(&my_termios, B115200);
		tcsetattr(m_portHandler, TCSANOW, &my_termios);
	}
	
	//!Default destructor
	~ArduimuDev(void)
	{
		ArduimuDev::finish();
	}
	
	//!Read (blocking) a complete message from the serial port
	int readData(ArduImuRawData &data)
	{
		int dataRead, size;
		char c;

		// Find the biginning of the stream 0x23
		c = 0;
		while(c != '#')
		{
			dataRead = read(m_portHandler, &c, 1);
			if(dataRead < 0)
				return -5;
		}

		// Conitue reading until 0x2A is received
		c = 0;
		size = 0;
		while(c != '*')
		{
			dataRead = read(m_portHandler, &c, 1);
			if(dataRead < 0)
				return -5;
			
			m_readBuff[size++] = c;			
			if(size == 256)
				size = 0;
		}
		
		// Parse message
		if(size != 23 && size != 17)
			return -1;
		data.time =  ((uint32_t *)m_readBuff)[0];
		data.acc_x = -((int16_t *)m_readBuff)[2]/8192.0;
		data.acc_y = -((int16_t *)m_readBuff)[3]/8192.0;
		data.acc_z = ((int16_t *)m_readBuff)[4]/8192.0;
		data.gyr_x = -((int16_t *)m_readBuff)[5]/65.5;
		data.gyr_y = -((int16_t *)m_readBuff)[6]/65.5;
		data.gyr_z = ((int16_t *)m_readBuff)[7]/65.5;
		data.mag_x = 0;
		data.mag_y = 0;
		data.mag_z = 0;
		data.accgyro_update = true;
		data.mag_update = false;
		if(size == 23)
		{
			data.mag_x = -((int16_t *)m_readBuff)[8];
			data.mag_y = -((int16_t *)m_readBuff)[9];
			data.mag_z = ((int16_t *)m_readBuff)[10];
			data.mag_update = true;
		}
				
		return 1;
	}

	//!Finish the serial communication 
	void finish(void)
	{
		// Close the port 
		if(m_portHandler > 0)
		{
			close(m_portHandler);
			m_portHandler = 0;
		}
	}

protected:
	
	//!Serial port hadler
	int m_portHandler;	
	
	//!Reading buffer
	char m_readBuff[256];
};


#endif




















