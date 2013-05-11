#ifndef IMU_HPP
#define IMU_HPP

#include <stdint.h>
#include <termios.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <fstream>

#include "imudata.hpp"

class IMU
{
public:
	IMU(const std::string& port = "/dev/ttyUSB0", int baud = B500000);
	~IMU();

	// Block until new measurement is avalible
	ImuData getReading();

	void setFactors(double gf, double af);

	bool connected;
protected:

private:
	int fd;
	struct termios oldtio;


	double gyro_factor;
	double acc_factor;

	char data[50];
	int dlen;

	double imu_step;

	ImuData imu_data;

	int16_t getShort14(char * dat);
	int16_t getShort12(char * dat);
	void interpret(char * dat);

};

#endif /* IMU_HPP */
