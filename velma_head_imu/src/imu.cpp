#include "imu.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "impconst.h"

#include <iostream>

using boost::lexical_cast;

IMU::IMU(const std::string& port, int baud)
{
	connected = false;
	dlen = 0;

	gyro_factor = 0.05 * M_PI / 180.0;
	acc_factor = 0.000333 * lib::G_ACC;
	imu_step = 0.0012;

	fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0) {
		tcgetattr(fd, &oldtio);

		// set up new settings
		struct termios newtio;
		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = CBAUD | CS8 | CLOCAL | CREAD;
		newtio.c_iflag = INPCK; //IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		if (cfsetispeed(&newtio, baud) < 0 || cfsetospeed(&newtio, baud) < 0) {
			fprintf(stderr, "Failed to set serial baud rate: %d\n", baud);
			tcsetattr(fd, TCSANOW, &oldtio);
			close(fd);
			fd = -1;
			return;
		}
		// activate new settings
		tcflush(fd, TCIFLUSH);
		tcsetattr(fd, TCSANOW, &newtio);
		connected = true;
	}
}

IMU::~IMU()
{
	// restore old port settings
	if (fd > 0)
		tcsetattr(fd, TCSANOW, &oldtio);
	close(fd);
}

void IMU::setFactors(double gf, double af)
{
	gyro_factor = gf;
	acc_factor = af;
}

ImuData IMU::getReading()
{
	for (;;) {
		fd_set rfds;

		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);

		// timeout
		struct timeval timeout;
		timeout.tv_sec = (time_t) 0;
		timeout.tv_usec = 50000;

		if(dlen < 0)
			dlen = 0;

		int select_retval = select(fd + 1, &rfds, NULL, NULL, &timeout);

		if (select_retval < 0) {
			printf("imu 16362 <0 !! \n");
			//	throw std::runtime_error("imu 16362 <0 !!!");
		} else if (select_retval == 0) {
			printf("imu 16362 ==0 timeout !!! \n");
			//	throw std::runtime_error("imu 16362 ==0  timeout !!!");
		} else {
			//printf("imu 16362 >0  \n");
			int ret = read(fd, data + dlen, 50 - dlen);
			if (ret <= 0)
				continue;
			//	printf("imu 16362 >0 2  \n");
			dlen += ret;
			//printf("dlen = %d\n", dlen);
			bool myexit = false;
			for (unsigned int i = 0; i < dlen; i++) {

				if (data[i] == '\n') {
					if (i > 21) {
						//printf("interpret\n");
						interpret(data + i - 22);
						myexit = true;
					}
					unsigned int k = 0;

					for (unsigned int j = i + 1; j < dlen; j++) {
						data[k++] = data[j];
					}

					dlen = k;

					i = 0;

				}
			}

			if (myexit) {
				//	printf("imu 16362 >0 3  \n");
				return imu_data;
			}

		}
	}
}

int16_t IMU::getShort14(char * dat)
{
	int16_t val = 0;

	((char *) &val)[0] = dat[1];
	((char *) &val)[1] = dat[0];

	val = val & 0x3FFF;

	if (val > (16383 / 2 - 1))
		val = val - 16383;
	return val;
}

int16_t IMU::getShort12(char * dat)
{
	int16_t val = 0;

	((char *) &val)[0] = dat[1];
	((char *) &val)[1] = dat[0];

	val = val & 0x0FFF;

	if (val > (4095 / 2 - 1))
		val = val - 4095;
	return val;
}

void IMU::interpret(char * dat)
{
	int16_t sup, gyroX, gyroY, gyroZ, acclX, acclY, acclZ, tempX, tempY, tempZ;

	static ImuData last_imu_data;

	sup = getShort12(&dat[0]);

	gyroX = getShort14(&dat[2]);
	gyroY = getShort14(&dat[4]);
	gyroZ = getShort14(&dat[6]);

	acclX = getShort14(&dat[8]);
	acclY = getShort14(&dat[10]);
	acclZ = getShort14(&dat[12]);

	tempX = getShort12(&dat[14]);
	tempY = getShort12(&dat[16]);
	tempZ = getShort12(&dat[18]);

	imu_data.angularVelocity[0] = gyroX * gyro_factor;
	imu_data.angularVelocity[1] = gyroY * gyro_factor;
	imu_data.angularVelocity[2] = gyroZ * gyro_factor;

	imu_data.linearAcceleration[0] = acclX * acc_factor;
	imu_data.linearAcceleration[1] = acclY * acc_factor;
	imu_data.linearAcceleration[2] = acclZ * acc_factor;

	imu_data.angularAcceleration[0] = -(imu_data.angularVelocity[0] - last_imu_data.angularVelocity[0]) / imu_step;
	imu_data.angularAcceleration[1] = -(imu_data.angularVelocity[1] - last_imu_data.angularVelocity[1]) / imu_step;
	imu_data.angularAcceleration[2] = -(imu_data.angularVelocity[2] - last_imu_data.angularVelocity[2]) / imu_step;

	last_imu_data = imu_data;

//	imu.tempX = 25.0 + tempX * 0.136;
//	imu.tempY = 25.0 + tempY * 0.136;
//	imu.tempZ = 25.0 + tempZ * 0.136;

}
