#include <fstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"

int main(int argc, char **argv)
{
	std_msgs::Int16MultiArray imu_data;
        char accel_data[6], gyro_data[6];
        char accelAddress[1] = {0x3B};
        char gyroAddress[1] = {0x43};
        int i,spiDev;
	struct spi_ioc_transfer xfer;

	if ((spiDev = open("/dev/spidev1.1", O_RDWR)) < 0)
	{
		perror("Failed to open spiDev");
		return 1;
	}

	memset(&xfer, 0, sizeof(xfer));

        ros::init(argc,argv,"mpu9255_reader");
        ros::NodeHandle n;

        ros::Publisher pub = n.advertise<std_msgs::Int16MultiArray>("mpu9255_raw_data",1000);

        imu_data.data.clear();
        for (i = 0;i < 6;i++) imu_data.data.push_back(0);

	while (ros::ok())
	{
		
	}

	return 0;
}


