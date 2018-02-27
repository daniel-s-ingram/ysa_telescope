#include <fstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"

#define MPU_ADDRESS 0x68

int main(int argc, char **argv)
{
	std_msgs::Int16MultiArray imu_data;
	char accel_data[6], gyro_data[6];
	char accelAddress[1] = {0x3B};
	char gyroAddress[1] = {0x43};
	int i,file;

	if ((file = open("/dev/i2c-1", O_RDWR)) < 0)
	{
		printf("Failed to connect to device\n");
		return 1;
	}

	if(ioctl(file, I2C_SLAVE, MPU_ADDRESS) < 0)
        {
                printf("Failed to set slave address: %m\n" );
                return 1;
        }

	ros::init(argc,argv,"mpu9255_reader");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Int16MultiArray>("mpu9255_raw_data",1000);

	imu_data.data.clear();
	for (i = 0;i < 6;i++) imu_data.data.push_back(0);

	while(ros::ok())
	{
		//Read the accelerometer data, value for x, y, and z directions stored as a high byte and a low byte in addresses 0x3B - 0x40
		if (write(file, accelAddress, 1) != 1)
		{
			printf("Failed to reset read address\n");
			return 1;
		}
		if (read(file, accel_data, 6) != 6)
		{
			printf("Failed to read from device\n");
			return 1;
		}
		
		//Read the gyroscope data, stored in addresses 0x43 - 0x49
                if (write(file, gyroAddress, 1) != 1)
                {
                        printf("Failed to reset read address\n");
                        return 1;
                }
                if (read(file, gyro_data, 6) != 6)
                {
                        printf("Failed to read from device\n");
                        return 1;
                }


		imu_data.data[0] = (accel_data[0] << 8) + accel_data[1];
  		imu_data.data[1] = (accel_data[2] << 8) + accel_data[3];
  		imu_data.data[2] = (accel_data[4] << 8) + accel_data[5];
  		imu_data.data[3] = (gyro_data[0] << 8) + gyro_data[1];
  		imu_data.data[4] = (gyro_data[2] << 8) + gyro_data[3];
  		imu_data.data[5] = (gyro_data[4] << 8) + gyro_data[5];

		pub.publish(imu_data);
	}

	close(file);
	return 0;
}


