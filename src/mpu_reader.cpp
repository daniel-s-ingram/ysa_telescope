#include <fstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>

int main(int argc, char **argv)
{
	std_msgs::Int16MultiArray mpu_msg;
	char accel_data[6], gyro_data[6];
	char accelAddress[1] = {0x3B};
	char gyroAddress[1] = {0x43};
	int i, mpu_dev;
	struct spi_ioc_transfer xfer;

	//Create ROS node with publisher for raw mpu data (to be processed by another node)
	ros::init(argc, argv, "mpu9255");
	ros::NodeHandle nh;
	ros::Publisher mpu_pub = nh.advertise<std_msgs::Int16MultiArray>("/mpu_raw_data", 10);

	//Allocate space in message for raw accelerometer and gyroscope data
	mpu_msg.data.clear();
	for (i = 0; i < 6; i++) imu_data.data.push_back(0);

	//Attempt to establish communication with MPU9255
	if ((mpu_dev = open("/dev/spidev1.1", O_RDWR)) < 0)
	{
		perror("Failed to open MPU");
		return 1;
	}

	//Read and publish data as long as roscore is running
	while (ros::ok())
	{
		mpu_msg.data[0] = (accel_data[0] << 8) + accel_data[1];
		mpu_msg.data[1] = (accel_data[2] << 8) + accel_data[3];
		mpu_msg.data[2] = (accel_data[4] << 8) + accel_data[5];

		mpu_msg.data[3] = (gyro_data[0] << 8) + gyro_data[1];
		mpu_msg.data[4] = (gyro_data[2] << 8) + gyro_data[3];
		mpu_msg.data[5] = (gyro_data[4] << 8) + gyro_data[5];

		try
		{
			mpu_pub.publish(mpu_msg);
		}
		catch (ros::Exception &e)
		{
			ROS_ERROR("Error occured: %s ", e.what());
		}
	}

	close(mpu)
	return 0;
}


