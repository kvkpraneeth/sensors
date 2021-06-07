#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>

#define Device_Address 0x68

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

class mpu6050 : public rclcpp::Node
{

        public: mpu6050() : Node("mpu6050")
        {
            fd = wiringPiI2CSetup(Device_Address); 
            wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);
            wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);
            wiringPiI2CWriteReg8 (fd, CONFIG, 0);       
            wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24); 
            wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);    
        }

        private: rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imupub = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 100);

        private: double f=10.0;
                 std::chrono::duration <double, std::ratio<1,1000>> frequency{f};
                 std::shared_ptr<rclcpp::TimerBase> timer = this->create_wall_timer(frequency, std::bind(&mpu6050::run, this));

        private: int fd;
                 float Acc_x,Acc_y,Acc_z;
                 float Gyro_x,Gyro_y,Gyro_z;
                 float Ax=0, Ay=0, Az=0;
                 float Gx=0, Gy=0, Gz=0;

        public: geometry_msgs::msg::Quaternion init;

        private: sensor_msgs::msg::Imu imu;

        private: short read_raw_data(int addr)
        {
            short high_byte,low_byte,value;
            high_byte = wiringPiI2CReadReg8(fd, addr);
            low_byte = wiringPiI2CReadReg8(fd, addr+1);
            value = (high_byte << 8) | low_byte;
            return value;
        }       

        private: void read()
        {
            Acc_x = read_raw_data(ACCEL_XOUT_H);
            Acc_y = read_raw_data(ACCEL_YOUT_H);
            Acc_z = read_raw_data(ACCEL_ZOUT_H);
            
            Gyro_x = read_raw_data(GYRO_XOUT_H);
            Gyro_y = read_raw_data(GYRO_YOUT_H);
            Gyro_z = read_raw_data(GYRO_ZOUT_H);
            
            Ax = Acc_x/16384.0 * 9.80665;
            Ay = Acc_y/16384.0 * 9.80665;
            Az = Acc_z/16384.0 * 9.80665;
            
            Gx = Gyro_x/131;
            Gy = Gyro_y/131;
            Gz = Gyro_z/131;
        }

        private: void write()
        {
            imu.header.frame_id = "imu";
            imu.header.stamp = now();

            imu.angular_velocity.x = Gx;
            imu.angular_velocity.y = Gy;
            imu.angular_velocity.z = Gz;
        
            imu.linear_acceleration.x = Ax;
            imu.linear_acceleration.y = Ay;
            imu.linear_acceleration.z = Az;

            imupub->publish(imu);
        }

        private: void run()
        {
            read(); 
            write();
        }

};

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<mpu6050> imuNode = std::make_shared<mpu6050>();

    exe.add_node(imuNode->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}

