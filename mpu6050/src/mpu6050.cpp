#include "rclcpp/rclcpp.hpp"
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>



class mpu6050 : public rclcpp::Node
{

    public:

        mpu6050() : Node("mpu6050"){};

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imupub = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 100);

        double f=10.0;
        std::chrono::duration <double, std::ratio<1,1000>> frequency{f};
        std::shared_ptr<rclcpp::TimerBase> timer = this->create_wall_timer(frequency, std::bind(&mpu6050::run, this));

        void imu()
        {
            
        }

        void run(){};

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

