#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "navXTimeSync/AHRS.h"

using namespace std::chrono_literals;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

class IMUNode : public rclcpp::Node
{
    public:
        // Create ROS Node that manages IMU data
        IMUNode()
        : Node("imu_node"), count_(0)
        {
            // Create publisher to continuously publish IMU data
            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu",10);
            // Create timer to manage continuous process and sending of IMU data
            timer_ = this->create_wall_timer(10ms, std::bind(&IMUNode::run_loop, this));
        }

    private:
        void run_loop()
        {
            // Create message object that stores the current IMU data
            auto message = sensor_msgs::msg::Imu();

            // Process current orientation (as quaternion)
            message.orientation.x = com.GetQuaternionX();
            message.orientation.y = com.GetQuaternionY();
            message.orientation.z = com.GetQuaternionZ();
            message.orientation.w = com.GetQuaternionW();

            // Determine angular velocity based on angular change between successive time
            // steps
            int64_t curr_imu_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            float curr_pitch = com.GetPitch();
            float curr_roll = com.GetRoll();
            float curr_yaw = com.GetYaw();
            message.angular_velocity.x = (curr_roll-prev_roll)/(curr_imu_time-prev_imu_time)*2000*M_PI/360;
            message.angular_velocity.y = (curr_pitch-prev_pitch)/(curr_imu_time-prev_imu_time)*2000*M_PI/360;
            message.angular_velocity.z = (curr_yaw-prev_yaw)/(curr_imu_time-prev_imu_time)*2000*M_PI/360;
            prev_imu_time = curr_imu_time;
            prev_pitch = curr_pitch;
            prev_roll = curr_roll;
            prev_yaw = curr_yaw;

            // Process linear acceleration and convert from G to m/s2
            message.linear_acceleration.x = com.GetWorldLinearAccelX()*G_TO_MS2_FACTOR;
            message.linear_acceleration.y = com.GetWorldLinearAccelY()*G_TO_MS2_FACTOR;
            message.linear_acceleration.z = com.GetWorldLinearAccelZ()*G_TO_MS2_FACTOR;

            // Publish message
            imu_publisher_->publish(message);
        }
        // Create publisher object
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        // Create timer object
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        // Establish connection to IMU via navXTimeSync library
        char IMU_PORT[13] = "/dev/ttyACM1";
        AHRS com = AHRS(IMU_PORT);
        float G_TO_MS2_FACTOR = 9.80665;

        // Declare variables to store angular data (used to compute angular velocity)
        int64_t prev_imu_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        float prev_pitch = com.GetPitch();
        float prev_roll = com.GetRoll();
        float prev_yaw = com.GetYaw();
};

int main(int argc, char * argv[])
/*
Main function to call IMU Node.
*/
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUNode>());
    rclcpp::shutdown();
    return 0;
}