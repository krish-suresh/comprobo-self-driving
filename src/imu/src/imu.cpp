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

class IMUPublisher : public rclcpp::Node
{
    public:
        IMUPublisher()
        : Node("imu_publisher"), count_(0)
        {
            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_yaw",10);
            timer_ = this->create_wall_timer(10ms, std::bind(&IMUPublisher::run_loop, this));
        }

    private:
        void run_loop()
        {
            auto message = sensor_msgs::msg::Imu();
            message.orientation.x = com.GetQuaternionX();
            message.orientation.y = com.GetQuaternionY();
            message.orientation.z = com.GetQuaternionZ();
            message.orientation.w = com.GetQuaternionW();

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
            // message.angular_velocity.x = com.GetRawGyroX()*2*M_PI/360;
            // message.angular_velocity.y = com.GetRawGyroY()*2*M_PI/360;
            // message.angular_velocity.z = com.GetRawGyroZ()*2*M_PI/360;

            message.linear_acceleration.x = com.GetWorldLinearAccelX()*G_TO_MS2_FACTOR;
            message.linear_acceleration.y = com.GetWorldLinearAccelY()*G_TO_MS2_FACTOR;
            message.linear_acceleration.z = com.GetWorldLinearAccelZ()*G_TO_MS2_FACTOR;
            imu_publisher_->publish(message);
        }
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        char IMU_PORT[13] = "/dev/ttyACM0";
        AHRS com = AHRS(IMU_PORT);
        float G_TO_MS2_FACTOR = 9.80665;

        int64_t prev_imu_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        float prev_pitch = com.GetPitch();
        float prev_roll = com.GetRoll();
        float prev_yaw = com.GetYaw();
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}