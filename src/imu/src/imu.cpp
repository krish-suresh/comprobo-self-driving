#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "navXTimeSync/AHRS.h"

using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node
{
    public:
        IMUPublisher()
        : Node("imu_publisher"), count_(0)
        {
            imu_publisher_ = this->create_publisher<std_msgs::msg::Float64>("imu_yaw",10);
            timer_ = this->create_wall_timer(10ms, std::bind(&IMUPublisher::run_loop, this));
        }

    private:
        void run_loop()
        {
            curr_pitch = com.GetFusedHeading();
            RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", curr_pitch);
            auto message = std_msgs::msg::Float64();
            message.data = curr_pitch;
            imu_publisher_->publish(message);
        }
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr imu_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;

        char IMU_PORT[13] = "/dev/ttyACM0";
        AHRS com = AHRS(IMU_PORT);

        float curr_pitch;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}