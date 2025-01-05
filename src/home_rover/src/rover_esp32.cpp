#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>

class rover_esp32 : public rclcpp::Node {
public:
    rover_esp32() : Node("rover_esp32"), i2cAddress(0x20) {
        this->declare_parameter<int>("frequency", 20);
        int timer_frequency = this->get_parameter("frequency").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Node initializing i2c");

        i2c_device = wiringPiI2CSetup(i2cAddress);
        if (i2c_device == -1) {
            throw std::runtime_error("Failed to initialize I2C.");
        }

        RCLCPP_INFO(this->get_logger(), "I2C Initialized");

        publisher_leftEncDist = this->create_publisher<std_msgs::msg::Int32>("left_enc_dist", 10);
        publisher_rightEncDist = this->create_publisher<std_msgs::msg::Int32>("right_enc_dist", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1) / timer_frequency,
            std::bind(&rover_esp32::timerCallback, this)
        );
    }

private:
    void timerCallback() {
        
        int32_t speeds[2];
        int nb = wiringPiI2CRawRead(i2c_device, (uint8_t*)speeds, sizeof(speeds));
        if(nb != sizeof(speeds)) 
        {
            // Log error with number of bytes read
            RCLCPP_ERROR(this->get_logger(), "Failed to read from I2C. Number of bytes read: %d", nb);
            return;
        }
        
        auto message = std_msgs::msg::Int32();
        message.data = speeds[0];
        publisher_leftEncDist->publish(message);

        message.data = speeds[1];
        publisher_rightEncDist->publish(message);
    }

    int i2cAddress;
    int i2c_device;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_leftEncDist;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_rightEncDist;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rover_esp32>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
