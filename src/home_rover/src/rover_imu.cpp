#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>


#define MPU6050_REG_PWR_MGMT_1 (0x6b)
#define MPU6050_REG_DATA_START (0x3b)
#define A_SCALE (16384.0)
#define ANG_SCALE (131.0)

class rover_imu : public rclcpp::Node {
public:
    rover_imu() : Node("rover_imu"), i2cAddress(0x68) {
        this->declare_parameter<int>("frequency", 50);
        int timer_frequency = this->get_parameter("frequency").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Node initializing i2c");

        i2c_device = wiringPiI2CSetup(i2cAddress);
        if (i2c_device == -1) {
            throw std::runtime_error("Failed to initialize I2C.");
        }

        RCLCPP_INFO(this->get_logger(), "I2C Initialized");

        
        // Perform I2C work, this is to wake up the MPU6050 ?
        wiringPiI2CWriteReg8(i2c_device, MPU6050_REG_PWR_MGMT_1, 0);

        publisher_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1) / timer_frequency,
            std::bind(&rover_imu::timerCallback, this)
        );
    }

private:
    void timerCallback() {
        
        uint8_t msb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START);
        uint8_t lsb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+1);
        short accelX = msb << 8 | lsb;

        msb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+2);
        lsb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+3);
        short accelY = msb << 8 | lsb;

        msb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+4);
        lsb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+5);
        short accelZ = msb << 8 | lsb;

        //msb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+6);
        //lsb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+7);
        //short temp = msb << 8 | lsb;

        msb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+8);
        lsb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+9);
        short gyroX = msb << 8 | lsb;

        msb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+10);
        lsb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+11);
        short gyroY = msb << 8 | lsb;

        msb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+12);
        lsb = wiringPiI2CReadReg8(i2c_device, MPU6050_REG_DATA_START+13);
        short gyroZ = msb << 8 | lsb;

        auto message = sensor_msgs::msg::Imu();
        message.angular_velocity.x = accelX / A_SCALE;
        message.angular_velocity.y = accelY / A_SCALE;
        message.angular_velocity.z = accelZ / A_SCALE;

        message.linear_acceleration.x = gyroX / ANG_SCALE;
        message.linear_acceleration.y = gyroY / ANG_SCALE;
        message.linear_acceleration.z = gyroZ / ANG_SCALE;
        publisher_imu->publish(message);
    }

    int i2cAddress;
    int i2c_device;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rover_imu>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
