#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "vectornav/Interface/Errors.hpp"
#include "vectornav/Interface/Registers.hpp"
#include "vectornav/Interface/Sensor.hpp"

using namespace VN;

#define PUBLISH_TIME_MS 1
#define AHRS_PORT "/dev/ttyUSB_AHRS"

bool firstEnsemble = false;

class AHRSPublisher : public rclcpp::Node
{
  public:
    AHRSPublisher() : Node("ahrs_publisher")
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/ahrs/quaternion", 10);
      timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_TIME_MS),
            std::bind(&AHRSPublisher::timer_callback, this));
      init_sensor();

      RCLCPP_INFO(this->get_logger(), "Running AHRS measurements");
    }

    ~AHRSPublisher()
    {
        sensor.disconnect();
    }

  private:
    void init_sensor()
    {
      Error latestError = sensor.autoConnect(AHRS_PORT);
      if (latestError != Error::None)
      {
        std::ostringstream oss;
        oss << latestError;
        RCLCPP_ERROR(this->get_logger(), oss.str().c_str());
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Connected to AHRS at %s", AHRS_PORT);
    }

    void timer_callback()
    {
      Registers::Attitude::Quaternion quaternionRegister;
      Error latestError = sensor.readRegister(&quaternionRegister);
      if (latestError != Error::None)
      {
        std::ostringstream oss;
        oss << latestError;
        RCLCPP_ERROR(this->get_logger(), oss.str().c_str());
        return;
      }

      geometry_msgs::msg::Quaternion quaternionMessage;
      quaternionMessage.x = quaternionRegister.quatX;
      quaternionMessage.y = quaternionRegister.quatY;
      quaternionMessage.z = quaternionRegister.quatZ;
      quaternionMessage.w = quaternionRegister.quatS;

      publisher_->publish(quaternionMessage);

      if (firstEnsemble == false){
        RCLCPP_INFO(this->get_logger(), "AHRS node is publishing succesfully");
        firstEnsemble = true; 
      }
    }

  //Variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_;
  Sensor sensor;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AHRSPublisher>());
  rclcpp::shutdown();
  return 0;
}
