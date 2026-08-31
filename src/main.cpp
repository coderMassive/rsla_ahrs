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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace VN;

#define PUBLISH_TIME_MS 1
#define AHRS_PORT "/dev/ttyUSB0"

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
        sensor_.disconnect();
    }

  private:
    void init_sensor()
    {
      Error latestError = sensor_.autoConnect(AHRS_PORT);
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
      Error latestError = sensor_.readRegister(&quaternionRegister);
      if (latestError != Error::None)
      {
        std::ostringstream oss;
        oss << latestError;
        RCLCPP_ERROR(this->get_logger(), oss.str().c_str());
        return;
      }

      tf2::Quaternion q_orig(
        quaternionRegister.quatX,
        quaternionRegister.quatY,
        quaternionRegister.quatZ,
        quaternionRegister.quatS
      );

      tf2::Quaternion q_rot(0, 0, -0.7071, 0.7071);
      tf2::Quaternion q_earth = q_orig * q_rot;
      
      if (!firstEnsemble) {
          double roll, pitch, yaw;
          tf2::Matrix3x3(q_earth).getRPY(roll, pitch, yaw);
          forward_.setRPY(0.0, 0.0, yaw);
      }

      tf2::Quaternion q = forward_.inverse() * q_earth;

      publisher_->publish(tf2::toMsg(q));
      if (firstEnsemble == false) {
        RCLCPP_INFO(this->get_logger(), "AHRS node is publishing succesfully");
        firstEnsemble = true; 
      }
    }

  //Variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_;
  Sensor sensor_;
  tf2::Quaternion forward_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AHRSPublisher>());
  rclcpp::shutdown();
  return 0;
}
