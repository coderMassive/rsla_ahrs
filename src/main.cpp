#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "vectornav/Interface/Errors.hpp"
#include "vectornav/Interface/Registers.hpp"
#include "vectornav/Interface/Sensor.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace VN;

#define PUBLISH_TIME_MS 1
#define BINARY_RATE_DIVISOR 8   // base rate is 800hz so this makes it 100hz
#define AHRS_PORT "/dev/ttyUSB0"

// accel and gyro noise from the VN-100 datasheet (noise density x bandwidth).
static constexpr double ACCEL_VAR = 4.3e-4;
static constexpr double GYRO_VAR  = 9.9e-7;

bool firstEnsemble = false;

class AHRSPublisher : public rclcpp::Node
{
  public:
    AHRSPublisher() : Node("ahrs_publisher")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/ahrs/imu", 10);
      init_sensor();
      timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_TIME_MS),
            std::bind(&AHRSPublisher::timer_callback, this));

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
        RCLCPP_ERROR(this->get_logger(), "%s", oss.str().c_str());
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Connected to AHRS at %s", AHRS_PORT);

      // stream quaternion, accel, gyro and attitude uncertainty as one binary message cuz according to the manual it can only be read with binary
      binaryOutput_.asyncMode.emplace();
      binaryOutput_.asyncMode->serial1 = true;
      binaryOutput_.asyncMode->serial2 = true;
      binaryOutput_.rateDivisor = BINARY_RATE_DIVISOR;
      binaryOutput_.imu.accel = true;
      binaryOutput_.imu.angularRate = true;
      binaryOutput_.attitude.quaternion = true;
      binaryOutput_.attitude.yprU = true;

      latestError = sensor_.writeRegister(&binaryOutput_);
      if (latestError != Error::None)
      {
        std::ostringstream oss;
        oss << latestError;
        RCLCPP_ERROR(this->get_logger(), "Binary output config failed: %s", oss.str().c_str());
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Binary output configured");
    }

    void timer_callback()
    {
      while (true)
      {
        Sensor::CompositeDataQueueReturn data = sensor_.getNextMeasurement(false);
        if (!data) break;
        if (!data->matchesMessage(binaryOutput_)) continue;
        if (!data->attitude.quaternion.has_value() || !data->imu.accel.has_value() ||
            !data->imu.angularRate.has_value()) continue;

        Quat qs = data->attitude.quaternion.value();
        Vec3f a = data->imu.accel.value();
        Vec3f g = data->imu.angularRate.value();

        tf2::Quaternion q_orig(qs.vector[0], qs.vector[1], qs.vector[2], qs.scalar);

        tf2::Quaternion q_rot(0, 0, -0.7071, 0.7071);
        tf2::Quaternion q_earth = q_orig * q_rot;

        if (!firstEnsemble) {
            double roll, pitch, yaw;
            tf2::Matrix3x3(q_earth).getRPY(roll, pitch, yaw);
            forward_.setRPY(0.0, 0.0, yaw);
        }

        tf2::Quaternion q = forward_.inverse() * q_earth;
        q.normalize();

        // orientation was rotated by q_rot, so rotate the body-frame vectors the same way
        tf2::Quaternion q_rot_inv = q_rot.inverse();
        tf2::Vector3 accel = tf2::quatRotate(q_rot_inv, tf2::Vector3(a[0], a[1], a[2]));
        tf2::Vector3 gyro  = tf2::quatRotate(q_rot_inv, tf2::Vector3(g[0], g[1], g[2]));

        // build the sensor_msgs/Imu
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "ahrs_link";

        msg.orientation = tf2::toMsg(q);

        msg.angular_velocity.x = gyro.x();
        msg.angular_velocity.y = gyro.y();
        msg.angular_velocity.z = gyro.z();

        msg.linear_acceleration.x = accel.x();
        msg.linear_acceleration.y = accel.y();
        msg.linear_acceleration.z = accel.z();

        // orientation covariance from yprU
        if (data->attitude.yprU.has_value())
        {
          Vec3f u = data->attitude.yprU.value();
          const double d2r = M_PI / 180.0;
          double yawVar   = std::pow(u[0] * d2r, 2);
          double pitchVar = std::pow(u[1] * d2r, 2);
          double rollVar  = std::pow(u[2] * d2r, 2);

          // the 90 degree frame rotation swaps the sensor roll and pitch axes
          msg.orientation_covariance[0] = pitchVar;  // about x
          msg.orientation_covariance[4] = rollVar;   // about y
          msg.orientation_covariance[8] = yawVar;    // about z
        }

        // angular velocity and linear acceleration covariance (datasheet estimates)
        msg.angular_velocity_covariance[0] = GYRO_VAR;
        msg.angular_velocity_covariance[4] = GYRO_VAR;
        msg.angular_velocity_covariance[8] = GYRO_VAR;

        msg.linear_acceleration_covariance[0] = ACCEL_VAR;
        msg.linear_acceleration_covariance[4] = ACCEL_VAR;
        msg.linear_acceleration_covariance[8] = ACCEL_VAR;

        publisher_->publish(msg);

        if (firstEnsemble == false) {
          RCLCPP_INFO(this->get_logger(), "AHRS node is publishing succesfully");
          firstEnsemble = true;
        }
      }
    }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  Sensor sensor_;
  Registers::System::BinaryOutput1 binaryOutput_;
  tf2::Quaternion forward_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AHRSPublisher>());
  rclcpp::shutdown();
  return 0;
}
