#include <chrono>
#include <memory>

#include "mpu9250driver/mpu9250driver.h"

using namespace std::chrono_literals;

MPU9250Driver::MPU9250Driver()
    : Node("mpu9250publisher"), mpu9250_{std::make_unique<MPU9250Sensor>()}
{
  // Declare parameters
  this->declare_parameter<bool>("calibrate", true);
  this->declare_parameter<int>("gyro_range", MPU9250Sensor::GyroRange::GYR_250_DEG_S);
  this->declare_parameter<int>("accel_range", MPU9250Sensor::AccelRange::ACC_2_G);
  this->declare_parameter<int>("dlpf_bandwidth", MPU9250Sensor::DlpfBandwidth::DLPF_260_HZ);
  this->declare_parameter<double>("gyro_x_offset", 0.0);
  this->declare_parameter<double>("gyro_y_offset", 0.0);
  this->declare_parameter<double>("gyro_z_offset", 0.0);
  this->declare_parameter<double>("accel_x_offset", 0.0);
  this->declare_parameter<double>("accel_y_offset", 0.0);
  this->declare_parameter<double>("accel_z_offset", 0.0);
  this->declare_parameter<int>("frequency", 0.0);
  // Set parameters
  mpu9250_->setGyroscopeRange(
      static_cast<MPU9250Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
  mpu9250_->setAccelerometerRange(
      static_cast<MPU9250Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
  mpu9250_->setDlpfBandwidth(
      static_cast<MPU9250Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
  mpu9250_->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
                               this->get_parameter("gyro_y_offset").as_double(),
                               this->get_parameter("gyro_z_offset").as_double());
  mpu9250_->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
                                   this->get_parameter("accel_y_offset").as_double(),
                                   this->get_parameter("accel_z_offset").as_double());
  // Check if we want to calibrate the sensor
  if (this->get_parameter("calibrate").as_bool()) {
    std::cout << "Calibrating...\n";
    mpu9250_->calibrate();
  }
  mpu9250_->printConfig();
  mpu9250_->printOffsets();
  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  std::chrono::duration<int64_t, std::milli> frequency =
      1000ms / this->get_parameter("gyro_range").as_int();
  timer_ = this->create_wall_timer(frequency, std::bind(&MPU9250Driver::handleInput, this));
}

void MPU9250Driver::handleInput()
{
  auto message = sensor_msgs::msg::Imu();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";
  message.linear_acceleration.x = mpu9250_->getAccelerationX();
  message.linear_acceleration.y = mpu9250_->getAccelerationY();
  message.linear_acceleration.z = mpu9250_->getAccelerationZ();
  message.angular_velocity.x = mpu9250_->getAngularVelocityX();
  message.angular_velocity.y = mpu9250_->getAngularVelocityY();
  message.angular_velocity.z = mpu9250_->getAngularVelocityZ();
  message.orientation.x = 0;
  message.orientation.y = 0;
  message.orientation.z = 0;
  message.orientation.w = 0;
  publisher_->publish(message);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPU9250Driver>());
  rclcpp::shutdown();
  return 0;
}