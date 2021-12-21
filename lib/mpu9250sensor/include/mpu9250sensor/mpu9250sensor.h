#ifndef MPU9250SENSOR_H
#define MPU9250SENSOR_H

#include <memory>
#include <string>
#include <unordered_map>

#include "I2cCommunicator.h"

class MPU9250Sensor {
 public:
  MPU9250Sensor(std::unique_ptr<I2cCommunicator> i2cBus);

  enum AccelRange { ACC_2_G, ACC_4_G, ACC_8_G, ACC_16_G };
  enum GyroRange { GYR_250_DEG_S, GYR_500_DEG_S, GYR_1000_DEG_S, GYR_2000_DEG_S };
  enum DlpfBandwidth {
    DLPF_260_HZ,
    DLPF_184_HZ,
    DLPF_94_HZ,
    DLPF_44_HZ,
    DLPF_21_HZ,
    DLPF_10_HZ,
    DLPF_5_HZ
  };

  void printConfig() const;
  void printOffsets() const;
  void setGyroscopeRange(GyroRange range);
  void setAccelerometerRange(AccelRange range);
  void setDlpfBandwidth(DlpfBandwidth bandwidth);
  double getAccelerationX() const;
  double getAccelerationY() const;
  double getAccelerationZ() const;
  double getAngularVelocityX() const;
  double getAngularVelocityY() const;
  double getAngularVelocityZ() const;
  double getMagneticFluxDensityX() const;
  double getMagneticFluxDensityY() const;
  double getMagneticFluxDensityZ() const;
  void setGyroscopeOffset(double gyro_x_offset, double gyro_y_offset, double gyro_z_offset);
  void setAccelerometerOffset(double accel_x_offset, double accel_y_offset, double accel_z_offset);
  void calibrate();

 private:
  void initImuI2c() const;
  void initMagnI2c() const;
  double convertRawGyroscopeData(int16_t gyro_raw) const;
  double convertRawAccelerometerData(int16_t accel_raw) const;
  double convertRawMagnetometerData(int16_t flux_raw) const;
  void setContinuousMeasurementMode100Hz();
  void enableBypassMode();
  int readGyroscopeRange();
  int readAccelerometerRange();
  int readDlpfConfig();

  std::unique_ptr<I2cCommunicator> i2cBus_;
  int accel_range_{2};
  int gyro_range_{250};
  int dlpf_range_{260};
  bool calibrated_{false};
  double gyro_x_offset_{0.0};
  double gyro_y_offset_{0.0};
  double gyro_z_offset_{0.0};
  double accel_x_offset_{0.0};
  double accel_y_offset_{0.0};
  double accel_z_offset_{0.0};

  // MPU9250 registers and addresses (s. datasheet for details)
  static constexpr int MPU9250_ADDRESS_DEFAULT = 0x68;
  static constexpr int AK8963_ADDRESS_DEFAULT = 0x0C;
  static constexpr int MPU9250_USER_CTRL = 0x6A;
  static constexpr int MPU9250_BYPASS_ADDR = 0x37;
  static constexpr int PWR_MGMT_1 = 0x6B;
  static constexpr int GYRO_CONFIG = 0x1B;
  static constexpr int ACCEL_CONFIG = 0x1C;
  static constexpr int ACCEL_XOUT_H = 0x3B;
  static constexpr int ACCEL_YOUT_H = 0x3D;
  static constexpr int ACCEL_ZOUT_H = 0x3F;
  static constexpr int GYRO_XOUT_H = 0x43;
  static constexpr int GYRO_YOUT_H = 0x45;
  static constexpr int GYRO_ZOUT_H = 0x47;
  static constexpr int MAGN_ADDR = 0x0C;
  static constexpr int MAGN_XOUT_L = 0x03;
  static constexpr int MAGN_YOUT_L = 0x05;
  static constexpr int MAGN_ZOUT_L = 0x07;
  static constexpr int MAGN_MEAS_MODE = 0x0A;
  static constexpr int DLPF_CONFIG = 0x1A;
  // Helper constants
  static constexpr int GYRO_CONFIG_SHIFT = 3;
  static constexpr int ACCEL_CONFIG_SHIFT = 3;
  static constexpr int MAX_RAW_MAGN_FLUX = 32760;
  static constexpr int MAX_CONV_MAGN_FLUX = 4912;
  static constexpr double GRAVITY = 9.81;
  const std::array<int, 4> ACCEL_RANGES{2, 4, 8, 16};
  const std::array<int, 4> GYRO_RANGES{250, 500, 1000, 2000};
  const std::array<int, 7> DLPF_RANGES{260, 184, 94, 44, 21, 10, 5};
  const std::unordered_map<int, int> ACCEL_SENS_MAP{{2, 16384}, {4, 8192}, {8, 4096}, {16, 2048}};
  const std::unordered_map<int, double> GYRO_SENS_MAP{
      {250, 131}, {500, 65.5}, {1000, 32.8}, {2000, 16.4}};
  static constexpr int CALIBRATION_COUNT{1000};
};

#endif  // MPU9250SENSOR_H
