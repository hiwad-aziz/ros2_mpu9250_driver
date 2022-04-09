#include "mpu9250sensor.h"

extern "C" {
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
}

#include <iostream>
#include <thread>

MPU9250Sensor::MPU9250Sensor(std::unique_ptr<I2cCommunicator> i2cBus) : i2cBus_(std::move(i2cBus))
{
  initImuI2c();
  // Wake up sensor
  int result = i2cBus_->write(PWR_MGMT_1, 0);
  if (result < 0)
  {
    std::cerr << "Error waking sensor" << std::endl;
  }
  // Enable bypass mode for magnetometer
  enableBypassMode();
  // Set magnetometer to 100 Hz continuous measurement mode
  setContinuousMeasurementMode100Hz();
  // Read current ranges from sensor
  readGyroscopeRange();
  readAccelerometerRange();
  readDlpfConfig();
}

void MPU9250Sensor::initImuI2c() const
{
  if (ioctl(i2cBus_->getFile(), I2C_SLAVE, MPU9250_ADDRESS_DEFAULT) < 0) {
    std::cerr << "Failed to find device address! Check device address!";
    exit(1);
  }
}

void MPU9250Sensor::initMagnI2c() const
{
  if (ioctl(i2cBus_->getFile(), I2C_SLAVE, AK8963_ADDRESS_DEFAULT) < 0) {
    std::cerr << "Failed to find device address! Check device address!";
    exit(1);
  }
}

void MPU9250Sensor::printConfig() const
{
  std::cout << "Accelerometer Range: +-" << accel_range_ << "g\n";
  std::cout << "Gyroscope Range: +-" << gyro_range_ << " degree per sec\n";
  std::cout << "DLPF Range: " << dlpf_range_ << " Hz\n";
}

void MPU9250Sensor::printOffsets() const
{
  std::cout << "Accelerometer Offsets: x: " << accel_x_offset_ << ", y: " << accel_y_offset_
            << ", z: " << accel_z_offset_ << "\n";
  std::cout << "Gyroscope Offsets: x: " << gyro_x_offset_ << ", y: " << gyro_y_offset_
            << ", z: " << gyro_z_offset_ << "\n";
}

void MPU9250Sensor::setContinuousMeasurementMode100Hz()
{
  initMagnI2c();
  // Set to power-down mode first before switching to another mode
  int result = i2cBus_->write(MAGN_MEAS_MODE, 0x00);
  if (result < 0)
  {
    std::cerr << "Error powering down magnometer" << std::endl;
  }
  // Wait until mode changes
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Switch to 100 Hz mode
  result = i2cBus_->write(MAGN_MEAS_MODE, 0x16);
  if (result < 0)
  {
    std::cerr << "Error reactivating magnometer" << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  initImuI2c();
}

void MPU9250Sensor::enableBypassMode()
{
  // Disable I2C master interface
  int result = i2cBus_->write(MPU9250_USER_CTRL, 0x00);
  if (result < 0)
  {
    std::cerr << "Error disabling i2c master interface" << std::endl;
  }
  
  // Enable bypass mode
  result = i2cBus_->write(MPU9250_BYPASS_ADDR, 0x02);
  if (result < 0)
  {
    std::cerr << "Error enabling bypass" << std::endl;
  }
}

int MPU9250Sensor::readGyroscopeRange()
{
  int range = i2cBus_->read(GYRO_CONFIG);
  range = range >> GYRO_CONFIG_SHIFT;
  gyro_range_ = GYRO_RANGES[range];
  return gyro_range_;
}

int MPU9250Sensor::readAccelerometerRange()
{
  int range = i2cBus_->read(ACCEL_CONFIG);
  range = range >> ACCEL_CONFIG_SHIFT;
  accel_range_ = ACCEL_RANGES[range];
  return accel_range_;
}

int MPU9250Sensor::readDlpfConfig()
{
  int range = i2cBus_->read(DLPF_CONFIG);
  range = range & 7;  // Read only first 3 bits
  dlpf_range_ = DLPF_RANGES[range];
  return dlpf_range_;
}

void MPU9250Sensor::setGyroscopeRange(MPU9250Sensor::GyroRange range)
{
  int result = i2cBus_->write(GYRO_CONFIG, range << GYRO_CONFIG_SHIFT);
  if (result < 0)
  {
    std::cerr << "Error setting gyroscope range" << std::endl;
  }
  gyro_range_ = GYRO_RANGES[static_cast<size_t>(range)];
}

void MPU9250Sensor::setAccelerometerRange(MPU9250Sensor::AccelRange range)
{
  int result = i2cBus_->write(ACCEL_CONFIG, range << ACCEL_CONFIG_SHIFT);
  if (result < 0)
  {
    std::cerr << "Error setting acc. range" << std::endl;
  }
  accel_range_ = ACCEL_RANGES[static_cast<size_t>(range)];
}

void MPU9250Sensor::setDlpfBandwidth(DlpfBandwidth bandwidth)
{
  int result = i2cBus_->write(DLPF_CONFIG, bandwidth);
  if (result < 0)
  {
    std::cerr << "Error setting bandwidth" << std::endl;
  }
  dlpf_range_ = DLPF_RANGES[static_cast<size_t>(bandwidth)];
}

double MPU9250Sensor::getAccelerationX() const
{
  int16_t accel_x_msb = i2cBus_->read(ACCEL_XOUT_H);
  int16_t accel_x_lsb = i2cBus_->read(ACCEL_XOUT_H + 1);
  int16_t accel_x = accel_x_lsb | accel_x_msb << 8;
  double accel_x_converted = convertRawAccelerometerData(accel_x);
  if (calibrated_) {
    return accel_x_converted - accel_x_offset_;
  }
  return accel_x_converted;
}

double MPU9250Sensor::getAccelerationY() const
{
  int16_t accel_y_msb = i2cBus_->read(ACCEL_YOUT_H);
  int16_t accel_y_lsb = i2cBus_->read(ACCEL_YOUT_H + 1);
  int16_t accel_y = accel_y_lsb | accel_y_msb << 8;
  double accel_y_converted = convertRawAccelerometerData(accel_y);
  if (calibrated_) {
    return accel_y_converted - accel_y_offset_;
  }
  return accel_y_converted;
}

double MPU9250Sensor::getAccelerationZ() const
{
  int16_t accel_z_msb = i2cBus_->read(ACCEL_ZOUT_H);
  int16_t accel_z_lsb = i2cBus_->read(ACCEL_ZOUT_H + 1);
  int16_t accel_z = accel_z_lsb | accel_z_msb << 8;
  double accel_z_converted = convertRawAccelerometerData(accel_z);
  if (calibrated_) {
    return accel_z_converted - accel_z_offset_;
  }
  return accel_z_converted;
}

double MPU9250Sensor::getAngularVelocityX() const
{
  int16_t gyro_x_msb = i2cBus_->read(GYRO_XOUT_H);
  int16_t gyro_x_lsb = i2cBus_->read(GYRO_XOUT_H + 1);
  int16_t gyro_x = gyro_x_lsb | gyro_x_msb << 8;
  double gyro_x_converted = convertRawGyroscopeData(gyro_x);
  if (calibrated_) {
    return gyro_x_converted - gyro_x_offset_;
  }
  return gyro_x_converted;
}

double MPU9250Sensor::getAngularVelocityY() const
{
  int16_t gyro_y_msb = i2cBus_->read(GYRO_YOUT_H);
  int16_t gyro_y_lsb = i2cBus_->read(GYRO_YOUT_H + 1);
  int16_t gyro_y = gyro_y_lsb | gyro_y_msb << 8;
  double gyro_y_converted = convertRawGyroscopeData(gyro_y);
  if (calibrated_) {
    return gyro_y_converted - gyro_y_offset_;
  }
  return gyro_y_converted;
}

double MPU9250Sensor::getAngularVelocityZ() const
{
  int16_t gyro_z_msb = i2cBus_->read(GYRO_ZOUT_H);
  int16_t gyro_z_lsb = i2cBus_->read(GYRO_ZOUT_H + 1);
  int16_t gyro_z = gyro_z_lsb | gyro_z_msb << 8;
  double gyro_z_converted = convertRawGyroscopeData(gyro_z);
  if (calibrated_) {
    return gyro_z_converted - gyro_z_offset_;
  }
  return gyro_z_converted;
}

double MPU9250Sensor::getMagneticFluxDensityX() const
{
  // TODO: check for overflow of magnetic sensor
  initMagnI2c();
  int16_t magn_flux_x_msb = i2cBus_->read(MAGN_XOUT_L + 1);
  int16_t magn_flux_x_lsb = i2cBus_->read(MAGN_XOUT_L);
  int16_t magn_flux_x = magn_flux_x_lsb | magn_flux_x_msb << 8;
  double magn_flux_x_converted = convertRawMagnetometerData(magn_flux_x);
  initImuI2c();
  return magn_flux_x_converted;
}

double MPU9250Sensor::getMagneticFluxDensityY() const
{
  initMagnI2c();
  int16_t magn_flux_y_msb = i2cBus_->read(MAGN_YOUT_L + 1);
  int16_t magn_flux_y_lsb = i2cBus_->read(MAGN_YOUT_L);
  int16_t magn_flux_y = magn_flux_y_lsb | magn_flux_y_msb << 8;
  double magn_flux_y_converted = convertRawMagnetometerData(magn_flux_y);
  initImuI2c();
  return magn_flux_y_converted;
}

double MPU9250Sensor::getMagneticFluxDensityZ() const
{
  initMagnI2c();
  int16_t magn_flux_z_msb = i2cBus_->read(MAGN_ZOUT_L + 1);
  int16_t magn_flux_z_lsb = i2cBus_->read(MAGN_ZOUT_L);
  int16_t magn_flux_z = magn_flux_z_lsb | magn_flux_z_msb << 8;
  double magn_flux_z_converted = convertRawMagnetometerData(magn_flux_z);
  initImuI2c();
  return magn_flux_z_converted;
}

double MPU9250Sensor::convertRawGyroscopeData(int16_t gyro_raw) const
{
  const double ang_vel_in_deg_per_s = static_cast<double>(gyro_raw) / GYRO_SENS_MAP.at(gyro_range_);
  return ang_vel_in_deg_per_s;
}

double MPU9250Sensor::convertRawMagnetometerData(int16_t flux_raw) const
{
  const double magn_flux_in_mu_tesla =
      static_cast<double>(flux_raw) * MAX_CONV_MAGN_FLUX / MAX_RAW_MAGN_FLUX;
  return magn_flux_in_mu_tesla;
}

double MPU9250Sensor::convertRawAccelerometerData(int16_t accel_raw) const
{
  const double accel_in_m_per_s =
      static_cast<double>(accel_raw) / ACCEL_SENS_MAP.at(accel_range_) * GRAVITY;
  return accel_in_m_per_s;
}

void MPU9250Sensor::setGyroscopeOffset(double gyro_x_offset, double gyro_y_offset,
                                       double gyro_z_offset)
{
  gyro_x_offset_ = gyro_x_offset;
  gyro_y_offset_ = gyro_y_offset;
  gyro_z_offset_ = gyro_z_offset;
}

void MPU9250Sensor::setAccelerometerOffset(double accel_x_offset, double accel_y_offset,
                                           double accel_z_offset)
{
  accel_x_offset_ = accel_x_offset;
  accel_y_offset_ = accel_y_offset;
  accel_z_offset_ = accel_z_offset;
}

void MPU9250Sensor::calibrate()
{
  int count = 0;
  while (count < CALIBRATION_COUNT) {
    gyro_x_offset_ += getAngularVelocityX();
    gyro_y_offset_ += getAngularVelocityY();
    gyro_z_offset_ += getAngularVelocityZ();
    accel_x_offset_ += getAccelerationX();
    accel_y_offset_ += getAccelerationY();
    accel_z_offset_ += getAccelerationZ();
    ++count;
  }
  gyro_x_offset_ /= CALIBRATION_COUNT;
  gyro_y_offset_ /= CALIBRATION_COUNT;
  gyro_z_offset_ /= CALIBRATION_COUNT;
  accel_x_offset_ /= CALIBRATION_COUNT;
  accel_y_offset_ /= CALIBRATION_COUNT;
  accel_z_offset_ /= CALIBRATION_COUNT;
  accel_z_offset_ -= GRAVITY;
  calibrated_ = true;
}
