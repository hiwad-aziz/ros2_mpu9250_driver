
# MPU9250 Driver for ROS2
This repository contains a ROS2 package that interfaces with an MPU9250 sensor over I2C. The accelerometer and gyroscope are calibrated on node startup (sensor needs to be on a plane with z-axis up and should not be moved during calibration). Calibration can be turned off in the parameters file. The orientation is calculated from accelerometer and magnetometer measurements and is currently unfiltered.

## Dependencies
-  libi2c-dev

## Setup
The number of iterations for calibration can be set up in `include/mpu9250driver/mpu9250sensor.h`.
Following other parameters and default values are listed here and can be changed in `params/mpu9250.yaml`.
```    calibrate: True
    gyro_range: 0
    accel_range: 0
    dlpf_bandwidth: 2
    gyro_x_offset: 0.0 # [deg/s]
    gyro_y_offset: 0.0 # [deg/s]
    gyro_z_offset: 0.0 # [deg/s]
    accel_x_offset: 0.0 # [m/s²]
    accel_y_offset: 0.0 # [m/s²]
    accel_z_offset: 0.0 # [m/s²]
    frequency: 100 # [Hz]
```

Build the package in your workspace:

    colcon build --packages-select mpu9250driver

Source setup.bash in your workspace:

    . install/setup.bash
    
Launch it:

    ros2 launch mpu9250driver mpu9250driver_launch.py
    
## Disclaimer
The project is still in **work in progress** state. I will spend some time every now and then when I'm free. Please don't expect everything to work flawlessly. Feel free to create a pull request if you want to contribute or fix bugs.

