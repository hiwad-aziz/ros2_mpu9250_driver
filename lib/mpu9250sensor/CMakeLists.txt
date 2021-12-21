# MPU9250 sensor library
add_library(mpu9250sensor src/mpu9250sensor.cpp)
target_include_directories(mpu9250sensor PUBLIC include/mpu9250sensor)
target_link_libraries(mpu9250sensor PRIVATE i2c_comm_interface)