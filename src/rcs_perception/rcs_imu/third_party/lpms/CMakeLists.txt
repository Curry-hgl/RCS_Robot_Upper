find_package(Threads REQUIRED)
find_library(UDEV_LIBRARY udev REQUIRED)
add_library(rcs_imu_lpms STATIC
    src/MicroMeasure.cpp src/SerialPortLinux.cpp src/LpmsIG1.cpp
    src/LpMatrix.c src/LpUtil.cpp)
target_include_directories(rcs_imu_lpms PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(rcs_imu_lpms PUBLIC cxx_std_17)
target_link_libraries(rcs_imu_lpms PUBLIC Threads::Threads rt ${UDEV_LIBRARY})
