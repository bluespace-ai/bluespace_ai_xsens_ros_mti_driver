
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(
    xda_test xdautils_test.cpp
)

target_link_libraries(
    xda_test
    xsens_mti_utils
)
