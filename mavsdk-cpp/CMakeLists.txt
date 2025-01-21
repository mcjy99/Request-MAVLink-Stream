cmake_minimum_required(VERSION 3.10.2)
# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(MavlinkRequest)
# Find the MAVSDK library
find_package(MAVSDK REQUIRED)

# Add the executable
add_executable(mavlink_request mav_req_lr_hb.cpp)

# Link MAVSDK to your project
target_link_libraries(mavlink_request MAVSDK::mavsdk)

if(NOT MSVC)
    add_compile_options(mavlink_request PRIVATE -Wall -Wextra)
else()
    add_compile_options(mavlink_request PRIVATE -W2)
endif()
