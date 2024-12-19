cmake_minimum_required(VERSION 3.10)
project(UDP_Examples)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform specific settings
if(WIN32)
    # Windows socket libraries
    set(PLATFORM_LIBS ws2_32 wsock32)
else()
    set(PLATFORM_LIBS pthread)
endif()

# For Windows vcpkg users
if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
    
    # Fix OpenCV_DIR path
    set(vcpkg_DIR "C:/Users/User/Documents/git/vcpkg")
    if(DEFINED vcpkg_DIR)
        set(OpenCV_DIR "${vcpkg_DIR}/installed/x64-windows/share/opencv4")
    else()
        message(FATAL_ERROR "vcpkg_DIR is not defined. Please make sure vcpkg is properly installed and integrated.")
    endif()
    set(Protobuf_DIR "${vcpkg_DIR}/installed/x64-windows/share/protobuf")
    set(Boost_DIR "${vcpkg_DIR}/installed/x64-windows/share/boost")
    set(quirc_DIR "${vcpkg_DIR}/installed/x64-windows/share/quirc")
    set(libjpeg_turbo_DIR "${vcpkg_DIR}/installed/x64-windows/share/libjpeg-turbo")
    set(libpng_DIR "${vcpkg_DIR}/installed/x64-windows/share/libpng")
    set(libtiff_DIR "${vcpkg_DIR}/installed/x64-windows/share/libtiff")
    set(libwebp_DIR "${vcpkg_DIR}/installed/x64-windows/share/libwebp")
    set(OpenEXR_DIR "${vcpkg_DIR}/installed/x64-windows/share/openexr")
endif()

# Find OpenCV package (for image examples)
find_package(OpenCV REQUIRED)

if(WIN32)
    set(OpenCV_LIBS ${OpenCV_LIBS})
else()
    set(OpenCV_LIBS ${OpenCV_LIBRARIES})
endif()

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

# Set source files
set(NETWORK_SOURCES
    src/network/udp_receiver.cpp
    src/network/udp_sender.cpp
)

set(SENSOR_SOURCES
    src/sensors/camera.cpp
    src/sensors/imu.cpp
    src/sensors/gps.cpp
    src/sensors/vehicle_state.cpp
    src/sensors/traffic_light.cpp
)

# Create executables - Basic examples
add_executable(basic_receiver 
    examples/udp_receiver_example.cpp
    ${NETWORK_SOURCES}
)
target_link_libraries(basic_receiver PRIVATE ${PLATFORM_LIBS})
    
add_executable(basic_sender 
    examples/udp_sender_example.cpp
    ${NETWORK_SOURCES}
)
target_link_libraries(basic_sender PRIVATE ${PLATFORM_LIBS})

# Create executables - Sensor examples
add_executable(camera_receiver
    examples/camera_receiver_example.cpp
    ${NETWORK_SOURCES}
    src/sensors/camera.cpp
)

target_link_libraries(camera_receiver PRIVATE 
    ${PLATFORM_LIBS}
    ${OpenCV_LIBS}
)

add_executable(imu_receiver
    examples/imu_receiver_example.cpp
    ${NETWORK_SOURCES}
    src/sensors/imu.cpp
)
target_link_libraries(imu_receiver PRIVATE 
    ${PLATFORM_LIBS}
)

# Create executables - GPS example
add_executable(gps_receiver
    examples/gps_receiver_example.cpp
    ${NETWORK_SOURCES}
    src/sensors/gps.cpp
)
target_link_libraries(gps_receiver PRIVATE 
    ${PLATFORM_LIBS}
)

add_executable(vehicle_state_receiver
    examples/vehicle_state_receiver_example.cpp
    ${NETWORK_SOURCES}
    src/sensors/vehicle_state.cpp
)
target_link_libraries(vehicle_state_receiver PRIVATE 
    ${PLATFORM_LIBS}
)

# Create executable - Traffic Light example
add_executable(traffic_light_receiver
    examples/traffic_light_receiver_example.cpp
    ${NETWORK_SOURCES}
    src/sensors/traffic_light.cpp
)
target_link_libraries(traffic_light_receiver PRIVATE 
    ${PLATFORM_LIBS}
)

# Ego Control Example
add_executable(ego_control_example
    examples/ego_control_example.cpp
    src/actuator/ego_ctl.cpp
    src/network/udp_sender.cpp
)

target_include_directories(ego_control_example
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ego_control_example
    PRIVATE
    ${REQUIRED_LIBS}  # 필요한 라이브러리들 (예: pthread 등)
)

# Debug/Release configuration
if(MSVC)
    # Visual Studio compiler options
    add_compile_options(
        $<$<CONFIG:>:/MT>
        $<$<CONFIG:Debug>:/MTd>
        $<$<CONFIG:Release>:/MT>
    )
else()
    # GCC/Clang compiler options
    add_compile_options(-Wall -Wextra)
endif()

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Installation settings
if(WIN32)
    install(TARGETS 
        basic_receiver 
        basic_sender
        camera_receiver
        imu_receiver
        gps_receiver
        vehicle_state_receiver
        ego_control_example
        traffic_light_receiver
        RUNTIME DESTINATION bin
    )
endif()
