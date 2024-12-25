cmake_minimum_required(VERSION 3.10)
project(morai_cpp_networks)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform specific settings
if(WIN32)
    # Windows socket libraries
    set(PLATFORM_LIBS ws2_32 wsock32)
    
    # For Windows vcpkg users
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
    
    # Fix OpenCV_DIR path
    set(vcpkg_DIR "C:/Users/User/Documents/git/vcpkg")
    if(DEFINED vcpkg_DIR)
        set(OpenCV_DIR "${vcpkg_DIR}/installed/x64-windows/share/opencv4")
    else()
        message(FATAL_ERROR "vcpkg_DIR is not defined. Please make sure vcpkg is properly installed and integrated.")
    endif()
else()
    set(PLATFORM_LIBS pthread)
    
    # Linux에서만 ROS 관련 설정 추가
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        cv_bridge
        image_transport
        sensor_msgs
        visualization_msgs
        geographic_msgs
        tf2
        tf2_ros
    )
    
    # Catkin specific configuration
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}_network
        CATKIN_DEPENDS 
            roscpp 
            cv_bridge 
            image_transport 
            sensor_msgs 
            visualization_msgs
            geographic_msgs
            tf2
            tf2_ros
        DEPENDS OpenCV
    )
endif()

# Find OpenCV package
find_package(OpenCV REQUIRED)

if(WIN32)
    set(OpenCV_LIBS ${OpenCV_LIBS})
else()
    set(OpenCV_LIBS ${OpenCV_LIBRARIES})
    
    # Catkin specific configuration
    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}_network
        CATKIN_DEPENDS 
            roscpp 
            cv_bridge 
            image_transport 
            sensor_msgs 
            visualization_msgs
            geographic_msgs
            tf2
            tf2_ros
        DEPENDS 
            OpenCV
    )
endif()

# Eigen
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

# Linux에서만 catkin include 추가
if(NOT WIN32)
    include_directories(${catkin_INCLUDE_DIRS})
endif()

# Network library sources
add_library(${PROJECT_NAME}_network
    src/network/udp_receiver.cpp
    src/network/udp_sender.cpp
)

# 네트워크 라이브러리에 대한 export 설정 추가
set_target_properties(${PROJECT_NAME}_network PROPERTIES
    EXPORT_NAME network
)

target_link_libraries(${PROJECT_NAME}_network
    ${PLATFORM_LIBS}
)

# Basic examples
add_executable(basic_receiver examples/udp_receiver_example.cpp)
target_link_libraries(basic_receiver ${PROJECT_NAME}_network ${PLATFORM_LIBS})
    
add_executable(basic_sender examples/udp_sender_example.cpp)
target_link_libraries(basic_sender ${PROJECT_NAME}_network ${PLATFORM_LIBS})

# Camera receiver
if(WIN32)
    add_executable(camera_receiver
        examples/camera_receiver_example.cpp
        src/sensors/camera.cpp
    )
    target_link_libraries(camera_receiver
        ${PROJECT_NAME}_network
        ${OpenCV_LIBS}
        ${PLATFORM_LIBS}
    )
else()
    # Linux에서는 ROS 노드로 빌
    add_executable(camera_receiver_node
        examples/camera_receiver_example.cpp
        src/sensors/camera.cpp
    )
    target_link_libraries(camera_receiver_node
        ${PROJECT_NAME}_network
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${PLATFORM_LIBS}
    )
endif()

# IMU receiver
if(WIN32)
    add_executable(imu_receiver
        examples/imu_receiver_example.cpp
        src/sensors/imu.cpp
    )
    target_link_libraries(imu_receiver 
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
    )
else()
    add_executable(imu_receiver_node
        examples/imu_receiver_example.cpp
        src/sensors/imu.cpp
    )
    target_link_libraries(imu_receiver_node
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
        ${catkin_LIBRARIES}
    )
endif()

# GPS receiver
if(WIN32)
    add_executable(gps_receiver
        examples/gps_receiver_example.cpp
        src/sensors/gps.cpp
    )
    target_link_libraries(gps_receiver 
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
    )
else()
    add_executable(gps_receiver_node
        examples/gps_receiver_example.cpp
        src/sensors/gps.cpp
    )
    target_link_libraries(gps_receiver_node
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
        ${catkin_LIBRARIES}
    )
endif()

# Vehicle state receiver
if(WIN32)
    add_executable(vehicle_state_receiver
        examples/vehicle_state_receiver_example.cpp
        src/sensors/vehicle_state.cpp
    )
    target_link_libraries(vehicle_state_receiver
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
    )
else()
    add_executable(vehicle_state_receiver_node
        examples/vehicle_state_receiver_example.cpp
        src/sensors/vehicle_state.cpp
    )
    target_link_libraries(vehicle_state_receiver_node
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
        ${catkin_LIBRARIES}
    )
endif()

# Traffic Light receiver
if(WIN32)
    add_executable(traffic_light_receiver
        examples/traffic_light_receiver_example.cpp
        src/sensors/traffic_light.cpp
    )
    target_link_libraries(traffic_light_receiver 
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
    )
else()
    add_executable(traffic_light_receiver_node
        examples/traffic_light_receiver_example.cpp
        src/sensors/traffic_light.cpp
    )
    target_link_libraries(traffic_light_receiver_node
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
        ${catkin_LIBRARIES}
    )
endif()

# Object Info receiver
if(WIN32)
    add_executable(object_info_receiver
        examples/object_info_receiver_example.cpp
        src/sensors/object_info.cpp
    )
    target_link_libraries(object_info_receiver
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
    )
else()
    add_executable(object_info_receiver_node
        examples/object_info_receiver_example.cpp
        src/sensors/object_info.cpp
    )
    target_link_libraries(object_info_receiver_node
        ${PROJECT_NAME}_network
        ${PLATFORM_LIBS}
        ${catkin_LIBRARIES}
    )
endif()

# Ego Control Example
add_executable(ego_control_example
    examples/ego_control_example.cpp
    src/actuator/ego_ctl.cpp
)

target_link_libraries(ego_control_example
    ${PROJECT_NAME}_network
    ${PLATFORM_LIBS}
)

# Debug/Release configuration
if(MSVC)
    add_compile_options(
        $<$<CONFIG:>:/MT>
        $<$<CONFIG:Debug>:/MTd>
        $<$<CONFIG:Release>:/MT>
    )
else()
    add_compile_options(-Wall -Wextra)
endif()

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Installation settings
if(WIN32)
    install(TARGETS 
        ${PROJECT_NAME}_network
        basic_receiver 
        basic_sender
        camera_receiver
        imu_receiver
        gps_receiver
        vehicle_state_receiver
        ego_control_example
        traffic_light_receiver
        object_info_receiver
        RUNTIME DESTINATION bin
    )
else()
    install(TARGETS 
        ${PROJECT_NAME}_network
        basic_receiver 
        basic_sender
        camera_receiver_node
        imu_receiver_node
        gps_receiver_node
        vehicle_state_receiver_node
        ego_control_example
        traffic_light_receiver_node
        object_info_receiver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )

    # Install launch files if they exist
    install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
    )

    # Install RViz config files
    install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        FILES_MATCHING PATTERN "*.rviz"
    )
endif()

# Sensor library sources
add_library(${PROJECT_NAME}_sensor
    src/sensors/camera.cpp
    src/sensors/gps.cpp
    src/sensors/imu.cpp
    src/sensors/object_info.cpp
    src/sensors/traffic_light.cpp
    src/sensors/vehicle_state.cpp
)

# 센서 라이브러리에 대한 export 설정 추가
set_target_properties(${PROJECT_NAME}_sensor PROPERTIES
    EXPORT_NAME sensor
)

target_link_libraries(${PROJECT_NAME}_sensor
    ${PLATFORM_LIBS}
    ${OpenCV_LIBS}
    ${Eigen3_LIBRARIES}
)

# Export 타겟 설정
install(TARGETS ${PROJECT_NAME}_network ${PROJECT_NAME}_sensor
    EXPORT MoraiCppNetworksTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Set include directory path for applications

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# 설치 경로 설정
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set(CONFIG_INSTALL_DIR ${LIB_INSTALL_DIR}/cmake/MoraiCppNetworks)

# Config 파일 생성
configure_package_config_file(
    cmake/MoraiCppNetworksConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/MoraiCppNetworksConfig.cmake
    INSTALL_DESTINATION ${CONFIG_INSTALL_DIR}
    PATH_VARS INCLUDE_INSTALL_DIR
)

# 타겟 export
install(EXPORT MoraiCppNetworksTargets
    FILE MoraiCppNetworksTargets.cmake
    NAMESPACE MoraiCppNetworks::
    DESTINATION ${CONFIG_INSTALL_DIR}
)

# Config 파일 설치
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/MoraiCppNetworksConfig.cmake
    DESTINATION ${CONFIG_INSTALL_DIR}
)

# 헤더 파일 설치
install(DIRECTORY include/
    DESTINATION ${INCLUDE_INSTALL_DIR}/morai_cpp_networks
)
