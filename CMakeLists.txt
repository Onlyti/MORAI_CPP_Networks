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
endif()

# Find OpenCV package
find_package(OpenCV REQUIRED)

if(WIN32)
    set(OpenCV_LIBS ${OpenCV_LIBS})
else()
    set(OpenCV_LIBS ${OpenCV_LIBRARIES})
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
    ${PROJECT_NAME}_network
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
set(INCLUDE_INSTALL_DIR include)
set(LIB_INSTALL_DIR lib)
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
