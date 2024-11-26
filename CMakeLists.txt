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
    set(nlohmann_json_DIR "${vcpkg_DIR}/installed/x64-windows/share/nlohmann_json")
endif()

# Find OpenCV package (for image examples)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)

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
)

# Create executables - Basic examples
add_executable(basic_receiver 
    examples/udp_receiver_example.cpp
    ${NETWORK_SOURCES}
)

add_executable(basic_sender 
    examples/udp_sender_example.cpp
    ${NETWORK_SOURCES}
)

# Create executables - Sensor examples
add_executable(camera_receiver
    examples/camera_receiver_example.cpp
    ${NETWORK_SOURCES}
    ${SENSOR_SOURCES}
)

add_executable(imu_receiver
    examples/imu_receiver_example.cpp
    ${NETWORK_SOURCES}
    ${SENSOR_SOURCES}
)

add_executable(gps_receiver
    examples/gps_receiver_example.cpp
    ${NETWORK_SOURCES}
    ${SENSOR_SOURCES}
)

add_executable(vehicle_state_receiver
    examples/vehicle_state_receiver_example.cpp
    ${NETWORK_SOURCES}
    ${SENSOR_SOURCES}
)

# Add sensor_recorder executable
add_executable(sensor_recorder
    dataset_generator/sensor_recording_to_morai_dataset.cpp
    ${NETWORK_SOURCES}
    ${SENSOR_SOURCES}
)

# Link libraries - Basic examples
target_link_libraries(basic_receiver PRIVATE ${PLATFORM_LIBS})
target_link_libraries(basic_sender PRIVATE ${PLATFORM_LIBS})

# Link libraries - Sensor examples
target_link_libraries(camera_receiver PRIVATE 
    ${PLATFORM_LIBS} 
    ${OpenCV_LIBS}
)

target_link_libraries(imu_receiver PRIVATE 
    ${PLATFORM_LIBS}
)

target_link_libraries(gps_receiver PRIVATE 
    ${PLATFORM_LIBS}
)

target_link_libraries(vehicle_state_receiver PRIVATE 
    ${PLATFORM_LIBS}
)

# Link libraries for sensor_recorder
target_link_libraries(sensor_recorder PRIVATE 
    ${PLATFORM_LIBS}
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
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
install(TARGETS 
    basic_receiver 
    basic_sender
    camera_receiver
    imu_receiver
    gps_receiver
    vehicle_state_receiver
    sensor_recorder
    RUNTIME DESTINATION bin
)

# Add test directory if testing is enabled
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# Generate documentation if Doxygen is found
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target(doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
endif()

if(MSVC)
    # Visual Studio specific settings for sensor_recorder
    set_target_properties(sensor_recorder PROPERTIES
        LINK_FLAGS "/SUBSYSTEM:CONSOLE"
    )
endif()
