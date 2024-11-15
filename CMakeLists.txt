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

# Create executables - Basic examples
add_executable(basic_receiver 
    examples/udp_receiver_example.cpp
    ${NETWORK_SOURCES}
)

add_executable(basic_sender 
    examples/udp_sender_example.cpp
    ${NETWORK_SOURCES}
)

add_executable(camera_receiver
    examples/camera_receiver_example.cpp
    src/sensors/camera.cpp
    ${NETWORK_SOURCES}
)

# Link libraries
target_link_libraries(basic_receiver PRIVATE ${PLATFORM_LIBS})
target_link_libraries(basic_sender PRIVATE ${PLATFORM_LIBS})
target_link_libraries(camera_receiver PRIVATE 
    ${PLATFORM_LIBS} 
    ${OpenCV_LIBS}
    opencv_core
    opencv_highgui
    opencv_imgcodecs
)

# For Windows vcpkg users
if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

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
    RUNTIME DESTINATION bin
)
