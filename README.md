# MORAI C++ Network Examples

이 프로젝트는 MORAI 시뮬레이터와의 통신을 위한 C++ 네트워크 예제들을 포함하고 있습니다.

## 빌드 요구사항

### 공통 요구사항:
- CMake (>= 3.10)
- OpenCV (>= 4.0)
- C++17 호환 컴파일러

### Windows 전용 요구사항:
- Visual Studio 2019 이상
- vcpkg
- Windows SDK

### Linux 전용 요구사항:
- ROS Noetic
- gcc/g++ (>= 7.5.0)

## Windows에서 빌드하기

1. vcpkg 설치 및 설정
   ~~~~ bash
   # vcpkg 클론
   git clone https://github.com/Microsoft/vcpkg.git
   cd vcpkg
   
   # vcpkg 부트스트랩
   .\bootstrap-vcpkg.bat
   
   # OpenCV 설치
   .\vcpkg install opencv4:x64-windows
   
   # 환경 변수 설정
   setx VCPKG_ROOT "C:\path\to\vcpkg"
   ~~~~

2. 빌드
   ~~~~ bash
   # 빌드 디렉토리 생성
   mkdir build
   cd build
   
   # CMake 구성
   cmake .. -G "Visual Studio 16 2019" -A x64 ^
   -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake
   
   # 빌드 수행
   cmake --build . --config Release
   ~~~~

3. 실행
   ~~~~ bash
   # camera_receiver 예제 실행 (포트: 7777)
   cd bin\Release
   camera_receiver.exe 7777
   ~~~~

## Linux(ROS)에서 빌드하기

1. ROS 워크스페이스 설정
   ~~~~ bash
   # ROS 워크스페이스 생성 (이미 있다면 생략)
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   
   # 프로젝트 클론 또는 복사
   git clone <repository_url>
   # 또는
   cp -r /path/to/MORAI_CPP_Networks .
   ~~~~

2. 의존성 설치
   ~~~~ bash
   # ROS 의존성 설치
   sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-transport
   
   # OpenCV 설치 (필요한 경우)
   sudo apt-get install libopencv-dev
   ~~~~

3. 빌드
   ~~~~ bash
   # 워크스페이스 루트로 이동
   cd ~/catkin_ws
   
   # 빌드 수행
   catkin_make
   # 또는
   catkin build
   
   # 환경 설정 소싱
   source devel/setup.bash
   ~~~~

4. 실행
   ~~~~ bash
   # camera_receiver 노드 실행 (포트: 7777)
   rosrun morai_cpp_networks camera_receiver_node _port:=7777
   ~~~~

## 실행 가능한 예제들

1. Basic Examples:
   - basic_receiver: 기본 UDP 수신 예제
   - basic_sender: 기본 UDP 송신 예제

2. Sensor Examples:
   - camera_receiver: 카메라 데이터 수신 예제
   - imu_receiver: IMU 데이터 수신 예제
   - gps_receiver: GPS 데이터 수신 예제
   - vehicle_state_receiver: 차량 상태 데이터 수신 예제
   - traffic_light_receiver: 신호등 데이터 수신 예제
   - object_info_receiver: 객체 정보 수신 예제

3. Control Examples:
   - ego_control_example: 차량 제어 예제

## 주의사항

1. Windows에서 실행 시:
   - 관리자 권한이 필요할 수 있습니다
   - 방화벽 설정이 필요할 수 있습니다

2. Linux에서 실행 시:
   - ROS 환경 설정이 필요합니다
   - 포트 접근 권한이 필요할 수 있습니다

3. 공통 사항:
   - MORAI 시뮬레이터가 실행 중이어야 합니다
   - 네트워크 설정이 올바르게 되어 있어야 합니다 

## 다른 프로젝트에서 사용하기

### 방법 1: 시스템에 설치하여 사용

당신의 프로젝트의 CMakeLists.txt에 다음과 같이 추가하세요:

~~~~ cmake
# 설치된 패키지 찾기
find_package(MoraiCppNetworks REQUIRED)

add_executable(your_project main.cpp)
target_link_libraries(your_project
    MoraiCppNetworks::network    # 네임스페이스가 있는 타겟 이름 사용
    MoraiCppNetworks::sensor
)
~~~~

설치 방법:
~~~~ bash
# 빌드 디렉토리 생성
mkdir build && cd build

# CMake 구성 및 빌드
cmake ..
cmake --build .

# 라이브러리 설치 (Linux의 경우)
sudo cmake --install .

# 라이브러리 설치 (Windows의 경우 관리자 권한으로 실행)
cmake --install .
~~~~

### 방법 2: FetchContent 사용 (CMake 3.11+)

당신의 프로젝트의 CMakeLists.txt에 다음과 같이 추가하세요:

~~~~ cmake
include(FetchContent)
FetchContent_Declare(
    morai_cpp_networks
    GIT_REPOSITORY https://github.com/Onlyti/morai_cpp_networks.git
    GIT_TAG main  # 또는 특정 태그/커밋
)
FetchContent_MakeAvailable(morai_cpp_networks)

add_executable(your_project main.cpp)
target_link_libraries(your_project
    morai_cpp_networks_network   # 직접 타겟 이름 사용
    morai_cpp_networks_sensor
)
~~~~

### 방법 3: add_subdirectory 사용

1. 먼저 프로젝트를 서브디렉토리로 가져옵니다:
~~~~ bash
# 프로젝트 루트 디렉토리에서
git clone https://github.com/Onlyti/morai_cpp_networks.git external/morai_cpp_networks
# 또는
git submodule add https://github.com/Onlyti/morai_cpp_networks.git external/morai_cpp_networks
~~~~

2. CMakeLists.txt에 다음과 같이 추가하세요:
~~~~ cmake
add_subdirectory(external/morai_cpp_networks)

add_executable(your_project main.cpp)
target_link_libraries(your_project
    morai_cpp_networks_network   # 직접 타겟 이름 사용
    morai_cpp_networks_sensor
)
~~~~

### 사용 예시

~~~~ cpp
#include <morai_cpp_networks/network/udp_receiver.hpp>
#include <morai_cpp_networks/sensors/camera.hpp>

int main() {
    // UDP 수신기 생성
    morai::network::UdpReceiver receiver("127.0.0.1", 7777);
    
    // 카메라 센서 데이터 처리
    morai::sensors::Camera camera;
    // ... 사용자 코드 ...
    
    return 0;
}
~~~~

방법 2와 3의 장점:
- 시스템 수준 설치가 필요 없음
- 프로젝트와 함께 버전 관리 가능
- 빌드 시점에 자동으로 의존성 해결
~~~~
