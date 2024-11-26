#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

#include "sensors/camera.hpp"
#include "sensors/gps.hpp"
#include "sensors/imu.hpp"
#include "sensors/vehicle_state.hpp"
#include "utils/dataset_directory.hpp"

using json = nlohmann::json;

#define DEG2RAD(deg) ((deg) * M_PI / 180.0)
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)

#define SMALL_IMAGE_WIDTH 200
#define SMALL_IMAGE_HEIGHT 112

struct Vector3
{
    float x, y, z;
};

struct SensorPosition
{
    Vector3 pos;
    Vector3 rot;
};

struct UDPConfig
{
    std::string hostIP;
    int hostPort;
    std::string destinationIP;
    int destinationPort;
    int receiveTimeout;
    bool isBufferThreadActivated;
    int receiveBufferSize;
    int sizeOfAuxHeader;
};

struct CameraIntrinsic
{
    float fx;  // focal length x (pixel)
    float fy;  // focal length y (pixel)
    float cx;  // principal point x
    float cy;  // principal point y
};

class SensorInfo
{
 public:
    int uniqueID;
    int sensorType;
    SensorPosition position;
    UDPConfig udpConfig;
    std::string frameID;
    float sensorPeriod;
    CameraIntrinsic camera_intrinsic;  // 3x3 intrinsic matrix 관련 파라미터
};

class SensorRecorder
{
 public:
    json sensor_config_;
    std::string config_path_;
    std::vector<SensorInfo> cameras_;
    std::vector<SensorInfo> imus_;
    std::vector<SensorInfo> gpss_;

    std::vector<std::unique_ptr<Camera>> camera_receivers_;
    std::vector<std::shared_ptr<cv::Mat>> camera_images_;
    std::vector<std::unique_ptr<IMU>> imu_receivers_;
    std::vector<std::unique_ptr<GPS>> gps_receivers_;
    std::atomic<bool> is_running_;

    // 스레드 관련 멤버 변수들
    std::vector<std::thread> camera_threads_;

    std::vector<std::thread> imu_threads_;
    std::vector<std::thread> gps_threads_;
    std::mutex data_mutex_;

    std::unique_ptr<VehicleState> vehicle_state_receiver_;
    std::thread vehicle_state_thread_;

    bool parseSensorCommon(const json& sensor, SensorInfo& info)
    {
        try
        {
            info.uniqueID = sensor["m_SensorUniqueID"];
            info.sensorType = sensor["m_SensorType"];

            // Parse position and rotation
            info.position.pos.x = std::stof(sensor["pos"]["x"].get<std::string>());
            info.position.pos.y = std::stof(sensor["pos"]["y"].get<std::string>());
            info.position.pos.z = std::stof(sensor["pos"]["z"].get<std::string>());

            info.position.rot.x = std::stof(sensor["rot"]["roll"].get<std::string>());
            info.position.rot.y = std::stof(sensor["rot"]["pitch"].get<std::string>());
            info.position.rot.z = std::stof(sensor["rot"]["yaw"].get<std::string>());

            return true;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing sensor common data: " << e.what() << std::endl;
            return false;
        }
    }

    bool parseIMU(const json& imu)
    {
        try
        {
            SensorInfo imu_info;
            if (parseSensorCommon(imu, imu_info))
            {
                const auto& ic = imu["ic"];
                imu_info.sensorPeriod = ic["sensorPeriod"];
                imu_info.frameID = ic["rosConfig"]["frameID"];

                // Parse UDP config
                const auto& udp = ic["udpConfig"];
                imu_info.udpConfig.hostIP = udp["hostIP"];
                imu_info.udpConfig.hostPort = udp["hostPort"];
                imu_info.udpConfig.destinationIP = udp["destinationIP"];
                imu_info.udpConfig.destinationPort = udp["destinationPort"];
                imu_info.udpConfig.sizeOfAuxHeader = udp["sizeOfAuxHeader"];

                imus_.push_back(imu_info);
                return true;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing IMU data: " << e.what() << std::endl;
        }
        return false;
    }

    bool parseGPS(const json& gps)
    {
        try
        {
            SensorInfo gps_info;
            if (parseSensorCommon(gps, gps_info))
            {
                const auto& gc = gps["gc"];
                gps_info.sensorPeriod = gc["sensorPeriod"];
                gps_info.frameID = gc["rosConfig"]["frameID"];

                // Parse UDP config
                const auto& udp = gc["udpConfig"];
                gps_info.udpConfig.hostIP = udp["hostIP"];
                gps_info.udpConfig.hostPort = udp["hostPort"];
                gps_info.udpConfig.destinationIP = udp["destinationIP"];
                gps_info.udpConfig.destinationPort = udp["destinationPort"];
                gps_info.udpConfig.sizeOfAuxHeader = udp["sizeOfAuxHeader"];

                gpss_.push_back(gps_info);
                return true;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing GPS data: " << e.what() << std::endl;
        }
        return false;
    }

    bool parseCamera(const json& camera, SensorInfo& camera_info)
    {
        try
        {
            if (parseSensorCommon(camera, camera_info))
            {
                const auto& cc = camera["cc"];
                camera_info.sensorPeriod = cc["sensorPeriod"];
                camera_info.frameID = cc["rosConfig"]["frameID"];

                // Parse UDP config
                const auto& udp = cc["udpConfig"];
                camera_info.udpConfig.hostIP = udp["hostIP"];
                camera_info.udpConfig.hostPort = udp["hostPort"];
                camera_info.udpConfig.destinationIP = udp["destinationIP"];
                camera_info.udpConfig.destinationPort = udp["destinationPort"];
                camera_info.udpConfig.sizeOfAuxHeader = udp["sizeOfAuxHeader"];

                // Parse camera intrinsic matrix parameters
                camera_info.camera_intrinsic.fx = cc["focalLengthpixel"];
                camera_info.camera_intrinsic.fy = cc["focalLengthpixel"];

                const auto& principal_point = cc["principalPoint"];
                camera_info.camera_intrinsic.cx = principal_point["x"];
                camera_info.camera_intrinsic.cy = principal_point["y"];

                return true;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error parsing camera data: " << e.what() << std::endl;
        }
        return false;
    }

    // 스레드 작업 함수들
    void processCameraData(size_t camera_index, const DatasetDirectory& dataset,
                           const DatasetDirectory::ScenarioInfo& scenario)
    {
        auto& camera = camera_receivers_[camera_index];
        const auto& camera_info = cameras_[camera_index];
        cv::Mat image;

        Camera::CameraData camera_data;

        // 이미지 저장 경로 생성
        std::string save_path =
            dataset.getSensorPath(scenario, "CAMERA", camera_info.uniqueID).string();

        while (is_running_)
        {
            if (camera->GetCameraData(camera_data))
            {
                // visualization을 위한 작은 이미지
                cv::resize(camera_data.image_data, image, cv::Size(), 0.125, 0.125);
                camera_images_[camera_index] = std::make_shared<cv::Mat>(image);

                // timestamp를 10ms 단위의 인덱스로 변환
                int timestamp_index =
                    static_cast<int>(camera_data.timestamp * 100);  // seconds to 10ms units

                // 원본 이미지 저장
                std::string filename = std::to_string(timestamp_index) + "_Intensity.jpeg";
                std::string full_path = save_path + "/" + filename;

                cv::imwrite(full_path, camera_data.image_data);

                std::cout << "Camera_" << camera_info.uniqueID << " saved: " << filename
                          << " timestamp: " << std::fixed << std::setprecision(6)
                          << camera_data.timestamp << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void processIMUData(size_t imu_index, const DatasetDirectory& dataset,
                        const DatasetDirectory::ScenarioInfo& scenario)
    {
        auto& imu = imu_receivers_[imu_index];
        const auto& imu_info = imus_[imu_index];

        while (is_running_)
        {
            try
            {
                IMU::IMUData data;
                if (imu->GetIMUData(data))
                {
                    // std::lock_guard<std::mutex> lock(data_mutex_);
                    // IMU 데이터 출력
                    std::cout << "\rIMU_" << imu_info.uniqueID << " | Acc: " << std::fixed
                              << std::setprecision(3) << "x=" << data.linear_acceleration_x
                              << " y=" << data.linear_acceleration_y
                              << " z=" << data.linear_acceleration_z << " | Gyro: "
                              << "x=" << data.angular_velocity_x << " y=" << data.angular_velocity_y
                              << " z=" << data.angular_velocity_z << std::endl
                              << std::flush;

                    // TODO: IMU 데이터 저장
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error in IMU thread " << imu_info.uniqueID << ": " << e.what()
                          << std::endl;
            }
        }
    }

    void processGPSData(size_t gps_index, const DatasetDirectory& dataset,
                        const DatasetDirectory::ScenarioInfo& scenario)
    {
        auto& gps = gps_receivers_[gps_index];
        const auto& gps_info = gpss_[gps_index];

        while (is_running_)
        {
            try
            {
                GPS::GPSData data;
                if (gps->GetGPSData(data))
                {
                    // std::lock_guard<std::mutex> lock(data_mutex_);
                    // GPS 데이터 출력
                    std::cout << "\rGPS_" << gps_info.uniqueID << " | Lat: " << std::fixed
                              << std::setprecision(6) << data.latitude << " Lon: " << data.longitude
                              << " | Time: " << data.timestamp << std::endl
                              << std::flush;

                    // TODO: GPS 데이터 저장
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error in GPS thread " << gps_info.uniqueID << ": " << e.what()
                          << std::endl;
            }
        }
    }

    void processVehicleStateData(const DatasetDirectory& dataset,
                                 const DatasetDirectory::ScenarioInfo& scenario)
    {
        VehicleState::VehicleData vehicle_data;

        // 데이터 저장 경로 생성
        std::string save_path = dataset.getSensorPath(scenario, "EGO_INFO", 0).string();

        while (is_running_)
        {
            try
            {
                if (vehicle_state_receiver_->GetVehicleState(vehicle_data))
                {
                    // timestamp를 10ms 단위의 인덱스로 변환
                    double timestamp =
                        vehicle_data.timestamp.seconds + vehicle_data.timestamp.nanoseconds * 1e-9;
                    int timestamp_index =
                        static_cast<int>(timestamp * 100);  // seconds to 10ms units

                    // 각 타임스탬프별 파일 생성
                    std::string filename =
                        save_path + "/" + std::to_string(timestamp_index) + ".txt";
                    std::ofstream state_file(filename);

                    if (!state_file.is_open())
                    {
                        std::cerr << "Failed to open vehicle state data file: " << filename
                                  << std::endl;
                        continue;
                    }

                    // 데이터 저장
                    state_file << "position: " << vehicle_data.position.x << " "
                               << vehicle_data.position.y << " " << vehicle_data.position.z << "\n";
                    state_file << "orientation: " << vehicle_data.rotation.x << " "
                               << vehicle_data.rotation.y << " " << vehicle_data.rotation.z << "\n";
                    state_file << "enu_velocity: " << vehicle_data.velocity.x << " "
                               << vehicle_data.velocity.y << " " << vehicle_data.velocity.z << "\n";
                    state_file << "velocity: " << vehicle_data.signed_velocity << " 0 0\n";
                    state_file << "angularVelocity: " << vehicle_data.angular_velocity.x << " "
                               << vehicle_data.angular_velocity.y << " "
                               << vehicle_data.angular_velocity.z << "\n";
                    state_file << "acceleration: " << vehicle_data.acceleration.x << " "
                               << vehicle_data.acceleration.y << " " << vehicle_data.acceleration.z
                               << "\n";
                    state_file << "accel: " << vehicle_data.accel_input << "\n";
                    state_file << "brake: " << vehicle_data.brake_input << "\n";
                    state_file << "steer: " << vehicle_data.steering << "\n";
                    state_file << "linkid: " << vehicle_data.mgeo_link_id << "\n";
                    state_file << "trafficlightid: null";  // TBD: 지도 데이터 바탕으로 전방 traffic
                                                           // light id 추가

                    state_file.close();

                    // 콘솔 출력
                    std::cout << "\rVehicle State"
                              << " Save: " << std::to_string(timestamp_index) + ".txt"
                              << " | Time: " << std::fixed << std::setprecision(3) << timestamp
                              << "ms"
                              << " | Pos: " << std::fixed << std::setprecision(3)
                              << "x=" << vehicle_data.position.x << " y=" << vehicle_data.position.y
                              << " z=" << vehicle_data.position.z << " | Rot: "
                              << "r=" << vehicle_data.rotation.x << " p=" << vehicle_data.rotation.y
                              << " y=" << vehicle_data.rotation.z
                              << " | Vel: " << vehicle_data.signed_velocity << " km/h" << std::endl
                              << std::flush;
                }
            }
            catch (const std::exception& e)
            {
                std::cerr << "Error in vehicle state thread: " << e.what() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

 public:
    SensorRecorder(const std::string& config_path) : config_path_(config_path), is_running_(false)
    {
    }

    bool initializeReceivers()
    {
        try
        {
            // 카메라 리시버 초기화
            for (const auto& camera : cameras_)
            {
                // 카메라 리시버 생성
                auto receiver = std::make_unique<Camera>(camera.udpConfig.destinationIP,
                                                         camera.udpConfig.destinationPort);
                camera_receivers_.push_back(std::move(receiver));
                cv::Mat dummy(SMALL_IMAGE_HEIGHT, SMALL_IMAGE_WIDTH, CV_8UC3);
                camera_images_.push_back(std::make_shared<cv::Mat>(dummy));
            }

            // IMU 리시버 초기화
            for (const auto& imu : imus_)
            {
                // IMU 리시버 생성
                auto receiver = std::make_unique<IMU>(imu.udpConfig.destinationIP,
                                                      imu.udpConfig.destinationPort);
                imu_receivers_.push_back(std::move(receiver));
            }

            // GPS 리시버 초기화
            for (const auto& gps : gpss_)
            {
                // GPS 리시버 생성
                auto receiver = std::make_unique<GPS>(gps.udpConfig.destinationIP,
                                                      gps.udpConfig.destinationPort);
                gps_receivers_.push_back(std::move(receiver));
            }

            // Vehicle State 리시버 초기화
            vehicle_state_receiver_ =
                std::make_unique<VehicleState>("127.0.0.1", 909);  // 포트 번호는 설정에 맞게 조정

            std::cout << "All sensor receivers initialized successfully" << std::endl;
            return true;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to initialize receivers: " << e.what() << std::endl;
            return false;
        }
    }

    bool loadSensorConfig()
    {
        try
        {
            // Read JSON file
            std::ifstream config_file(config_path_);
            if (!config_file.is_open())
            {
                std::cerr << "Failed to open config file: " << config_path_ << std::endl;
                return false;
            }

            config_file >> sensor_config_;

            // Parse cameras
            for (const auto& camera : sensor_config_["cameraList"])
            {
                SensorInfo camera_info;
                if (parseCamera(camera, camera_info))
                {
                    cameras_.push_back(camera_info);
                }
            }

            // Parse IMUs
            for (const auto& imu : sensor_config_["IMUList"])
            {
                parseIMU(imu);
            }

            // Parse GPSs
            for (const auto& gps : sensor_config_["GPSList"])
            {
                parseGPS(gps);
            }

            // Print loaded configuration for verification
            std::cout << "Loaded sensor configuration:" << std::endl;
            std::cout << "Number of cameras: " << cameras_.size() << std::endl;
            std::cout << "Number of IMUs: " << imus_.size() << std::endl;
            std::cout << "Number of GPSs: " << gpss_.size() << std::endl;

            // Print camera details with intrinsic matrix
            for (const auto& camera : cameras_)
            {
                std::cout << "\nCamera ID: " << camera.uniqueID << std::endl;
                std::cout << "  Frame ID: " << camera.frameID << std::endl;
                std::cout << "  UDP Host: " << camera.udpConfig.hostIP << ":"
                          << camera.udpConfig.destinationPort << std::endl;

                // Print intrinsic matrix
                std::cout << "  Intrinsic Matrix:" << std::endl;
                std::cout << "    [" << camera.camera_intrinsic.fx << ", 0, "
                          << camera.camera_intrinsic.cx << "]" << std::endl;
                std::cout << "    [0, " << camera.camera_intrinsic.fy << ", "
                          << camera.camera_intrinsic.cy << "]" << std::endl;
                std::cout << "    [0, 0, 1]" << std::endl;
            }

            // Print IMU details
            for (const auto& imu : imus_)
            {
                std::cout << "\nIMU ID: " << imu.uniqueID << std::endl;
                std::cout << "  Frame ID: " << imu.frameID << std::endl;
                std::cout << "  UDP Host: " << imu.udpConfig.hostIP << ":"
                          << imu.udpConfig.destinationPort << std::endl;
            }

            // Print GPS details
            for (const auto& gps : gpss_)
            {
                std::cout << "\nGPS ID: " << gps.uniqueID << std::endl;
                std::cout << "  Frame ID: " << gps.frameID << std::endl;
                std::cout << "  UDP Host: " << gps.udpConfig.hostIP << ":"
                          << gps.udpConfig.destinationPort << std::endl;
            }

            return initializeReceivers();
        }
        catch (const json::exception& e)
        {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
            return false;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error loading sensor config: " << e.what() << std::endl;
            return false;
        }
    }

    const std::vector<SensorInfo>& getCameras() const { return cameras_; }

    const std::vector<SensorInfo>& getIMUs() const { return imus_; }

    const std::vector<SensorInfo>& getGPSs() const { return gpss_; }

    const std::vector<std::unique_ptr<Camera>>& getCameraReceivers() const
    {
        return camera_receivers_;
    }
    const std::vector<std::unique_ptr<IMU>>& getIMUReceivers() const { return imu_receivers_; }
    const std::vector<std::unique_ptr<GPS>>& getGPSReceivers() const { return gps_receivers_; }

    bool startRecording(const DatasetDirectory& dataset,
                        const DatasetDirectory::ScenarioInfo& scenario)
    {
        is_running_ = true;

        // 카메라 스레드 시작
        for (size_t i = 0; i < camera_receivers_.size(); ++i)
        {
            camera_threads_.emplace_back(&SensorRecorder::processCameraData, this, i,
                                         std::ref(dataset), scenario);
            camera_threads_.back().detach();
        }

        // IMU 스레드 시작
        for (size_t i = 0; i < imu_receivers_.size(); ++i)
        {
            imu_threads_.emplace_back(&SensorRecorder::processIMUData, this, i, std::ref(dataset),
                                      scenario);
            imu_threads_.back().detach();
        }

        // GPS 스레드 시작
        for (size_t i = 0; i < gps_receivers_.size(); ++i)
        {
            gps_threads_.emplace_back(&SensorRecorder::processGPSData, this, i, std::ref(dataset),
                                      scenario);
            gps_threads_.back().detach();
        }

        // Vehicle State 스레드 시작
        vehicle_state_thread_ = std::thread(&SensorRecorder::processVehicleStateData, this,
                                            std::ref(dataset), scenario);
        vehicle_state_thread_.detach();

        return true;
    }

    void stopRecording()
    {
        is_running_ = false;

        // OpenCV 윈도우 정리
        for (const auto& camera : cameras_)
        {
            cv::destroyWindow("Camera_" + std::to_string(camera.uniqueID));
        }

        // 스레드 정리
        for (auto& thread : camera_threads_)
        {
            if (thread.joinable()) thread.join();
        }

        for (auto& thread : imu_threads_)
        {
            if (thread.joinable()) thread.join();
        }
        for (auto& thread : gps_threads_)
        {
            if (thread.joinable()) thread.join();
        }

        camera_threads_.clear();
        imu_threads_.clear();
        gps_threads_.clear();

        // Vehicle State 스레드 정리
        if (vehicle_state_thread_.joinable())
        {
            vehicle_state_thread_.join();
        }
    }

    ~SensorRecorder() { stopRecording(); }
};

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_sensor_config.json> <path_to_dataset>"
                  << std::endl;
        return 1;
    }

    std::string config_path = argv[1];
    std::string dataset_path = argv[2];
    SensorRecorder recorder(config_path);

    if (!recorder.loadSensorConfig())
    {
        std::cerr << "Failed to load sensor configuration" << std::endl;
        return 1;
    }

    // 데이터셋 디렉토리 생성
    DatasetDirectory dataset(dataset_path);

    // 시나리오 정보 설정
    DatasetDirectory::ScenarioInfo scenario;
    scenario.name = "R_KR_PG_KATRI__HMG_Sample_Scenario_7_0";
    scenario.time = "NOON";
    scenario.weather = "SUNNY";

    // 시나리오 추가
    dataset.addScenario(scenario);

    // 기본 구조 생성
    if (!dataset.createBaseStructure())
    {
        std::cerr << "Failed to create base structure" << std::endl;
        return 1;
    }

    // 센서 정보 가져오기
    const auto& cameras = recorder.getCameras();
    const auto& imus = recorder.getIMUs();
    const auto& gpss = recorder.getGPSs();

    // 각 카메라의 캘리브레이션 데이터 저장 및 센서 폴더 생성
    for (const auto& camera : cameras)
    {
        // Intrinsic matrix 생성
        Eigen::Matrix3f intrinsic = Eigen::Matrix3f::Zero();
        intrinsic(0, 0) = camera.camera_intrinsic.fx;
        intrinsic(1, 1) = camera.camera_intrinsic.fy;
        intrinsic(0, 2) = camera.camera_intrinsic.cx;
        intrinsic(1, 2) = camera.camera_intrinsic.cy;
        intrinsic(2, 2) = 1.0f;

        // Extrinsic matrix 생성
        Eigen::Matrix4f extrinsic = Eigen::Matrix4f::Identity();
        // TODO: position.rot에서 roll, pitch, yaw를 rotation matrix로 변환

        Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
        rotation = Eigen::AngleAxisf(DEG2RAD(camera.position.rot.z), Eigen::Vector3f::UnitZ()) *
                   Eigen::AngleAxisf(DEG2RAD(camera.position.rot.y), Eigen::Vector3f::UnitY()) *
                   Eigen::AngleAxisf(DEG2RAD(camera.position.rot.x), Eigen::Vector3f::UnitX());

        // 위치 설정
        extrinsic.block<3, 1>(0, 3) =
            Eigen::Vector3f(camera.position.pos.x, camera.position.pos.y, camera.position.pos.z);
        extrinsic.block<3, 3>(0, 0) = rotation;

        std::string camera_name = "CAMERA_" + std::to_string(camera.uniqueID);

        // 캘리브레이션 데이터 저장
        if (!dataset.saveCalibrationData(camera_name, intrinsic, extrinsic, config_path))
        {
            std::cerr << "Failed to save calibration data for " << camera_name << std::endl;
            return 1;
        }

        // 센서 데이터 폴더 생성
        if (!dataset.createSensorFolder(scenario, "CAMERA", camera.uniqueID))
        {
            std::cerr << "Failed to create sensor folder for " << camera_name << std::endl;
            return 1;
        }
    }

    // IMU 센서 폴더 생성
    for (const auto& imu : imus)
    {
        if (!dataset.createSensorFolder(scenario, "IMU", imu.uniqueID))
        {
            std::cerr << "Failed to create sensor folder for IMU_" << imu.uniqueID << std::endl;
            return 1;
        }
    }

    // GPS 센서 폴더 생성
    for (const auto& gps : gpss)
    {
        if (!dataset.createSensorFolder(scenario, "GPS", gps.uniqueID))
        {
            std::cerr << "Failed to create sensor folder for GPS_" << gps.uniqueID << std::endl;
            return 1;
        }
    }

    std::cout << "Dataset directory structure created successfully at: " << dataset_path
              << std::endl;

    // 데이터 수집 시작
    if (!recorder.startRecording(dataset, scenario))
    {
        std::cerr << "Failed to start recording" << std::endl;
        return 1;
    }

    std::cout << "\nRecording started. Press 'q' to stop..." << std::endl;

    // 종료 조건 체크
    char key;
    while (true)
    {
        key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            std::cout << "\nStopping recording..." << std::endl;
            break;
        }
        // 6장 이미지 합쳐서 visualization 표시 (SMALL_IMAGE_WIDTHxSMALL_IMAGE_HEIGHT 해상도 6장)
        // (2,3) 배열로 600x224 크기의 결과 이미지 생성
        if (recorder.camera_images_.size() >= 6)
        {
            cv::Mat combined(224, 600, CV_8UC3);  // 크기 수정
            recorder.camera_images_[1]->copyTo(
                combined(cv::Rect(0, 0, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT)));  // 2
            recorder.camera_images_[0]->copyTo(combined(
                cv::Rect(SMALL_IMAGE_WIDTH, 0, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT)));  // 1
            recorder.camera_images_[2]->copyTo(
                combined(cv::Rect(400, 0, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT)));  // 3
            recorder.camera_images_[4]->copyTo(combined(
                cv::Rect(0, SMALL_IMAGE_HEIGHT, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT)));  // 5
            recorder.camera_images_[3]->copyTo(
                combined(cv::Rect(SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT, SMALL_IMAGE_WIDTH,
                                  SMALL_IMAGE_HEIGHT)));  // 4
            recorder.camera_images_[5]->copyTo(combined(
                cv::Rect(400, SMALL_IMAGE_HEIGHT, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT)));  // 6
            cv::imshow("Combined", combined);
            cv::waitKey(1);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 데이터 수집 종료
    recorder.stopRecording();
    std::cout << "Recording stopped successfully" << std::endl;

    return 0;
}
