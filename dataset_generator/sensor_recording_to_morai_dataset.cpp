#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <queue>
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

struct Vector3 {
    float x, y, z;
};

struct SensorPosition {
    Vector3 pos;
    Vector3 rot;
};

struct UDPConfig {
    std::string hostIP;
    int hostPort;
    std::string destinationIP;
    int destinationPort;
    int receiveTimeout;
    bool isBufferThreadActivated;
    int receiveBufferSize;
    int sizeOfAuxHeader;
};

struct CameraIntrinsic {
    float fx; // focal length x (pixel)
    float fy; // focal length y (pixel)
    float cx; // principal point x
    float cy; // principal point y
};

class SensorInfo {
public:
    int uniqueID;
    int sensorType;
    SensorPosition position;
    UDPConfig udpConfig;
    std::string frameID;
    float sensorPeriod;
    CameraIntrinsic camera_intrinsic; // 3x3 intrinsic matrix 관련 파라미터
};

class SensorRecorder {
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

    // 콜백을 위한 dataset/scenario 정보 저장
    std::shared_ptr<DatasetDirectory> dataset_;
    DatasetDirectory::ScenarioInfo current_scenario_;

    // 데이터 저장을 위한 큐와 뮤텍스
    struct SensorData {
        enum class Type { CAMERA, IMU, GPS, VEHICLE };
        Type type;
        int sensor_id;
        double timestamp;
        std::vector<uint8_t> data; // 직렬화된 데이터
    };

    std::queue<SensorData> data_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread storage_thread_;

    // 데이터 저장 스레드 함수
    void storageThreadFunc() {
        while (is_running_ || !data_queue_.empty()) {
            std::unique_lock<std::mutex> lock(queue_mutex_);

            if (data_queue_.empty()) {
                // 100ms 동안 대기하거나 새로운 데이터가 들어올 때까지
                queue_cv_.wait_for(lock, std::chrono::milliseconds(100));
                continue;
            }

            auto data = std::move(data_queue_.front());
            data_queue_.pop();
            lock.unlock();

            switch (data.type) {
            case SensorData::Type::CAMERA:
                storeCameraData(data);
                break;
            case SensorData::Type::IMU:
                storeIMUData(data);
                break;
            case SensorData::Type::GPS:
                storeGPSData(data);
                break;
            case SensorData::Type::VEHICLE:
                storeVehicleData(data);
                break;
            }
        }
    }

    // 각 센서 타입별 저장 함수
    void storeCameraData(const SensorData& data) {
        const auto& camera_info = cameras_[data.sensor_id];

        // 데이터 역직렬화
        cv::Mat image_data;
        std::vector<uint8_t> jpeg_data(data.data.begin(), data.data.end());
        image_data = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        // 저장 경로 생성
        int timestamp_index = static_cast<int>(data.timestamp * 100);
        std::string save_path = dataset_->getSensorPath(current_scenario_, "CAMERA", camera_info.uniqueID).string();
        std::string filename = std::to_string(timestamp_index) + "_Intensity.jpeg";
        std::string full_path = save_path + "/" + filename;

        cv::imwrite(full_path, image_data);

        std::cout << "Camera_" << camera_info.uniqueID << " saved: " << filename << " timestamp: " << std::fixed
                  << std::setprecision(6) << data.timestamp << std::endl;
    }

    void storeIMUData(const SensorData& data) {
        const auto& imu_info = imus_[data.sensor_id];

        // IMU 데이터 역직렬화 및 저장
        IMU::IMUData imu_data;
        std::memcpy(&imu_data, data.data.data(), sizeof(IMU::IMUData));

        // TODO: IMU 데이터 파일 저장 구현

        std::cout << "IMU_" << imu_info.uniqueID << " data saved, timestamp: " << data.timestamp << std::endl;
    }

    void storeGPSData(const SensorData& data) {
        const auto& gps_info = gpss_[data.sensor_id];

        // GPS 데이터 역직렬화 및 저장
        GPS::GPSData gps_data;
        std::memcpy(&gps_data, data.data.data(), sizeof(GPS::GPSData));

        // TODO: GPS 데이터 파일 저장 구현

        std::cout << "GPS_" << gps_info.uniqueID << " data saved, timestamp: " << data.timestamp << std::endl;
    }

    void storeVehicleData(const SensorData& data) {
        // Vehicle 데이터 역직렬화
        VehicleState::VehicleData vehicle_data;
        std::memcpy(&vehicle_data, data.data.data(), sizeof(VehicleState::VehicleData));

        int timestamp_index = static_cast<int>(data.timestamp * 100);
        std::string save_path = dataset_->getSensorPath(current_scenario_, "EGO_INFO", 0).string();
        std::string filename = save_path + "/" + std::to_string(timestamp_index) + ".txt";

        std::ofstream state_file(filename);
        if (state_file.is_open()) {
            state_file << "position: " << vehicle_data.position.x << " " << vehicle_data.position.y << " "
                       << vehicle_data.position.z << "\n";
            // ... 나머지 데이터 저장 ...
            state_file.close();
        }
    }

    // 콜백 함수들 수정
    void onCameraData(const Camera::CameraData& data, int camera_id) {
        if (!is_running_) return;

        // visualization을 위한 작은 이미지
        cv::Mat small_image;
        cv::resize(data.image_data, small_image, cv::Size(), 0.125, 0.125);
        camera_images_[camera_id] = std::make_shared<cv::Mat>(small_image);

        // 이미지를 JPEG으로 인코딩
        std::vector<uint8_t> jpeg_buffer;
        cv::imencode(".jpg", data.image_data, jpeg_buffer);

        // 큐에 데이터 추가
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            data_queue_.push({SensorData::Type::CAMERA, camera_id, data.timestamp, std::move(jpeg_buffer)});
        }
        queue_cv_.notify_one();
    }

    void onIMUData(const IMU::IMUData& data, int imu_id) {
        if (!is_running_) return;

        // 현재 시간을 타임스탬프로 사용
        double current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count() /
                              1000.0;

        // IMU 데이터 직렬화
        std::vector<uint8_t> serialized_data(sizeof(IMU::IMUData));
        std::memcpy(serialized_data.data(), &data, sizeof(IMU::IMUData));

        // SensorData 객체 생성 및 초기화
        SensorData sensor_data;
        sensor_data.type = SensorData::Type::IMU;
        sensor_data.sensor_id = imu_id;
        sensor_data.timestamp = current_time;
        sensor_data.data = std::move(serialized_data);

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            data_queue_.push(std::move(sensor_data));
        }
        queue_cv_.notify_one();
    }

    void onGPSData(const GPS::GPSData& data, int gps_id) {
        if (!is_running_) return;

        double current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count() /
                              1000.0;

        std::vector<uint8_t> serialized_data(sizeof(GPS::GPSData));
        std::memcpy(serialized_data.data(), &data, sizeof(GPS::GPSData));

        SensorData sensor_data;
        sensor_data.type = SensorData::Type::GPS;
        sensor_data.sensor_id = gps_id;
        sensor_data.timestamp = current_time;
        sensor_data.data = std::move(serialized_data);

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            data_queue_.push(std::move(sensor_data));
        }
        queue_cv_.notify_one();
    }

    void onVehicleStateData(const VehicleState::VehicleData& data) {
        if (!is_running_) return;

        double current_time = data.timestamp.seconds + data.timestamp.nanoseconds * 1e-9;

        std::vector<uint8_t> serialized_data(sizeof(VehicleState::VehicleData));
        std::memcpy(serialized_data.data(), &data, sizeof(VehicleState::VehicleData));

        SensorData sensor_data;
        sensor_data.type = SensorData::Type::VEHICLE;
        sensor_data.sensor_id = 0; // Vehicle state는 단일 센서
        sensor_data.timestamp = current_time;
        sensor_data.data = std::move(serialized_data);

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            data_queue_.push(std::move(sensor_data));
        }
        queue_cv_.notify_one();
    }

public:
    SensorRecorder(const std::string& config_path) : config_path_(config_path), is_running_(false) {}

    bool initializeReceivers() {
        try {
            // 카메라 리시버 초기화
            for (const auto& camera : cameras_) {
                // 카메라 리시버 생성
                auto receiver =
                        std::make_unique<Camera>(camera.udpConfig.destinationIP, camera.udpConfig.destinationPort);
                camera_receivers_.push_back(std::move(receiver));
                cv::Mat dummy(SMALL_IMAGE_HEIGHT, SMALL_IMAGE_WIDTH, CV_8UC3);
                camera_images_.push_back(std::make_shared<cv::Mat>(dummy));
            }

            // IMU 리시버 초기화
            for (const auto& imu : imus_) {
                // IMU 리시버 생성
                auto receiver = std::make_unique<IMU>(imu.udpConfig.destinationIP, imu.udpConfig.destinationPort);
                imu_receivers_.push_back(std::move(receiver));
            }

            // GPS 리시버 초기화
            for (const auto& gps : gpss_) {
                // GPS 리시버 생성
                auto receiver = std::make_unique<GPS>(gps.udpConfig.destinationIP, gps.udpConfig.destinationPort);
                gps_receivers_.push_back(std::move(receiver));
            }

            // Vehicle State 리시버 초기화
            vehicle_state_receiver_ = std::make_unique<VehicleState>("127.0.0.1", 909); // 포트 번호는 설정에 맞게 조정

            std::cout << "All sensor receivers initialized successfully" << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize receivers: " << e.what() << std::endl;
            return false;
        }
    }

    bool loadSensorConfig() {
        try {
            // Read JSON file
            std::ifstream config_file(config_path_);
            if (!config_file.is_open()) {
                std::cerr << "Failed to open config file: " << config_path_ << std::endl;
                return false;
            }

            config_file >> sensor_config_;

            // Parse cameras
            for (const auto& camera : sensor_config_["cameraList"]) {
                SensorInfo camera_info;
                if (parseCamera(camera, camera_info)) {
                    cameras_.push_back(camera_info);
                }
            }

            // Parse IMUs
            for (const auto& imu : sensor_config_["IMUList"]) {
                parseIMU(imu);
            }

            // Parse GPSs
            for (const auto& gps : sensor_config_["GPSList"]) {
                parseGPS(gps);
            }

            // Print loaded configuration for verification
            std::cout << "Loaded sensor configuration:" << std::endl;
            std::cout << "Number of cameras: " << cameras_.size() << std::endl;
            std::cout << "Number of IMUs: " << imus_.size() << std::endl;
            std::cout << "Number of GPSs: " << gpss_.size() << std::endl;

            // Print camera details with intrinsic matrix
            for (const auto& camera : cameras_) {
                std::cout << "\nCamera ID: " << camera.uniqueID << std::endl;
                std::cout << "  Frame ID: " << camera.frameID << std::endl;
                std::cout << "  UDP Host: " << camera.udpConfig.hostIP << ":" << camera.udpConfig.destinationPort
                          << std::endl;

                // Print intrinsic matrix
                std::cout << "  Intrinsic Matrix:" << std::endl;
                std::cout << "    [" << camera.camera_intrinsic.fx << ", 0, " << camera.camera_intrinsic.cx << "]"
                          << std::endl;
                std::cout << "    [0, " << camera.camera_intrinsic.fy << ", " << camera.camera_intrinsic.cy << "]"
                          << std::endl;
                std::cout << "    [0, 0, 1]" << std::endl;
            }

            // Print IMU details
            for (const auto& imu : imus_) {
                std::cout << "\nIMU ID: " << imu.uniqueID << std::endl;
                std::cout << "  Frame ID: " << imu.frameID << std::endl;
                std::cout << "  UDP Host: " << imu.udpConfig.hostIP << ":" << imu.udpConfig.destinationPort
                          << std::endl;
            }

            // Print GPS details
            for (const auto& gps : gpss_) {
                std::cout << "\nGPS ID: " << gps.uniqueID << std::endl;
                std::cout << "  Frame ID: " << gps.frameID << std::endl;
                std::cout << "  UDP Host: " << gps.udpConfig.hostIP << ":" << gps.udpConfig.destinationPort
                          << std::endl;
            }

            return initializeReceivers();
        } catch (const json::exception& e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
            return false;
        } catch (const std::exception& e) {
            std::cerr << "Error loading sensor config: " << e.what() << std::endl;
            return false;
        }
    }

    const std::vector<SensorInfo>& getCameras() const { return cameras_; }

    const std::vector<SensorInfo>& getIMUs() const { return imus_; }

    const std::vector<SensorInfo>& getGPSs() const { return gpss_; }

    const std::vector<std::unique_ptr<Camera>>& getCameraReceivers() const { return camera_receivers_; }
    const std::vector<std::unique_ptr<IMU>>& getIMUReceivers() const { return imu_receivers_; }
    const std::vector<std::unique_ptr<GPS>>& getGPSReceivers() const { return gps_receivers_; }

    bool startRecording(const DatasetDirectory& dataset, const DatasetDirectory::ScenarioInfo& scenario) {
        is_running_ = true;
        dataset_ = std::make_shared<DatasetDirectory>(dataset);
        current_scenario_ = scenario;

        // 저장 스레드 시작
        storage_thread_ = std::thread(&SensorRecorder::storageThreadFunc, this);

        // 콜백 등록
        for (size_t i = 0; i < camera_receivers_.size(); ++i) {
            camera_receivers_[i]->RegisterCameraCallback(
                    std::bind(&SensorRecorder::onCameraData, this, std::placeholders::_1, i));
        }

        for (size_t i = 0; i < imu_receivers_.size(); ++i) {
            imu_receivers_[i]->RegisterCallback(std::bind(&SensorRecorder::onIMUData, this, std::placeholders::_1, i));
        }

        for (size_t i = 0; i < gps_receivers_.size(); ++i) {
            gps_receivers_[i]->RegisterCallback(std::bind(&SensorRecorder::onGPSData, this, std::placeholders::_1, i));
        }

        vehicle_state_receiver_->RegisterCallback(
                std::bind(&SensorRecorder::onVehicleStateData, this, std::placeholders::_1));

        return true;
    }

    void stopRecording() {
        is_running_ = false;
        queue_cv_.notify_all();

        if (storage_thread_.joinable()) {
            storage_thread_.join();
        }

        // OpenCV 윈도우 정리
        for (const auto& camera : cameras_) {
            cv::destroyWindow("Camera_" + std::to_string(camera.uniqueID));
        }
    }

    ~SensorRecorder() { stopRecording(); }

private:
    // parse 함수들을 private으로 이동
    bool parseSensorCommon(const json& sensor, SensorInfo& info) {
        try {
            info.uniqueID = sensor["m_SensorUniqueID"];
            info.sensorType = sensor["m_SensorType"];

            info.position.pos.x = std::stof(sensor["pos"]["x"].get<std::string>());
            info.position.pos.y = std::stof(sensor["pos"]["y"].get<std::string>());
            info.position.pos.z = std::stof(sensor["pos"]["z"].get<std::string>());

            info.position.rot.x = std::stof(sensor["rot"]["roll"].get<std::string>());
            info.position.rot.y = std::stof(sensor["rot"]["pitch"].get<std::string>());
            info.position.rot.z = std::stof(sensor["rot"]["yaw"].get<std::string>());

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error parsing sensor common data: " << e.what() << std::endl;
            return false;
        }
    }

    bool parseIMU(const json& imu) {
        try {
            SensorInfo imu_info;
            if (parseSensorCommon(imu, imu_info)) {
                const auto& ic = imu["ic"];
                imu_info.sensorPeriod = ic["sensorPeriod"];
                imu_info.frameID = ic["rosConfig"]["frameID"];

                const auto& udp = ic["udpConfig"];
                imu_info.udpConfig.hostIP = udp["hostIP"];
                imu_info.udpConfig.hostPort = udp["hostPort"];
                imu_info.udpConfig.destinationIP = udp["destinationIP"];
                imu_info.udpConfig.destinationPort = udp["destinationPort"];
                imu_info.udpConfig.sizeOfAuxHeader = udp["sizeOfAuxHeader"];

                imus_.push_back(imu_info);
                return true;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing IMU data: " << e.what() << std::endl;
        }
        return false;
    }

    bool parseGPS(const json& gps) {
        try {
            SensorInfo gps_info;
            if (parseSensorCommon(gps, gps_info)) {
                const auto& gc = gps["gc"];
                gps_info.sensorPeriod = gc["sensorPeriod"];
                gps_info.frameID = gc["rosConfig"]["frameID"];

                const auto& udp = gc["udpConfig"];
                gps_info.udpConfig.hostIP = udp["hostIP"];
                gps_info.udpConfig.hostPort = udp["hostPort"];
                gps_info.udpConfig.destinationIP = udp["destinationIP"];
                gps_info.udpConfig.destinationPort = udp["destinationPort"];
                gps_info.udpConfig.sizeOfAuxHeader = udp["sizeOfAuxHeader"];

                gpss_.push_back(gps_info);
                return true;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing GPS data: " << e.what() << std::endl;
        }
        return false;
    }

    bool parseCamera(const json& camera, SensorInfo& camera_info) {
        try {
            if (parseSensorCommon(camera, camera_info)) {
                const auto& cc = camera["cc"];
                camera_info.sensorPeriod = cc["sensorPeriod"];
                camera_info.frameID = cc["rosConfig"]["frameID"];

                const auto& udp = cc["udpConfig"];
                camera_info.udpConfig.hostIP = udp["hostIP"];
                camera_info.udpConfig.hostPort = udp["hostPort"];
                camera_info.udpConfig.destinationIP = udp["destinationIP"];
                camera_info.udpConfig.destinationPort = udp["destinationPort"];
                camera_info.udpConfig.sizeOfAuxHeader = udp["sizeOfAuxHeader"];

                camera_info.camera_intrinsic.fx = cc["focalLengthpixel"];
                camera_info.camera_intrinsic.fy = cc["focalLengthpixel"];
                const auto& principal_point = cc["principalPoint"];
                camera_info.camera_intrinsic.cx = principal_point["x"];
                camera_info.camera_intrinsic.cy = principal_point["y"];

                return true;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error parsing camera data: " << e.what() << std::endl;
        }
        return false;
    }
};

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <path_to_sensor_config.json> <path_to_dataset>" << std::endl;
        return 1;
    }

    std::string config_path = argv[1];
    std::string dataset_path = argv[2];
    SensorRecorder recorder(config_path);

    if (!recorder.loadSensorConfig()) {
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
    if (!dataset.createBaseStructure()) {
        std::cerr << "Failed to create base structure" << std::endl;
        return 1;
    }

    // 센서 정보 가져오기
    const auto& cameras = recorder.getCameras();
    const auto& imus = recorder.getIMUs();
    const auto& gpss = recorder.getGPSs();

    // 각 카메라의 캘리브레이션 데이터 저장 및 센서 폴더 생성
    for (const auto& camera : cameras) {
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

        Eigen::Matrix3f vehicle_to_camera_x_forward_rotation = Eigen::Matrix3f::Identity();
        vehicle_to_camera_x_forward_rotation =
                Eigen::AngleAxisf(DEG2RAD(camera.position.rot.z), Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(DEG2RAD(camera.position.rot.y), Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(DEG2RAD(camera.position.rot.x), Eigen::Vector3f::UnitX());
        // z전방 카메라 좌표로 변환
        Eigen::Matrix3f z_forward_rotation = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f x_forward_to_z_forward;
        x_forward_to_z_forward << 0, 0, 1, -1, 0, 0, 0, -1, 0;

        Eigen::Matrix3f rotation = vehicle_to_camera_x_forward_rotation * x_forward_to_z_forward;

        // 위치 설정
        extrinsic.block<3, 1>(0, 3) =
                Eigen::Vector3f(camera.position.pos.x, camera.position.pos.y, camera.position.pos.z);
        extrinsic.block<3, 3>(0, 0) = rotation;

        std::string camera_name = "CAMERA_" + std::to_string(camera.uniqueID);

        // 캘리브레이션 데이터 저장
        if (!dataset.saveCalibrationData(camera_name, intrinsic, extrinsic, config_path)) {
            std::cerr << "Failed to save calibration data for " << camera_name << std::endl;
            return 1;
        }

        // 센서 데이터 폴더 생성
        if (!dataset.createSensorFolder(scenario, "CAMERA", camera.uniqueID)) {
            std::cerr << "Failed to create sensor folder for " << camera_name << std::endl;
            return 1;
        }
    }

    // IMU 센서 폴더 생성
    for (const auto& imu : imus) {
        if (!dataset.createSensorFolder(scenario, "IMU", imu.uniqueID)) {
            std::cerr << "Failed to create sensor folder for IMU_" << imu.uniqueID << std::endl;
            return 1;
        }
    }

    // GPS 센서 폴더 생성
    for (const auto& gps : gpss) {
        if (!dataset.createSensorFolder(scenario, "GPS", gps.uniqueID)) {
            std::cerr << "Failed to create sensor folder for GPS_" << gps.uniqueID << std::endl;
            return 1;
        }
    }

    std::cout << "Dataset directory structure created successfully at: " << dataset_path << std::endl;

    // 데이터 수집 시작
    if (!recorder.startRecording(dataset, scenario)) {
        std::cerr << "Failed to start recording" << std::endl;
        return 1;
    }

    std::cout << "\nRecording started. Press 'q' to stop..." << std::endl;

    // 종료 조건 체크
    char key;
    while (true) {
        key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            std::cout << "\nStopping recording..." << std::endl;
            break;
        }
        // 6장 이미지 합쳐서 visualization 표시 (SMALL_IMAGE_WIDTHxSMALL_IMAGE_HEIGHT 해상도 6장)
        // (2,3) 배열로 600x224 크기의 결과 이미지 생성
        cv::Mat combined(224, 600, CV_8UC3); // 크기 수정
        if (recorder.camera_images_.size() >= 1) {
            recorder.camera_images_[0]->copyTo(
                    combined(cv::Rect(SMALL_IMAGE_WIDTH, 0, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT))); // 1
        }
        if (recorder.camera_images_.size() >= 2) {
            recorder.camera_images_[1]->copyTo(combined(cv::Rect(0, 0, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT))); // 2
        }
        if (recorder.camera_images_.size() >= 3) {
            recorder.camera_images_[2]->copyTo(combined(cv::Rect(400, 0, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT))); // 3
        }
        if (recorder.camera_images_.size() >= 4) {
            recorder.camera_images_[3]->copyTo(
                    combined(cv::Rect(SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT, SMALL_IMAGE_WIDTH,
                                      SMALL_IMAGE_HEIGHT))); // 4
        }
        if (recorder.camera_images_.size() >= 5) {
            recorder.camera_images_[4]->copyTo(
                    combined(cv::Rect(0, SMALL_IMAGE_HEIGHT, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT))); // 5
        }
        if (recorder.camera_images_.size() >= 6) {
            recorder.camera_images_[5]->copyTo(
                    combined(cv::Rect(400, SMALL_IMAGE_HEIGHT, SMALL_IMAGE_WIDTH, SMALL_IMAGE_HEIGHT))); // 6
        }

        cv::imshow("Combined", combined);
        cv::waitKey(1);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 데이터 수집 종료
    recorder.stopRecording();
    std::cout << "Recording stopped successfully" << std::endl;

    return 0;
}
