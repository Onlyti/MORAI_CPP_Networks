#pragma once

#include <string>
#include <filesystem>
#include <vector>
#include <iostream>
#include "npy_writer.hpp"

namespace fs = std::filesystem;

class DatasetDirectory {
public:
    struct ScenarioInfo {
        std::string name;        // 시나리오 이름 (예: R_KR_PG_KATRI__HMG_Sample_Scenario_7_0)
        std::string time;        // 시간대 (예: NOON)
        std::string weather;     // 날씨 (예: SUNNY)
    };

private:
    fs::path base_path_;         // 기본 경로
    std::vector<ScenarioInfo> scenarios_;  // 시나리오 정보 리스트

    // 센서 타입별 폴더명 생성
    std::string getSensorFolderName(int sensor_id, const std::string& sensor_type) const {
        if (sensor_type == "CAMERA") {
            return "CAMERA_" + std::to_string(sensor_id);
        } else if (sensor_type == "LIDAR") {
            return "LIDAR_" + std::to_string(sensor_id);
        } else if (sensor_type == "IMU") {
            return "IMU_" + std::to_string(sensor_id);
        } else if (sensor_type == "GPS") {
            return "GPS_" + std::to_string(sensor_id);
        }
        return sensor_type;
    }

public:
    DatasetDirectory(const std::string& base_path) : base_path_(base_path) {}

    // 기본 디렉토리 구조 생성
    bool createBaseStructure() {
        try {
            // 기본 경로 생성
            fs::create_directories(base_path_);

            // Calibration 폴더 생성
            fs::create_directories(base_path_ / "Calibration");

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error creating base structure: " << e.what() << std::endl;
            return false;
        }
    }

    // 시나리오 추가
    bool addScenario(const ScenarioInfo& scenario) {
        scenarios_.push_back(scenario);
        return true;
    }

    // 센서 폴더 생성
    bool createSensorFolder(const ScenarioInfo& scenario, 
                          const std::string& sensor_type, 
                          int sensor_id) {
        try {
            fs::path scenario_path = base_path_ / scenario.name / 
                                   scenario.time / scenario.weather;
            
            // 센서 폴더 생성
            fs::path sensor_folder = getSensorFolderName(sensor_id, sensor_type);
            fs::path sensor_path = scenario_path / sensor_folder;
            
            fs::create_directories(sensor_path);

            // 시나리오 관련 추가 폴더 생성
            fs::create_directories(scenario_path / "EGO_INFO");
            fs::create_directories(scenario_path / "OBJECT_INFO");
            fs::create_directories(scenario_path / "TRAFFIC_INFO");

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error creating sensor folder: " << e.what() << std::endl;
            return false;
        }
    }

    // 전체 디렉토리 구조 생성
    bool createFullStructure() {
        if (!createBaseStructure()) {
            return false;
        }

        for (const auto& scenario : scenarios_) {
            fs::path scenario_path = base_path_ / scenario.name / 
                                   scenario.time / scenario.weather;
            
            try {
                // 시나리오 기본 폴더 생성
                fs::create_directories(scenario_path);

                // 기본 정보 폴더 생성
                fs::create_directories(scenario_path / "EGO_INFO");
                fs::create_directories(scenario_path / "OBJECT_INFO");
                fs::create_directories(scenario_path / "TRAFFIC_INFO");

            } catch (const std::exception& e) {
                std::cerr << "Error creating scenario structure: " << e.what() << std::endl;
                return false;
            }
        }

        return true;
    }

    // 경로 가져오기
    fs::path getScenarioPath(const ScenarioInfo& scenario) const {
        return base_path_ / scenario.name / scenario.time / scenario.weather;
    }

    fs::path getSensorPath(const ScenarioInfo& scenario, 
                          const std::string& sensor_type, 
                          int sensor_id) const {
        return getScenarioPath(scenario) / getSensorFolderName(sensor_id, sensor_type);
    }

    fs::path getCalibrationPath() const {
        return base_path_ / "Calibration";
    }

    bool saveCalibrationData(const std::string& camera_name,
                           const Eigen::Matrix3f& intrinsic,
                           const Eigen::Matrix4f& extrinsic,
                           const std::string& config_json_path) {
        try {
            fs::path calib_path = base_path_ / "Calibration";
            fs::create_directories(calib_path);

            // Save intrinsic matrix as .npy
            std::string intrinsic_filename = camera_name + "_intrinsic.npy";
            fs::path intrinsic_path = calib_path / intrinsic_filename;
            if (!NPYWriter::saveMatrix(intrinsic_path.string(), intrinsic)) {
                return false;
            }

            // Save extrinsic matrix as .npy
            std::string extrinsic_filename = camera_name + "_extrinsic.npy";
            fs::path extrinsic_path = calib_path / extrinsic_filename;
            if (!NPYWriter::saveMatrix(extrinsic_path.string(), extrinsic)) {
                return false;
            }

            // Copy configuration JSON file
            fs::path json_dest = calib_path / fs::path(config_json_path).filename();
            fs::copy_file(config_json_path, json_dest, 
                         fs::copy_options::overwrite_existing);

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error saving calibration data: " << e.what() << std::endl;
            return false;
        }
    }
}; 