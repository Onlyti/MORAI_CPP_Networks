#include <iostream>
#include <opencv2/opencv.hpp>
#include <iomanip>

#include "sensors/camera.hpp"

int main(int argc, char** argv)
{
    // Wait for debug attach
    // std::cout << "Waiting for debug attach..." << std::endl;
    // std::cin.get();

    int port = argc > 1 ? std::stoi(argv[1]) : 7777;

    printf("port: %d\n", port);
    try
    {
        std::cout << "Camera receiver example" << std::endl;
        // UDP 수신을 위한 Camera 객체 생성 (IP와 포트 설정)
        Camera camera("127.0.0.1", port);  // 실제 MORAI 시뮬레이터의 IP와 포트로 변경하세요

        std::cout << "Waiting for camera data..." << std::endl;

        while (true)
        {
            Camera::CameraData camera_data;
            Camera::BoundingBoxData bbox_data;

            // 카메라 데이터 수신
            if (camera.GetSyncData(camera_data, bbox_data) == false)
            {
                std::cout << "Failed to get sync data" << std::endl;
                continue;
            }
            else if (camera_data.image_data.empty() == false)
            {
                std::cout << "Received camera data" << std::endl;
                std::cout << "Camera timestamp: " << std::endl
                          << "\t" << std::fixed << std::setprecision(3) << camera_data.timestamp << std::endl;
                std::cout << "Bounding box timestamp: " << std::endl
                          << "\t" << std::fixed << std::setprecision(3) << bbox_data.timestamp << std::endl;

                // 이미지에 2D 바운딩 박스 그리기
                for (size_t i = 0; i < bbox_data.bbox_2d.size(); i++)
                {
                    const auto& bbox = bbox_data.bbox_2d[i];
                    const auto& cls = bbox_data.classes[i];

                    // 바운딩 박스 그리기
                    cv::rectangle(camera_data.image_data, cv::Point(bbox.x_min, bbox.y_min),
                                  cv::Point(bbox.x_max, bbox.y_max), cv::Scalar(0, 255, 0), 2);

                    // 클래스 정보 표시
                    std::string class_text = "Class: " + std::to_string(cls.group_id) + "," +
                                             std::to_string(cls.class_id) + "," +
                                             std::to_string(cls.sub_class_id);
                    cv::putText(camera_data.image_data, class_text,
                                cv::Point(bbox.x_min, bbox.y_min - 10), cv::FONT_HERSHEY_SIMPLEX,
                                0.5, cv::Scalar(0, 255, 0), 1);
                }

                // 이미지 표시
                cv::imshow("MORAI Camera", camera_data.image_data);

                // 'q' 키를 누르면 종료
                if (cv::waitKey(1) == 'q')
                {
                    break;
                }
            }
            else
            {
                std::cout << "No image data" << std::endl;
                // std::cout << "Camera size: " << camera_data.image_data.size() << std::endl;
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    cv::destroyAllWindows();
    return 0;
}