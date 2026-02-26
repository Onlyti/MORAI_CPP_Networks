#include "sensors/camera2.hpp"

#include <iostream>

using namespace MoraiCppUdp;

Camera2::Camera2(const std::string& ip_address, uint16_t port)
    : UDPReceiver(ip_address, port), is_running_(false), camera_callback_(nullptr),
      has_assembling_seq_(false), assembling_seq_(0), assembling_total_chunks_(0),
      assembling_last_chunk_no_(0), assembling_expected_image_size_(0) {
    is_running_ = true;
    thread_camera_udp_receiver_ = std::thread(&Camera2::ThreadCameraUdpReceiver, this);
}

Camera2::Camera2(const std::string& ip_address, uint16_t port, CameraDataCallback callback)
    : UDPReceiver(ip_address, port), is_running_(false), camera_callback_(callback),
      has_assembling_seq_(false), assembling_seq_(0), assembling_total_chunks_(0),
      assembling_last_chunk_no_(0), assembling_expected_image_size_(0) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    camera_callback_ = callback;
    is_running_ = true;
    thread_camera_udp_receiver_ = std::thread(&Camera2::ThreadCameraUdpReceiver, this);
}

Camera2::~Camera2() {
    is_running_ = false;
    if (thread_camera_udp_receiver_.joinable()) {
        thread_camera_udp_receiver_.join();
    }
    Close();
}

void Camera2::ThreadCameraUdpReceiver() {
    char packet_buffer[MAX_PACKET_SIZE];

    while (is_running_) {
        // Receive data from UDP
        size_t received_size = 0;
        if (!Receive(packet_buffer, MAX_PACKET_SIZE, received_size)) {
            std::cerr << "Failed to receive camera data" << std::endl;
            continue;
        }

        if (received_size > 2) {
            CameraData temp_data;
            if (ParseCameraData(packet_buffer, received_size, temp_data)) {
                // 콜백 호출
                std::lock_guard<std::mutex> callback_lock(callback_mutex_);
                if (camera_callback_) { // 콜백이 등록되어 있으면 호출
                    camera_callback_(temp_data);
                }
            }
        }
    }
}

bool Camera2::ParseCameraData(const char* buffer, size_t received_size, CameraData& data) {
    constexpr size_t kSingleHeaderSize = sizeof(int32_t);                     // size
    constexpr size_t kFragBaseHeaderSize = sizeof(int32_t) + sizeof(uint16_t) + sizeof(uint16_t); // seq+chunk+total
    constexpr size_t kFragFirstExtraSize = sizeof(int32_t);                   // full image length in first chunk

    auto reset_assembly = [this]() {
        has_assembling_seq_ = false;
        assembling_seq_ = 0;
        assembling_total_chunks_ = 0;
        assembling_last_chunk_no_ = 0;
        assembling_expected_image_size_ = 0;
        assembling_image_data_.clear();
    };

    if (buffer == nullptr || received_size == 0) {
        std::cerr << "Invalid camera packet: buffer is null or too small" << std::endl;
        return false;
    }

    // Case 1) single packet format: [size(int32_t)] + [jpeg bytes]
    int32_t single_size = 0;
    if (received_size >= kSingleHeaderSize) {
        std::memcpy(&single_size, buffer, kSingleHeaderSize);
    }
    const bool looks_single = (single_size > 0 &&
                               received_size == 2 * (kSingleHeaderSize + static_cast<size_t>(single_size)));

    if (looks_single) {
        const uint8_t* payload_ptr = reinterpret_cast<const uint8_t*>(buffer + kSingleHeaderSize);
        std::vector<uint8_t> image_data(payload_ptr, payload_ptr + static_cast<size_t>(single_size));

        cv::Mat decoded_image;
        try {
            decoded_image = cv::imdecode(image_data, cv::IMREAD_COLOR);
        } catch (const cv::Exception& e) {
            std::cerr << "Failed to decode single camera packet: " << e.what() << std::endl;
            return false;
        }
        if (decoded_image.empty()) {
            std::cerr << "Failed to decode single camera packet: empty" << std::endl;
            return false;
        }

        data.image_data = decoded_image;
        data.height = decoded_image.rows;
        data.width = decoded_image.cols;
        return true;
    }

    // Case 2) fragmented packet format:
    // first chunk(chunk_no=0): [seq(4)] [chunk_no(2)] [total_chunks(2)] [image_length(4)] [jpeg bytes]
    // other chunks: [seq(4)] [chunk_no(2)] [total_chunks(2)] [jpeg bytes]
    if (received_size < kFragBaseHeaderSize) {
        std::cerr << "Invalid fragmented packet: too small. size=" << received_size << std::endl;
        return false;
    }

    int32_t seq = 0;
    uint16_t chunk_no = 0;
    uint16_t total_chunks = 0;
    std::memcpy(&seq, buffer, sizeof(int32_t));
    std::memcpy(&chunk_no, buffer + sizeof(int32_t), sizeof(uint16_t));
    std::memcpy(&total_chunks, buffer + sizeof(int32_t) + sizeof(uint16_t), sizeof(uint16_t));

    if (total_chunks == 0 || chunk_no >= total_chunks) {
        std::cerr << "Invalid fragmented packet header. seq=" << seq
                  << ", chunk_no=" << chunk_no
                  << ", total_chunks=" << total_chunks << std::endl;
        reset_assembly();
        return false;
    }

    size_t payload_offset = kFragBaseHeaderSize;
    size_t first_chunk_expected_image_size = 0;
    if (chunk_no == 0) {
        if (received_size < (kFragBaseHeaderSize + kFragFirstExtraSize)) {
            std::cerr << "Invalid first fragmented packet: missing image length. seq=" << seq << std::endl;
            reset_assembly();
            return false;
        }
        int32_t image_length = 0;
        std::memcpy(&image_length, buffer + kFragBaseHeaderSize, sizeof(int32_t));
        if (image_length <= 0) {
            std::cerr << "Invalid image length in first fragmented packet. seq=" << seq
                      << ", image_length=" << image_length << std::endl;
            reset_assembly();
            return false;
        }
        payload_offset += kFragFirstExtraSize;
        first_chunk_expected_image_size = static_cast<size_t>(image_length);
    }

    if (received_size <= payload_offset) {
        std::cerr << "Invalid fragmented packet payload. seq=" << seq
                  << ", chunk_no=" << chunk_no
                  << ", received_size=" << received_size
                  << ", payload_offset=" << payload_offset << std::endl;
        reset_assembly();
        return false;
    }
    const uint8_t* payload_ptr = reinterpret_cast<const uint8_t*>(buffer + payload_offset);
    const size_t payload_size = received_size - payload_offset;

    // Start/restart frame assembly.
    if (chunk_no == 0) {
        reset_assembly();
        has_assembling_seq_ = true;
        assembling_seq_ = seq;
        assembling_total_chunks_ = total_chunks;
        assembling_last_chunk_no_ = 0;
        assembling_expected_image_size_ = first_chunk_expected_image_size;
    } else if (!has_assembling_seq_ || assembling_seq_ != seq || assembling_total_chunks_ != total_chunks) {
        std::cerr << "Fragment sequence mismatch. seq=" << seq
                  << ", chunk_no=" << chunk_no
                  << ", total_chunks=" << total_chunks << std::endl;
        reset_assembly();
        return false;
    }

    if (chunk_no > 0) {
        const uint16_t expected_next_chunk = static_cast<uint16_t>(assembling_last_chunk_no_ + 1);
        if (chunk_no != expected_next_chunk) {
            std::cerr << "Fragment order mismatch. seq=" << seq
                      << ", expected_chunk=" << expected_next_chunk
                      << ", got_chunk=" << chunk_no << std::endl;
            reset_assembly();
            return false;
        }
    }

    assembling_image_data_.insert(assembling_image_data_.end(), payload_ptr, payload_ptr + payload_size);
    assembling_last_chunk_no_ = chunk_no;

    if (chunk_no != static_cast<uint16_t>(total_chunks - 1)) {
        return false;
    }

    if (assembling_image_data_.empty()) {
        std::cerr << "Assembled fragmented image is empty. seq=" << seq << std::endl;
        reset_assembly();
        return false;
    }
    if (assembling_expected_image_size_ > 0 && assembling_image_data_.size() != assembling_expected_image_size_) {
        std::cerr << "Assembled size mismatch. seq=" << seq
                  << ", expected=" << assembling_expected_image_size_
                  << ", actual=" << assembling_image_data_.size() << std::endl;
        reset_assembly();
        return false;
    }
    if (assembling_image_data_.size() < 2) {
        reset_assembly();
        return false;
    }

    const bool has_jpeg_soi =
            (assembling_image_data_[0] == 0xFFu && assembling_image_data_[1] == 0xD8u);
    const bool has_jpeg_eoi = (assembling_image_data_.size() >= 2 &&
                               assembling_image_data_[assembling_image_data_.size() - 2] == 0xFFu &&
                               assembling_image_data_[assembling_image_data_.size() - 1] == 0xD9u);

    if (!has_jpeg_soi) {
        std::cerr << "Invalid JPEG SOI marker. seq=" << seq
                  << ", size=" << assembling_image_data_.size() << std::endl;
        reset_assembly();
        return false;
    }
    if (!has_jpeg_eoi) {
        std::cerr << "Missing JPEG EOI marker at frame end. seq=" << seq
                  << ", size=" << assembling_image_data_.size() << std::endl;
        reset_assembly();
        return false;
    }

    // Decode image using OpenCV
    cv::Mat decoded_image;
    try {
        decoded_image = cv::imdecode(assembling_image_data_, cv::IMREAD_COLOR);
    } catch (const cv::Exception& e) {
        std::cerr << "Failed to decode fragmented image: " << e.what() << std::endl;
        reset_assembly();
        return false;
    }
    if (decoded_image.empty()) {
        std::cerr << "Failed to decode fragmented image: empty" << std::endl;
        reset_assembly();
        return false;
    }

    reset_assembly();
    data.image_data = decoded_image;
    data.height = decoded_image.rows;
    data.width = decoded_image.cols;
    return true;
}