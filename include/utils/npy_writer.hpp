#pragma once

#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>

class NPYWriter {
public:
    static bool saveMatrix(const std::string& filename, 
                         const Eigen::MatrixXf& matrix) {
        try {
            std::ofstream file(filename, std::ios::binary);
            if (!file.is_open()) {
                return false;
            }

            // numpy는 row-major, Eigen은 column-major이므로 전치해서 저장
            Eigen::MatrixXf transposed_matrix = matrix.transpose();

            // Write magic string and version
            const char magic[] = "\x93NUMPY";
            const char version[] = "\x01\x00";
            file.write(magic, 6);
            file.write(version, 2);

            // Create header (전치된 행렬의 shape 사용)
            std::string header = "{'descr': '<f4', 'fortran_order': False, 'shape': (" + 
                               std::to_string(transposed_matrix.rows()) + ", " + 
                               std::to_string(transposed_matrix.cols()) + ")}";
            
            // Pad header to 64 bytes alignment
            int remainder = (10 + header.size()) % 64;
            int pad_len = remainder == 0 ? 0 : 64 - remainder;
            header.append(pad_len, ' ');
            header += "\n";

            // Write header length and header
            uint16_t header_len = static_cast<uint16_t>(header.size());
            file.write(reinterpret_cast<char*>(&header_len), sizeof(uint16_t));
            file.write(header.c_str(), header.size());

            // Write transposed data
            file.write(reinterpret_cast<const char*>(transposed_matrix.data()), 
                      transposed_matrix.size() * sizeof(float));

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error saving NPY file: " << e.what() << std::endl;
            return false;
        }
    }
}; 