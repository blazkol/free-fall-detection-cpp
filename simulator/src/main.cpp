#include <iostream>
#include <fstream>
#include <sstream>
#include <cstddef>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

void read_data(std::ifstream &file, std::vector<std::string> &headers, std::streampos &data_start_pos, const std::string &column_name) {
    // Find column index
    int column_index = -1;
    for (std::size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == column_name) {
            column_index = static_cast<int>(i);
            break;
        }
    }
    if (column_index == -1) {
        std::cerr << "Column not found: " << column_name << std::endl;
        return;
    }

    // Read next data line
    std::vector<std::string> data_row;
    std::string line, cell;

    if (!std::getline(file, line)) {
        file.clear(); // Clear EOF flag
        file.seekg(data_start_pos);
        std::getline(file, line);
    }
    std::stringstream ss(line);
    
    while (std::getline(ss, cell, ',')) {
        data_row.push_back(cell);
    }

    std::cout << "Value for column '" << column_name << "': " << data_row[column_index] << std::endl;
}
   

int main() {
    std::cout << "IMU simulator started." << std::endl;

    // Open the CSV file
    std::string filename = "./data/2023-01-16-15-33-09-imu.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return 0;
    }

    // Read header line
    std::vector<std::string> headers;
    std::string line, cell;

    std::getline(file, line);
    std::stringstream ss(line);

    while (std::getline(ss, cell, ',')) {
        boost::algorithm::trim(cell);
        headers.push_back(cell);
    }

    std::cout << "Headers:" << std::endl;
    for (std::string i: headers) {
        std::cout << '[' << i << ']';
    }
    std::cout << std::endl;

    std::streampos data_start_pos = file.tellg();

    read_data(file, headers, data_start_pos, "ax");
    read_data(file, headers, data_start_pos, "ay");
    read_data(file, headers, data_start_pos, "az");
    read_data(file, headers, data_start_pos, "gx");
    read_data(file, headers, data_start_pos, "gy");
    read_data(file, headers, data_start_pos, "gz");

    file.close();

    std::cout << "IMU simulator finished." << std::endl;

    return 0;
}