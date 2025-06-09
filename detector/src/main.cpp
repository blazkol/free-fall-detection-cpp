#include <iostream>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

namespace bip = boost::interprocess;

struct imu_sample {
    float ax, ay, az;
    float gx, gy, gz;
};

int main() {
    std::cout << "Free fall detector started." << std::endl;

    bip::shared_memory_object imu_shm_obj(bip::open_only, "imu_shm", bip::read_only);

    bip::mapped_region imu_region(imu_shm_obj, bip::read_only);

    imu_sample* sample = static_cast<imu_sample*>(imu_region.get_address());

    std::cout << "Read sample: ax =" << sample->ax << ", gy = " << sample->gy << std::endl;

    bip::shared_memory_object::remove("imu_shm");

    std::cout << "Free fall detector finished." << std::endl;

    return 0;
}