#include "python_interface.h"

UnitreeLidarPython::UnitreeLidarPython() {
    lreader = createUnitreeLidarReader();
}

UnitreeLidarPython::~UnitreeLidarPython() {
    // Pokud SDK vyžaduje explicitní delete, učiňte tak zde
    // delete lreader; 
}

bool UnitreeLidarPython::initialize(std::string lidar_ip, std::string local_ip, 
                                   unsigned short lidar_port, unsigned short local_port) {
    return lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip) == 0;
}

void UnitreeLidarPython::startRotation() { lreader->startLidarRotation(); }
void UnitreeLidarPython::stopRotation() { lreader->stopLidarRotation(); }
void UnitreeLidarPython::setWorkMode(uint32_t mode) { lreader->setLidarWorkMode(mode); }

py::dict UnitreeLidarPython::getNextData() {
    py::dict result_dict;
    result_dict["type"] = "none";

    int result = lreader->runParse();
    
    if (result == LIDAR_IMU_DATA_PACKET_TYPE) {
        LidarImuData imu;
        if (lreader->getImuData(imu)) {
            result_dict["type"] = "imu";
            result_dict["quaternion"] = std::vector<float>{imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]};
            result_dict["gyro"] = std::vector<float>{imu.angular_velocity[0], imu.angular_velocity[1], imu.angular_velocity[2]};
            result_dict["accel"] = std::vector<float>{imu.linear_acceleration[0], imu.linear_acceleration[1], imu.linear_acceleration[2]};
        }
    } 
    else if (result == LIDAR_POINT_DATA_PACKET_TYPE) {
        PointCloudUnitree cloud;
        if (lreader->getPointCloud(cloud)) {
            result_dict["type"] = "cloud";
            result_dict["stamp"] = cloud.stamp;
            
            // Převedeme body na list listů [x, y, z, intensity]
            py::list points_list;
            for (const auto& p : cloud.points) {
                points_list.append(py::make_tuple(p.x, p.y, p.z, p.intensity, p.ring));
            }
            result_dict["points"] = points_list;
        }
    }

    return result_dict;
}

// Makro pro vytvoření modulu "unilidar_py"
PYBIND11_MODULE(unilidar_py, m) {
    py::class_<UnitreeLidarPython>(m, "UnitreeLidar")
        .def(py::init<>())
        .def("initialize", &UnitreeLidarPython::initialize)
        .def("start_rotation", &UnitreeLidarPython::startRotation)
        .def("stop_rotation", &UnitreeLidarPython::stopRotation)
        .def("set_work_mode", &UnitreeLidarPython::setWorkMode)
        .def("get_next_data", &UnitreeLidarPython::getNextData);
}