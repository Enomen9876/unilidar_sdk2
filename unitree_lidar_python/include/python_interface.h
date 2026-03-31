#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // Pro automatický převod std::vector na Python list
#include "unitree_lidar_sdk.h"

namespace py = pybind11;
using namespace unilidar_sdk2;

class UnitreeLidarPython {
public:
    UnitreeLidarPython();
    ~UnitreeLidarPython();

    // Inicializace UDP spojení
    bool initialize(std::string lidar_ip, std::string local_ip, 
                    unsigned short lidar_port, unsigned short local_port);

    // Metody pro ovládání
    void startRotation();
    void stopRotation();
    void setWorkMode(uint32_t mode);
    
    // Metoda pro získání dat - vrací dictionary v Pythonu
    py::dict getNextData();

private:
    UnitreeLidarReader *lreader;
};