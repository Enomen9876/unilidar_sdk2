cmake_minimum_required(VERSION 3.0)
project(unitree_lidar_sdk)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG") 

# --- Hledání závislostí pro Python ---
# Tip: Musíte mít nainstalováno: pip install pybind11
find_package(PythonLibs REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(include)
# Přidat cesty pro pybind11 a python
include_directories(${pybind11_INCLUDE_DIRS})

link_directories(lib/${CMAKE_SYSTEM_PROCESSOR})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})


# Vytvoří soubor např. unilidar_py.cpython-310-x86_64-linux-gnu.so
pybind11_add_module(unilidar_py 
    interface/python_interface.cpp
)

# Linkování SDK knihovny k python modulu
# Pokud je libunilidar_sdk2.a statická knihovna, musíte zajistit, 
# aby byla kompilována s -fPIC (což u SDK od Unitree pro x86/arm obvykle je)
target_link_libraries(unilidar_py PRIVATE libunilidar_sdk2.a)

# Nastavení výstupu modulu do kořene nebo bin složky, aby šel snadno importovat
set_target_properties(unilidar_py PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/python_examples"
)
