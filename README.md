![Static Badge](https://img.shields.io/badge/status-experimental-critical)![Static Badge](https://img.shields.io/badge/Platform-Apple_silicon-magenta)![Static Badge](https://img.shields.io/badge/Tested-Apple)![Static Badge](https://img.shields.io/badge/Platform-Ubuntu_x64-orange)![Static Badge](https://img.shields.io/badge/tested-green)![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.8.0--rev0-orange)![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![GitHub License](https://img.shields.io/github/license/dqrobotics/cpp-interface-coppeliasim-zmq)![Static Badge](https://img.shields.io/badge/based_on-ZeroMQ_remote_API-blue)


# cpp-interface-coppeliasim-zmq

A DQ Robotics interface based on the ZeroMQ remote API to connect with CoppeliaSim. This API provides more functionalities than the legacy remote API (the one used by the [DQ Robotics interface](https://github.com/dqrobotics/cpp-interface-vrep)).

|  ![Static Badge](https://img.shields.io/badge/CoppeliaSim-4.8.0--rev0-orange)  | SO | Status (C++17) | 
| ------------- | ------------- |------------- |
| ![Static Badge](https://img.shields.io/badge/Apple_silicon-magenta)| macOS ![Static Badge](https://img.shields.io/badge/Apple_silicon-magenta) | ![Static Badge](https://img.shields.io/badge/beta-yellow)|
| ![Static Badge](https://img.shields.io/badge/x64-blue) ![Static Badge](https://img.shields.io/badge/arm64-blue)   | Ubuntu {22.04, 24.04} LTS ![Static Badge](https://img.shields.io/badge/x64-blue) ![Static Badge](https://img.shields.io/badge/arm64-blue)  |  ![Static Badge](https://img.shields.io/badge/beta-yellow)|
| ![Static Badge](https://img.shields.io/badge/x64-blue) ![Static Badge](https://img.shields.io/badge/arm64-blue)   | Windows 11 ![Static Badge](https://img.shields.io/badge/x64-blue) ![Static Badge](https://img.shields.io/badge/arm64-blue)   |  ![Static Badge](https://img.shields.io/badge/unsupported-gray) | 

# Instructions for Developers 

## Basic requirements (for C++ users)

- MacOS users require [Homebrew](https://brew.sh/)
- Windows users require [vcpkg](https://vcpkg.io/en/index.html) (C:\vcpkg)

  If you do not have vcpkg:

```shell
cd C:/
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg; .\bootstrap-vcpkg.bat
.\vcpkg.exe integrate install
```

- Download and install CoppeliaSim ≥ v4.7.0-rev0 (Use CoppeliaSim arm64 for Apple Silicon Macs)

### Install [DQ Robotics](https://github.com/dqrobotics/cpp) for C++ 

Skip these steps if you already have DQ Robotics installed.

#### MacOS (Apple Silicon)

```shell
brew install eigen
```

```shell
git clone https://github.com/dqrobotics/cpp.git
cd cpp
mkdir build && cd build
cmake ..
make -j16
sudo make install
```

#### Ubuntu 

```shell
sudo add-apt-repository ppa:dqrobotics-dev/development -y
sudo apt-get update
sudo apt-get install libdqrobotics
```

#### Windows 

```shell
Instructions missing here!
```

### Install [cpp-interface-coppeliasim](https://github.com/dqrobotics/cpp-interface-coppeliasim)

#### UNIX

```shell
git clone https://github.com/dqrobotics/cpp-interface-coppeliasim.git
cd cpp-interface-coppeliasim
mkdir build && cd build
cmake ..
make -j16
sudo make install
```

#### Windows 

```shell
Instructions missing here!
```


## Additional requirements:

### MacOS (Apple Silicon)

```shell
brew install pkg-config cppzmq eigen boost
```

### Ubuntu 


```shell
sudo apt install libzmq3-dev libboost-all-dev
```

### Windows 

Required vcpkg packages:

```shell
.\vcpkg install cppzmq
```


## Build and Install (UNIX)

Example for coppeliasim-v4.8.0-rev0. Note: :warning: replace coppeliasim-v4.8.0-rev0 with your CoppeliaSim version (≥ v4.7.0-rev0). 

```shell
git clone https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq.git --recursive
cd cpp-interface-coppeliasim-zmq/submodules/zmqRemoteApi
git checkout coppeliasim-v4.8.0-rev0
cd ../.. 
mkdir build && cd build
cmake ..
make -j16
sudo make install
```

Additional step for Ubuntu users:
```shell
sudo ldconfig
```

### To Uninstall 

Go to the build folder, and run:

```shell
sudo xargs rm < install_manifest.txt
```

## Build and Install (Windows)

Run powershell as administrator:

```shell
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake ..
cmake --build . --config Release
cmake --install .
```


# Example (Find more examples [here](https://github.com/juanjqo/dqrobotics-interface-coppeliasim-examples))

1) Open CoppeliaSim. (You do not need to load a specific scene).
2) Run and enjoy!

![ezgif com-video-to-gif-converter (1)](https://github.com/juanjqo/cpp-interface-coppeliasim/assets/23158313/c916025a-de3d-4058-8edf-14976d23584a)

```cpp
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h>

using namespace DQ_robotics;
using namespace Eigen;


int main()
{
    auto vi = std::make_shared<DQ_CoppeliaSimInterfaceZMQExperimental>();
    vi->connect();

    //To enable experimental methods
    // Load the models only if they are not already on the scene.
    vi->load_from_model_browser("/robots/non-mobile/UR5.ttm", "/UR5");
    vi->load_from_model_browser("/other/reference frame.ttm", "/Current_pose");
    vi->load_from_model_browser("/other/reference frame.ttm", "/Desired_pose");


    auto jointnames = vi->get_jointnames_from_parent_object("/UR5");
    vi->set_joint_modes(jointnames, DQ_CoppeliaSimInterfaceZMQExperimental::JOINT_MODE::DYNAMIC);
    vi->set_joint_control_modes(jointnames, DQ_CoppeliaSimInterfaceZMQExperimental::JOINT_CONTROL_MODE::VELOCITY);
    vi->enable_dynamics(true);
    vi->set_engine(DQ_CoppeliaSimInterfaceZMQExperimental::ENGINE::MUJOCO);
    vi->set_simulation_time_step(0.05);
    vi->set_stepping_mode(true);
    vi->draw_trajectory(jointnames.back(), 2, {1,0,1}, 1000);

    vi->start_simulation();


    auto qd = (VectorXd(6)<< 0.5, 0.5, 0.5,0.5,0.5,0.5).finished();
    auto q = vi->get_joint_positions(jointnames);
    VectorXd error = qd-q;
    double k = 0.1;

    double epsilon = 0.1;

    while (error.norm() > epsilon)
    {
        q = vi->get_joint_positions(jointnames);
        error = qd-q;
        auto u = k*error;
        vi->set_joint_target_velocities(jointnames, u);
        vi->trigger_next_simulation_step();
        std::cout<<"error: "<<error.norm()<<std::endl;
    }
    vi->stop_simulation();
}

```


```cmake
if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if(APPLE) #APPLE
    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3
    )
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
    # The library is installed here when using the regular cmake ., make, sudo make install
    LINK_DIRECTORIES(
        /usr/local/lib/
        /opt/homebrew/lib/
        )
endif()



if(WIN32)
    include(C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    find_package(cppzmq CONFIG REQUIRED)

    set(DQROBOTICS_PATH "C:/Program Files (x86)/dqrobotics")
    add_library(dqrobotics SHARED IMPORTED)
    set_target_properties(dqrobotics PROPERTIES
        IMPORTED_LOCATION ${DQROBOTICS_PATH}/bin/dqrobotics.dll
        IMPORTED_IMPLIB   ${DQROBOTICS_PATH}/lib/dqrobotics.lib
        INTERFACE_INCLUDE_DIRECTORIES ${DQROBOTICS_PATH}/include)

    set(DQROBOTICS_COPPELIASIM_PATH "C:/Program Files (x86)/dqrobotics-interface-coppeliasim")
    add_library(dqrobotics-interface-coppeliasim SHARED IMPORTED)
    set_target_properties(dqrobotics-interface-coppeliasim PROPERTIES
        IMPORTED_LOCATION ${DQROBOTICS_COPPELIASIM_PATH}/bin/dqrobotics-interface-coppeliasim.dll
        IMPORTED_IMPLIB   ${DQROBOTICS_COPPELIASIM_PATH}/lib/dqrobotics-interface-coppeliasim.lib
        INTERFACE_INCLUDE_DIRECTORIES ${DQROBOTICS_COPPELIASIM_PATH}/include)
    target_link_libraries(dqrobotics-interface-coppeliasim INTERFACE cppzmq)
endif()

add_executable(${CMAKE_PROJECT_NAME} main.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME}
                      dqrobotics
                      dqrobotics-interface-coppeliasim-zmq)
```






