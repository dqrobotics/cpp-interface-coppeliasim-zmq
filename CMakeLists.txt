cmake_minimum_required(VERSION 3.5...3.26)
# https://cmake.org/cmake/help/latest/variable/BUILD_SHARED_LIBS.html
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
endif()

project(dqrobotics-interface-coppeliasim-zmq LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
if (APPLE)
    find_package(cppzmq REQUIRED)
    # The DQ Robotics library is installed in /usr/local/include/ on macOS platforms.
    # The following directives are required to find and link the DQ Robotics to the
    # dqrobotics-interface-coppeliasim-zmq. Otherwise, this project could not compile
    # from sources on macOS, as reported in
    # https://github.com/dqrobotics/cpp-interface-coppeliasim-zmq/issues/16
    INCLUDE_DIRECTORIES(/usr/local/include/)
    LINK_DIRECTORIES(/usr/local/lib/)
endif()
if (WIN32)
    find_package(Boost REQUIRED COMPONENTS format)
    find_package(cppzmq CONFIG REQUIRED)
    # A must so that we don't need to manually export each public part of the library
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    set(DQROBOTICS_PATH "C:/Program Files (x86)/dqrobotics")
    INCLUDE_DIRECTORIES(${DQROBOTICS_PATH}/include/)
    LINK_DIRECTORIES(${DQROBOTICS_PATH}/lib/)
endif ()


add_library(${PROJECT_NAME} 
    src/dqrobotics/interfaces/coppeliasim/internal/_zmq_wrapper.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.cpp
#    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.cpp
#    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.cpp
#    src/dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.cpp
#    src/dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimZMQRobot.cpp
    submodules/zmqRemoteApi/clients/cpp/RemoteAPIClient.cpp

  )

target_compile_definitions(${PROJECT_NAME} PRIVATE
        -DSIM_REMOTEAPICLIENT_OBJECTS
)

target_include_directories(${PROJECT_NAME} PRIVATE
        include
        submodules/jsoncons/include
        submodules/zmqRemoteApi/clients/cpp
)

if(APPLE)
    target_link_libraries(${PROJECT_NAME}
            ${Boost_LIBRARIES}
            cppzmq
            dqrobotics
            Eigen3::Eigen
    )
endif()
if(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME}
            ${Boost_LIBRARIES}
            zmq
            dqrobotics
            Eigen3::Eigen
    )
endif()
if(WIN32)
    target_link_libraries(${PROJECT_NAME}
                            Boost::format
                            cppzmq
                            dqrobotics
                            )
endif()


#include_directories(${SOURCE_DIR})
#set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.h)
#set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${SOURCE_DIR}/RemoteAPIObjects.cpp)

SET_TARGET_PROPERTIES(${PROJECT_NAME}
    PROPERTIES PUBLIC_HEADER
    "include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h"
    )

INSTALL(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics/interfaces/coppeliasim"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

if(WIN32)
INSTALL(TARGETS ${PROJECT_NAME}
# https://stackoverflow.com/questions/21592361/cmake-install-is-not-installing-libraries-on-windows
    RUNTIME DESTINATION "bin"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
endif()


################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

# Other Headers
INSTALL(FILES
    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.h
    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h
#    include/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.h
    DESTINATION "include/dqrobotics/interfaces/coppeliasim")

# Robots
INSTALL(FILES
#    include/dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.h
#    include/dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimZMQRobot.h
    DESTINATION "include/dqrobotics/interfaces/coppeliasim/robots")

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# base folder
INSTALL(FILES
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.cpp
    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.cpp
#    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.cpp
#    src/dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobot.cpp
    DESTINATION "src/dqrobotics/interfaces/coppeliasim")

# robots folder
INSTALL(FILES
#    src/dqrobotics/interfaces/coppeliasim/robots/FrankaEmikaPandaCoppeliaSimZMQRobot.cpp
#    src/dqrobotics/interfaces/coppeliasim/robots/URXCoppeliaSimZMQRobot.cpp
    DESTINATION "src/dqrobotics/interfaces/coppeliasim/robots")


