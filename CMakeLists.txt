# Copyright 2024 Anton Bredenbeck, Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.14)

#%% input arguments

# ADD YOUR NEW AGENT TO THIS SEMICOLON-SEPARATED LIST!
set(ALL_MOCAPS "optitrack;qualisys;test")
set(ALL_AGENTS "ivy;udp;mavlink;ros2;ros2px4;console;log")

# process arguments: use all mocaps/agents if unset
if(DEFINED MOCAPS)
    list(TRANSFORM MOCAPS STRIP)
else()
    message(STATUS "MOCAPS is not set. Continuing with all mocaps")
    set(MOCAPS ${ALL_MOCAPS} CACHE STRING "Mocaps to build into the binary")
endif(DEFINED MOCAPS)

foreach(x ${MOCAPS})
    list(FIND ALL_MOCAPS ${x} index)
    if(index EQUAL -1)
        message(FATAL_ERROR "Mocap ${x} not in supported agent list: ${ALL_MOCAPS}")
    endif()

    string(TOUPPER ${x} uppercase)
    add_definitions(-DUSE_MOCAP_${uppercase})
endforeach()

message(STATUS "Building MOCAPS: ${MOCAPS}")


if(DEFINED AGENTS)
    list(TRANSFORM AGENTS STRIP)
else()
    message(STATUS "AGENTS is not set. Continuing with all agents")
    set(AGENTS ${ALL_AGENTS} CACHE STRING "Agents to build into the binary")
endif(DEFINED AGENTS)

foreach(x ${AGENTS})
    string(REPLACE " " "" stripped "${x}")

    list(FIND ALL_AGENTS ${stripped} index)
    if(index EQUAL -1)
        message(FATAL_ERROR "Agent ${x} not in supported agent list: ${ALL_AGENTS}")
    endif()

    string(TOUPPER ${stripped} uppercase)
    add_definitions(-DUSE_AGENT_${uppercase})
endforeach()

message(STATUS "Building AGENTS: ${AGENTS}")


#%% top level project setup

project(mocap-router LANGUAGES CXX)

# sources and executable
file(GLOB SOURCES "src/*.cpp" "src/agents/*.cpp" "src/common/*.cpp" "src/mocaps/*.cpp")
add_executable(mocap-router ${SOURCES})

# includes
include_directories(include)
include_directories(src/agents)
include_directories(src/mocaps)
include_directories(scripts)

# compiler settings
set (CMAKE_CXX_STANDARD 14)
if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


#%% find external libraries and include for all agents

# common libraries (boost and pthread)
find_package(Boost REQUIRED COMPONENTS program_options filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(mocap-router ${Boost_LIBRARIES})

find_package(Threads REQUIRED)
target_link_libraries(mocap-router Threads::Threads)

# mocap vendor specific libraries
if ("optitrack" IN_LIST MOCAPS)
    include(FetchContent)
    fetchcontent_declare(natnet_sdk_content
      URL https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.1_ubuntu.tar
      #DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    fetchcontent_makeavailable(natnet_sdk_content)

    add_library(natnet_sdk SHARED IMPORTED)
    set_target_properties(natnet_sdk PROPERTIES IMPORTED_LOCATION ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNet.so)
    target_include_directories(natnet_sdk INTERFACE ${natnet_sdk_content_SOURCE_DIR}/include)
    target_link_libraries(mocap-router natnet_sdk)
endif()

if ("qualisys" IN_LIST MOCAPS)
    # Include ExternalProject module
    include(ExternalProject)

    # Add an external project
    ExternalProject_Add(qualisys_cpp_sdk
        GIT_REPOSITORY https://github.com/qualisys/qualisys_cpp_sdk/
        GIT_TAG rt_protocol_1.25
        GIT_SUBMODULES_RECURSE TRUE
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=install
        INSTALL_COMMAND cmake --build . --target install
    )

    add_dependencies(mocap-router qualisys_cpp_sdk)

    target_include_directories(mocap-router PUBLIC ${CMAKE_BINARY_DIR}/qualisys_cpp_sdk-prefix/src/qualisys_cpp_sdk-build/install/include/qualisys_cpp_sdk)

    #find_package(qualisys_cpp_sdk REQUIRED)
    target_link_libraries(mocap-router ${CMAKE_BINARY_DIR}/qualisys_cpp_sdk-prefix/src/qualisys_cpp_sdk-build/install/lib/libqualisys_cpp_sdk.a)

endif()

# Agent-specific libraries
if ("ivy" IN_LIST AGENTS)
    find_library(IVY_LIB NAMES ivy)
    target_link_libraries(mocap-router ivy)

    find_package(PkgConfig REQUIRED)
    pkg_search_module(GLIB REQUIRED glib-2.0)

    include_directories(${GLIB_INCLUDE_DIRS})
    link_directories(${GLIB_LIBRARY_DIRS})

    add_definitions(${GLIB_CFLAGS_OTHER})
    target_link_libraries(mocap-router ${GLIB_LIBRARIES})
endif()

if ("udp" IN_LIST AGENTS)
    find_package(Boost REQUIRED COMPONENTS system)
endif()

if ("mavlink" IN_LIST AGENTS)
    include(ExternalProject)
    ExternalProject_Add(mavlink
        GIT_REPOSITORY https://github.com/mavlink/mavlink
        GIT_TAG 23eb98ca00032a7edc8b6ff1eb8914ec27c7b268
        GIT_SUBMODULES_RECURSE TRUE
        CMAKE_ARGS -H. -DCMAKE_INSTALL_PREFIX=install -DMAVLINK_DIALECT=common -DMAVLINK_VERSION=2.0
        INSTALL_COMMAND cmake --build . --target install
    )

    add_dependencies(mocap-router mavlink)
    # silence lots of warnings
    target_compile_options(mocap-router PRIVATE "-Wno-address-of-packed-member")
    target_include_directories(mocap-router PUBLIC ${CMAKE_BINARY_DIR}/mavlink-prefix/src/mavlink-build/install/include/)
endif()

if ("ros2" IN_LIST AGENTS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(geometry_msgs REQUIRED)

    ament_target_dependencies(mocap-router rclcpp geometry_msgs)
endif()

if ("ros2px4" IN_LIST AGENTS)

    # General ROS 2 dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    ament_target_dependencies(mocap-router rclcpp)

    # Specific PX4 msgs dependencies
    include(FetchContent)
    # px4msgs is BSD 3 clause licensed
    # Fetch the px4_msgs repo, but only the required message
    FetchContent_Declare(
      px4_msgs
      GIT_REPOSITORY https://github.com/PX4/px4_msgs
      GIT_TAG release/1.15
      DOWNLOAD_COMMAND
        git clone --no-checkout --branch release/1.15 https://github.com/PX4/px4_msgs ${FETCHCONTENT_BASE_DIR}/px4_msgs-src &&
        git -C ${FETCHCONTENT_BASE_DIR}/px4_msgs-src checkout HEAD CMakeLists.txt package.xml msg/VehicleOdometry.msg
      CMAKE_ARGS
        -DBUILD_TESTING=OFF
        -DTHIRDPARTY=ON
        -DPYTHON_EXECUTABLE=""
        -DPYTHON_BINDINGS=OFF
    )
    FetchContent_MakeAvailable(px4_msgs)
    # Add the dependency to make sure it is compiled
    add_dependencies(mocap-router px4_msgs)

    # Add the libraries
    add_library(px4_msgs_typesupport SHARED IMPORTED)
    set_target_properties(px4_msgs_typesupport PROPERTIES IMPORTED_LOCATION 
        ${CMAKE_BINARY_DIR}/_deps/px4_msgs-build/libpx4_msgs__rosidl_typesupport_cpp.so
    )
    add_library(px4_msgs_fastrtps SHARED IMPORTED)
    set_target_properties(px4_msgs_fastrtps PROPERTIES IMPORTED_LOCATION 
        ${CMAKE_BINARY_DIR}/_deps/px4_msgs-build/libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so
    )
    # Link them to the target
    target_link_libraries(mocap-router
        px4_msgs_typesupport
        px4_msgs_fastrtps
    )
    # Finally also define the include directories
    target_include_directories(mocap-router PUBLIC ${CMAKE_BINARY_DIR}/_deps/px4_msgs-build/rosidl_generator_cpp)
endif()
