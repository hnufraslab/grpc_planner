cmake_minimum_required(VERSION 3.10)
project(grpc_planner_service)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Find Required Packages
# ============================================================================

# Find Protobuf
find_package(Protobuf REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")

# Find gRPC manually (Ubuntu package doesn't provide CMake config)
find_library(GRPC_LIBRARY NAMES grpc REQUIRED)
find_library(GRPCPP_LIBRARY NAMES grpc++ REQUIRED)
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED)
message(STATUS "Found gRPC libraries: ${GRPC_LIBRARY}, ${GRPCPP_LIBRARY}")
message(STATUS "Found gRPC plugin: ${GRPC_CPP_PLUGIN}")

# Find OMPL
find_package(ompl REQUIRED)
if(NOT OMPL_LIBRARIES)
    set(OMPL_LIBRARIES ompl)
endif()
include_directories(${OMPL_INCLUDE_DIRS})
# Add OMPL version-specific include directory
include_directories(/usr/local/include/ompl-1.6)

# Find PCL
find_package(PCL REQUIRED COMPONENTS common io surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# ============================================================================
# Generate Protobuf and gRPC files
# ============================================================================

set(PROTO_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/planner.proto
)

# Generate protobuf files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Generate gRPC files
set(GRPC_SRCS "${CMAKE_CURRENT_BINARY_DIR}/planner.grpc.pb.cc")
set(GRPC_HDRS "${CMAKE_CURRENT_BINARY_DIR}/planner.grpc.pb.h")

add_custom_command(
    OUTPUT "${GRPC_SRCS}" "${GRPC_HDRS}"
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
         --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
         -I "${CMAKE_CURRENT_SOURCE_DIR}"
         --plugin=protoc-gen-grpc="${GRPC_CPP_PLUGIN}"
         "${PROTO_FILES}"
    DEPENDS "${PROTO_FILES}"
)

# ============================================================================
# Include Directories
# ============================================================================

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../surface_fitting/include
)

# Add library search path for surface_contact
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../surface_fitting/build)

# ============================================================================
# Source Files
# ============================================================================

set(SERVICE_SOURCES
    planner_service.cpp
    ${PROTO_SRCS}
    ${GRPC_SRCS}
)

set(SERVER_SOURCES
    grpc_server.cpp
)

# ============================================================================
# Link Libraries
# ============================================================================

set(LINK_LIBRARIES
    ${GRPCPP_LIBRARY}
    ${GRPC_LIBRARY}
    ${Protobuf_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${PCL_LIBRARIES}
    surface_contact
)

# ============================================================================
# Build Targets
# ============================================================================

# Build planner service library
add_library(planner_service_lib ${SERVICE_SOURCES})
target_link_libraries(planner_service_lib ${LINK_LIBRARIES})

# Build gRPC server executable
add_executable(grpc_planner_server ${SERVER_SOURCES})
target_link_libraries(grpc_planner_server
    planner_service_lib
    ${LINK_LIBRARIES}
)

# ============================================================================
# Installation
# ============================================================================

install(TARGETS grpc_planner_server
    RUNTIME DESTINATION bin
)

# ============================================================================
# Print Configuration Summary
# ============================================================================

message(STATUS "")
message(STATUS "========================================")
message(STATUS "gRPC Planner Service Configuration")
message(STATUS "========================================")
message(STATUS "Protobuf version: ${Protobuf_VERSION}")
message(STATUS "gRPC libraries: ${GRPCPP_LIBRARY}")
message(STATUS "OMPL libraries: ${OMPL_LIBRARIES}")
message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "========================================")
message(STATUS "")
