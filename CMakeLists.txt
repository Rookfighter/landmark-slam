cmake_minimum_required(VERSION 3.5)

project(ekf-slam)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall")

set(BIN_NAME "ekf-slam")
set(DEP_DIR "${CMAKE_CURRENT_LIST_DIR}/dep")
add_subdirectory("dep")

include_directories(
    "src"
    "${CSVCPP_INCLUDE_DIR}"
    "${EIGEN_INCLUDE_DIR}"
)

set(SRC "src/main.cpp")
add_executable(${BIN_NAME} ${SRC})
target_link_libraries(${BIN_NAME} ${CSVCPP_LIBRARY})
