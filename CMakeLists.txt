cmake_minimum_required(VERSION 3.5)

project(landmark-slam)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall")

set(LIB_NAME "landmark-slam")
set(DEP_DIR "${CMAKE_CURRENT_LIST_DIR}/dep")
add_subdirectory("dep")

find_package(Threads REQUIRED)

include_directories(
    "src"
    "${CSVCPP_INCLUDE_DIR}"
    "${EIGEN_INCLUDE_DIR}"
    "${SPDLOG_INCLUDE_DIR}"
)

set(SRC
    "src/Data.cpp"
    "src/Log.cpp"
    "src/Math.cpp"
    "src/Models.cpp"
    "src/State.cpp"
)

add_library(${LIB_NAME} ${SRC})
target_link_libraries(${LIB_NAME} ${CSVCPP_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

add_executable(ekf "src/ekf.cpp")
target_link_libraries(ekf ${LIB_NAME})
