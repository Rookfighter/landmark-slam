cmake_minimum_required(VERSION 3.5)

project(landmark-slam)
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Wall")

set(LIB_NAME "landmark-slam")

set(DEP_DIR "${CMAKE_CURRENT_LIST_DIR}/dep")

add_subdirectory("dep")

find_package(Threads REQUIRED)
find_package(PythonLibs 3 REQUIRED)

include_directories(
    "src"
    "${CSVCPP_INCLUDE_DIR}"
    "${EIGEN_INCLUDE_DIR}"
    "${SPDLOG_INCLUDE_DIR}"
    "${MATPLOTLIBCPP_INCLUDE_DIR}"
    "${PYTHON_INCLUDE_DIR}"
)

set(SRC
    "src/Data.cpp"
    "src/Log.cpp"
    "src/Math.cpp"
    "src/Models.cpp"
    "src/State.cpp"
    "src/Plot.cpp"
    "src/EKFSlam.cpp"
    "src/FastSlam.cpp"
)

add_library(${LIB_NAME} ${SRC})
target_link_libraries(${LIB_NAME}
    ${CSVCPP_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ${PYTHON_LIBRARY}
)

add_executable(slam "src/main.cpp")
target_link_libraries(slam ${LIB_NAME})
