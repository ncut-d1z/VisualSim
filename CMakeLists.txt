cmake_minimum_required(VERSION 3.20)
project(VisualSim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs imgproc)

add_executable(visual_sim
    src/main.cpp
)
target_include_directories(visual_sim PRIVATE src)
target_link_libraries(visual_sim PRIVATE Eigen3::Eigen ${OpenCV_LIBS})

include(CTest)
if(BUILD_TESTING)
  add_executable(trajectory_tests tests/TrajectoryTests.cpp)
  target_include_directories(trajectory_tests PRIVATE src)
  target_link_libraries(trajectory_tests PRIVATE Eigen3::Eigen)
  add_test(NAME trajectory_tests COMMAND trajectory_tests)
endif()
