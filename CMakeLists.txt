cmake_minimum_required(VERSION 3.25)
project(VisualSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)

add_executable(visual_sim src/main.cpp)
target_include_directories(visual_sim PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(visual_sim PRIVATE Eigen3::Eigen ${OpenCV_LIBS})

if(MSVC)
  add_compile_options(/utf-8)
  target_compile_options(visual_sim PRIVATE /W4 /utf-8 /permissive-)
  target_compile_definitions(visual_sim PRIVATE NOMINMAX)
endif()

if(BUILD_TESTING)
  add_executable(trajectory_tests tests/TrajectoryTests.cpp)
  target_include_directories(trajectory_tests PRIVATE src)
  target_link_libraries(trajectory_tests PRIVATE Eigen3::Eigen)
  if(MSVC)
    target_compile_options(trajectory_tests PRIVATE /W4 /utf-8 /permissive-)
    target_compile_definitions(trajectory_tests PRIVATE NOMINMAX)
  endif()
  add_test(NAME trajectory_tests COMMAND trajectory_tests)
endif()
