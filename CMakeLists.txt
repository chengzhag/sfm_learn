cmake_minimum_required(VERSION 2.8)
project(sfm_learn)

set(CMAKE_CXX_STANDARD 14)

# (un)set: cmake -DCVV_DEBUG_MODE=OFF ..
OPTION(CVV_DEBUG_MODE "cvvisual-debug-mode" ON)
if(CVV_DEBUG_MODE MATCHES ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCVVISUAL_DEBUGMODE")
endif()

# (un)set: cmake -DDEBUG_MODE=OFF ..
OPTION(DEBUG_MODE "debug-mode" ON)
if(DEBUG_MODE MATCHES ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")
endif()



list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Eigen
include_directories("/usr/include/eigen3")
# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# Sophus 
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
# Boost
find_package(Boost      REQUIRED COMPONENTS system chrono filesystem unit_test_framework program_options)
include_directories(${Boost_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        g2o_core g2o_stuff g2o_types_sba
        ${Boost_LIBRARIES}
        opencv_cvv
        )

add_executable(sfm_learn main.cpp Camera.cpp Camera.h common_include.h Frame.cpp Frame.h MapPoint.cpp MapPoint.h Map.cpp Map.h SFM.cpp SFM.h)
target_link_libraries(sfm_learn ${THIRD_PARTY_LIBS})