get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)

string(REPLACE " " "_" ProjectId ${ProjectId})

SET(CMAKE_BUILD_TYPE Debug)

project(${ProjectId} C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.5)
find_package( OpenCV REQUIRED )
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    )

    add_library(${PROJECT_NAME} SHARED
    src/Converter.cpp
    src/Feature.cpp
    src/FeatureExtractor.cpp
    src/Frame.cpp
    src/FrameDrawer.cpp
    src/g2o_Imitiator.cpp
    src/Initializer.cpp
    src/Map.cpp
    src/MapDrawer.cpp
    src/Mapper.cpp
    src/MapPoint.cpp
    src/Matcher.cpp
    src/Optimizer.cpp
    src/QuadTree.cpp
    src/System.cpp
    src/Tracker.cpp
    src/Utils.cpp
    src/Viewer.cpp
    src/VitamineFunction.cpp
    )
 
add_executable(main main.cpp)
target_link_libraries( main ${PROJECT_NAME}
${OpenCV_LIBS} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/thirdParty/g2o/lib/libg2o.so
)