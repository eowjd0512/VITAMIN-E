get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)

string(REPLACE " " "_" ProjectId ${ProjectId})

project(${ProjectId} C CXX)

set (CMAKE_CXX_STANDARD 11)
cmake_minimum_required(VERSION 3.5)
find_package( OpenCV REQUIRED )
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    )

    add_library(${PROJECT_NAME} SHARED
    src/Frame.cpp
    src/FrameDrawer.cpp
    src/Initializer.cpp
    src/Map.cpp
    src/MapDrawer.cpp
    src/Mapper.cpp
    src/MapPoint.cpp
    src/Matcher.cpp
    src/Optimizer.cpp
    src/System.cpp
    src/Tracker.cpp
    src/Utils.cpp
    src/Viewer.cpp
    src/VitamineFunction.cpp
    )
 
add_executable(main main.cpp)
target_link_libraries( main ${PROJECT_NAME}
${OpenCV_LIBS} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES}
)