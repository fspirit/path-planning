project(path-planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/WebSocketMessageHandler.cpp src/KeepLanePathPlanner.cpp src/KeepLanePathPlanner.h src/OutputPath.h
        src/PathPlannerInput.h src/CartesianPoint.h src/HighwayMap.cpp src/FrenetPoint.h src/PathPlanner.h src/SimpleSplineBasedPlanner.cpp
        src/SimpleSplineBasedPlanner.h src/spline.h src/OtherCar.h src/BehaviorPlanner/States/BehaviorState.h
        src/BehaviorPlanner/FSMPlanner.cpp src/BehaviorPlanner/FSMPlanner.h src/BehaviorPlanner/States/KeepLaneState.cpp
        src/BehaviorPlanner/States/KeepLaneState.h src/BehaviorPlanner/CostCalculator.cpp src/BehaviorPlanner/CostCalculator.h
        src/BehaviorPlanner/States/ChangeLaneToLeftState.cpp src/BehaviorPlanner/States/ChangeLaneToLeftState.h
        src/BehaviorPlanner/CostFunctions/KeepMaxSpeed.h src/SplineBasedLeftChangeCapablePlanner.h src/Settings.h src/BehaviorPlanner/CostFunctions/AvoidCollision.h src/BehaviorPlanner/CostFunctions/SelectFastestLane.h src/PathGenerators/KeepLanePathGenerator.cpp src/PathGenerators/KeepLanePathGenerator.h src/PathGenerators/SplinePathGenerator.cpp src/PathGenerators/SplinePathGenerator.h src/PathGenerators/ChangeLanePathGenerator.cpp src/PathGenerators/ChangeLanePathGenerator.h src/PathGenerators/LeftChangeLanePathGenerator.h src/PathGenerators/RightChangeLanePathGenerator.h src/BehaviorPlanner/States/ChangeLaneToRightState.cpp src/BehaviorPlanner/States/ChangeLaneToRightState.h src/BehaviorPlanner/CostFunctions/ChangeLaneOnProperSpeed.h)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(path-planning ${sources})

target_link_libraries(path-planning z ssl uv uWS)
