cmake_minimum_required(VERSION 2.8.3)
project(voxblox_rrt_planner_rt)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# c++14 required by ompl
add_definitions(-std=c++14)

# ############
# LIBRARIES #
# ############
cs_add_library(${PROJECT_NAME}
  src/voxblox_ompl_rrt.cpp
  src/voxblox_rrt_planner.cpp
  src/voxblox_hl_planner.cpp
  src/skeleton_graph_planner.cpp
)
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})

# ###########
# BINARIES #
# ###########
cs_add_executable(voxblox_rrt_planner_rt_node
  src/voxblox_rrt_planner_node.cpp
)
target_link_libraries(voxblox_rrt_planner_rt_node ${PROJECT_NAME})

cs_add_executable(voxblox_hl_planner_node
  src/voxblox_hl_planner_node.cpp
)
target_link_libraries(voxblox_hl_planner_node ${PROJECT_NAME})

# #########
# EXPORT #
# #########
cs_install()
cs_export(LIBRARIES ${OMPL_LIBRARIES})
