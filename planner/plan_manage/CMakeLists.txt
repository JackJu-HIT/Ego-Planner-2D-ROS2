
include_directories(include)
include_directories(../bspline_opt/include)
include_directories(../path_searching/include)
include_directories(../plan_env/include)
include_directories(./include/plan_manage)
include_directories(../include)

link_directories(/usr/include/python2.7)

add_library(    ego_planner  STATIC
                src/planner_interface.cpp
)

target_link_libraries(ego_planner 
                      bspline_opt 
                       path_searching
                       local_map )

