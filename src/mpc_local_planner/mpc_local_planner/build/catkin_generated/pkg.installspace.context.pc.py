# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;mpc_local_planner_msgs;control_box_rst;teb_local_planner".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmpc_local_planner_numerics;-lmpc_local_planner_optimization".split(';') if "-lmpc_local_planner_numerics;-lmpc_local_planner_optimization" != "" else []
PROJECT_NAME = "mpc_local_planner"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.3"
