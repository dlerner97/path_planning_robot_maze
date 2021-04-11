execute_process(COMMAND "/home/dani/Documents/ENPM661/path_planning_robot_maze/build/turtlebot3_example/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/dani/Documents/ENPM661/path_planning_robot_maze/build/turtlebot3_example/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
