execute_process(COMMAND "/home/nvidia/realsense_ws/src/cv_bridge/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/nvidia/realsense_ws/src/cv_bridge/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
