execute_process(COMMAND "/home/umang/ardupilot_ws/build/iq_gnc/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/umang/ardupilot_ws/build/iq_gnc/catkin_generated/python_distutils_install.sh) returned error code ")
endif()