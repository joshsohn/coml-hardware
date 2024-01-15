execute_process(COMMAND "/home/raphaelpeabody/r/simulation_ws/build/python_example/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/raphaelpeabody/r/simulation_ws/build/python_example/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
