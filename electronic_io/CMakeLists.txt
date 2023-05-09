cmake_minimum_required(VERSION 3.10.2)
project(electronic_io)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

#catkin_lint: ignore uninstalled_script
catkin_install_python(PROGRAMS
  nodes/devices
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  # catkin_lint - checks validity of package.xml and CMakeLists.txt
  # ROS buildfarm calls this without any environment and with empty rosdep cache,
  # so we have problems reading the list of packages from env
  # see https://github.com/ros-infrastructure/ros_buildfarm/issues/923
  if(DEFINED ENV{ROS_HOME})
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{ROS_HOME}")
  else()
    #catkin_lint: ignore_once env_var
    set(ROS_HOME "$ENV{HOME}/.ros")
  endif()

  #catkin_lint: ignore_once env_var
  if(DEFINED ENV{ROS_ROOT} AND EXISTS "${ROS_HOME}/rosdep/sources.cache")
    roslint_custom(catkin_lint "-W2" .)
  endif()

  set(ROSLINT_PYTHON_OPTS "--max-line-length=120")
  roslint_python(
    nodes/devices
    nodes/fake_io_board
    src/${PROJECT_NAME}/__init__.py
    src/${PROJECT_NAME}/device.py
    src/${PROJECT_NAME}/io_board_client.py
    src/${PROJECT_NAME}/io_board_server.py
    src/${PROJECT_NAME}/pins.py
    src/${PROJECT_NAME}/devices/ampere_meter.py
    src/${PROJECT_NAME}/devices/battery.py
    src/${PROJECT_NAME}/devices/dimmable_led.py
    src/${PROJECT_NAME}/devices/output_group.py
    src/${PROJECT_NAME}/devices/power_switch.py
    src/${PROJECT_NAME}/devices/thermometer.py
    src/${PROJECT_NAME}/devices/voltmeter.py
    src/${PROJECT_NAME}/virtual_pins/digital_pin_combo.py
    src/${PROJECT_NAME}/virtual_pins/linear_adc_pin.py
  )

  roslint_add_test()
endif()