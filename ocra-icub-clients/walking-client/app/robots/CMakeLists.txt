option(CODYCO_INSTALL_ALL_ROBOTS "Installation of all available robots" ON)

set(ROBOT_NAME "$ENV{YARP_ROBOT_NAME}" CACHE PATH "Name of your robot")
set(ROBOT_DIR "${CMAKE_CURRENT_SOURCE_DIR/${ROBOT_NAME}}")

if(CODYCO_INSTALL_ALL_ROBOTS)
  add_subdirectory(icubGazeboSim)
  add_subdirectory(icubSim)
  add_subdirectory(iCubGenova02)
else()
  if(ROBOT_NAME)
    add_subdirectory($ROBOT_NAME)
  endif()
endif()
