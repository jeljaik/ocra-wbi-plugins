set(robotname iCubGenova02)
file(GLOB ini ${CMAKE_CURRENT_SOURCE_DIR}/*.ini)
yarp_install(FILES ${ini} DESTINATION ${CODYCO_ROBOTS_INSTALL_DIR}/${robotname})
