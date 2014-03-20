
FIND_PATH(youBot_Driver_INCLUDE_DIR NAMES youbot/YouBotBase.hpp
  PATHS
  $ENV{YOUBOT_DIR}
  NO_DEFAULT_PATH
)

FIND_LIBRARY(youBot_Driver_LIBRARIES NAMES YouBotDriver soem
  PATHS
  $ENV{YOUBOT_DIR}/lib 
  NO_DEFAULT_PATH
)

SET(youBot_Driver_INCLUDE_DIR ${youBot_Driver_INCLUDE_DIR} ${youBot_Driver_INCLUDE_DIR}/soem/src/)

#Set FOUND to true, if all pathes were found
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(youBot_Driver DEFAULT_MSG
                                  youBot_Driver_LIBRARIES youBot_Driver_INCLUDE_DIR)
