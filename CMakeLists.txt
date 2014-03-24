PROJECT(YouBot_Arm_Controller)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#Load and compile youBot API
include(ExternalProject)

ExternalProject_Add(youBot
  PREFIX youbot_driver
  URL ${PROJECT_SOURCE_DIR}/thirdparty/youbot_driver.zip
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/youbot_driver
)

include_directories(
  ${CMAKE_BINARY_DIR}/youbot_driver/include/youbot/youbot 
  ${CMAKE_BINARY_DIR}/youbot_driver/include/youbot/youbot/src
)
link_directories(${CMAKE_BINARY_DIR}/youbot_driver/lib)

#QT Gui Stuff
SET(GUI_FILES src/JointController.ui)
SET(QT_HEADER_FILES src/JointController.h)
SET(SRC_FILES src/main.cpp src/JointController.cpp src/Manipulator.cpp src/OfflineManipulator.cpp)

# Define if kinematics solver implemented
SET(KINEMTAIC_SRC src/KinematicsSolver.cpp)

#QT Gui stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_WRAP_UI(GUI_HEADER ${GUI_FILES})
QT4_WRAP_CPP(QT_MOCS ${QT_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(${PROJECT_NAME} ${SRC_FILES} ${QT_MOCS} ${GUI_HEADER} ${KINEMTAIC_SRC})
target_link_libraries(${PROJECT_NAME} YouBotDriver soem ${Boost_LIBRARIES} ${QT_LIBRARIES})

ADD_DEPENDENCIES(${PROJECT_NAME} youBot)
