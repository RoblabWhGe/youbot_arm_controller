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

# Load boost stuff
find_package(Boost COMPONENTS filesystem system thread REQUIRED)

# Define source files
SET(SRC_FILES src/main.cpp src/JointController.cpp src/Manipulator.cpp src/OfflineManipulator.cpp)
SET(KINEMTAIC_SRC src/KinematicsSolver.cpp)
SET(GUI_FILES ui/JointController.ui)
SET(QT_HEADER_FILES src/JointController.h)
SET(QT_RES_FILES ui/KukaLogo.qrc)

#QT Gui stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_WRAP_UI(GUI_HEADER ${GUI_FILES})
QT4_WRAP_CPP(QT_MOCS ${QT_HEADER_FILES})
QT4_ADD_RESOURCES(QT_RES ${QT_RES_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Create binary
add_executable(${PROJECT_NAME} ${SRC_FILES} ${QT_MOCS} ${GUI_HEADER} ${QT_RES} ${KINEMTAIC_SRC})
target_link_libraries(${PROJECT_NAME} YouBotDriver soem ${Boost_LIBRARIES} ${QT_LIBRARIES})

# Compile API first
ADD_DEPENDENCIES(${PROJECT_NAME} youBot)
