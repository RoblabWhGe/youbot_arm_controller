PROJECT(YouBot_Arm_Controller)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules")

find_package(youBot_Driver REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system)
include_directories(${youBot_Driver_INCLUDE_DIR})

SET(GUI_FILES src/JointController.ui)
SET(QT_HEADER_FILES src/JointController.h)
SET(SRC_FILES src/main.cpp src/JointController.cpp src/Manipulator.cpp src/OfflineManipulator.cpp)

#QT Gui stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_WRAP_UI(GUI_HEADER ${GUI_FILES})
QT4_WRAP_CPP(QT_MOCS ${QT_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(${PROJECT_NAME} ${SRC_FILES} ${QT_MOCS} ${GUI_HEADER})
target_link_libraries(${PROJECT_NAME} ${youBot_Driver_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES})
