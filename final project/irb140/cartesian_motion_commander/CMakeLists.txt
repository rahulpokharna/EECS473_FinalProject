cmake_minimum_required(VERSION 2.8.3)
project(cartesian_motion_commander)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)   
cs_add_library(cartesian_motion_commander src/cart_motion_commander.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(example_generic_cartesian_move_ac src/example_generic_cart_move_ac.cpp)
cs_add_executable(example_generic_cartesian_move_ac2 src/example_generic_cart_move_ac2.cpp)
cs_add_executable(example_generic_cartesian_move_ac3 src/example_generic_cart_move_ac3.cpp)
cs_add_executable(example_generic_cartesian_move_ac4 src/example_generic_cart_move_ac4.cpp)
cs_add_executable(example_block_grabber src/example_block_grabber.cpp)

cs_add_executable(cart_move_ac_irb5400 src/cart_move_ac_irb5400.cpp)


#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(example_generic_cartesian_move_ac cartesian_motion_commander)
target_link_libraries(example_generic_cartesian_move_ac2 cartesian_motion_commander)
target_link_libraries(example_generic_cartesian_move_ac3 cartesian_motion_commander)
target_link_libraries(example_generic_cartesian_move_ac4 cartesian_motion_commander)
target_link_libraries(example_block_grabber cartesian_motion_commander)

target_link_libraries(cart_move_ac_irb5400 cartesian_motion_commander)

cs_install()
cs_export()
    
