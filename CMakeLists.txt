cmake_minimum_required(VERSION 2.6 FATAL_ERROR)


project(registrator)


find_package(PCL 1.5 REQUIRED)
find_package(Eigen3)
include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(BOOST_LIBS program_options serialization random system)
FIND_PACKAGE(Boost 1.48.0 COMPONENTS ${BOOST_LIBS} REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
MESSAGE("Boost information:") 
MESSAGE("  Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}") 
MESSAGE("  Boost_LIBRARIES: ${Boost_LIBRARIES}") 
MESSAGE("  Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}") 

add_executable (registrator registrator.cpp) 
add_executable (transform transform.cpp) 

target_link_libraries(registrator  ${Boost_LIBRARIES} ${PCL_LIBRARIES} )
target_link_libraries(transform  ${Boost_LIBRARIES} ${PCL_LIBRARIES} )