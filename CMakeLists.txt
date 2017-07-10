cmake_minimum_required(VERSION 2.8)

################################################################################
# Set a default build type if none was specified
################################################################################
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

################################################################################
# Compiler option
################################################################################
add_compile_options(-std=c++11)

################################################################################
# Find source and header files
################################################################################
file(GLOB HEADERS src/*.h)
file(GLOB SOURCES 
  	src/COS.cpp 
  	src/CKB.cpp 
  	src/CAS.cpp 
  	src/CData.cpp 
  	src/CGraph.cpp 
  	src/algo.cpp 
  	src/DataFilter.cpp
  	src/VTKExtra.cpp 
  	src/print.cpp
  	src/core.cpp
  	src/TestCase.cpp 
  	src/Evaluate.cpp 
  	src/ReadFile.cpp 
  	src/WriteFile.cpp 
  	src/DataParser.cpp 
  	src/Test.cpp 
  	src/Train.cpp 
  	src/TrainLA.cpp 
  	src/TrainSM.cpp 
  	src/ObjectPrediction.cpp  
    src/ActionPrediction.cpp 
  	src/ActionParser.cpp 
  	src/DBSCAN.cpp 
  	)

################################################################################
# Build libraries
################################################################################
if(false)
project(hmar)

# Find source files
file(GLOB HEADERS src/*.h)
file(GLOB SOURCES 
  	src/COS.cpp 
  	src/CKB.cpp 
  	src/CAS.cpp 
  	src/CData.cpp 
  	src/CGraph.cpp 
  	src/algo.cpp 
  	src/DataFilter.cpp
  	src/VTKExtra.cpp 
  	src/print.cpp
  	src/core.cpp
  	src/TestCase.cpp 
  	src/Evaluate.cpp 
  	src/ReadFile.cpp 
  	src/WriteFile.cpp 
  	src/DataParser.cpp 
  	src/Test.cpp 
  	src/Train.cpp 
  	src/TrainLA.cpp 
  	src/TrainSM.cpp 
  	src/ObjectPrediction.cpp  
    src/ActionPrediction.cpp 
  	src/ActionParser.cpp 
  	src/DBSCAN.cpp 
  	)

SET(VTK_DIR /home/chen/vtksrc/VTK-build)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# Include header files
include_directories(${HEADERS})
include_directories(
	/usr/local/include/eigen3
	/home/chen/TTS/festival/src/include
	/home/chen/TTS/speech_tools/include
	/home/chen/gsl/include)

link_directories(
	/home/chen/gsl/lib
	/home/chen/TTS/festival/src/lib
	/home/chen/TTS/speech_tools/lib
	)

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES} gsl gslcblas Festival estools estbase eststring ncurses)


# Install library
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_BINARY_DIR}/../lib)

# Install library headers
install(FILES ${HEADERS} DESTINATION ${CMAKE_BINARY_DIR}/../include)
endif()


################################################################################
# Build project
################################################################################
if(true)

project(HMAR_Location)

SET(VTK_DIR /home/chen/vtksrc/VTK-build)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(
#	${CMAKE_BINARY_DIR}/../include
	/usr/local/include/eigen3
	/home/chen/TTS/festival/src/include
	/home/chen/TTS/speech_tools/include
	/home/chen/gsl/include)

link_directories(
	/home/chen/gsl/lib
	/home/chen/TTS/festival/src/lib
	/home/chen/TTS/speech_tools/lib
#	${CMAKE_BINARY_DIR}/../lib
	)

add_executable(HMAR
	src/main.cpp
  	src/COS.cpp 
  	src/CKB.cpp 
  	src/CAS.cpp 
  	src/CData.cpp 
  	src/CGraph.cpp 
  	src/algo.cpp 
  	src/DataFilter.cpp
  	src/VTKExtra.cpp 
  	src/print.cpp
  	src/core.cpp
  	src/TestCase.cpp 
  	src/Evaluate.cpp 
  	src/ReadFile.cpp 
  	src/WriteFile.cpp 
  	src/DataParser.cpp 
  	src/Test.cpp 
  	src/Train.cpp 
  	src/TrainLA.cpp 
  	src/TrainSM.cpp 
  	src/ObjectPrediction.cpp  
    src/ActionPrediction.cpp 
  	src/ActionParser.cpp 
  	src/DBSCAN.cpp 
 )

target_link_libraries(HMAR ${VTK_LIBRARIES} gsl gslcblas Festival estools estbase eststring ncurses)

#set_property(TARGET s../HMAR PROPERTY C_STANDARD 99)
endif()