cmake_minimum_required(VERSION 2.8)

## Set a default build type if none was specified
#if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
#  message(STATUS "Setting build type to 'Debug' as none was specified.")
#  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
#  # Set the possible values of build type for cmake-gui
#  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
#    "MinSizeRel" "RelWithDebInfo")
#endif()
#
add_compile_options(-std=c++11)

project(HMAR_Location)

SET(VTK_DIR /home/chen/vtksrc/VTK-build)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(/usr/local/include/eigen3 /home/chen/gsl/include)

link_directories(/home/chen/gsl/lib)

# FALSE for PC
if(TRUE)
  add_executable(../HMAR 
  	./src/CKB.cpp 
  	./src/CAS.cpp 
  	./src/CData.cpp 
  	./src/CGraph.cpp 
  	./src/VTKExtra.cpp 
  	./src/algo.cpp 
  	./src/print.cpp
  	./src/core.cpp
  	./src/TestCase.cpp 
  	./src/Evaluate.cpp 
  	./src/ReadFile.cpp 
  	./src/WriteFile.cpp 
  	./src/DataParser.cpp 
  	./src/Test.cpp 
  	./src/Train.cpp 
  	./src/TrainLA.cpp 
  	./src/TrainSM.cpp 
  	./src/DataFilter.cpp 
  	./src/ActionPrediction.cpp 
  	./src/ActionParser.cpp 
  	./src/DBSCAN.cpp 
  	./src/main.cpp)
  if(VTK_LIBRARIES)
    target_link_libraries(../HMAR ${VTK_LIBRARIES} gsl gslcblas)
  else()
    target_link_libraries(../HMAR vtkHybrid vtkWidgets gsl gslcblas)
  endif()
else()
  add_executable(HMAR 
  	../src/Prediction.cpp 
  	../src/Graph.cpp
 # 	../src/dbscan.cpp
 # 	../src/util.cpp
  	../src/vtkExtra.cpp
  	../src/algo.cpp
 #  	../src/readWriteFile.cpp
 # 	../src/labeling_loc.cpp
#  	../src/labeling_sec.cpp
 #  	../src/predicting.cpp
  	../src/misc.cpp
  	../src/core.cpp
  	../src/TestCase.cpp 
  	../src/Evaluate.cpp 
  	../src/ReadFile.cpp 
  	../src/WriteFile.cpp 
  	../src/DataParser.cpp 
  	../src/Test.cpp 
  	../src/Train.cpp 
  	../src/TrainLA.cpp 
  	../src/TrainSM.cpp 
  	../src/DataFilter.cpp 
  	../src/DataContainer.cpp 
  	../src/ActionPrediction.cpp 
  	../src/Deployment.cpp 
  	../src/DBSCAN.cpp 
  	../src/main.cpp)
  if(VTK_LIBRARIES)
    target_link_libraries(HMAR ${VTK_LIBRARIES} gsl gslcblas)
  else()
    target_link_libraries(HMAR vtkHybrid vtkWidgets gsl gslcblas)
  endif()
endif()


install (TARGETS ../HMAR DESTINATION bin)

#set_property(TARGET s../HMAR PROPERTY C_STANDARD 99)
