cmake_minimum_required(VERSION 2.8)
 
PROJECT(HMAR_Location)

add_compile_options(-std=c++11)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
include_directories(/home/chen/gsl/include)

link_directories(/home/chen/gsl/lib)

# FALSE for PC
if(TRUE)
  add_executable(../HMAR 
  	./src/Prediction.cpp 
  	./src/Graph.cpp 
#  	./src/dbscan.cpp 
 # 	./src/util.cpp 
  	./src/vtkExtra.cpp 
  	./src/algo.cpp 
 #  	./src/readWriteFile.cpp 
 # 	./src/labeling_loc.cpp
 # 	./src/labeling_sec.cpp
 #  	./src/predicting.cpp
  	./src/misc.cpp
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
  	./src/DataContainer.cpp 
  	./src/ActionPrediction.cpp 
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
  	../src/DBSCAN.cpp 
  	../src/main.cpp)
  if(VTK_LIBRARIES)
    target_link_libraries(HMAR ${VTK_LIBRARIES} gsl gslcblas)
  else()
    target_link_libraries(HMAR vtkHybrid vtkWidgets gsl gslcblas)
  endif()
endif()

#set_property(TARGET s../HMAR PROPERTY C_STANDARD 99)
