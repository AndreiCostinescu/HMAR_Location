cmake_minimum_required(VERSION 2.8)
 
PROJECT(HMAR_Location)

add_compile_options(-std=c++11)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(HMAR ./src/Graph.cpp ./src/svm.cpp ./src/dbscan.cpp ./src/util.cpp ./src/main.cpp)
 
if(VTK_LIBRARIES)
  target_link_libraries(HMAR ${VTK_LIBRARIES})
else()
  target_link_libraries(HMAR vtkHybrid vtkWidgets)
endif()


#set_property(TARGET s../HMAR PROPERTY C_STANDARD 99)
