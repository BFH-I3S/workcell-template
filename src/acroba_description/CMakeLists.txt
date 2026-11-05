cmake_minimum_required(VERSION 3.16)
project(acroba_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch meshes DESTINATION share/${PROJECT_NAME})

# The workcell's cells (cells/<cell>.yaml and the generated cells/<cell>/),
# each registered in the ament index so cell_description's composition finds
# it by name (resource type cell_compositions, content this package).
install(DIRECTORY cells DESTINATION share/${PROJECT_NAME})
file(GLOB cell_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/cells ${CMAKE_CURRENT_SOURCE_DIR}/cells/*.yaml)
foreach(cell_file ${cell_files})
  string(REGEX REPLACE "\\.yaml$" "" cell ${cell_file})
  ament_index_register_resource(cell_compositions PACKAGE_NAME ${cell} CONTENT ${PROJECT_NAME})
endforeach()

ament_package()
