cmake_minimum_required(VERSION 3.5.2)
project(ur_description)

find_package(catkin REQUIRED COMPONENTS
  cerberus
)

catkin_package(
  CATKIN_DEPENDS cerberus urdf xacro
)

set(LINKS "base" "forearm" "shoulder" "upperarm" "wrist1" "wrist2" "wrist3")

function(build_meshes model)
    set(DAE_INPUT_PATH "${CMAKE_SOURCE_DIR}/meshes/${model}/visual")
    set(MESH_OUTPUT_PATH "${DAE_INPUT_PATH}")
    cerberus_build_meshes(${DAE_INPUT_PATH} "dae"
                          ${MESH_OUTPUT_PATH}
                          ${LINKS})
endfunction(build_meshes)

build_meshes(ur3)
build_meshes(ur5)
build_meshes(ur10)
build_meshes(ur3e)
build_meshes(ur5e)
build_meshes(ur10e)

install(DIRECTORY launch meshes urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
