cmake_minimum_required (VERSION 3.8)

project("Globe3D")

find_package(OGRE REQUIRED COMPONENTS Bites CONFIG)

add_executable(Globe3D main.cpp "Globe3D.h" "Globe3D.cpp")

target_link_libraries(Globe3D OgreBites)
