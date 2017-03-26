cmake_minimum_required(VERSION 2.8.12)
project (hello)

# --------------------------------------------------------------

set(CMAKE_BUILD_TYPE "release")

# VTK ----------------------------------------------------------

find_package(VTK REQUIRED)

# GTK ----------------------------------------------------------

# Use the package PkgConfig to detect GTK+ headers/library files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

# Setup CMake to use GTK+, tell the compiler where to look for headers
# and to the linker where to look for libraries
include_directories(${GTK3_INCLUDE_DIRS})
link_directories(${GTK3_LIBRARY_DIRS})

# Add other flags to the compiler
add_definitions(${GTK3_CFLAGS_OTHER})

# Add an executable compiled from hello.c
#add_executable(viewer "src/viewer.cc")
# Link the target to the GTK+ libraries
#target_link_libraries(viewer ${GTK3_LIBRARIES})

#---------------------------------------------------------------

#add_definitions(-Wall)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -ggdb -DELPP_THREADSAFE -DELPP_STACKTRACE_ON_CRASH -DELPP_NO_DEFAULT_LOG_FILE")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

set(tools_PREFIX "${CMAKE_SOURCE_DIR}/../tools/install")

#Bring the headers
include(${VTK_USE_FILE})
include_directories(${CMAKE_SOURCE_DIR}/src ${tools_PREFIX}/include)
link_directories(${tools_PREFIX}/lib)

#For the shared library:
#set (PROJECT_LINK_LIBS libtest.so)
#link_directories(build)

add_executable(sebes "src/sebes.cc" "src/imagestore.cc" "src/dcmtk.cc" "src/util.cc")
target_link_libraries(sebes dcmqrdb dcmjpls charls dcmjpeg ijg8 ijg12 ijg16 dcmimage dcmimgle dcmnet dcmdata oflog ofstd pthread wrap z nsl)

add_executable(huz "src/huz.cc" "src/imagestore.cc" "src/dcmtk.cc" "src/util.cc")
target_link_libraries(huz dcmqrdb dcmjpls charls dcmjpeg ijg8 ijg12 ijg16 dcmimage dcmimgle dcmnet dcmdata oflog ofstd pthread wrap z nsl)
