cmake_minimum_required(VERSION 2.8.12)
project (sebestools)

find_package(Git)
if(GIT_FOUND)
  message("git found: ${GIT_EXECUTABLE}")
endif()

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For tools ----------------------------------------------------

set(INSTALL_DIR "${CMAKE_SOURCE_DIR}/install")
set(BUILD_SHARED_LIBS OFF)

# Building DCMTK -----------------------------------------------

include(ExternalProject)

set(dcmtk_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dcmtk")
set(dcmtk_CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
  -DDCMTK_USE_CXX11_STL=ON
  -DDCMTK_ENABLE_BUILTIN_DICTIONARY=ON
  -DDCMTK_ENABLE_EXTERNAL_DICTIONARY=OFF
  -DDCMTK_WITH_THREADS=OFF
  -DDCMTK_ENABLE_PRIVATE_TAGS=ON)
                   
message("dcmtk_PREFIX='${dcmtk_PREFIX}'")
message("dcmtk_INSTALL_DIR='${INSTALL_DIR}'")
message("dcmtk_CMAKE_ARGS='${dcmtk_CMAKE_ARGS}'")
message("DCMTK_ENABLE_BUILTIN_DICTIONARY='${DCMTK_ENABLE_BUILTIN_DICTIONARY}'")
message("GIT_EXECUTABLE='${GIT_EXECUTABLE}'")

ExternalProject_Add(dcmtk
  PREFIX ${dcmtk_PREFIX}
  URL http://dicom.offis.de/download/dcmtk/release/dcmtk-3.6.2.tar.gz
#  GIT_REPOSITORY git://git.dcmtk.org/dcmtk
  INSTALL_DIR ${dcmtk_INSTALL_DIR}
#  PATCH_COMMAND patch -p1 -i ${CMAKE_SOURCE_DIR}/setsockopt.patch 
  CMAKE_ARGS ${dcmtk_CMAKE_ARGS}
)

message("INSTALL_DIR='${INSTALL_DIR}'")

# Building OpenJPEG --------------------------------------------

ExternalProject_Add(ojpg
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
  # can not be switched off as it is explicitely in their CMakeLists.txt -DBUILD_SHARED_LIBS:BOOL=OFF
  -DBUILD_SHARED_LIBS:BOOL=OFF
  PREFIX "openjpeg"
  #  GIT_REPOSITORY https://github.com/uclouvain/openjpeg.git
  URL https://github.com/uclouvain/openjpeg/archive/v2.1.2.tar.gz
)

ExternalProject_Get_Property(${openjpeg} install_dir)

# Building fmjpeg2koj ------------------------------------------

ExternalProject_Add(fmjpeg2koj DEPENDS dcmtk ojpg
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
  -DOPENJPEG=${INSTALL_DIR}
  -DDCMTK_DIR=${INSTALL_DIR}
  -DCMAKE_CXX_STANDARD=11
  -DBUILD_SHARED_LIBS:BOOL=OFF
  PREFIX "fmjpeg2koj"
  GIT_REPOSITORY https://github.com/DraconPern/fmjpeg2koj.git
)

ExternalProject_Get_Property(${fmjpeg2koj} install_dir)


# Building Easylogging++ ---------------------------------------

ExternalProject_Add(easylogging
  PREFIX "easylogging"
  GIT_REPOSITORY https://github.com/easylogging/easyloggingpp.git
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND cp ../easylogging/src/easylogging++.h  ${INSTALL_DIR}/include
)

# --------------------------------------------------------------
