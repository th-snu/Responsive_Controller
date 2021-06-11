cmake_minimum_required(VERSION 3.16)
project(Reactive_Controller)

# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE "Release" CACHE STRING
#       "Choose the type of build, options are: Debug Release
# 		RelWithDebInfo MinSizeRel."
#       FORCE)
# endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_STANDARD 14)
set(CENVDIR ${PROJECT_SOURCE_DIR}/c_env)

add_definitions()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_PREFIX_PATH ${CENVDIR})
set(FCL_INCLUDE_DIRS ${CENVDIR}/include/fcl)
set(FCL_LIBRARIES ${CENVDIR}/lib/libfcl.so)
set(CCD_INCLUDE_DIRS ${CENVDIR}/include/ccd)
set(CCD_LIBRARIES ${CENVDIR}/lib/libccd.so)
set(CMAKE_LIBRARY_PATH ${CENVDIR}/lib)

add_compile_options(-fPIC)
add_compile_options(-std=gnu++17)
add_compile_options(-Wdeprecated-declarations)
add_compile_options(-lm)
add_compile_options(-lboost_python)

add_definitions(-DPROJECT_DIR="${CMAKE_HOME_DIRECTORY}")
set(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/cmake_module")

# for eigen intellisense
add_definitions(-DENABLE_PRECOMPILED_HEADERS=OFF)

add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

find_package(DART REQUIRED COMPONENTS gui optimizer-ipopt collision-bullet collision-ode CONFIG)
find_package(Boost COMPONENTS filesystem python38 numpy38 program_options)
IF(NOT Boost_FOUND)
find_package(Boost COMPONENTS filesystem python36 numpy36 program_options)
ENDIF()

find_package(PythonLibs 3.6 REQUIRED)
find_package(PythonInterp 3.6 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(TinyXML REQUIRED)
find_package(pybind11 REQUIRED)


add_subdirectory( sim )
add_subdirectory( network )
add_subdirectory( render )
#add_subdirectory( render_qt )



