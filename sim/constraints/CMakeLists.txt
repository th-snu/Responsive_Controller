cmake_minimum_required(VERSION 2.8.6)
project(constraints)

SET(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#	FORCE
	)

add_compile_options(-DHAVE_CSTDDEF)

set(TinyDNN_USE_SERIALIZER ON)
find_package(TinyDNN QUIET)
if(NOT TinyDNN_FOUND)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Failed to find TinyDNN. glut_human_joint_limits is disabled.")
  endif()
  return()
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads QUIET)
if(NOT Threads_FOUND)
  if(DART_VERBOSE OR NOT DART_IN_SOURCE_BUILD)
    message(STATUS "Failed to find Threads. glut_human_joint_limits is disabled.")
  endif()
  return()
endif()

include_directories(${DART_INCLUDE_DIRS})
file(GLOB srcs "*.hpp" "*.cpp")
add_library(constraints ${srcs})
target_link_libraries(constraints ${DART_LIBRARIES})
