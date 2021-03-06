
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(LinearSLAM)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# postfix, based on type
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
SET(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# work out the postfix; required where we use OUTPUT_NAME
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(EXE_POSTFIX)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  SET(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  SET(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# On the Mac platform, configure the RPATH as per the install, to
# avoid the problem of loading both the built and installed versions
# of the shared targets
IF(APPLE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  SET(CMAKE_INSTALL_RPATH "")
ENDIF(APPLE)

# Set the output directory for the build executables and libraries
SET(LinearSLAM_RUNTIME_OUTPUT_DIRECTORY ${LinearSLAM_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")
IF(WIN32)
  SET(gLinearSLAM_LIBRARY_OUTPUT_DIRECTORY ${LinearSLAM_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
ELSE(WIN32)
  SET(LinearSLAM_LIBRARY_OUTPUT_DIRECTORY ${LinearSLAM_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
ENDIF(WIN32)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LinearSLAM_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LinearSLAM_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${LinearSLAM_RUNTIME_OUTPUT_DIRECTORY})

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
#LIST(APPEND CMAKE_MODULE_PATH ${LinearSLAM_SOURCE_DIR} /cmake/modules)
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})  

# Detect OS and define macros appropriately
IF(WIN32)
  ADD_DEFINITIONS(-DWINDOWS)
  MESSAGE(STATUS "Compiling on Windows")
  IF(MSVC)
    MESSAGE(STATUS "Compiling with MSVC")
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    IF (BUILD_SHARED_LIBS)
      # disable warning on missing DLL interfaces
      ADD_DEFINITIONS("/wd4251")
      # SSE2 optimizations
      ADD_DEFINITIONS("/arch:SSE2")
    ENDIF()
  ENDIF(MSVC)
ELSEIF(CYGWIN)
  MESSAGE(STATUS "Compiling on Cygwin")
  ADD_DEFINITIONS(-DCYGWIN)
ELSEIF(APPLE)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on OSX")
ELSEIF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(WIN32)

# detect Android Cross Compiler
# based on android-cmake which sets the variable ANDROID for us
IF(ANDROID)
  ADD_DEFINITIONS(-DANDROID)
  MESSAGE(STATUS "Cross compiling for Android")
ENDIF()

# For building the CHOLMOD / CSPARSE solvers
FIND_PACKAGE(Cholmod)
FIND_PACKAGE(BLAS)
FIND_PACKAGE(LAPACK)
#FIND_PACKAGE(GOTO2)
IF(CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
  MESSAGE(STATUS "Found CHOLMOD and its dependencies")
  SET(CHOLMOD_FOUND TRUE)
ELSE()
  SET(CHOLMOD_FOUND FALSE)
ENDIF()

OPTION(BUILD_LGPL_SHARED_LIBS "Build LGPL Code as Shared Libraries (LGPL Code)" ON)
SET (LinearSLAM_LGPL_LIB_TYPE STATIC)
IF (BUILD_LGPL_SHARED_LIBS)
  SET (LinearSLAM_LGPL_LIB_TYPE SHARED)
ELSE()
  MESSAGE(STATUS "Building LGPL code as static library (affects license of the binary)")
ENDIF()


# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  MESSAGE(STATUS "Compiling with GCC")
  
  IF(NOT ${ARCH} MATCHES arm)
    # Generic settings for optimisation
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4")
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4")
  ENDIF()
  # switch off optimization for debug builds
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

  # OS X
  #IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}") 
    #SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}") 
  #ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  IF(NOT ${ARCH} MATCHES arm AND ${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native") 
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native") 
  ENDIF()
  # activate warnings !!!
  SET(LinearSLAM_C_FLAGS "${LinearSLAM_C_FLAGS} -Wall -W")
  SET(LinearSLAM_CXX_FLAGS "${LinearSLAM_CXX_FLAGS} -Wall -W")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

IF(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  MESSAGE(STATUS "Compiling with Clang")

  # activate all warnings
  #SET(LinearSLAM_C_FLAGS "${LinearSLAM_C_FLAGS} -Weverything")
  #SET(LinearSLAM_CXX_FLAGS "${LinearSLAM_CXX_FLAGS} -Weverything")
  SET(LinearSLAM_C_FLAGS "${LinearSLAM_C_FLAGS} -Wall")
  SET(LinearSLAM_CXX_FLAGS "${LinearSLAM_CXX_FLAGS} -Wall")
ENDIF()


# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LinearSLAM_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${LinearSLAM_C_FLAGS}")


# Set up the top-level include directories
INCLUDE_DIRECTORIES(${LinearSLAM_SOURCE_DIR} ${LinearSLAM_EIGEN3_INCLUDE})

# Generate config.h
SET(LinearSLAM_HAVE_CHOLMOD ${CHOLMOD_FOUND})
SET(LinearSLAM_SHARED_LIBS ${BUILD_SHARED_LIBS})
SET(LinearSLAM_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
SET(LinearSLAM_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(config.h.in ${PROJECT_BINARY_DIR}/LinearSLAM/config.h)
include_directories(${PROJECT_BINARY_DIR})
INSTALL(FILES ${PROJECT_BINARY_DIR}/LinearSLAM/config.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/LinearSLAM)

# Include the subdirectories
ADD_SUBDIRECTORY(src)  
