cmake_minimum_required (VERSION 2.6)

project (EV3Simulator)


set(ODE_DIRECTORIY ${ODE_DIRECTORIY} CACHE PATH "set ODE_DIRECTORIY ")

if(NOT ODE_DIRECTORIY)
	message(FATAL_ERROR "Please set ODE_DIRECTORIY.")
endif()





set(ODEDir ${ODE_DIRECTORIY})

set(ODELIB optimized;ode;debug;oded;optimized;drawstuff;debug;drawstuffd;;user32.lib;winmm;opengl32;glu32)





find_package(OpenRTM HINTS /usr/lib64/openrtm-1.1/cmake)
if(${OpenRTM_FOUND})
  MESSAGE(STATUS "OpenRTM configuration Found")
else(${OpenRTM_FOUND})
  message(STATUS "Use cmake/Modules/FindOpenRTM.cmake in the project")
  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
  find_package(OpenRTM REQUIRED)
endif(${OpenRTM_FOUND})





if (DEFINED OPENRTM_INCLUDE_DIRS)
  string(REGEX REPLACE "-I" ";"
    OPENRTM_INCLUDE_DIRS "${OPENRTM_INCLUDE_DIRS}")
  string(REGEX REPLACE " ;" ";"
    OPENRTM_INCLUDE_DIRS "${OPENRTM_INCLUDE_DIRS}")
endif (DEFINED OPENRTM_INCLUDE_DIRS)

if (DEFINED OPENRTM_LIBRARY_DIRS)
  string(REGEX REPLACE "-L" ";"
    OPENRTM_LIBRARY_DIRS "${OPENRTM_LIBRARY_DIRS}")
  string(REGEX REPLACE " ;" ";"
    OPENRTM_LIBRARY_DIRS "${OPENRTM_LIBRARY_DIRS}")
endif (DEFINED OPENRTM_LIBRARY_DIRS)

if (DEFINED OPENRTM_LIBRARIES)
  string(REGEX REPLACE "-l" ";"
    OPENRTM_LIBRARIES "${OPENRTM_LIBRARIES}")
  string(REGEX REPLACE " ;" ";"
    OPENRTM_LIBRARIES "${OPENRTM_LIBRARIES}")
endif (DEFINED OPENRTM_LIBRARIES)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
include_directories(${PROJECT_BINARY_DIR})
include_directories(${PROJECT_BINARY_DIR}/idl)
include_directories(${OPENRTM_INCLUDE_DIRS})
include_directories(${OMNIORB_INCLUDE_DIRS})


include_directories(${ODEDir}/include)

add_definitions(${OPENRTM_CFLAGS})
add_definitions(${OMNIORB_CFLAGS})



link_directories(${OPENRTM_LIBRARY_DIRS})
link_directories(${OMNIORB_LIBRARY_DIRS})
link_directories(${ODEDir}/lib/Release)
link_directories(${ODEDir}/lib/Debug)


add_executable (EV3Simulator EV3Simulator.cpp EV3SimulatorObj.cpp EV3SimulatorObj.h DrawThread.h DrawThread.cpp searchFile.cpp searchFile.h ${ODEDir}/drawstuff/src/resources.rc)

target_link_libraries(EV3Simulator ${OPENRTM_LIBRARIES} ${ODELIB} shlwapi.lib)

