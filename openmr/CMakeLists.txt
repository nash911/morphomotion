cmake_minimum_required (VERSION 2.6.0)
project(openMR)
find_package(OpenRAVE 0.8 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT COMPONENTS iostreams python thread)

IF (NOT EXISTS "${PROJECT_SOURCE_DIR}/lib")
    execute_process( COMMAND ${CMAKE_COMMAND} -E make_directory  "${PROJECT_SOURCE_DIR}/lib")
ENDIF(NOT EXISTS "${PROJECT_SOURCE_DIR}/lib")

# Set output paths:
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

# Add subdirectories
add_subdirectory(plugin)

