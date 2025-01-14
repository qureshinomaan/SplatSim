# pybind11Config.cmake
# --------------------
#
# PYBIND11 cmake module.
# This module sets the following variables in your project::
#
#   pybind11_FOUND - true if pybind11 and all required components found on the system
#   pybind11_VERSION - pybind11 version in format Major.Minor.Release
#   pybind11_INCLUDE_DIRS - Directories where pybind11 and python headers are located.
#   pybind11_INCLUDE_DIR - Directory where pybind11 headers are located.
#   pybind11_DEFINITIONS - Definitions necessary to use pybind11, namely USING_pybind11.
#   pybind11_LIBRARIES - compile flags and python libraries (as needed) to link against.
#   pybind11_LIBRARY - empty.
#   CMAKE_MODULE_PATH - appends location of accompanying FindPythonLibsNew.cmake and
#                       pybind11Tools.cmake modules.
#
#
# Available components: None
#
#
# Exported targets::
#
# If pybind11 is found, this module defines the following :prop_tgt:`IMPORTED`
# interface library targets::
#
#   pybind11::module - for extension modules
#   pybind11::embed - for embedding the Python interpreter
#
# Python headers, libraries (as needed by platform), and the C++ standard
# are attached to the target. Set PythonLibsNew variables to influence
# python detection and PYBIND11_CPP_STANDARD (-std=c++11 or -std=c++14) to
# influence standard setting. ::
#
#   find_package(pybind11 CONFIG REQUIRED)
#   message(STATUS "Found pybind11 v${pybind11_VERSION}: ${pybind11_INCLUDE_DIRS}")
#
#   # Create an extension module
#   add_library(mylib MODULE main.cpp)
#   target_link_libraries(mylib pybind11::module)
#
#   # Or embed the Python interpreter into an executable
#   add_executable(myexe main.cpp)
#   target_link_libraries(myexe pybind11::embed)
#
# Suggested usage::
#
# find_package with version info is not recommended except for release versions. ::
#
#   find_package(pybind11 CONFIG)
#   find_package(pybind11 2.0 EXACT CONFIG REQUIRED)
#
#
# The following variables can be set to guide the search for this package::
#
#   pybind11_DIR - CMake variable, set to directory containing this Config file
#   CMAKE_PREFIX_PATH - CMake variable, set to root directory of this package
#   PATH - environment variable, set to bin directory of this package
#   CMAKE_DISABLE_FIND_PACKAGE_pybind11 - CMake variable, disables
#     find_package(pybind11) when not REQUIRED, perhaps to force internal build

# @PACKAGE_INIT@

set(PN pybind11)

if(NOT DEFINED PYBIND11_PYTHON_VERSION)
	message("-- PYBIND11_PYTHON_VERSION was not defined, assuming python3")
	execute_process(COMMAND python3 -c "import pybind11; print(pybind11.get_include(True))" OUTPUT_VARIABLE pybind11_INCLUDE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

if("${PYBIND11_PYTHON_VERSION}" LESS "3")
	message("-- Using python2")
	execute_process(COMMAND python -c "import pybind11; print(pybind11.get_include(True))" OUTPUT_VARIABLE pybind11_INCLUDE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

set(${PN}_LIBRARY "")
set(${PN}_DEFINITIONS USING_${PN})

# make detectable the FindPythonLibsNew.cmake module
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(pybind11_INCLUDE_DIR "${pybind11_INCLUDE_DIR}" CACHE INTERNAL "pybind include directory")

include(pybind11Tools)
