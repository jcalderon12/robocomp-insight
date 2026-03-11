cmake_minimum_required(VERSION 3.10)
project(episodic_memory_python
        VERSION 1.0.0
        DESCRIPTION "Episodic Memory API Python wrapper"
        LANGUAGES CXX)

# Check ROBOCOMP environment variable
if("$ENV{ROBOCOMP}" STREQUAL "")
    message(WARNING "ROBOCOMP variable not set. Using default: /home/robocomp/robocomp")
    set(ENV{ROBOCOMP} "/home/robocomp/robocomp/")
endif()

# Verify RoboComp installation
if(NOT EXISTS "$ENV{ROBOCOMP}/cmake")
    message(FATAL_ERROR "Could not find RoboComp in $ENV{ROBOCOMP}! Please set ROBOCOMP environment variable.")
endif()

# Include RoboComp CMake configuration
include($ENV{ROBOCOMP}/cmake/robocomp.cmake)

# Find required packages
find_package(PythonLibs REQUIRED)
find_package(Python3 REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(Qt6 COMPONENTS Core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Create Python module
add_library(episodic_memory_api MODULE)

target_sources(episodic_memory_api
        PRIVATE
        python_binding.cpp
        ../src/dsr_episodic_api.cpp
)

target_link_libraries(episodic_memory_api
        PRIVATE
        Qt6::Core
        Eigen3::Eigen
        dsr_core
        dsr_api
        ${PYTHON_LIBRARIES}
)

target_include_directories(episodic_memory_api
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src/>
        $<BUILD_INTERFACE:$ENV{ROBOCOMP}/cortex/core/include/>
        $<BUILD_INTERFACE:$ENV{ROBOCOMP}/cortex/api/include/>
        PRIVATE
        ${PYTHON_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/
)

set_target_properties(episodic_memory_api PROPERTIES
        CMAKE_CXX_STANDARD 23
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS ON
        PREFIX ""                    # No lib prefix
        SUFFIX ".so"                 # Explicit .so extension
)

target_compile_options(episodic_memory_api PRIVATE -O3 -std=c++23 -fmax-errors=5)

# Installation
set(EPISODIC_INSTALL_PYTHONDIR "${Python3_SITEARCH}")
message(STATUS "Python installation directory: ${EPISODIC_INSTALL_PYTHONDIR}")

install(TARGETS episodic_memory_api 
        DESTINATION ${EPISODIC_INSTALL_PYTHONDIR} 
        COMPONENT module)

# Install __init__.py if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/__init__.py")
    install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/__init__.py"
            DESTINATION ${EPISODIC_INSTALL_PYTHONDIR}
            COMPONENT module)
endif()

