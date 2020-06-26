cmake_minimum_required(VERSION 3.1)
project(MyAwesomeComponents VERSION 1.0)

# SOFA packages containing our dependencies
find_package(SofaFramework REQUIRED) # Needed to use SofaCore, SofaHelper and SofaDefaultType
find_package(SofaGeneral REQUIRED) # Needed to use SofaBaseTopology

# Config files (tell SOFA this is a plugin)
set(CONFIG_FILES
    src/MyAwesomeComponents/config.h
    src/MyAwesomeComponents/initMyAwesomeComponents.cpp
    )
# Our nice new SOFA Components
set(HEADER_FILES
    src/MyAwesomeComponents/ExplicitBDFSolver.h
    src/MyAwesomeComponents/FanForceField.h
    src/MyAwesomeComponents/FanForceField.inl
    )
set(SOURCE_FILES
    src/MyAwesomeComponents/ExplicitBDFSolver.cpp
    src/MyAwesomeComponents/FanForceField.cpp
    )

# SOFA plugin are basically libraries
add_library(${PROJECT_NAME} SHARED ${CONFIG_FILES} ${HEADER_FILES} ${SOURCE_FILES})

# Dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
    SofaCore
    SofaHelper
    SofaDefaultType
    SofaBaseTopology
    )

# Needed to build on Windows (dllimport/dllexport mechanism)
target_compile_definitions(${PROJECT_NAME} PRIVATE "-DSOFA_BUILD_MYAWESOMECOMPONENTS")

# Set include directories to be able to resolve "#include <MyAwesomeComponents/XXX.h>"
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>")

# nested-namespace-definition requires compiler flag '/std:c++17'
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

# Create package Config, Version & Target files.
# Deploy the headers, resources, scenes & examples.
sofa_generate_package(
    NAME ${PROJECT_NAME}
    TARGETS ${PROJECT_NAME}
    VERSION ${PROJECT_VERSION}
    INCLUDE_SOURCE_DIR src/MyAwesomeComponents
    INCLUDE_INSTALL_DIR MyAwesomeComponents
    )

