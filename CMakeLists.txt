cmake_minimum_required(VERSION 3.12)
project(MyAwesomeComponents VERSION 1.0)

# SOFA packages containing our dependencies
find_package(SofaFramework REQUIRED) # Needed to use SofaCore, SofaHelper and SofaDefaultType


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

# nested-namespace-definition requires compiler flag '/std:c++17'
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    )

# Create package Config, Version & Target files.
# Deploy the headers, resources, scenes & examples.
sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    EXAMPLE_INSTALL_DIR "examples"
    RELOCATABLE "plugins"
    )
