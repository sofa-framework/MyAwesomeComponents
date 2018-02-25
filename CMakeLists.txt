cmake_minimum_required(VERSION 3.1)
project(MyAwesomeComponents)

# SOFA packages containing our dependencies
find_package(SofaFramework REQUIRED) # Needed to include SofaCore, SofaHelper and SofaDefaultType modules
find_package(SofaGeneral REQUIRED) # Needed to include SofaBaseTopology modules

# Config files (tell SOFA this is a plugin)
set(CONFIG_FILES
    config.h
    initMyAwesomeComponents.cpp
)

# Our nice new SOFA Components
set(HEADER_FILES
    src/ExplicitBDFSolver.h
    src/FanForceField.h
    src/FanForceField.inl
)
set(SOURCE_FILES
    src/ExplicitBDFSolver.cpp
    src/FanForceField.cpp
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
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_MYAWESOMECOMPONENTS")
