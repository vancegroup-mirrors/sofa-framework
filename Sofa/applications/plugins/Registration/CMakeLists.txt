include(${SOFA_CMAKE_DIR}/preProject.cmake)


set(HEADER_FILES
    initRegistration.h
    RegistrationContact.h
    RegistrationContact.inl
    RegistrationContactForceField.h
    RegistrationContactForceField.inl
    ClosestPointRegistrationForceField.h
    ClosestPointRegistrationForceField.inl
    RegistrationExporter.h
    GroupwiseRegistrationEngine.h)

set(SOURCE_FILES
    initRegistration.cpp
    RegistrationContact.cpp
    RegistrationContactForceField.cpp
    ClosestPointRegistrationForceField.cpp
    RegistrationExporter.cpp
    GroupwiseRegistrationEngine.cpp)

set(README_FILES registration.txt)

if(SOFA-PLUGIN_IMAGE)
    list(APPEND HEADER_FILES "IntensityProfileRegistrationForceField.h")
    list(APPEND HEADER_FILES "IntensityProfileRegistrationForceField.inl")
    list(APPEND SOURCE_FILES "IntensityProfileRegistrationForceField.cpp")
    AddLinkerDependencies(image)
endif()

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})
AddCompilerDefinitions("SOFA_BUILD_REGISTRATION")
AddLinkerDependencies(SofaGuiQt)


include(${SOFA_CMAKE_DIR}/postProject.cmake)
