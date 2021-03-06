include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(SOURCE_FILES
   AffinePatch_test.cpp
   AffineDeformationMapping_test.cpp
   HexahedraMaterial_test.cpp
   Material_test.cpp
   PointDeformationMapping_test.cpp
   Patch_test.cpp
   StrainMappings_test.cpp
   RigidDeformationMapping_test.cpp
        )

if(APPLE)
#    set(RC_FILES "runSOFA.icns")
endif()

include_directories("${SOFA_APPLICATIONS_PROJECTS_DIR}/Standard_test")

if(SOFA-PLUGIN_SOFAPYTHON)
    AddCompilerDefinitions("FLEXIBLE_TEST_PYTHON_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/scenes/python\"")
    list(APPEND SOURCE_FILES "python_test_list.cpp")
    AddLinkerDependencies(SofaPython)
    include(./scenes/python/python.cmake)
endif()

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${RC_FILES} ${PYTHON_FILES})
AddCompilerDefinitions("FLEXIBLE_TEST_SCENES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/scenes\"")
AddLinkerDependencies(Flexible SofaTest)

#Needed by python test
AddCompilerDefinitions("SOFA_SRC_DIR=${SOFA_SRC_DIR}")

include(${SOFA_CMAKE_DIR}/postProject.cmake)
