include(${SOFA_CMAKE_DIR}/preProject.cmake)

set(SOURCE_FILES

    Main.cpp

    )
    

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

include(${SOFA_CMAKE_DIR}/postProject.cmake)

