cmake_minimum_required(VERSION 2.8)

project("SofaComponentAdvanced")

include(${SOFA_CMAKE_DIR}/pre.cmake)

set(HEADER_FILES

    initComponentAdvanced.h
    
    )
    
set(SOURCE_FILES

    initComponentAdvanced.cpp

    )
    
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})

set(COMPILER_DEFINES "SOFA_BUILD_COMPONENT_ADVANCED")
set(LINKER_DEPENDENCIES SofaEulerianFluid SofaSphFluid SofaVolumetricData SofaNonUniformFem SofaEigen2Solver)

include(${SOFA_CMAKE_DIR}/post.cmake)