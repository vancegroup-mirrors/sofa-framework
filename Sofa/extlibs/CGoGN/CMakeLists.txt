include(${SOFA_CMAKE_DIR}/preProject.cmake)
# cmake_minimum_required(VERSION 2.8)

# project(CGoGN)

#SET ( CMAKE_VERBOSE_MAKEFILE 1 )

SET(CGoGN_ROOT_DIR ${SOFA_EXTLIBS_DIR}/CGoGN )

SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} ${CGoGN_ROOT_DIR}/cmake_modules/" )

# for shared or not shared
# SET ( BUILD_SHARED_LIBS OFF CACHE BOOL "if used all library are build as shared type (.so/.dylib/.dll)" )

# SET ( WITH_ASSIMP ON CACHE BOOL "build CGoGN with Assimp" )
# SET ( WITH_ZINRI ON CACHE BOOL "build CGoGN with Zinri lib" )
# SET ( WITH_QT ON CACHE BOOL "build CGoGN with Qt lib" )
# SET ( DESIRED_QT_VERSION "4" CACHE STRING "4: QT4/ 5 not yet implemented" )
# SET ( DONOT_COMPILE_SANDBOX OFF CACHE BOOL "do not compile the SandBox directory" )


# for CGoGN MR
# SET ( FORCE_MR "0" CACHE STRING "0: normal mode / 1 multires mode" )

# #create one big lib
# SET ( ONELIB OFF CACHE BOOL "build CGoGN in one lib" )

# SET ( WITH_GLEWMX OFF CACHE BOOL "use multi-contex GLEW" )

SET ( BUILD_SHARED_LIBS OFF )
SET ( WITH_ASSIMP OFF)
SET ( WITH_QT OFF)
SET ( DESIRED_QT_VERSION "4")
SET ( DONOT_COMPILE_SANDBOX ON)
SET ( FORCE_MR "0" )
SET ( ONELIB ON)
SET ( WITH_GLEWMX OFF)

IF(WIN32)
	SET ( CMAKE_PREFIX_PATH ${CGoGN_ROOT_DIR}/windows_dependencies CACHE STRING "path to dependencies" )
ENDIF(WIN32)

find_package(OpenGL REQUIRED)

if(NOT (SOFA-EXTERNAL_BOOST_PATH STREQUAL ""))
	message(FATAL_ERROR "CGoGN needs the full boost version. You need to set SOFA-EXTERNAL_BOOST_PATH to \"\"")
endif()
find_package(Boost COMPONENTS regex thread system REQUIRED)

find_package(ZLIB REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(GLEW REQUIRED)



IF (DEFINED ASSERTON)
	add_definitions(-DCGOGN_ASSERT_BOOL=${ASSERTON})
ELSE (DEFINED ASSERTON)
	add_definitions(-DCGOGN_ASSERT_BOOL=false)
ENDIF (DEFINED ASSERTON)

IF (FORCE_MR EQUAL 0)
	file(WRITE ${CGoGN_ROOT_DIR}/include/cgogn_mr.h "0" )
ENDIF (FORCE_MR EQUAL 0)

IF (FORCE_MR EQUAL 1)
	add_definitions(-DCGoGN_FORCE_MR=1)
	file(WRITE ${CGoGN_ROOT_DIR}/include/cgogn_mr.h "1" )
ENDIF (FORCE_MR EQUAL 1)

IF (ONELIB)
	file(WRITE ${CGoGN_ROOT_DIR}/include/cgogn_onelib.h "1" )
ELSE (ONELIB)
	file(WRITE ${CGoGN_ROOT_DIR}/include/cgogn_onelib.h "0" )
ENDIF (ONELIB)

IF (WITH_GLEWMX)
	file(WRITE ${CGoGN_ROOT_DIR}/include/cgogn_gmx.h "1" )
ELSE (WITH_GLEWMX)
	file(WRITE ${CGoGN_ROOT_DIR}/include/cgogn_gmx.h "0" )
ENDIF (WITH_GLEWMX)


add_definitions(-DSHADERPATH="${CGoGN_ROOT_DIR}/lib/Shaders/")

# define includes of external libs 
SET (CGoGN_EXT_INCLUDES
	${CGoGN_ROOT_DIR}/ThirdParty/include
	${OPENGL_INCLUDE_DIR}
	${GLEW_INCLUDE_DIRS}
	${ZLIB_INCLUDE_DIRS}
	${LIBXML2_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${SOFA-EXTERNAL_EIGEN_PATH}
)

include_directories(
	${CGoGN_ROOT_DIR}/ThirdParty/include
	${OPENGL_INCLUDE_DIR}
	${GLEW_INCLUDE_DIRS}
	${ZLIB_INCLUDE_DIRS}
	${LIBXML2_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${SOFA-EXTERNAL_EIGEN_PATH}
)



# define libs for external libs
SET (CGoGN_EXT_LIBS
    nl
	${OPENGL_LIBRARY}
	${GLEW_LIBRARIES}
	${ZLIB_LIBRARIES}
	${LIBXML2_LIBRARIES}
	${Boost_SYSTEM_LIBRARY}
	${Boost_REGEX_LIBRARY}
	${Boost_THREAD_LIBRARY}
)

IF(WIN32)
	SET (CGoGN_EXT_INCLUDES ${CGoGN_EXT_INCLUDES} ${CGoGN_ROOT_DIR}/windows_dependencies/include/suitesparse)
	SET (CGoGN_EXT_LIBS ${CGoGN_EXT_LIBS} 
		${CGoGN_ROOT_DIR}/windows_dependencies/lib/suitesparse.lib
		${CGoGN_ROOT_DIR}/windows_dependencies/lib/metis.lib
		${CGoGN_ROOT_DIR}/windows_dependencies/lib/liblapack.lib
		${CGoGN_ROOT_DIR}/windows_dependencies/lib/libblas.lib		)
ELSE(WIN32)
	find_package(SuiteSparse REQUIRED)
	SET (CGoGN_EXT_INCLUDES ${CGoGN_EXT_INCLUDES} ${SUITESPARSE_INCLUDE_DIRS})
	SET (CGoGN_EXT_LIBS ${CGoGN_EXT_LIBS} ${SUITESPARSE_LIBRARIES})
ENDIF(WIN32)


#optionnal libs
# IF (WITH_ASSIMP)
# 	add_definitions(-DWITH_ASSIMP)	
# 	SET (CGoGN_EXT_LIBS ${CGoGN_EXT_LIBS} assimp)
# 	INSTALL (DIRECTORY ThirdParty/Assimp/include  DESTINATION .)
# ENDIF (WITH_ASSIMP)

IF (WITH_ZINRI)
	add_definitions(-DWITH_ZINRI)
	SET (CGoGN_EXT_LIBS ${CGoGN_EXT_LIBS} Zinri)
	# INSTALL (DIRECTORY ThirdParty/Zinri/ DESTINATION include FILES_MATCHING PATTERN "*.h")
ENDIF (WITH_ZINRI)

IF (WITH_QT)
	add_definitions(-DWITH_QT)
	find_package(Qt REQUIRED)
	SET(QT_USE_QTCORE TRUE)
	SET(QT_USE_QTOPENGL TRUE)
	SET(QT_USE_QTSVG TRUE)
	SET(QT_USE_QTXML TRUE )
	INCLUDE(${QT_USE_FILE})
	ADD_DEFINITIONS(${QT_DEFINITIONS})
	find_package(QGLViewer REQUIRED)
	SET (CGoGN_EXT_INCLUDES ${CGoGN_EXT_INCLUDES} ${QT_INCLUDE_DIR} ${QGLVIEWER_INCLUDE_DIR} )
	SET (CGoGN_EXT_LIBS ${CGoGN_EXT_LIBS} ${QT_LIBRARIES} ${QGLVIEWER_LIBRARIES} )
ENDIF (WITH_QT)

IF (WITH_GLEWMX)
	ADD_DEFINITIONS(-DGLEW_MX)
ENDIF (WITH_GLEWMX)


IF(APPLE)
# attention a changer pour chercher la bonne version automatiquement
	SET(CMAKE_OSX_SYSROOT "/Developer/SDKs/MacOSX10.6.sdk" CACHE STRING "developer SDK")
	SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-undefined -Wl,dynamic_lookup")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMAC_OSX")
	SET(CMAKE_OSX_ARCHITECTURES x86_64)
	SET (CGoGN_EXT_LIBS ${CGoGN_EXT_LIBS} "-framework Foundation -framework cocoa" )
ENDIF(APPLE)

add_subdirectory(ThirdParty)


IF(WIN32)
	add_subdirectory(Release)
	# add_subdirectory(${CGoGN_ROOT_DIR}/Apps Apps)
	set(CMAKE_CONFIGURATION_TYPES Release Debug)
	set(CMAKE_CONFIGURATION_TYPES "${CMAKE_CONFIGURATION_TYPES}" CACHE STRING "Only Release or Debug" FORCE)
#	set(CMAKE_CONFIGURATION_TYPES "Release Debug" CACHE STRING "Only Release or Debug" FORCE)
ELSE(WIN32)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wclobbered -Wempty-body -Wignored-qualifiers -Wmissing-field-initializers -Wsign-compare -Wtype-limits -Wuninitialized -fPIC")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Woverride-init -fPIC")
	if(CMAKE_BUILD_TYPE STREQUAL Release)
		add_subdirectory(Release)
	else()
		add_subdirectory(Debug)
	endif()
ENDIF(WIN32)


include(${SOFA_CMAKE_DIR}/postProject.cmake)