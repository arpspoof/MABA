cmake_minimum_required(VERSION 3.15)

set (CMAKE_CXX_STANDARD 11)

project(pyPhysX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# Example install directory for linux:
# ---/PhysX/physx/install/linux
set(PHYSX_INSTALL_DIR "" CACHE PATH "PhysX install directory, like /../PhysX/physx/install/linux")
if (PHYSX_INSTALL_DIR STREQUAL "")
    message(FATAL_ERROR "No PhysX install directory specified")
endif()

# Example build directory for linux:
# ---/PhysX/physx/bin/linux.clang/release
set(PHYSX_BUILD_DIR "" CACHE PATH "PhysX build directory, like /../PhysX/physx/bin/linux.clang/release")
if (PHYSX_BUILD_DIR STREQUAL "")
    message(FATAL_ERROR "No PhysX build directory specified")
endif()

set(EIGEN_DIR "" CACHE PATH "Source directory to eigen")
if (EIGEN_DIR STREQUAL "")
    message(FATAL_ERROR "No Eigen directory specified")
endif()

set(SRC_PATH "${PROJECT_SOURCE_DIR}/src")
set(RESOURCES_PATH "${PROJECT_SOURCE_DIR}/resources")

set(COMMON_PATH "${SRC_PATH}/common")
set(ACTORS_PATH "${SRC_PATH}/actors")
set(ARTICULATION_PATH "${SRC_PATH}/articulation")
set(CORE_PATH "${SRC_PATH}/core")

set(RENDERER_PATH "${SRC_PATH}/renderer")
set(RENDERER_GLUT_PATH "${RENDERER_PATH}/glut")
set(RENDERER_UNITY_PATH "${RENDERER_PATH}/unity")

include_directories(SYSTEM "${PHYSX_INSTALL_DIR}/PhysX/include")
include_directories(SYSTEM "${PHYSX_INSTALL_DIR}/PhysX/source/foundation/include")
include_directories(SYSTEM "${PHYSX_INSTALL_DIR}/PhysX/source/fastxml/include")
include_directories(SYSTEM "${PHYSX_INSTALL_DIR}/PxShared/include")
include_directories(SYSTEM "${EIGEN_DIR}")

include_directories("${COMMON_PATH}/include")
include_directories("${ACTORS_PATH}/include")
include_directories("${ARTICULATION_PATH}/include")
include_directories("${CORE_PATH}/include")
include_directories("${RENDERER_GLUT_PATH}/include")
include_directories("${RENDERER_UNITY_PATH}/include")

link_directories("${PHYSX_BUILD_DIR}")

if (${CMAKE_BUILD_TYPE} MATCHES "Debug")
    add_compile_definitions(_DEBUG)
else()
    add_compile_definitions(NDEBUG)
endif()

set(PX_LIBS 
    libPhysX_static_64.a 
    libPhysXCommon_static_64.a
    libPhysXCharacterKinematic_static_64.a
    libPhysXFoundation_static_64.a
    libPhysXPvdSDK_static_64.a
    libPhysXCooking_static_64.a
    libPhysXExtensions_static_64.a
    libPhysXVehicle_static_64.a
)

set(SYS_LIBS
    dl
    pthread
    GL
    GLU
    glut
    X11
    rt
#    cuda
)

function(createLibrary varName varSrcPath)
    file(GLOB_RECURSE TMP_SRC
        "${varSrcPath}/include/*.h"
        "${varSrcPath}/*.cpp"
    )
    add_library(${varName} ${TMP_SRC})
    target_link_libraries(${varName} ${ARGN})
    target_compile_options(${varName} PRIVATE -Wall -Wextra -pedantic -Werror -fPIC)
endfunction(createLibrary)

function(createExecutable varName varSrcPath)
    file(GLOB_RECURSE TMP_SRC
        "${varSrcPath}/include/*.h"
        "${varSrcPath}/*.cpp"
    )
    add_executable(${varName} ${TMP_SRC})
    target_link_libraries(${varName} ${ARGN})
    target_compile_options(${varName} PRIVATE -Wall -Wextra -pedantic -Werror -fPIC)
endfunction(createExecutable)


set(ENABLE_UNITY_KINEMATICS OFF CACHE BOOL "Choose whether to enable UnityKinematics renderer")

if(${ENABLE_UNITY_KINEMATICS})

set(RPCLIB_INCLUDE_DIR "/usr/local/include" CACHE PATH "rpclib include directory")
set(RPCLIB_LIBS_DIR "/usr/local/lib" CACHE PATH "rpclib library directory")
set(UNITY_KINEMATICS_INCLUDE "" CACHE PATH "UnityKinematics include directory")
set(UNITY_KINEMATICS_LIB "" CACHE PATH "UnityKinematics lib directory")

if (UNITY_KINEMATICS_INCLUDE STREQUAL "")
    message(FATAL_ERROR "No UnityKinematics include directory specified")
endif()
if (UNITY_KINEMATICS_LIB STREQUAL "")
    message(FATAL_ERROR "No UnityKinematics lib directory specified")
endif()
if (RPCLIB_INCLUDE_DIR STREQUAL "")
    message(FATAL_ERROR "No rpclib include directory specified")
endif()
if (RPCLIB_LIBS_DIR STREQUAL "")
    message(FATAL_ERROR "No rpclib library directory specified")
endif()

include_directories(SYSTEM "${UNITY_KINEMATICS_INCLUDE}")
include_directories(SYSTEM "${RPCLIB_INCLUDE_DIR}")
link_directories("${UNITY_KINEMATICS_LIB}")
link_directories("${RPCLIB_LIBS_DIR}")

add_compile_definitions(ENABLE_UNITY_KINEMATICS)

endif()

createLibrary(common ${COMMON_PATH})
createLibrary(actors ${ACTORS_PATH})
createLibrary(articulation ${ARTICULATION_PATH} common)
createLibrary(core ${CORE_PATH} articulation)
createLibrary(renderer_glut ${RENDERER_GLUT_PATH})
if(${ENABLE_UNITY_KINEMATICS})
createLibrary(renderer_unity ${RENDERER_UNITY_PATH} articulation rpc client)
endif()

set(ALL_MODULE_NO_RENDERER common actors articulation core)
set(ALL_MODULE ${ALL_MODULE_NO_RENDERER} renderer_glut)
if(${ENABLE_UNITY_KINEMATICS})
set(ALL_MODULE ${ALL_MODULE} renderer_unity)
endif()
set(ALL_EXT_LIBS ${PX_LIBS} ${PX_LIBS} ${SYS_LIBS})
if(${ENABLE_UNITY_KINEMATICS})
set(ALL_EXT_LIBS ${ALL_EXT_LIBS} client)
endif()


add_compile_definitions(ENABLE_SPD_ABA)


set(EXAMPLE_PATH "${SRC_PATH}/examples")

set(EXAMPLE_POSE_TRACKING_PATH "${EXAMPLE_PATH}/poseTracking")
createExecutable(poseTracking ${EXAMPLE_POSE_TRACKING_PATH} ${ALL_MODULE} ${ALL_EXT_LIBS})

set(EXAMPLE_SNAKE_PATH "${EXAMPLE_PATH}/snake")
createExecutable(snake ${EXAMPLE_SNAKE_PATH} ${ALL_MODULE} ${ALL_EXT_LIBS})



set(BUILD_API OFF CACHE BOOL "Choose whether python binding API should be built")
set(BUILD_API_DIR "${CMAKE_BINARY_DIR}/pyPhysX" CACHE PATH "Directory to place generated python API files")

if (${BUILD_API})

find_package(PythonLibs 3.7.4 REQUIRED)
find_package(PythonInterp ${PYTHONLIBS_VERSION_STRING} REQUIRED)

include_directories(${PYTHON_INCLUDE_PATH})

find_package(SWIG 4.0.1 REQUIRED)
include(${SWIG_USE_FILE})

set_source_files_properties("${SRC_PATH}/api.i" PROPERTIES CPLUSPLUS ON)

swig_add_library(pyPhysX 
    LANGUAGE python 
    OUTPUT_DIR "${BUILD_API_DIR}"
    OUTFILE_DIR "${CMAKE_BINARY_DIR}/pybuild"
    SOURCES "${SRC_PATH}/api.i"
)

swig_link_libraries(pyPhysX rpc ${ALL_MODULE} ${ALL_EXT_LIBS})

add_custom_target(copy_so ALL
    COMMAND cp -f "${CMAKE_BINARY_DIR}/_pyPhysX.so" "${BUILD_API_DIR}"
)

add_dependencies(copy_so pyPhysX)

endif()

add_custom_target(copy_resources ALL
    COMMAND cp -rf "${RESOURCES_PATH}" "${CMAKE_BINARY_DIR}"
    DEPENDS "${RESOURCES_PATH}"
)
