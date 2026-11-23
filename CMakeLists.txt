cmake_minimum_required(VERSION 3.20)

project(LISRobotOperation LANGUAGES C CXX)

## dependencies come from the pixi environment (see pixi.toml); never from ~/.local or the system
option(BOTOP_REQUIRE_PIXI "only configure inside a pixi environment of this repo" ON)
if(BOTOP_REQUIRE_PIXI)
  if(NOT DEFINED ENV{CONDA_PREFIX} OR NOT "$ENV{CONDA_PREFIX}" MATCHES "/\\.pixi/envs/")
    message(FATAL_ERROR "[botop] configure via 'pixi run build' (or inside 'pixi shell'). "
      "Set -DBOTOP_REQUIRE_PIXI=OFF only if you know what you are doing.")
  endif()
endif()
if(DEFINED ENV{CONDA_PREFIX})
  set(ENV_PREFIX "$ENV{CONDA_PREFIX}")
  list(PREPEND CMAKE_PREFIX_PATH "${ENV_PREFIX}")
  if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${ENV_PREFIX}" CACHE PATH "install prefix" FORCE)
  endif()
endif()

## generate rai unity sources (same content as 'make -C rai unityAll'); the file is only
## rewritten when its content changes, so build/default and build/hw do not trigger rebuilds of each other
set(RAI_UNITY_MODULES Core Optim Geo Algo Search PathAlgos Gui Kin KOMO LGP Logic Control DataGen Perception ry)
foreach(m ${RAI_UNITY_MODULES})
  execute_process(
    COMMAND sh -c "find . -maxdepth 1 -name '*.h' -printf '#include \"%f\"\\n' | sort; find . -maxdepth 1 -name '*.cpp' -printf '#include \"%f\"\\n' | sort"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rai/src/${m}
    OUTPUT_VARIABLE unity_content
    COMMAND_ERROR_IS_FATAL ANY)
  file(CONFIGURE OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/rai/src/${m}/unity.cxx CONTENT "${unity_content}" @ONLY)
endforeach()

## variable default values
set(CMAKE_INSTALL_MESSAGE LAZY)
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "\$ORIGIN;\$ORIGIN/../lib;${CMAKE_INSTALL_PREFIX}/lib")

## options
option(USE_PYBIND "build python bindings" ON)
option(USE_PHYSX "link to physx" ON)
option(USE_OPENCV "link to opencv" ON)
option(USE_REALSENSE "link to realsense2 driver library" ON)
option(USE_LIBFRANKA "link to libfranka" ON)
option(USE_ROBOTIQ "Robotiq gripper driver (serial)" ON)
option(USE_OMNIBASE "Omnibase driver" ON)
option(USE_OPTITRACK "OptiTrack motion capture driver" ON)
option(USE_H5 "link to h5 library" ON)
option(USE_FREETYPE "link to freetype2 library" ON)
option(BUILD_SKILLS "build libbotop_skills (skills/: motion, control, perception helpers on top of BotOp)" ON)
option(BUILD_TESTS "build the test/examples/*/main.cpp executables (needs BUILD_SKILLS)" OFF)

## compile options
add_compile_options(
  -O3 -g -Wall -Wpedantic -fPIC
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-terminate>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-array-bounds>
  )

################################################################################

add_definitions(
  -DRAI_ROOT_PATH=\"${CMAKE_SOURCE_DIR}/rai\"
  -DRAI_X11 -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD
  -DRAI_GL
  -DRAI_GLFW -DRAI_ANN -DRAI_QHULL8 -DRAI_ASSIMP -DNO_CYCLE_DETECTION -DRAI_PNG -DRAI_PLY -DRAI_YAML
  -DRAI_GJK -DRAI_CCD -DRAI_FCL -DRAI_Lewiner -DRAI_VHACD
  -DRAI_PORTAUDIO
  -DBOOST_ERROR_CODE_HEADER_ONLY
  -DRAI_BotOp
)

include_directories(
  rai/src
  )

################################################################################

file(GLOB SRC_AlgoLewiner rai/src/Algo/Lewiner/*.cpp)
file(GLOB SRC_GeoGJK rai/src/Geo/GJK/*.c)
file(GLOB SRC_GeoPly rai/src/Geo/ply/*.c)
file(GLOB SRC_GeoCcd rai/src/Geo/ccd_rai/*.c)
file(GLOB SRC_OptLBFGS rai/src/Optim/liblbfgs/*.c)
file(GLOB SRC_OptCMA rai/src/Optim/CMA/*.c)

add_library(rai SHARED
  rai/src/Core/unity.cxx
  rai/src/Optim/unity.cxx
  rai/src/Geo/unity.cxx
  ${SRC_AlgoLewiner}
  ${SRC_GeoGJK}
  ${SRC_GeoPly}
  ${SRC_GeoCcd}
  ${SRC_OptLBFGS}
  ${SRC_OptCMA}
  rai/src/Algo/unity.cxx
  rai/src/Search/unity.cxx
  rai/src/PathAlgos/unity.cxx
  rai/src/Gui/unity.cxx
  rai/src/Kin/unity.cxx
  rai/src/KOMO/unity.cxx
  rai/src/LGP/unity.cxx
  rai/src/Logic/unity.cxx
  rai/src/Control/unity.cxx
  rai/src/DataGen/unity.cxx
  rai/src/Perception/unity.cxx
  )

################################################################################

find_package(PkgConfig REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GLUT REQUIRED)
find_package(Boost REQUIRED CONFIG)
find_package(assimp REQUIRED CONFIG)  # not via pkg-config: assimp.pc has a broken Libs.private
pkg_check_modules(DEPS REQUIRED IMPORTED_TARGET
  glfw3 glew glu gl x11 xrandr xxf86vm xi
  yaml-0.1 libpng ccd portaudio-2.0 hidapi-hidraw)
find_library(FCL_LIB NAMES fcl REQUIRED)  # not via pkg-config: fcl.pc forces -std=c++11

find_library(ANN_LIB NAMES ANN ann REQUIRED)
find_library(QHULL_LIB NAMES qhullstatic REQUIRED)
find_library(LAPACK_LIB NAMES lapack REQUIRED)
find_library(BLAS_LIB NAMES blas REQUIRED)

target_link_libraries(rai PUBLIC
  Eigen3::Eigen
  ${FCL_LIB}
  PkgConfig::DEPS
  assimp::assimp
  GLUT::GLUT
  Boost::headers
  ${ANN_LIB} ${QHULL_LIB} ${LAPACK_LIB} ${BLAS_LIB}
  pthread rt dl
  )

if(USE_FREETYPE)
  find_package(Freetype REQUIRED)
  target_compile_definitions(rai PUBLIC RAI_FREETYPE)
  target_link_libraries(rai PUBLIC Freetype::Freetype)
endif()

if(USE_H5)
  find_package(HDF5 REQUIRED COMPONENTS C CXX)
  target_compile_definitions(rai PUBLIC RAI_H5)
  target_include_directories(rai PUBLIC ${HDF5_INCLUDE_DIRS})
  target_link_libraries(rai PUBLIC ${HDF5_CXX_LIBRARIES} ${HDF5_C_LIBRARIES})
endif()

if(USE_PHYSX)
  find_path(PHYSX_INCLUDE_DIR PxPhysicsAPI.h REQUIRED)
  set(PHYSX_LIB_DIR "${PHYSX_INCLUDE_DIR}/../lib/PhysX")
  if(NOT EXISTS "${PHYSX_LIB_DIR}/libPhysX_static_64.a")
    message(FATAL_ERROR "[botop] PhysX not found in ${PHYSX_LIB_DIR} -- run 'pixi run deps'")
  endif()
  target_compile_definitions(rai PUBLIC RAI_PHYSX NDEBUG PX_DISABLE_FLUIDS CORELIB PX32 LINUX)
  target_include_directories(rai PUBLIC ${PHYSX_INCLUDE_DIR})
  foreach(px PhysXExtensions PhysX PhysXCooking PhysXCommon PhysXFoundation)
    target_link_libraries(rai PUBLIC "${PHYSX_LIB_DIR}/lib${px}_static_64.a")
  endforeach()
endif()

if(USE_OPENCV)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio objdetect)
  target_compile_definitions(rai PUBLIC RAI_OPENCV)
  target_link_libraries(rai PUBLIC ${OpenCV_LIBS})
endif()

################################################################################
## botop: robot interface (src/BotOp), simulation (src/Sim) and drivers (src/<driver>)
## a driver built without its option/SDK compiles to a stub

file(GLOB SRC_botop
  src/BotOp/*.cpp src/Sim/*.cpp
  src/Franka/*.cpp src/Robotiq/*.cpp src/Omnibase/*.cpp src/OptiTrack/*.cpp src/RealSense/*.cpp src/Audio/*.cpp)
add_library(botop SHARED ${SRC_botop})
target_include_directories(botop PUBLIC src)
target_link_libraries(botop PUBLIC rai)

if(USE_REALSENSE)
  find_package(realsense2 REQUIRED)
  target_compile_definitions(botop PUBLIC RAI_REALSENSE)
  target_link_libraries(botop PUBLIC realsense2::realsense2)
endif()

if(USE_LIBFRANKA)
  find_package(Franka REQUIRED)
  target_compile_definitions(botop PUBLIC RAI_FRANKA)
  target_link_libraries(botop PUBLIC Franka::Franka)
endif()

foreach(drv ROBOTIQ OMNIBASE OPTITRACK)
  if(USE_${drv})
    target_compile_definitions(botop PUBLIC RAI_${drv})
  endif()
endforeach()

################################################################################
## botop_skills: motion/control/perception helpers that use BotOp; #include <skills/...>

if(BUILD_SKILLS)
  file(GLOB SRC_skills skills/*/*.cpp)
  add_library(botop_skills SHARED ${SRC_skills})
  target_include_directories(botop_skills PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(botop_skills PUBLIC botop)
endif()

################################################################################

if(USE_PYBIND)
  find_package(Python 3 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_robotic
    rai/src/ry/unity.cxx
    src/BotOp/py-BotOp.cxx
  )
  target_compile_definitions(_robotic PUBLIC RAI_PYBIND)
  target_link_libraries(_robotic PRIVATE botop)
  target_link_options(_robotic PRIVATE -flto=auto)  # parallel LTO link (pybind11 adds plain -flto)

  set(PY_LIB_SITE "${Python_SITEARCH}")
  message(STATUS "[botop] python ${Python_VERSION}, installing package 'robotic' in ${PY_LIB_SITE}")

  add_custom_target(docstrings
    DEPENDS _robotic
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND env PYTHONPATH="." ${Python_EXECUTABLE} -m pybind11_stubgen --ignore-all-errors _robotic
    COMMAND mv stubs/_robotic*/__init__.pyi _robotic.pyi
    COMMAND mv stubs/_robotic*/*.pyi .)
endif()

################################################################################

get_property(_defs DIRECTORY PROPERTY COMPILE_DEFINITIONS)
message(STATUS "[botop] compiler flags: " "${_defs}")
message(STATUS "[botop] install prefix: " "${CMAKE_INSTALL_PREFIX}")

################################################################################

add_executable(kinEdit rai/bin/src_kinEdit/main.cpp)
target_link_libraries(kinEdit rai)

add_executable(meshTool rai/bin/src_meshTool/main.cpp)
target_link_libraries(meshTool rai)

add_executable(bot bin/bot/main.cpp)
target_link_libraries(bot botop)

if(BUILD_TESTS)
  if(NOT BUILD_SKILLS)
    message(FATAL_ERROR "[botop] BUILD_TESTS needs BUILD_SKILLS=ON")
  endif()
  file(GLOB tests test/examples/*/main.cpp)
  foreach(file ${tests})
    get_filename_component(path ${file} DIRECTORY)
    get_filename_component(name ${path} NAME)
    add_executable(test_${name} ${file})
    target_link_libraries(test_${name} botop_skills)
    set_target_properties(test_${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${path})
  endforeach()
endif()

################################################################################

file(GLOB SRC_botopSrc rai/src/* src/*)

if(USE_PYBIND)
  install(TARGETS _robotic DESTINATION ${PY_LIB_SITE}/robotic)
  install(DIRECTORY python/robotic/ DESTINATION ${PY_LIB_SITE}/robotic)
  install(
    DIRECTORY rai-robotModels
    DESTINATION ${PY_LIB_SITE}/robotic
    FILES_MATCHING
    PATTERN "*.g" PATTERN "*.ply"
    PATTERN "retired" EXCLUDE
    )
endif()

install(
  DIRECTORY ${SRC_botopSrc}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/rai
  FILES_MATCHING
  PATTERN "*.h" PATTERN "*.ipp"
  PATTERN "*retired*" EXCLUDE
  )

if(BUILD_SKILLS)
  install(DIRECTORY skills DESTINATION ${CMAKE_INSTALL_PREFIX}/include/rai FILES_MATCHING PATTERN "*.h")
  install(TARGETS botop_skills DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()

install(TARGETS rai botop DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(TARGETS kinEdit meshTool bot DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
