cmake_minimum_required(VERSION 3.15)

project(alpine CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
set(CMAKE_SUPPRESS_REGENERATION true)

add_compile_options(/WX)

option(ALPINE_BUILD_APPS "Builds alpine sample applications" OFF)
option(ALPINE_ENABLE_EMBREE "Enables Embree as an accelerator backend option" OFF)
option(ALPINE_ENABLE_OIDN "Enables a denoising option via OIDN" OFF)
option(ALPINE_ENABLE_ISPC "Enables ISPC-based SIMD code generation" OFF)

if (ALPINE_BUILD_APPS)
  # set directory for output executables
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

  # set directory for output libraries
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
endif()

set(SOURCES
  src/alpine.cpp
  src/alpine_config.h
  src/animation.cpp
  src/animation.h
  src/camera.cpp
  src/camera.h
  src/sampler.cpp
  src/sampler.h
  src/image.cpp
  src/image.h
  src/intersection.h
  src/ray.h
  src/texture.h
)

set(ACCELERATORS
  src/accelerators/accelerator.h
  src/accelerators/bvh.cpp
  src/accelerators/bvh.h
  src/accelerators/bvh4.h
  src/accelerators/bvh_common.cpp
  src/accelerators/bvh_common.h
  src/accelerators/embree.h
)

if (ALPINE_ENABLE_EMBREE)
list(APPEND ACCELERATORS src/accelerators/embree.cpp)
else()
list(APPEND ACCELERATORS src/accelerators/embree_dummy.cpp)
endif()

if (ALPINE_ENABLE_ISPC)
list(APPEND ACCELERATORS src/accelerators/bvh4.cpp)
else()
list(APPEND ACCELERATORS src/accelerators/bvh4_dummy.cpp)
endif()
set(DENOISERS src/denoisers/denoiser.h)

if (ALPINE_ENABLE_OIDN)
list(APPEND DENOISERS src/denoisers/oidn.cpp)
else()
list(APPEND DENOISERS src/denoisers/dummy.cpp)
endif()

set(LIGHT_SAMPLERS
  src/light_samplers/light_sampler.h
  src/light_samplers/bvh_light_sampler.cpp
  src/light_samplers/bvh_light_sampler.h
  src/light_samplers/power_light_sampler.cpp
  src/light_samplers/power_light_sampler.h
  src/light_samplers/uniform_light_sampler.cpp
  src/light_samplers/uniform_light_sampler.h
)

set(LIGHTS
  src/lights/disk_light.cpp
  src/lights/disk_light.h
  src/lights/light.h
  src/lights/point_light.cpp
  src/lights/point_light.h
)

set(LOADERS
  src/loaders/file_loader.h
  src/loaders/gltf_loader.cpp
  src/loaders/obj_loader.cpp
)

set(MATERIALS
  src/materials/matte.cpp
  src/materials/matte.h
  src/materials/material.h
  src/materials/metal.cpp
  src/materials/metal.h
)

set(MATH
  src/math/matrix.h
  src/math/vector.h
)

set(SCENE
  src/scenes/debug_scene.cpp
  src/scenes/debug_scene.h
  src/scenes/scene.h
)

set(SHAPES
  src/shapes/mesh.cpp
  src/shapes/mesh.h
  src/shapes/shape.h
  src/shapes/sphere.cpp
  src/shapes/sphere.h
)

set(UTILS
  src/utils/alias_table.cpp
  src/utils/alias_table.h
  src/utils/bounding_box.h
  src/utils/bvh_util.h
  src/utils/util.cpp
  src/utils/util.h
)

# ispc compile setting
if (ALPINE_ENABLE_ISPC)
set(ISPC_EXE ${CMAKE_CURRENT_SOURCE_DIR}/ext/ispc/bin/ispc.exe)

set(ISPC_FILE_NAMES
  bounding_box
  triangle)

set(ISPC_SRCS "")
set(ISPC_OBJS "")

foreach(ISPC_FILE_NAME ${ISPC_FILE_NAMES})
  set(ISPC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/ispc/${ISPC_FILE_NAME}.ispc)
  set(ISPC_OBJ ${CMAKE_CURRENT_BINARY_DIR}/ispc/${ISPC_FILE_NAME}.obj)
  set(ISPC_H ${CMAKE_CURRENT_BINARY_DIR}/ispc/${ISPC_FILE_NAME}.h)

  add_custom_command(
      OUTPUT ${ISPC_OBJ} ${ISPC_H}
      COMMAND ${ISPC_EXE} ${ISPC_SRC} -o ${ISPC_OBJ} -h ${ISPC_H}
      DEPENDS ${ISPC_SRC}
      COMMENT "Compiling ISPC file ${ISPC_SRC}"
  )
  
  list(APPEND ISPC_SRCS ${ISPC_SRC})
  list(APPEND ISPC_OBJS ${ISPC_OBJ})
endforeach()

endif ()

source_group("" FILES ${SOURCES})
source_group("accelerators" FILES ${ACCELERATORS})
source_group("denoisers" FILES ${DENOISERS})
source_group("light_samplers" FILES ${LIGHT_SAMPLERS})
source_group("lights" FILES ${LIGHTS})
source_group("loaders" FILES ${LOADERS})
source_group("materials" FILES ${MATERIALS})
source_group("math" FILES ${MATH})
source_group("scene" FILES ${SCENE})
source_group("shapes" FILES ${SHAPES})
source_group("utils" FILES ${UTILS})
source_group("ispc" FILES ${ISPC_SRCS})

add_library(alpine STATIC
  ${SOURCES}
  ${ACCELERATORS}
  ${DENOISERS}
  ${LIGHT_SAMPLERS}
  ${LIGHTS}
  ${LOADERS}
  ${MATERIALS}
  ${MATH}
  ${SCENE}
  ${SHAPES}
  ${UTILS}
  ${ISPC_OBJS}
  ${ISPC_SRCS}
)

target_include_directories(alpine
  PUBLIC
    include
    ext
  PRIVATE
    src
    ${CMAKE_CURRENT_BINARY_DIR}
)

# embree libraries
if (ALPINE_ENABLE_EMBREE)
target_include_directories(alpine PRIVATE ext/embree/include)
target_link_directories(alpine PUBLIC ext/embree/lib)
target_link_libraries(alpine PUBLIC embree4 tbb12)

add_custom_command(TARGET alpine POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/embree/bin/embree4.dll
  $<TARGET_FILE_DIR:alpine>

  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/embree/bin/tbb12.dll
  $<TARGET_FILE_DIR:alpine>
)
endif()

# oidn libraries
if (ALPINE_ENABLE_OIDN)
target_include_directories(alpine PRIVATE ext/oidn/include)
target_link_directories(alpine PUBLIC ext/oidn/lib)
target_link_libraries(alpine PUBLIC OpenImageDenoise)

add_custom_command(TARGET alpine POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/oidn/bin/OpenImageDenoise.dll
  $<TARGET_FILE_DIR:alpine>

  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/oidn/bin/OpenImageDenoise_core.dll
  $<TARGET_FILE_DIR:alpine>

  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/oidn/bin/OpenImageDenoise_device_cpu.dll
  $<TARGET_FILE_DIR:alpine>
)
endif()

# applications
if (ALPINE_BUILD_APPS)
  add_subdirectory(applications/simple)
  add_subdirectory(applications/viewer)
endif()
