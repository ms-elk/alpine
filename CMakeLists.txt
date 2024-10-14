cmake_minimum_required(VERSION 3.1.0)

project(alpine CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
set(CMAKE_SUPPRESS_REGENERATION true)

add_compile_options(/WX)

option(ALPINE_BUILD_APPS "build applications of alpine" OFF)

if (ALPINE_BUILD_APPS)
  # set directory for output executables
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Debug)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)

  # set directory for output libraries
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Debug)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)
endif()

set(SOURCES
  src/alpine.cpp
  src/camera.cpp
  src/camera.h
  src/sampler.cpp
  src/sampler.h
  src/image.cpp
  src/image.h
  src/ray.h
  src/texture.h
)

set(KERNEL
  src/kernel/kernel.h
  src/kernel/embree.cpp
)

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
  src/utils/util.cpp
  src/utils/util.h
)

source_group("" FILES ${SOURCES})
source_group("kernel" FILES ${KERNEL})
source_group("light_samplers" FILES ${LIGHT_SAMPLERS})
source_group("lights" FILES ${LIGHTS})
source_group("loaders" FILES ${LOADERS})
source_group("materials" FILES ${MATERIALS})
source_group("math" FILES ${MATH})
source_group("scene" FILES ${SCENE})
source_group("shapes" FILES ${SHAPES})
source_group("utils" FILES ${UTILS})

add_library(alpine STATIC
  ${SOURCES}
  ${KERNEL}
  ${LIGHT_SAMPLERS}
  ${LIGHTS}
  ${LOADERS}
  ${MATERIALS}
  ${MATH}
  ${SCENE}
  ${SHAPES}
  ${UTILS}
)

target_include_directories(alpine
  PUBLIC
    include
    ext
  PRIVATE
    ext/embree/include
    ext/oidn/include
    src
)

target_link_directories(alpine PUBLIC
  ext/embree/lib
  ext/oidn/lib
)

target_link_libraries(alpine PUBLIC
  embree4
  OpenImageDenoise
  tbb12
)

add_custom_command(TARGET alpine POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/embree/bin/embree4.dll
  $<TARGET_FILE_DIR:alpine>

  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/embree/bin/tbb12.dll
  $<TARGET_FILE_DIR:alpine>

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

# applications
if (ALPINE_BUILD_APPS)
  add_subdirectory(applications/simple)
  add_subdirectory(applications/viewer)
endif()
