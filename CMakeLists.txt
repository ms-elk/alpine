cmake_minimum_required(VERSION 3.1.0)

project(alpine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
set(CMAKE_SUPPRESS_REGENERATION true)

add_compile_options(/WX)

option(ALPINE_BUILD_APPS "build applications of alpine" ON)

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

add_library(alpine STATIC
  src/alpine.cpp
  src/camera.cpp
  src/camera.h
  src/debug_scene.cpp
  src/debug_scene.h
  src/disk_light.cpp
  src/disk_light.h
  src/embree.cpp
  src/file_loader.h
  src/gltf_loader.cpp
  src/image.cpp
  src/image.h
  src/kernel.h
  src/light.h
  src/material.h
  src/matrix.h
  src/matte.cpp
  src/matte.h
  src/mesh.cpp
  src/mesh.h
  src/metal.cpp
  src/metal.h
  src/obj_loader.cpp
  src/point_light.cpp
  src/point_light.h
  src/ray.h
  src/sampler.cpp
  src/sampler.h
  src/scene.h
  src/shape.h
  src/sphere.cpp
  src/sphere.h
  src/texture.h
  src/util.cpp
  src/util.h
  src/vector.h
)

target_include_directories(alpine
  PUBLIC
    include
    ext
  PRIVATE
    ext/embree/include
    ext/oidn/include
)

target_link_directories(alpine PUBLIC
  ext/embree/lib
  ext/oidn/lib
)

target_link_libraries(alpine PUBLIC
  embree3
  OpenImageDenoise
  tbb
)

add_custom_command(TARGET alpine POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${CMAKE_CURRENT_SOURCE_DIR}/ext/embree/bin/embree3.dll
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
