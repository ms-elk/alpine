add_library(alpine STATIC
  alpine.cpp
  camera.cpp
  camera.h
  debug_scene.cpp
  debug_scene.h
  disk_light.cpp
  disk_light.h
  embree.cpp
  file_loader.h
  gltf_loader.cpp
  image.cpp
  image.h
  kernel.h
  light.h
  material.h
  matrix.h
  matte.cpp
  matte.h
  mesh.cpp
  mesh.h
  metal.cpp
  metal.h
  obj_loader.cpp
  point_light.cpp
  point_light.h
  ray.h
  sampler.cpp
  sampler.h
  scene.h
  shape.h
  sphere.cpp
  sphere.h
  texture.h
  util.cpp
  util.h
  vector.h
)

target_include_directories(alpine
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/ext
  PRIVATE
    ${PROJECT_SOURCE_DIR}/ext/embree/include
    ${PROJECT_SOURCE_DIR}/ext/oidn/include
)

target_link_directories(alpine PUBLIC
  ${PROJECT_SOURCE_DIR}/ext/embree/lib
  ${PROJECT_SOURCE_DIR}/ext/oidn/lib
)

target_link_libraries(alpine PUBLIC
  embree3
  OpenImageDenoise
  tbb
)

add_custom_command(TARGET alpine POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PROJECT_SOURCE_DIR}/ext/embree/bin/embree3.dll
  $<TARGET_FILE_DIR:alpine>

  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PROJECT_SOURCE_DIR}/ext/embree/bin/tbb12.dll
  $<TARGET_FILE_DIR:alpine>

  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PROJECT_SOURCE_DIR}/ext/oidn/bin/OpenImageDenoise.dll
  $<TARGET_FILE_DIR:alpine>

  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PROJECT_SOURCE_DIR}/ext/oidn/bin/OpenImageDenoise_core.dll
  $<TARGET_FILE_DIR:alpine>

  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PROJECT_SOURCE_DIR}/ext/oidn/bin/OpenImageDenoise_device_cpu.dll
  $<TARGET_FILE_DIR:alpine>
)
