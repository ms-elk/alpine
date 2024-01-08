add_executable(viewer main.cpp)

target_include_directories(viewer PRIVATE
  ../../ext/glfw/include
)

target_link_directories(viewer PRIVATE
  ../../ext/glfw/lib-vc2022
)

target_link_libraries(viewer
  alpine
  glfw3dll
  OpenGL32
)

add_custom_command(TARGET viewer POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${PROJECT_SOURCE_DIR}/ext/glfw/lib-vc2022/glfw3.dll
  $<TARGET_FILE_DIR:viewer>
)
