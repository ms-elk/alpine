set(IMGUI_SRCS
  ../../ext/imgui/imgui.cpp
  ../../ext/imgui/imgui_draw.cpp
  ../../ext/imgui/imgui_tables.cpp
  ../../ext/imgui/imgui_widgets.cpp
  ../../ext/imgui/backends/imgui_impl_glfw.cpp
  ../../ext/imgui/backends/imgui_impl_opengl3.cpp
)

add_executable(viewer
  main.cpp
  ${IMGUI_SRCS}
)

target_include_directories(viewer PRIVATE
  ../../ext/glfw/include
  ../../ext/imgui
  ../../ext/imgui/backends
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
