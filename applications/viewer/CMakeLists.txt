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

if(MSVC)
  target_link_directories(viewer PRIVATE
    ../../ext/glfw/lib-vc2022
  )
elseif(APPLE)
  target_link_directories(viewer PRIVATE
    ../../ext/glfw/lib-arm64
  )
endif()

find_package(OpenGL REQUIRED)

target_link_libraries(viewer
  alpine
  OpenGL::GL
)

if(MSVC)
  target_link_libraries(viewer
    glfw3dll
  )
elseif(APPLE)
  find_library(COCOA_FRAMEWORK Cocoa)
  find_library(IOKIT_FRAMEWORK IOKit)
  find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
  find_library(COREVIDEO_FRAMEWORK CoreVideo)

  target_link_libraries(viewer
    glfw3
    ${COCOA_FRAMEWORK}
    ${IOKIT_FRAMEWORK}
    ${COREFOUNDATION_FRAMEWORK}
    ${COREVIDEO_FRAMEWORK}
  )

  target_compile_definitions(viewer PRIVATE GL_SILENCE_DEPRECATION)
endif()

if(MSVC)
  add_custom_command(TARGET viewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${PROJECT_SOURCE_DIR}/ext/glfw/lib-vc2022/glfw3.dll
    $<TARGET_FILE_DIR:viewer>
  )
elseif(APPLE)
  add_custom_command(TARGET viewer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${PROJECT_SOURCE_DIR}/ext/glfw/lib-arm64/libglfw.3.dylib
    $<TARGET_FILE_DIR:viewer>
  )
endif()
