add_executable(rtcamp9
  main.cpp
  ${PROJECT_SOURCE_DIR}/ext/fpng/src/fpng.cpp
)

target_include_directories(rtcamp9 PRIVATE
  ${PROJECT_SOURCE_DIR}/ext/fpng/src
)

target_link_libraries(rtcamp9 alpine)
