add_executable(simple main.cpp)

target_include_directories(simple PRIVATE
  ../../ext/CLI11/include
)

target_link_libraries(simple alpine)
