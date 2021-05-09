add_executable(i2c_experiment main.cpp)
target_link_libraries(
  i2c_experiment
  PRIVATE project_options
          project_warnings)
