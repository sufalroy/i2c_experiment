add_executable(i2c_experiment main.cpp)
target_link_libraries(
  i2c_experiment
  PRIVATE i2c 
  	project_options
    project_warnings
	spdlog::spdlog)
