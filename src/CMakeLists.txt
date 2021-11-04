find_package(spdlog)

add_executable(experiment main.cpp)
target_link_libraries(experiment PRIVATE i2c bluetooth project_options project_warnings spdlog::spdlog)
