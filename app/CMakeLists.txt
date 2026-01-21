add_executable(run_localizer main.cc)
target_link_libraries(run_localizer
        ${PROJECT_NAME} gflags
        )