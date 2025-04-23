find_package(Threads)

if(Threads_FOUND)
    add_executable(
            rl_environments_oil_platform_sac_standalone
            training.cpp
    )
    target_link_libraries(
            rl_environments_oil_platform_sac_standalone
            PRIVATE rl_tools Threads::Threads
    )
    target_compile_definitions(
            rl_environments_oil_platform_sac_standalone
            PRIVATE
            RL_TOOLS_BACKEND_DISABLE_BLAS
            RL_TOOLS_DISABLE_HDF5
            RL_TOOLS_DISABLE_TENSORBOARD
    )
    RL_TOOLS_INSTALL(rl_environments_oil_platform_sac_standalone)

    if(RL_TOOLS_BACKEND_ENABLE_BLAS)
        add_executable(
                rl_environments_oil_platform_sac_blas
                training.cpp
        )
        target_link_libraries(
                rl_environments_oil_platform_sac_blas
                PRIVATE rl_tools Threads::Threads
        )
        target_compile_definitions(
                rl_environments_oil_platform_sac_blas
                PRIVATE
                RL_TOOLS_DISABLE_HDF5
                RL_TOOLS_DISABLE_TENSORBOARD
        )
        RL_TOOLS_INSTALL(rl_environments_oil_platform_sac_blas)
    endif()
endif()
