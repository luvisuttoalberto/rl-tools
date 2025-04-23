add_executable(
 rl_environments_multi_agent_oil_platform_monitoring
 oil_platform_monitoring.cpp
)
target_link_libraries(
 rl_environments_multi_agent_oil_platform_monitoring
 PRIVATE rl_tools
)

# also build the SAC trainer
add_subdirectory(sac/cpu)

if(RL_TOOLS_BACKEND_ENABLE_CUDA)
    add_subdirectory(sac/cuda)
endif()
