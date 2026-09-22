#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_OIL_PLATFORM_V1_ENVIRONMENT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_OIL_PLATFORM_V1_ENVIRONMENT_H

#include <rl_tools/rl/environments/multi_agent/oil_platform/operations_cpu.h>
#include <type_traits>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1 {
    namespace rlt = rl_tools;
    template<typename T, typename TI>
    struct VariableSwarmParameters: rlt::rl::environments::multi_agent::oil_platform::DefaultParameters<T, TI> {
        static constexpr TI MAX_AGENTS = 6;
        static constexpr TI N_AGENTS = MAX_AGENTS;
        static constexpr TI MIN_AGENTS = 2;
        static constexpr bool RANDOMIZE_SWARM_SIZE = true;
        static constexpr bool NORMALIZE_SWARM_REWARD = true;
        static_assert(MIN_AGENTS >= 2 && MIN_AGENTS <= MAX_AGENTS);
    };
    template <typename DEVICE, typename TYPE_POLICY, typename TI, bool USE_PRIVILEGED_CRITIC_OBSERVATION = false, bool VARIABLE_SWARM = false>
    struct ENVIRONMENT_FACTORY {
        using T = typename TYPE_POLICY::DEFAULT;
        using PARAMETERS = std::conditional_t<VARIABLE_SWARM, VariableSwarmParameters<T, TI>, rlt::rl::environments::multi_agent::oil_platform::DefaultParameters<T, TI>>;
        using OBSERVATION = rlt::rl::environments::multi_agent::oil_platform::Observation<PARAMETERS>;
        using OBSERVATION_PRIVILEGED_RAW = rlt::rl::environments::multi_agent::oil_platform::ObservationPrivileged<PARAMETERS>;
        using OBSERVATION_PRIVILEGED = typename std::conditional<
                USE_PRIVILEGED_CRITIC_OBSERVATION,
                OBSERVATION_PRIVILEGED_RAW,
                OBSERVATION
        >::type;
        using ENVIRONMENT_SPEC = rlt::rl::environments::multi_agent::oil_platform::Specification<T, TI, PARAMETERS, OBSERVATION, OBSERVATION_PRIVILEGED>;
        using ENVIRONMENT      = rlt::rl::environments::multi_agent::OilPlatform<ENVIRONMENT_SPEC>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
