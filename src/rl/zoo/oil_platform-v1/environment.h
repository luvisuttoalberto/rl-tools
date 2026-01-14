#include <rl_tools/version.h>
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ZOO_OIL_PLATFORM_V1_ENVIRONMENT_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ZOO_OIL_PLATFORM_V1_ENVIRONMENT_H

#include <rl_tools/rl/environments/multi_agent/oil_platform/operations_cpu.h>
#include <type_traits>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1 {
    namespace rlt = rl_tools;
    template <typename DEVICE, typename T, typename TI, bool USE_PRIVILEGED_CRITIC_OBSERVATION = false>
    struct ENVIRONMENT_FACTORY {
        using PARAMETERS = rlt::rl::environments::multi_agent::oil_platform::DefaultParameters<T, TI>;
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
