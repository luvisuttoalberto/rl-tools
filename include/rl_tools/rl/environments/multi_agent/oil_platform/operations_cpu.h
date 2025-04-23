#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_CPU_H

#include "oil_platform.h"
#include "operations_generic.h"
#include <string>
#include <sstream>
#include <cstddef>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template <typename DEVICE, typename SPEC>
    struct OilPlatformCPU: public rl::environments::multi_agent::OilPlatform<SPEC> {
        using ENVIRONMENT = rl::environments::multi_agent::OilPlatform<SPEC>;
        using State = typename ENVIRONMENT::State;
        using Parameters = typename ENVIRONMENT::Parameters;
        using TI = typename DEVICE::index_t;
    };

    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, rl::environments::multi_agent::OilPlatform<SPEC>& env, const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters) {
        std::string result = "{";
        result += "\"N_AGENTS\":" + std::to_string(SPEC::PARAMETERS::N_AGENTS) + ",";
        result += "\"ACTIVE_DRONES\":" + std::to_string(SPEC::PARAMETERS::ACTIVE_DRONES) + ",";
        result += "\"SENSOR_RANGE\":" + std::to_string(SPEC::PARAMETERS::SENSOR_RANGE) + ",";
        result += "\"GRID_SIZE_X\":" + std::to_string(SPEC::PARAMETERS::GRID_SIZE_X) + ",";
        result += "\"GRID_SIZE_Y\":" + std::to_string(SPEC::PARAMETERS::GRID_SIZE_Y) + ",";
        result += "\"HIGH_PRIORITY_X\":" + std::to_string(SPEC::PARAMETERS::HIGH_PRIORITY_X) + ",";
        result += "\"HIGH_PRIORITY_Y\":" + std::to_string(SPEC::PARAMETERS::HIGH_PRIORITY_Y) + ",";
        result += "\"HIGH_PRIORITY_RADIUS\":" + std::to_string(SPEC::PARAMETERS::HIGH_PRIORITY_RADIUS);
        result += "}";
        return result;
    }

    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, rl::environments::multi_agent::OilPlatform<SPEC>& env, const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters, const typename rl::environments::multi_agent::oil_platform::State<SPEC>& state) {
        using TI = typename DEVICE::index_t;
        std::string drone_states = "[";
        for(TI i = 0; i < SPEC::PARAMETERS::N_AGENTS; i++) {
            if(i > 0) {
                drone_states += ",";
            }
            std::string drone_state = "{";
            drone_state += "\"position\": [" + std::to_string(state.drone_states[i].position[0]) + "," + std::to_string(state.drone_states[i].position[1]) + "],";
            drone_state += "\"velocity\": [" + std::to_string(state.drone_states[i].velocity[0]) + "," + std::to_string(state.drone_states[i].velocity[1]) + "],";
            drone_state += "\"acceleration\": [" + std::to_string(state.drone_states[i].acceleration[0]) + "," + std::to_string(state.drone_states[i].acceleration[1]) + "],";
            drone_state += "\"mode\": " + std::to_string(static_cast<int>(state.drone_states[i].mode)) + ",";
            drone_state += "\"disaster_detected\": " + std::string(state.drone_states[i].disaster_detected ? "true" : "false");
            drone_state += "}";
            drone_states += drone_state;
        }
        drone_states += "]";

        std::string disaster = "{";
        disaster += "\"active\": " + std::string(state.disaster.active ? "true" : "false") + ",";
        disaster += "\"position\": [" + std::to_string(state.disaster.position[0]) + "," + std::to_string(state.disaster.position[1]) + "]";
        disaster += "}";

        std::string result = "{";
        result += "\"drone_states\": " + drone_states + ",";
        result += "\"disaster\": " + disaster;
        result += "}";
        return result;
    }

    template <typename DEVICE, typename SPEC>
    std::string get_ui(DEVICE& /*device*/, rl::environments::multi_agent::OilPlatform<SPEC>& /*env*/) {
        // Dummy UI: preserves init & render interface but does nothing.
        return R"RL_TOOLS_LITERAL(
export async function init(canvas, options) {
    // no-op
    return {};
}
export async function render(ui_state, parameters, state, action) {
    // no-op
}
)RL_TOOLS_LITERAL";
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif
