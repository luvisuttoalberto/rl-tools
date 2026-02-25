#pragma once

// This header provides PPO-specific observation construction for the oil_platform environment.
// It is included from operations_generic.h when RL_TOOLS_USE_MULTI_AGENT_PPO is defined.

template<typename DEVICE, typename SPEC, typename OBS_SPEC, typename OBS_PARAMETERS, typename RNG>
RL_TOOLS_FUNCTION_PLACEMENT static void observe(
        DEVICE &device,
        const rl::environments::multi_agent::OilPlatform<SPEC> &env,
        const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
        const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
        const rl::environments::multi_agent::oil_platform::Observation<OBS_PARAMETERS> &,
        Matrix<OBS_SPEC> &observation,
        RNG &rng
) {
    using OBS = rl::environments::multi_agent::oil_platform::Observation<OBS_PARAMETERS>;
    static_assert(OBS_SPEC::ROWS == 1);
    static_assert(OBS_SPEC::COLS == OBS::DIM);
    using T = typename SPEC::T;
    using TI = typename SPEC::TI;
    constexpr TI PER_AGENT_OBS_DIM = OBS::PER_AGENT_DIM;
    constexpr TI SHARED_DIM = OBS::SHARED_DIM;
    constexpr TI OTHER_AGENTS_DIM = OBS::OTHER_AGENTS_DIM;
    constexpr TI PER_OTHER_AGENT_DIM = OBS::PER_OTHER_AGENT_DIM;
    constexpr TI PER_AGENT_TOTAL_DIM = OBS::PER_AGENT_TOTAL_DIM;
    using PARAMS = typename SPEC::PARAMETERS;

    // Pre-compute shared observations once
    T shared_obs[SHARED_DIM];
    shared_obs[0] = state.disaster_detected_global ? 1 : -1;
    shared_obs[1] = !state.disaster_detected_global ? 10 : 2 * (state.last_detected_disaster_position[0]/PARAMS::GRID_SIZE_X) - 1;
    shared_obs[2] = !state.disaster_detected_global ? 10 : 2 * (state.last_detected_disaster_position[1]/PARAMS::GRID_SIZE_Y) - 1;
    if constexpr (PARAMS::ACTOR_OBSERVE_CHARGING_STATION_POSITION) {
        shared_obs[3] = 2 * (state.charging_station_position[0] / PARAMS::GRID_SIZE_X) - 1;
        shared_obs[4] = 2 * (state.charging_station_position[1] / PARAMS::GRID_SIZE_Y) - 1;
    }

    for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; agent_i++) {
        const auto &agent_state = state.drone_states[agent_i];
        TI base_offset = agent_i * PER_AGENT_TOTAL_DIM;

        // Per-agent own observations (8 dimensions)
        set(observation, 0, base_offset + 0, 2 * (agent_state.position[0] / PARAMS::GRID_SIZE_X) - 1);  // Normalized position [-1,1]
        set(observation, 0, base_offset + 1, 2 * (agent_state.position[1] / PARAMS::GRID_SIZE_Y) - 1);  // Normalized position [-1,1]
        set(observation, 0, base_offset + 2, agent_state.dead ? 0 : agent_state.velocity[0] / PARAMS::MAX_SPEED);
        set(observation, 0, base_offset + 3, agent_state.dead ? 0 : agent_state.velocity[1] / PARAMS::MAX_SPEED);
        set(observation, 0, base_offset + 4, agent_state.dead ? -1 : (agent_state.is_detecting ? 1 : -1));
        set(observation, 0, base_offset + 5, 2 * (agent_state.battery / 100) - 1);
        set(observation, 0, base_offset + 6, agent_state.dead ? 1 : -1);
        set(observation, 0, base_offset + 7, agent_state.dead ? -1 : agent_state.is_charging ? 1 : -1);

        // Other agents' observations (6 dimensions per other agent: pos_x, pos_y, vel_x, vel_y, battery, charging)
        TI other_agent_offset = 0;
        for (TI other_i = 0; other_i < PARAMS::N_AGENTS; other_i++) {
            if (other_i == agent_i) continue;  // Skip self

            const auto &other_state = state.drone_states[other_i];
            TI offset = base_offset + PER_AGENT_OBS_DIM + other_agent_offset * PER_OTHER_AGENT_DIM;

            set(observation, 0, offset + 0, 2 * (other_state.position[0] / PARAMS::GRID_SIZE_X) - 1);  // Normalized position [-1,1]
            set(observation, 0, offset + 1, 2 * (other_state.position[1] / PARAMS::GRID_SIZE_Y) - 1);  // Normalized position [-1,1]
            set(observation, 0, offset + 2, other_state.dead ? 0 : other_state.velocity[0] / PARAMS::MAX_SPEED);
            set(observation, 0, offset + 3, other_state.dead ? 0 : other_state.velocity[1] / PARAMS::MAX_SPEED);
            set(observation, 0, offset + 4, 2 * (other_state.battery / 100) - 1);
            set(observation, 0, offset + 5, other_state.dead ? -1 : other_state.is_charging ? 1 : -1);

            other_agent_offset++;
        }

        // Shared observations duplicated for each agent (5 dimensions)
        for (TI shared_i = 0; shared_i < SHARED_DIM; shared_i++) {
            set(observation, 0, base_offset + PER_AGENT_OBS_DIM + OTHER_AGENTS_DIM + shared_i, shared_obs[shared_i]);
        }
    }

    utils::assert_exit(device, !is_nan(device, observation), "Observation is nan");
}
