#pragma once

#include "../../../../version.h"
#include "../../../../rl_tools.h"
#include "../../../../containers/matrix/matrix.h"
#include "../../../../random/operations_generic.h"
#include "oil_platform.h"

#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || \
     !defined(RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_GENERIC_H)) && \
    (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_GENERIC_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    // Pull in the inner-namespace types so we can refer to them unqualified:
    using rl::environments::multi_agent::oil_platform::DroneMode;
    using rl::environments::multi_agent::oil_platform::Observation;

    // 1) Nothing to malloc/free/init at the env‐level for now
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void malloc(
            DEVICE & device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env
    ) {}

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void free(
            DEVICE & device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env
    ) {}

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void init(
            DEVICE & device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env
    ) {}

    // 2) Parameters (default‐only for now)
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_parameters(
            DEVICE & device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            RNG & rng
    ) {
        parameters = typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters{};
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_parameters(
            DEVICE & device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters
    ) {
        parameters = typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters{};
    }

    // 3) Initial state (split drones into NORMAL vs RECHARGING, clear occupancy, zero step_count)
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_state(
            DEVICE & device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            RNG & rng
    ) {
        using T  = typename SPEC::T;
        using TI = typename SPEC::TI;

        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            auto &d = state.drone_states[i];
            d.position[0]       = T(0);
            d.position[1]       = T(0);
            d.velocity[0]       = T(0);
            d.velocity[1]       = T(0);
            d.acceleration[0]   = T(0);
            d.acceleration[1]   = T(0);
            d.mode              = (i < parameters.ACTIVE_DRONES ? DroneMode::NORMAL : DroneMode::RECHARGING);
            d.disaster_detected = false;
            d.battery           = T(100);
            d.recharge_count    = 0;
        }

        state.disaster.active      = false;
        state.disaster.position[0] = T(0);
        state.disaster.position[1] = T(0);

        for (TI idx = 0; idx < parameters.GRID_SIZE_X * parameters.GRID_SIZE_Y; ++idx) {
            state.last_visit[idx] = TI(0);
        }
        state.step_count = 0;
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_state(
            DEVICE & device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state
    ) {
        using T  = typename SPEC::T;
        using TI = typename SPEC::TI;

        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            auto &d = state.drone_states[i];
            d.position[0]       = T(0);
            d.position[1]       = T(0);
            d.velocity[0]       = T(0);
            d.velocity[1]       = T(0);
            d.acceleration[0]   = T(0);
            d.acceleration[1]   = T(0);
            d.mode              = (i < parameters.ACTIVE_DRONES ? DroneMode::NORMAL : DroneMode::RECHARGING);
            d.disaster_detected = false;
            d.battery           = T(100);
            d.recharge_count    = 0;
        }

        state.disaster.active      = false;
        state.disaster.position[0] = T(0);
        state.disaster.position[1] = T(0);

        for (TI idx = 0; idx < parameters.GRID_SIZE_X * parameters.GRID_SIZE_Y; ++idx) {
            state.last_visit[idx] = TI(0);
        }
        state.step_count = 0;
    }

    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(
            DEVICE & device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            RNG & rng
    ) {
        initial_state(device, env, parameters, state, rng);
    }

    // 4) Step + reward + detection + battery + swapping + exploration + high‑priority bonus
    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T step(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & /*env*/,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            const Matrix<ACTION_SPEC> &action,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State &next_state,
            RNG &rng
    ) {
        using T  = typename SPEC::T;
        using TI = typename SPEC::TI;

        T total_reward = T(0);

        // (1) Copy last-visited timestamps
        for (TI idx = 0; idx < parameters.GRID_SIZE_X * parameters.GRID_SIZE_Y; ++idx) {
            next_state.last_visit[idx] = state.last_visit[idx];
        }

        // (2) Disaster: 2% to start in any HIGH‑PRIORITY area, then drift at fixed vx,vy
        {
            const T cx = parameters.GRID_SIZE_X / T(2);
            const T cy = parameters.GRID_SIZE_Y / T(2);
            const T vx = T(1), vy = T(0);

            if (!state.disaster.active) {
                if (random::uniform_real_distribution(device.random, T(0), T(1), rng) < T(0.02)) {
                    T x, y;
                    bool inHigh;
                    do {
                        x = random::uniform_real_distribution(device.random, T(0), T(parameters.GRID_SIZE_X), rng);
                        y = random::uniform_real_distribution(device.random, T(0), T(parameters.GRID_SIZE_Y), rng);
                        T adx = std::abs(x - cx), ady = std::abs(y - cy);
                        bool inPlat = (adx <= parameters.PLATFORM_HALF_SIZE && ady <= parameters.PLATFORM_HALF_SIZE);
                        bool inPipeH = (ady <= parameters.PIPE_WIDTH/2   && adx >= parameters.PLATFORM_HALF_SIZE);
                        bool inPipeV = (adx <= parameters.PIPE_WIDTH/2   && ady >= parameters.PLATFORM_HALF_SIZE);
                        inHigh = inPlat || inPipeH || inPipeV;
                    } while (!inHigh);
                    next_state.disaster.active      = true;
                    next_state.disaster.position[0] = x;
                    next_state.disaster.position[1] = y;
                } else {
                    next_state.disaster.active = false;
                }
            } else {
                next_state.disaster.active      = true;
                next_state.disaster.position[0] = state.disaster.position[0] + vx * parameters.DT;
                next_state.disaster.position[1] = state.disaster.position[1] + vy * parameters.DT;
            }
        }

        // (3) Disaster detection
        bool detected = false;
        if (next_state.disaster.active) {
            for (TI i = 0; i < parameters.N_AGENTS; i++) {
                T dx = state.drone_states[i].position[0] - next_state.disaster.position[0];
                T dy = state.drone_states[i].position[1] - next_state.disaster.position[1];
                if (math::sqrt(device.math, dx*dx + dy*dy) < parameters.SENSOR_RANGE) {
                    detected = true;
                    break;
                }
            }
        }

        // (4) Per‑drone update: battery, recharge_count, dynamics, flags
        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            const auto &old_d = state.drone_states[i];
            auto       &new_d = next_state.drone_states[i];

            // battery
            if (old_d.mode == DroneMode::RECHARGING) {
                new_d.battery = std::min(T(100), old_d.battery + parameters.RECHARGE_RATE);
            } else {
                new_d.battery = std::max(T(0), old_d.battery - parameters.DISCHARGE_RATE);
            }

            // recharge counter
            if (old_d.mode == DroneMode::RECHARGING && new_d.battery >= T(100)) {
                new_d.recharge_count = old_d.recharge_count + 1;
            } else {
                new_d.recharge_count = 0;
            }

            // acceleration
            T ax = T(0), ay = T(0);
            if (old_d.mode == DroneMode::RECHARGING) {
                T dx = -old_d.position[0], dy = -old_d.position[1];
                T n = math::sqrt(device.math, dx*dx + dy*dy) + 1e-6f;
                ax = dx/n * parameters.MAX_ACCELERATION;
                ay = dy/n * parameters.MAX_ACCELERATION;
            } else {
                ax = get(action, 0, i*2 + 0) * parameters.MAX_ACCELERATION;
                ay = get(action, 0, i*2 + 1) * parameters.MAX_ACCELERATION;
            }

            // integrate
            new_d.velocity[0] = old_d.velocity[0] + ax * parameters.DT;
            new_d.velocity[1] = old_d.velocity[1] + ay * parameters.DT;
            new_d.position[0] = old_d.position[0] + new_d.velocity[0] * parameters.DT;
            new_d.position[1] = old_d.position[1] + new_d.velocity[1] * parameters.DT;

            // clamp
            new_d.position[0] = math::clamp(device.math, new_d.position[0], T(0), T(parameters.GRID_SIZE_X));
            new_d.position[1] = math::clamp(device.math, new_d.position[1], T(0), T(parameters.GRID_SIZE_Y));

            new_d.acceleration[0]   = ax;
            new_d.acceleration[1]   = ay;
            new_d.mode              = old_d.mode;
            new_d.disaster_detected = detected;
        }

        // (5) Swap fully charged ↔ lowest battery
        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            const auto &od = state.drone_states[i];
            if (od.mode == DroneMode::RECHARGING && od.recharge_count >= parameters.FULLY_CHARGED_STEPS) {
                TI low_i = 0;
                T   low_b = std::numeric_limits<T>::infinity();
                for (TI j = 0; j < parameters.N_AGENTS; j++) {
                    const auto &c = next_state.drone_states[j];
                    if (c.mode != DroneMode::RECHARGING && c.battery < low_b) {
                        low_b = c.battery;
                        low_i = j;
                    }
                }
                if (low_b < T(50)) {
                    next_state.drone_states[i].mode     = DroneMode::NORMAL;
                    next_state.drone_states[low_i].mode = DroneMode::RECHARGING;
                    next_state.drone_states[low_i].recharge_count = 0;
                }
            }
        }

        // (6) Two-phase reward logic
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            const auto &d = next_state.drone_states[i];
            if (d.mode == DroneMode::RECHARGING) continue;

            // map continuous pos to grid cell
            TI cx = math::clamp(device.math,
                                TI(d.position[0] / parameters.GRID_CELL_SIZE),
                                TI(0), parameters.GRID_SIZE_X - 1);
            TI cy = math::clamp(device.math,
                                TI(d.position[1] / parameters.GRID_CELL_SIZE),
                                TI(0), parameters.GRID_SIZE_Y - 1);
            TI idx = cx + cy * parameters.GRID_SIZE_X;

            if (state.disaster.active) {
                // Phase 1: all drones swarm to the spill
                T dx = d.position[0] - next_state.disaster.position[0];
                T dy = d.position[1] - next_state.disaster.position[1];
                T dist = math::sqrt(device.math, dx * dx + dy * dy);
                total_reward += std::max(
                        parameters.DISASTER_PRIORITY * (parameters.SENSOR_RANGE - dist),
                        T(0)
                );
            } else {
                // Phase 2: continuous exploration
                if (state.step_count - state.last_visit[idx] > parameters.REVISIT_THRESHOLD) {
                    total_reward += parameters.EXPLORATION_BONUS;
                    next_state.last_visit[idx] = next_state.step_count;
                }

                // priority vs general cells
                T x = d.position[0], y = d.position[1];
                T adx = std::abs(x - parameters.GRID_SIZE_X / 2);
                T ady = std::abs(y - parameters.GRID_SIZE_Y / 2);
                bool inPlat = (adx <= parameters.PLATFORM_HALF_SIZE && ady <= parameters.PLATFORM_HALF_SIZE);
                bool inPipeH = (ady <= parameters.PIPE_WIDTH / 2 && adx >= parameters.PLATFORM_HALF_SIZE);
                bool inPipeV = (adx <= parameters.PIPE_WIDTH / 2 && ady >= parameters.PLATFORM_HALF_SIZE);

                if (inPlat || inPipeH || inPipeV) {
                    total_reward += parameters.HIGH_PRIORITY_BONUS;
                } else {
                    total_reward += parameters.GENERAL_AREA_BONUS;
                }
            }
        }

        // (7) Advance step count
        next_state.step_count = state.step_count + 1;

        return total_reward;
    }

    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            const Matrix<ACTION_SPEC> &action,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State &next_state,
            RNG &rng
    ) {
        return step(device, env, parameters, state, action, next_state, rng);
    }

    // 7) Observe()
    template<typename DEVICE, typename SPEC, typename OBS_SPEC, typename OBS_PARAMETERS, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & /*env*/,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            const rl::environments::multi_agent::oil_platform::Observation<OBS_PARAMETERS> & /*obs_spec*/,
            Matrix<OBS_SPEC> &observation,
            RNG & /*rng*/
    ) {
        using T  = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI D = Observation<OBS_PARAMETERS>::PER_AGENT_DIM;

        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            const auto &d = state.drone_states[i];
            set(observation, 0, i*D + 0, d.position[0]);
            set(observation, 0, i*D + 1, d.position[1]);
            set(observation, 0, i*D + 2, d.velocity[0]);
            set(observation, 0, i*D + 3, d.velocity[1]);
            set(observation, 0, i*D + 4, d.acceleration[0]);
            set(observation, 0, i*D + 5, d.acceleration[1]);
            set(observation, 0, i*D + 6, (T)d.mode);
            set(observation, 0, i*D + 7, d.disaster_detected ? T(1) : T(0));
            set(observation, 0, i*D + 8, state.disaster.active ? state.disaster.position[0] : T(0));
            set(observation, 0, i*D + 9, state.disaster.active ? state.disaster.position[1] : T(0));
        }
    }

    // 8) Termination: just the step limit
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(
            DEVICE & /*device*/,
            const rl::environments::multi_agent::OilPlatform<SPEC> & /*env*/,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            RNG & /*rng*/
    ) {
        // bring in the T and TI types from SPEC:
        using T  = typename SPEC::T;
        using TI = typename SPEC::TI;

        // 1) end immediately if a drone runs out of battery
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            if (state.drone_states[i].battery <= T(0)) {
                return true;
            }
        }
        // 2) otherwise standard step‐limit check
        return state.step_count >= parameters.EPISODE_STEP_LIMIT;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif  // RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_GENERIC_H
