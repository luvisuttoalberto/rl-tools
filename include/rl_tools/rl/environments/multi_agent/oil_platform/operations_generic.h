#pragma once

#include "../../../../version.h"
#include "../../../../rl_tools.h"
#include "../../../../containers/matrix/matrix.h"
#include "../../../../random/operations_generic.h"
#include "oil_platform.h"
#include <array>

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
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> &env
    ) {}

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void free(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> &env
    ) {}

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void init(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> &env
    ) {}

    // 2) Parameters (default‐only for now)
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_parameters(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> &env,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            RNG &rng
    ) {
        parameters = typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters{};
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_parameters(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> &env,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters
    ) {
        parameters = typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters{};
    }

    // 3) Initial state (split drones into NORMAL vs RECHARGING, clear occupancy, zero step_count)
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_state(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> &env,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            RNG &rng
    ) {
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;

        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            auto &d = state.drone_states[i];
            d.position[0] = T(parameters.GRID_SIZE_X / 2);
            d.position[1] = T(parameters.GRID_SIZE_Y / 2);
            d.velocity[0] = T(0);
            d.velocity[1] = T(0);
            d.acceleration[0] = T(0);
            d.acceleration[1] = T(0);
            d.mode = (i < parameters.ACTIVE_DRONES ? DroneMode::NORMAL : DroneMode::RECHARGING);
            d.disaster_detected = false;
            d.battery = T(95);  // Start with higher battery level
            d.last_detected_disaster_position[0] = T(-1);
            d.last_detected_disaster_position[1] = T(-1);
        }

        state.disaster.active = false;
        state.disaster.position[0] = T(0);
        state.disaster.position[1] = T(0);
//        state.disaster_undetected_steps = 0;

//        for (TI idx = 0; idx < parameters.GRID_SIZE_X * parameters.GRID_SIZE_Y; ++idx) {
//            state.last_visit[idx] = TI(0);
//        }
        state.step_count = 0;
    }

    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static void initial_state(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> &env,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state
    ) {
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;

        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            auto &d = state.drone_states[i];
            d.position[0] = T(parameters.GRID_SIZE_X / 2);
            d.position[1] = T(parameters.GRID_SIZE_Y / 2);
            d.velocity[0] = T(0);
            d.velocity[1] = T(0);
            d.acceleration[0] = T(0);
            d.acceleration[1] = T(0);
            d.mode = (i < parameters.ACTIVE_DRONES ? DroneMode::NORMAL : DroneMode::RECHARGING);
            d.disaster_detected = false;
            d.battery = T(95);  // Start with higher battery level
            d.last_detected_disaster_position[0] = T(-1);
            d.last_detected_disaster_position[1] = T(-1);
        }

        state.disaster.active = false;
        state.disaster.position[0] = T(0);
        state.disaster.position[1] = T(0);
//        state.disaster_undetected_steps = 0;

//        for (TI idx = 0; idx < parameters.GRID_SIZE_X * parameters.GRID_SIZE_Y; ++idx) {
//            state.last_visit[idx] = TI(0);
//        }
        state.step_count = 0;
    }

    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_initial_state(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> &env,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            RNG &rng
    ) {
        initial_state(device, env, parameters, state, rng);
    }

    // Helper function to compute magnitude of a 2D vector
    template<typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static T magnitude(DEVICE& device, T x, T y) {
        return math::sqrt(device.math, x * x + y * y);
    }

    // Helper function to clamp velocity to maximum speed
    template<typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static void clamp_velocity(DEVICE& device, T& vx, T& vy, T max_speed) {
        T speed = magnitude(device, vx, vy);
        if (speed > max_speed) {
            T scale = max_speed / speed;
            vx *= scale;
            vy *= scale;
        }
    }

    // 4) Step - ONLY handles physics and state transitions, returns DT value
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
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;

        // per-agent detection flags, must live until we assign each new_d.disaster_detected
        std::array<bool, SPEC::PARAMETERS::N_AGENTS> local_detect{};
        bool any_local = false;

        // Maximum speed for drones
        const T MAX_SPEED = T(2.0);

        // (1) Copy last-visited timestamps
//        for (TI idx = 0; idx < parameters.GRID_SIZE_X * parameters.GRID_SIZE_Y; ++idx) {
//            next_state.last_visit[idx] = state.last_visit[idx];
//        }

        // (2) Disaster: 2% to start in any HIGH‑PRIORITY area, remains static once created
        {
            const T cx = parameters.GRID_SIZE_X / T(2);
            const T cy = parameters.GRID_SIZE_Y / T(2);

            if (!state.disaster.active) {
                if (random::uniform_real_distribution(device.random, T(0), T(1), rng) < T(0.02)) {
                    T x, y;
                    bool inHigh;
                    do {
                        x = random::uniform_real_distribution(device.random, T(0), T(parameters.GRID_SIZE_X), rng);
                        y = random::uniform_real_distribution(device.random, T(0), T(parameters.GRID_SIZE_Y), rng);
                        T adx = std::abs(x - cx), ady = std::abs(y - cy);
                        bool inPlat = (adx <= parameters.PLATFORM_HALF_SIZE && ady <= parameters.PLATFORM_HALF_SIZE);
                        bool inPipeH = (ady <= parameters.PIPE_WIDTH / 2 && adx >= parameters.PLATFORM_HALF_SIZE);
                        bool inPipeV = (adx <= parameters.PIPE_WIDTH / 2 && ady >= parameters.PLATFORM_HALF_SIZE);
                        inHigh = inPlat || inPipeH || inPipeV;
                    } while (!inHigh);
                    next_state.disaster.active = true;
                    next_state.disaster.position[0] = x;
                    next_state.disaster.position[1] = y;
                } else {
                    next_state.disaster.active = false;
                }
            } else {
                // Disaster remains static once created
                next_state.disaster.active = true;
                next_state.disaster.position[0] = state.disaster.position[0];
                next_state.disaster.position[1] = state.disaster.position[1];
            }

            for (TI i = 0; i < parameters.N_AGENTS; ++i) {
                const auto &od = state.drone_states[i];
                bool see = false;
                if (next_state.disaster.active && od.mode != DroneMode::RECHARGING) {
                    T dx = od.position[0] - next_state.disaster.position[0];
                    T dy = od.position[1] - next_state.disaster.position[1];
                    if (magnitude(device, dx, dy) < parameters.SENSOR_RANGE) {
                        see = true;
                    }
                }
                local_detect[i] = see;
                any_local      |= see;
            }

            // stamp *all* agents’ last_seen coords if anyone sees it
            if (any_local) {
                for (TI j = 0; j < parameters.N_AGENTS; ++j) {
                    next_state.drone_states[j]
                            .last_detected_disaster_position[0]
                            = next_state.disaster.position[0];
                    next_state.drone_states[j]
                            .last_detected_disaster_position[1]
                            = next_state.disaster.position[1];
                }
            }
        }

//        // (3) Disaster detection
//        bool detected = false;
//        if (next_state.disaster.active) {
//            const T centerX = parameters.GRID_SIZE_X / T(2);
//            const T centerY = parameters.GRID_SIZE_Y / T(2);
//            for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//                const auto &d = state.drone_states[i];
//
//                // Skip any drone that is in RECHARGING mode at the charging station
//                T dx0 = d.position[0] - centerX;
//                T dy0 = d.position[1] - centerY;
//                if (d.mode == DroneMode::RECHARGING
//                    && math::sqrt(device.math, dx0*dx0 + dy0*dy0) < T(1.0)) {
//                    continue;
//                }
//                // Otherwise check normal sensor range
//                T dx = d.position[0] - next_state.disaster.position[0];
//                T dy = d.position[1] - next_state.disaster.position[1];
//                if (math::sqrt(device.math, dx*dx + dy*dy) < parameters.SENSOR_RANGE) {
//                    detected = true;
//                    break;
//                }
//            }
//        }

        // Track how long a disaster has been active but undetected
//        if (next_state.disaster.active) {
//            if (!detected) {
//                next_state.disaster_undetected_steps = state.disaster_undetected_steps + 1;
//            } else {
//                next_state.disaster_undetected_steps = 0;
//            }
//        } else {
//            next_state.disaster_undetected_steps = 0;
//        }

        // (4) Per‑drone update: battery, dynamics, flags
        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            const auto &old_d = state.drone_states[i];
            auto &new_d = next_state.drone_states[i];

            // Improved battery management
            // Switch to RECHARGING mode when battery gets low
            if (old_d.mode != DroneMode::RECHARGING && old_d.battery < T(40)) {
                new_d.mode = DroneMode::RECHARGING;
            }

            // Handle recharging and battery updates
            if (old_d.mode == DroneMode::RECHARGING) {
                // Check if the drone is at the recharging station (center of the grid)
//                const T centerX = parameters.GRID_SIZE_X / T(2);
//                const T centerY = parameters.GRID_SIZE_Y / T(2);
                const T chargeX = T(0);
                const T chargeY = T(0);
                const T dx = old_d.position[0] - chargeX;
                const T dy = old_d.position[1] - chargeY;
                const T distance = math::sqrt(device.math, dx * dx + dy * dy);

                // Only recharge if the drone is close enough to the center
                if (distance < 1.0) {
                    new_d.battery = std::min(T(100), old_d.battery + parameters.RECHARGE_RATE);
                } else {
                    // Still discharge but at a lower rate when in RECHARGING mode
                    new_d.battery = std::max(T(0), old_d.battery - parameters.DISCHARGE_RATE * T(0.5));
                }
            } else {
                new_d.battery = std::max(T(0), old_d.battery - parameters.DISCHARGE_RATE);
            }

            // Calculate acceleration based on drone mode and current state
            T ax = T(0), ay = T(0);

            if (old_d.mode == DroneMode::RECHARGING) {
                // Get vector to base station (center of grid)
                T chargeX = T(0);
                T chargeY = T(0);
                T dx = chargeX - old_d.position[0];
                T dy = chargeY - old_d.position[1];
//                T baseX = parameters.GRID_SIZE_X / T(2);
//                T baseY = parameters.GRID_SIZE_Y / T(2);
//                T dx = baseX - old_d.position[0];
//                T dy = baseY - old_d.position[1];
                T dist = math::sqrt(device.math, dx * dx + dy * dy) + 1e-6f;

                if (dist < 0.5) {
                    // When very close to base, apply strong braking force
                    new_d.velocity[0] = old_d.velocity[0] * T(0.3); // Damping factor
                    new_d.velocity[1] = old_d.velocity[1] * T(0.3);

                    if (math::sqrt(device.math, new_d.velocity[0]*new_d.velocity[0] + new_d.velocity[1]*new_d.velocity[1]) < T(0.1)) {
                        new_d.velocity[0] = T(0);
                        new_d.velocity[1] = T(0);
                    }
                }
                else if (dist < 2.0) {
                    T approach_speed = std::min(dist * T(0.8), T(1.0));
                    T target_vx = dx / dist * approach_speed;
                    T target_vy = dy / dist * approach_speed;
                    ax = (target_vx - old_d.velocity[0]) / parameters.DT * T(0.7);
                    ay = (target_vy - old_d.velocity[1]) / parameters.DT * T(0.7);
                }
                else {
                    T approach_speed = std::min(T(2.0), dist * T(0.2));
                    T target_vx = dx / dist * approach_speed;
                    T target_vy = dy / dist * approach_speed;
                    ax = (target_vx - old_d.velocity[0]) / parameters.DT * T(0.7);
                    ay = (target_vy - old_d.velocity[1]) / parameters.DT * T(0.7);
                }

                // Limit acceleration magnitude to MAX_ACCELERATION
                T accel_mag = magnitude(device, ax, ay);
                if (accel_mag > parameters.MAX_ACCELERATION) {
                    T scale = parameters.MAX_ACCELERATION / accel_mag;
                    ax *= scale;
                    ay *= scale;
                }
            } else {
                ax = get(action, 0, i * 2 + 0) * parameters.MAX_ACCELERATION;
                ay = get(action, 0, i * 2 + 1) * parameters.MAX_ACCELERATION;
            }

            // integrate acceleration to get new velocity
            new_d.velocity[0] = old_d.velocity[0] + ax * parameters.DT;
            new_d.velocity[1] = old_d.velocity[1] + ay * parameters.DT;

            // clamp velocity magnitude to MAX_SPEED
            T speed = math::sqrt(device.math, new_d.velocity[0]*new_d.velocity[0] + new_d.velocity[1]*new_d.velocity[1]);
            if (speed > MAX_SPEED) {
                T scale = MAX_SPEED / speed;
                new_d.velocity[0] *= scale;
                new_d.velocity[1] *= scale;
            }

            // integrate velocity to get new position
            new_d.position[0] = old_d.position[0] + new_d.velocity[0] * parameters.DT;
            new_d.position[1] = old_d.position[1] + new_d.velocity[1] * parameters.DT;

            // clamp position to environment boundaries
            new_d.position[0] = math::clamp(device.math, new_d.position[0], T(0), T(parameters.GRID_SIZE_X));
            new_d.position[1] = math::clamp(device.math, new_d.position[1], T(0), T(parameters.GRID_SIZE_Y));

            new_d.acceleration[0] = ax;
            new_d.acceleration[1] = ay;
            new_d.mode = old_d.mode;
            new_d.disaster_detected = local_detect[i];
            // If this agent (and only this agent) sees the disaster now, record its coords:
            if ( new_d.disaster_detected
                 && new_d.mode != DroneMode::RECHARGING)
            {
                new_d.last_detected_disaster_position[0] = next_state.disaster.position[0];
                new_d.last_detected_disaster_position[1] = next_state.disaster.position[1];
            }
            // otherwise leave whatever it had before (including –1,–1 if still never seen)
        }

        // (5) Immediate swap: any drone that reaches 100% battery swaps with lowest-charge active drone
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            const auto &old_d = state.drone_states[i];
            if (old_d.mode != DroneMode::RECHARGING) continue;
            const auto &new_d = next_state.drone_states[i];
            if (new_d.battery >= T(100)) {
                TI low_i = -1;
                T low_b = std::numeric_limits<T>::infinity();
                for (TI j = 0; j < parameters.N_AGENTS; ++j) {
                    const auto &cand = next_state.drone_states[j];
                    if (cand.mode == DroneMode::RECHARGING) continue;
                    if (cand.battery < low_b) {
                        low_b = cand.battery;
                        low_i = j;
                    }
                }
                if (low_i >= 0) {
                    next_state.drone_states[i].mode = DroneMode::NORMAL;
                    next_state.drone_states[low_i].mode = DroneMode::RECHARGING;
                }
            }
        }

        // (7) Advance step count
        next_state.step_count = state.step_count + 1;

        // Return DT value from step function (NOT the reward!)
        return parameters.DT;
    }

    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC>& /*env*/,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& state,
            const Matrix<ACTION_SPEC>& /*action*/,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& next_state,
            RNG &/*rng*/
    ) {
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;

        // Count active drones
        TI active_count = TI(0);
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            if (next_state.drone_states[i].mode != DroneMode::RECHARGING) {
                ++active_count;
            }
        }

        if (active_count == 0) return T(0);

        // Base reward that applies in all situations
        T reward = T(0.1);  // Small positive reward for staying alive

        // DISASTER MODE
        if (next_state.disaster.active) {
            // Track detection and distances
            TI detecting_count = 0;
            T min_distance = std::numeric_limits<T>::max();
            T sum_distances = T(0);

            for (TI i = 0; i < parameters.N_AGENTS; ++i) {
                const auto &d = next_state.drone_states[i];
                if (d.mode == DroneMode::RECHARGING) continue;

                // Calculate distance to disaster
                T dx = d.position[0] - next_state.disaster.position[0];
                T dy = d.position[1] - next_state.disaster.position[1];
                T dist = magnitude(device, dx, dy);

                sum_distances += dist;
                min_distance = std::min(min_distance, dist);

                if (d.disaster_detected) {
                    detecting_count++;
                }
            }

            // Progressive reward for number of drones that detect the disaster
            if (detecting_count > 0) {
                // Linear scaling: more detection = higher reward
                reward += T(0.5) * T(detecting_count);

                // Bonus for ALL active drones detecting
                if (detecting_count == active_count) {
                    reward += T(1.0);
                }

                // Proximity reward - scaled by detection (importance grows with more detection)
                T proximity_factor = T(1.0) - (min_distance / parameters.SENSOR_RANGE);
                proximity_factor = std::max(T(0), proximity_factor); // Clamp to non-negative
                reward += T(1.0) * proximity_factor * T(detecting_count) / T(active_count);

                // Swarm reward - average distance (only if multiple active drones)
                if (active_count > 1 && detecting_count > 1) {
                    T avg_distance = sum_distances / T(active_count);
                    T swarm_factor = T(1.0) - (avg_distance / (T(2.0) * parameters.SENSOR_RANGE));
                    swarm_factor = std::max(T(0), swarm_factor); // Clamp to non-negative
                    reward += T(0.5) * swarm_factor;
                }
            }
        }
            // EXPLORATION MODE
        else {
            // Calculate coverage of important areas
            const T cx = parameters.GRID_SIZE_X / T(2);
            const T cy = parameters.GRID_SIZE_Y / T(2);

            // Track coverage
            bool platform_covered = false;
            bool pipe_h_covered = false;
            bool pipe_v_covered = false;

            // Track spatial distribution
            T min_agent_distance = (active_count > 1) ? std::numeric_limits<T>::max() : T(0);

            for (TI i = 0; i < parameters.N_AGENTS; ++i) {
                const auto &d = next_state.drone_states[i];
                if (d.mode == DroneMode::RECHARGING) continue;

                // Check if drone is in high-priority area
                T x = d.position[0], y = d.position[1];
                T adx = std::abs(x - cx);
                T ady = std::abs(y - cy);

                // Platform coverage
                if (adx <= parameters.PLATFORM_HALF_SIZE && ady <= parameters.PLATFORM_HALF_SIZE) {
                    platform_covered = true;
                }

                // Pipe coverage (simplified to horizontal/vertical)
                if (ady <= parameters.PIPE_WIDTH / 2) pipe_h_covered = true;
                if (adx <= parameters.PIPE_WIDTH / 2) pipe_v_covered = true;

                // Calculate minimum distance between agents (for spatial distribution)
                if (active_count > 1) {
                    for (TI j = i + 1; j < parameters.N_AGENTS; ++j) {
                        const auto &d2 = next_state.drone_states[j];
                        if (d2.mode == DroneMode::RECHARGING) continue;

                        T dx = d.position[0] - d2.position[0];
                        T dy = d.position[1] - d2.position[1];
                        T dist = magnitude(device, dx, dy);

                        min_agent_distance = std::min(min_agent_distance, dist);
                    }
                }
            }

            // Count covered areas
            TI areas_covered = (platform_covered ? 1 : 0) +
                               (pipe_h_covered ? 1 : 0) +
                               (pipe_v_covered ? 1 : 0);

            // Coverage reward
            reward += T(0.2) * T(areas_covered);

            // Spatial distribution reward (if more than one drone active)
            if (active_count > 1) {
                // Optimal separation is around the sensor range
                T optimal_distance = parameters.SENSOR_RANGE;
                T distance_factor = min_agent_distance / optimal_distance;

                // We want to encourage some separation, but not too much
                // Peak reward at optimal distance, less reward for too close or too far
                if (distance_factor > T(1.5)) {
                    // Too far apart
                    reward += T(0.1);
                } else if (distance_factor > T(0.5)) {
                    // Good range
                    reward += T(0.2) * (T(1.0) - std::abs(distance_factor - T(1.0)));
                } else {
                    // Too close
                    reward += T(0.05) * distance_factor;
                }
            }
        }

        return reward;
    }


//    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
//    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(
//            DEVICE &device,
//            const rl::environments::multi_agent::OilPlatform<SPEC>& /*env*/,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& /*state*/,
//            const Matrix<ACTION_SPEC>& /*action*/,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& next_state,
//            RNG &/*rng*/
//    ) {
//        using T  = typename SPEC::T;
//        using TI = typename SPEC::TI;
//
//        // count how many drones are active (not recharging)
//        TI active_count = TI(0);
//        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//            if (next_state.drone_states[i].mode != DroneMode::RECHARGING) {
//                ++active_count;
//            }
//        }
//
//        // no active drones → no penalty
//        if (active_count == TI(0)) {
//            return T(0);
//        }
//
//        // worst‐case single‐drone distance = diagonal of the grid
//        T max_d = magnitude(device, T(parameters.GRID_SIZE_X), T(parameters.GRID_SIZE_Y));
//        // total worst‐case sum = diagonal × active_count
//        T max_sum = max_d * static_cast<T>(active_count);
//
//        // if the disaster hasn't spawned yet, apply maximal normalized penalty
//        if (!next_state.disaster.active) {
//            return T(0);
//        }
//
//        // once active, accumulate each active drone's distance to the true disaster
//        T sum_dist = T(0);
//        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//            const auto &d = next_state.drone_states[i];
//            if (d.mode == DroneMode::RECHARGING) {
//                continue;
//            }
//            if (!d.disaster_detected){
//                sum_dist += max_d;
//            }
//            else{
//                T dx = d.position[0] - next_state.disaster.position[0];
//                T dy = d.position[1] - next_state.disaster.position[1];
//                sum_dist += magnitude(device, dx, dy);
//            }
//        }
//
//        // normalize into [0,1] and return its negative in [0,-1]
//        return - (sum_dist / max_sum) +1;
//    }

//    // 5) Reward function - calculates and returns reward based on state transition
//    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
//    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(
//            DEVICE &device,
//            const rl::environments::multi_agent::OilPlatform<SPEC> &env,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
//            const Matrix<ACTION_SPEC> &action,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &next_state,
//            RNG &rng
//    ) {
//        using T = typename SPEC::T;
//        using TI = typename SPEC::TI;
//
//        T total_reward = T(0);
//        T collective_reward = T(0);
//
////        // One-time major penalty when disaster detection grace period is exceeded
////        if (next_state.disaster.active &&
////            !next_state.drone_states[0].disaster_detected && // Use first drone's disaster_detected flag (they're all the same)
////            next_state.disaster_undetected_steps == parameters.DISASTER_DETECTION_GRACE_PERIOD + 1) {
////            total_reward -= T(15.0);
////        }
//
//        // Calculate rewards for each active drone
//        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//            const auto &d = next_state.drone_states[i];
//            if (d.mode == DroneMode::RECHARGING) continue;
//
//            T agent_reward = T(0);  // Individual agent reward
//
//            // map continuous pos to grid cell
//            TI cx = math::clamp(device.math,
//                                TI(d.position[0] / parameters.GRID_CELL_SIZE),
//                                TI(0), parameters.GRID_SIZE_X - 1);
//            TI cy = math::clamp(device.math,
//                                TI(d.position[1] / parameters.GRID_CELL_SIZE),
//                                TI(0), parameters.GRID_SIZE_Y - 1);
//            TI idx = cx + cy * parameters.GRID_SIZE_X;
//
//            if (next_state.disaster.active) {
//                // Phase 1: disaster tracking reward (individual component)
//                T dx = d.position[0] - next_state.disaster.position[0];
//                T dy = d.position[1] - next_state.disaster.position[1];
//                T dist = math::sqrt(device.math, dx * dx + dy * dy);
//
//                // Individual reward for proximity to disaster
//                agent_reward += std::max(
//                        parameters.DISASTER_PRIORITY * (parameters.SENSOR_RANGE - dist),
//                        T(0)
//                );
//
//                // Add to disaster detection count for collective reward
//                if (dist < parameters.SENSOR_RANGE) {
//                    collective_reward += T(0.5); // Bonus for each additional drone detecting
//                }
//            } else {
//                // Phase 2: continuous exploration (individual component)
////                if (state.step_count - state.last_visit[idx] > parameters.REVISIT_THRESHOLD) {
////                    agent_reward += parameters.EXPLORATION_BONUS;
////                }
//
//                // priority vs general cells
//                T x = d.position[0], y = d.position[1];
//                T adx = std::abs(x - parameters.GRID_SIZE_X / 2);
//                T ady = std::abs(y - parameters.GRID_SIZE_Y / 2);
//                bool inPlat = (adx <= parameters.PLATFORM_HALF_SIZE && ady <= parameters.PLATFORM_HALF_SIZE);
//                bool inPipeH = (ady <= parameters.PIPE_WIDTH / 2 && adx >= parameters.PLATFORM_HALF_SIZE);
//                bool inPipeV = (adx <= parameters.PIPE_WIDTH / 2 && ady >= parameters.PLATFORM_HALF_SIZE);
//
//                if (inPlat || inPipeH || inPipeV) {
//                    agent_reward += parameters.HIGH_PRIORITY_BONUS;
//                } else {
//                    agent_reward += parameters.GENERAL_AREA_BONUS;
//                }
//            }
//
//            // Improved battery management rewards
//            if (d.battery < T(15)) {
//                agent_reward -= T(2.0); // Slightly reduced penalty for low battery
//            } else if (d.battery > T(50)) {
//                agent_reward += T(0.05); // Increased bonus for maintaining good battery levels
//            }
//
//            // Spatial distribution bonus (if not during disaster)
//            if (!next_state.disaster.active) {
//                T min_distance = std::numeric_limits<T>::max();
//                for (TI j = 0; j < parameters.N_AGENTS; ++j) {
//                    if (i == j || next_state.drone_states[j].mode == DroneMode::RECHARGING) continue;
//
//                    T dx = d.position[0] - next_state.drone_states[j].position[0];
//                    T dy = d.position[1] - next_state.drone_states[j].position[1];
//                    T dist = math::sqrt(device.math, dx * dx + dy * dy);
//                    min_distance = std::min(min_distance, dist);
//                }
//                T optimal_distance = T(5.0);
//                agent_reward += T(0.15) * std::min(T(1.0), min_distance / optimal_distance);
//            }
//
//            total_reward += agent_reward;
//        }
//
//        // Add collective component to total reward
//        total_reward += collective_reward;
//
//        return total_reward;
//    }

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
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI D = Observation<OBS_PARAMETERS>::PER_AGENT_DIM;

        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            const auto &d = state.drone_states[i];
            set(observation, 0, i * D + 0, d.position[0]);
            set(observation, 0, i * D + 1, d.position[1]);
            set(observation, 0, i * D + 2, d.velocity[0]);
            set(observation, 0, i * D + 3, d.velocity[1]);
            set(observation, 0, i * D + 4, d.acceleration[0]);
            set(observation, 0, i * D + 5, d.acceleration[1]);
            set(observation, 0, i * D + 6, (T) d.mode);
            set(observation, 0, i * D + 7, d.disaster_detected ? T(1) : T(0));
            set(observation, 0, i * D + 8, state.disaster.active ? state.disaster.position[0] : T(0));
            set(observation, 0, i * D + 9, state.disaster.active ? state.disaster.position[1] : T(0));

            // Uncomment if you have 11 dimensions in your observation
            // set(observation, 0, i*D + 10, d.battery / T(100)); // Normalize to 0-1 range
        }
    }

    // 8) Termination: improved conditions
    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(
            DEVICE & /*device*/,
            const rl::environments::multi_agent::OilPlatform<SPEC> & /*env*/,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            RNG & /*rng*/
    ) {
        // bring in the T and TI types from SPEC:
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;

        // Check for critically low battery (below 5% instead of 0%)
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            if (state.drone_states[i].battery <= T(5) && state.drone_states[i].mode != DroneMode::RECHARGING) {
                return true;
            }
        }

        // Check if disaster left the environment
        if (state.disaster.active) {
            if (state.disaster.position[0] < 0 ||
                state.disaster.position[0] >= parameters.GRID_SIZE_X ||
                state.disaster.position[1] < 0 ||
                state.disaster.position[1] >= parameters.GRID_SIZE_Y) {
                return true;
            }
        }

        // Standard step‐limit check
        return state.step_count >= parameters.EPISODE_STEP_LIMIT;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif  // RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_GENERIC_H