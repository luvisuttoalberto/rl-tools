#pragma once

#include "../../../../version.h"
#include "../../../../rl_tools.h"
#include "../../../../containers/matrix/matrix.h"
#include "../../../../random/operations_generic.h"
#include "oil_platform.h"
#include <array>
#include <vector>
#include <algorithm>

#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || \
     !defined(RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_GENERIC_H)) && \
    (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_GENERIC_H

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    // Pull in the inner-namespace types so we can refer to them unqualified:
//    using rl::environments::multi_agent::oil_platform::DroneMode;
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

//        // Create a vector of indices and shuffle it
//        std::vector<TI> drone_indices(parameters.N_AGENTS);
//        for (TI i = 0; i < parameters.N_AGENTS; i++) {
//            drone_indices[i] = i;
//        }
//        std::shuffle(drone_indices.begin(), drone_indices.end(), rng);

        // Initialize all drone states
        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            auto &d = state.drone_states[i];
            d.position[0] = T(parameters.GRID_SIZE_X / 2);
            d.position[1] = T(parameters.GRID_SIZE_Y / 2);
            d.velocity[0] = T(0);
            d.velocity[1] = T(0);
            d.acceleration[0] = T(0);
            d.acceleration[1] = T(0);

            // Randomly assign modes based on shuffled indices
//            d.mode = (std::find(drone_indices.begin(), drone_indices.begin() + parameters.ACTIVE_DRONES, i) != drone_indices.begin() + parameters.ACTIVE_DRONES)
//                     ? DroneMode::NORMAL : DroneMode::RECHARGING;

            d.battery = T(95);  // Start with higher battery level
            d.is_charging = false;
            d.charging_station_entry_step = -1;
            d.last_detected_disaster_position[0] = T(-1);
            d.last_detected_disaster_position[1] = T(-1);
            d.dead = false;
        }

        state.disaster.active = false;
        state.disaster.position[0] = T(0);
        state.disaster.position[1] = T(0);
        // ADDED: Initialize disaster velocity (will be set randomly when disaster spawns)
        state.disaster.velocity[0] = T(0);
        state.disaster.velocity[1] = T(0);
        state.step_count = 0;
        state.disaster_undetected_steps = 0;
        state.charging_occupancy = 0;
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
            d.battery = T(95);  // Start with higher battery level
            d.is_charging = false;
            d.charging_station_entry_step = -1;
            d.last_detected_disaster_position[0] = T(-1);
            d.last_detected_disaster_position[1] = T(-1);
            d.dead = false;
        }

        state.disaster.active = false;
        state.disaster.position[0] = T(0);
        state.disaster.position[1] = T(0);
        // ADDED: Initialize disaster velocity (will be set randomly when disaster spawns)
        state.disaster.velocity[0] = T(0);
        state.disaster.velocity[1] = T(0);
        state.step_count = 0;
        state.disaster_undetected_steps = 0;
        state.charging_occupancy = 0;
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

    // 4) Step - handles physics and state transitions, returns DT value
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

        // Local variable to track if any drone detects the disaster
        bool any_detection = false;

        // Maximum speed for drones
        const T MAX_SPEED = parameters.MAX_SPEED;

        // (1) Disaster: 2% to start in any HIGH‑PRIORITY area, update position
        const T cx = parameters.GRID_SIZE_X / T(2);
        const T cy = parameters.GRID_SIZE_Y / T(2);

        if (!state.disaster.active) {
            if (state.step_count >= 200 && random::uniform_real_distribution(device.random, T(0), T(1), rng) < T(0.02)) {
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

                // Generate random direction for disaster movement
                T angle = random::uniform_real_distribution(device.random, T(0), T(2) * T(M_PI), rng);
                next_state.disaster.velocity[0] = 0;
                next_state.disaster.velocity[1] = 0;
//                next_state.disaster.velocity[0] = parameters.DISASTER_MAX_SPEED * math::cos(device.math, angle);
//                next_state.disaster.velocity[1] = parameters.DISASTER_MAX_SPEED * math::sin(device.math, angle);
            } else {
                next_state.disaster.active = false;
                next_state.disaster.position[0] = state.disaster.position[0];
                next_state.disaster.position[1] = state.disaster.position[1];
                next_state.disaster.velocity[0] = state.disaster.velocity[0];
                next_state.disaster.velocity[1] = state.disaster.velocity[1];
            }
        } else {
            // Update disaster position based on velocity
            next_state.disaster.active = true;
            next_state.disaster.position[0] = state.disaster.position[0] + state.disaster.velocity[0] * parameters.DT;
            next_state.disaster.position[1] = state.disaster.position[1] + state.disaster.velocity[1] * parameters.DT;
            next_state.disaster.velocity[0] = state.disaster.velocity[0];
            next_state.disaster.velocity[1] = state.disaster.velocity[1];
        }

        // (2) Handle charging station entry/exit tracking and assignment
        const T station_x = T(0);  // Charging station at origin
        const T station_y = T(0);

        next_state.charging_occupancy = 0;

        // Update entry timestamps and handle charge/leave actions
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            // Skip processing if drone is already dead
            if (state.drone_states[i].dead) {
                next_state.drone_states[i].charging_station_entry_step = -1;
                next_state.drone_states[i].is_charging = false;
                continue;
            }

            T dx = state.drone_states[i].position[0] - station_x;
            T dy = state.drone_states[i].position[1] - station_y;
            T dist = magnitude(device, dx, dy);

            bool was_in_range = (state.drone_states[i].charging_station_entry_step != -1);
            bool is_in_range = (dist < parameters.CHARGING_STATION_RANGE);

            // Check if drone wants to charge (action[2] <= 0) or leave (action[2] > 0)
            T charging_action = get(action, 0, i * 3 + 2);
            bool wants_to_charge = (charging_action >= T(0.0));

            if (!was_in_range && is_in_range && wants_to_charge) {
                // Just entered and wants to charge
                next_state.drone_states[i].charging_station_entry_step = state.step_count;
            } else if (was_in_range && (!wants_to_charge || !is_in_range)) {
                // Wants to leave OR physically left the range
                next_state.drone_states[i].charging_station_entry_step = -1;
            } else if (was_in_range && is_in_range && wants_to_charge) {
                // Still in range and wants to keep charging
                next_state.drone_states[i].charging_station_entry_step = state.drone_states[i].charging_station_entry_step;
            } else {
                // Not in range or doesn't want to charge
                next_state.drone_states[i].charging_station_entry_step = -1;
            }
        }

        // Find drones that want to charge and sort by entry time (first-come-first-served)
        std::vector<std::pair<TI, TI>> entry_order; // (entry_step, drone_id)
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            if (next_state.drone_states[i].charging_station_entry_step != -1) {
                entry_order.push_back({next_state.drone_states[i].charging_station_entry_step, i});
            }
        }
        std::sort(entry_order.begin(), entry_order.end()); // Sort by entry step

        // Reset all charging status first
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            next_state.drone_states[i].is_charging = false;
        }

        // Assign first MAX_CHARGING_SLOTS drones to charge
        for (size_t j = 0; j < entry_order.size() && j < static_cast<size_t>(parameters.MAX_CHARGING_SLOTS); ++j) {
            TI drone_id = entry_order[j].second;
            next_state.drone_states[drone_id].is_charging = true;
            next_state.charging_occupancy++;
        }

        // (3) Per‑drone update: battery, dynamics, flags
        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            const auto &old_d = state.drone_states[i];
            auto &new_d = next_state.drone_states[i];

            // MODIFIED: Check if already dead first
            if (old_d.dead) {
                // Dead drone: copy dead state, ensure it stays dead and motionless
                new_d.position[0] = old_d.position[0];
                new_d.position[1] = old_d.position[1];
                new_d.velocity[0] = T(0);
                new_d.velocity[1] = T(0);
                new_d.acceleration[0] = T(0);
                new_d.acceleration[1] = T(0);
                new_d.battery = T(0);
                new_d.dead = true;
                new_d.is_charging = false;
                new_d.charging_station_entry_step = -1;
                continue;
            }

            // Update battery - only charge if assigned to charging slot
            if (new_d.is_charging) {
                new_d.battery = std::min(T(100), old_d.battery + parameters.CHARGING_RATE);
            } else {
                new_d.battery = std::max(T(0), old_d.battery - parameters.DISCHARGE_RATE);
            }

            // ADDED: Check if drone dies after battery update
            if (new_d.battery <= T(0)) {
                new_d.dead = true;
                new_d.position[0] = old_d.position[0];
                new_d.position[1] = old_d.position[1];
                new_d.velocity[0] = T(0);
                new_d.velocity[1] = T(0);
                new_d.acceleration[0] = T(0);
                new_d.acceleration[1] = T(0);
                new_d.battery = T(0);
                new_d.is_charging = false;
                new_d.charging_station_entry_step = -1;
                continue;
            }

            // Movement: restrict movement while charging
            if (new_d.is_charging) {
                // Override actions - force drone to stay still while charging
                new_d.velocity[0] = T(0);
                new_d.velocity[1] = T(0);
                new_d.acceleration[0] = T(0);
                new_d.acceleration[1] = T(0);
                // Keep position unchanged
                new_d.position[0] = old_d.position[0];
                new_d.position[1] = old_d.position[1];
                new_d.dead = false; // Explicitly set alive
            } else {
                T ax = get(action, 0, i * 3 + 0) * parameters.MAX_ACCELERATION;
                T ay = get(action, 0, i * 3 + 1) * parameters.MAX_ACCELERATION;

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
                new_d.dead = false; // Explicitly set alive
            }
        }

        // (4) Check for disaster detection using next_state positions
        if (next_state.disaster.active) {
            for (TI i = 0; i < parameters.N_AGENTS; ++i) {
                const auto &d = next_state.drone_states[i];

                // MODIFIED: Skip dead drones for disaster detection
                if (d.dead) continue;

                // MODIFIED: Use next_state positions for both drone and disaster
                T dx = d.position[0] - next_state.disaster.position[0];
                T dy = d.position[1] - next_state.disaster.position[1];
                T dist = magnitude(device, dx, dy);

                // Check if this drone detects the disaster
                if (dist < parameters.SENSOR_RANGE) {
                    any_detection = true;
                    break;
                }
            }

            // If any drone detects, update all drones' last detected position
            if (any_detection) {
                next_state.disaster_undetected_steps = 0;

                for (TI j = 0; j < parameters.N_AGENTS; ++j) {
                    next_state.drone_states[j].last_detected_disaster_position[0] = next_state.disaster.position[0];
                    next_state.drone_states[j].last_detected_disaster_position[1] = next_state.disaster.position[1];
                }
            } else {
                next_state.disaster_undetected_steps = state.disaster_undetected_steps + 1;
            }
        } else {
            next_state.disaster_undetected_steps = 0;
        }

        // Copy last_detected_disaster_position for drones if no detection occurred
        if (!any_detection) {
            for (TI i = 0; i < parameters.N_AGENTS; ++i) {
                next_state.drone_states[i].last_detected_disaster_position[0] = state.drone_states[i].last_detected_disaster_position[0];
                next_state.drone_states[i].last_detected_disaster_position[1] = state.drone_states[i].last_detected_disaster_position[1];
            }
        }

        // (5) Advance step count
        next_state.step_count = state.step_count + 1;

        // Return DT value from step function
        return parameters.DT;
    }

//    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
//    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(
//            DEVICE &device,
//            const rl::environments::multi_agent::OilPlatform<SPEC>& /*env*/,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& state,
//            const Matrix<ACTION_SPEC>& /*action*/,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& next_state,
//            RNG &/*rng*/
//    ) {
//        using T = typename SPEC::T;
//        using TI = typename SPEC::TI;
//
//        TI active_count = 0;
//        T total_reward = T(0);
//        T death_penalty = T(0);
//
//        // Count active agents and apply death penalty
//        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//            const auto &d = next_state.drone_states[i];
//            const auto &prev_d = state.drone_states[i];
//
//            if (!d.dead) {
//                active_count++;
//            } else if (!prev_d.dead && d.dead) {
//                // KEPT: One-time death penalty when drone just died
//                death_penalty -= T(10.0);
//            }
//        }
//
//        // KEPT: No change - multiple death penalties in same step are intentional
//        // If no agents alive, return large negative reward
////        if (active_count == 0) {
////            return T(-100.0);
////        }
//
//        // MODIFIED: Charging inefficiency penalty - now with smooth curve instead of hard threshold
//        T charging_inefficiency_penalty = T(0);
//
//        // MODIFIED: Check if ANY living drone has detected disaster before (was only checking drone 0)
//        bool disaster_known_to_agents = false;
//        if (active_count > 0) {
//            // Check if any living drone has detected a disaster before (last_detected_position != (-1,-1))
//            for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//                if (!next_state.drone_states[i].dead &&
//                    next_state.drone_states[i].last_detected_disaster_position[0] > T(-1)) {
//                    disaster_known_to_agents = true;
//                    break;
//                }
//            }
//        }
//
//        // Check if any drone currently detects the disaster
//        TI detecting_count = 0;
//        std::vector<bool> detecting(active_count, false);
//        std::vector<T> distances(active_count, std::numeric_limits<T>::max());
//        std::vector<TI> detecting_indices;
//
//        if (next_state.disaster.active) {
//            for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//                const auto &d = next_state.drone_states[i];
//                if (d.dead) continue;
//
//                T dx = d.position[0] - next_state.disaster.position[0];
//                T dy = d.position[1] - next_state.disaster.position[1];
//                T dist = magnitude(device, dx, dy);
//                distances[i] = dist;
//
//                if (dist < parameters.SENSOR_RANGE) {
//                    detecting[i] = true;
//                    detecting_count++;
//                    detecting_indices.push_back(i);
//                }
//
//                // MODIFIED: Progressive charging penalty instead of hard threshold
//                if (d.is_charging && d.battery >= T(90)) {
//                    // Smooth penalty that increases from 90% to 100%
//                    T excess = (d.battery - T(90)) / T(10);  // 0 to 1 range
//                    T penalty_factor = excess * excess;  // Quadratic growth
//                    charging_inefficiency_penalty += T(0.5) * penalty_factor;  // Base penalty
//
//                    // Extra penalty if agents know about a disaster (either currently detected or previously known)
//                    if (disaster_known_to_agents) {
//                        charging_inefficiency_penalty += T(0.5) * penalty_factor;  // Additional penalty when disaster is known
//                    }
//                }
//            }
//        }
//        total_reward -= charging_inefficiency_penalty;
//
//        // DISASTER DETECTION MODE - when disaster is detected
//        if (detecting_count > 0) {
//            // Base reward for detection
//            total_reward += T(1.0) * T(detecting_count);
//
//            // MODIFIED: Consolidated proximity rewards to avoid double-counting
//            for (TI i = 0; i < active_count; ++i) {
//                if (detecting[i]) {
//                    T norm_dist = distances[i] / parameters.SENSOR_RANGE;
//                    T proximity_factor = T(1.0) - norm_dist;
//
//                    // Steep reward for being close (cubic for strong gradient)
//                    total_reward += T(3.0) * proximity_factor * proximity_factor * proximity_factor;
//
//                    // Add significant bonus for being at least halfway inside the detection radius
//                    if (norm_dist < 0.5) {
//                        total_reward += T(1.5);
//                    }
//
//                    // Bonus for being in optimal monitoring zone (70-90% of sensor range)
//                    if (norm_dist >= T(0.7) && norm_dist <= T(0.9)) {
//                        total_reward += T(1.0);  // Sweet spot bonus
//                    }
//                }
//            }
//
//            // Strong bonus for complete swarm detection
//            if (detecting_count == active_count) {
//                total_reward += T(5.0);
//            } else {
//                T missing_ratio = T(active_count - detecting_count) / T(active_count);
//                total_reward -= T(3.0) * missing_ratio;
//            }
//
//            // REMOVED: Old separate proximity rewards section to avoid double-counting
//
//            // Very close bonus
//            for (TI i = 0; i < active_count; ++i) {
//                if (distances[i] < parameters.SENSOR_RANGE * T(0.3)) {
//                    total_reward += T(1.0);
//                }
//            }
//
//            // MODIFIED: Formation reward for 360-degree coverage with fixed angle calculation
//            if (detecting_count >= 2) {
//                T formation_reward = T(0);
//
//                // Calculate angles of detecting drones relative to disaster
//                std::vector<T> angles;
//                for (TI idx : detecting_indices) {
//                    const auto &d = next_state.drone_states[idx];
//                    T dx = d.position[0] - next_state.disaster.position[0];
//                    T dy = d.position[1] - next_state.disaster.position[1];
//                    T angle = math::atan2(device.math, dy, dx);
//                    // Normalize angle to [0, 2π)
//                    if (angle < T(0)) {
//                        angle += T(2) * T(M_PI);
//                    }
//                    angles.push_back(angle);
//                }
//
//                // Sort angles for better analysis
//                std::sort(angles.begin(), angles.end());
//
//                // Calculate angular separation between consecutive drones
//                T ideal_separation = T(2) * T(M_PI) / T(detecting_count);
//                T total_angle_error = T(0);
//
//                for (size_t i = 0; i < angles.size(); ++i) {
//                    size_t next_i = (i + 1) % angles.size();
//                    T actual_separation;
//
//                    // FIXED: Proper wraparound handling for angle calculation
//                    if (next_i == 0) {
//                        // Wraparound case: from last angle to first angle
//                        actual_separation = (T(2) * T(M_PI) + angles[0]) - angles[i];
//                    } else {
//                        actual_separation = angles[next_i] - angles[i];
//                    }
//
//                    // Calculate error from ideal separation
//                    T error = std::abs(actual_separation - ideal_separation);
//                    total_angle_error += error;
//                }
//
//                // Reward for good angular distribution (lower error = higher reward)
//                T max_error = T(2) * T(M_PI);  // Maximum possible error
//                T normalized_error = total_angle_error / max_error;
//                formation_reward = T(3.0) * (T(1.0) - normalized_error);
//
//                // Additional reward for maintaining optimal distance while in formation
//                T distance_consistency_reward = T(0);
//                if (detecting_count >= 2) {
//                    T ideal_radius = parameters.SENSOR_RANGE * T(0.75);  // Optimal monitoring distance
//                    T distance_variance = T(0);
//
//                    for (TI idx : detecting_indices) {
//                        T deviation = std::abs(distances[idx] - ideal_radius);
//                        distance_variance += deviation;
//                    }
//
//                    T avg_deviation = distance_variance / T(detecting_count);
//                    T max_deviation = parameters.SENSOR_RANGE;
//                    T normalized_deviation = avg_deviation / max_deviation;
//                    distance_consistency_reward = T(1.5) * (T(1.0) - normalized_deviation);
//                }
//
//                total_reward += formation_reward + distance_consistency_reward;
//            }
//        }
//            // EXPLORATION MODE - no disaster detected (regardless of disaster.active)
//        else {
//            const T cx = parameters.GRID_SIZE_X / T(2);
//            const T cy = parameters.GRID_SIZE_Y / T(2);
//
//            // Check if disaster has been detected before (shared knowledge)
////            bool disaster_previously_detected = false;
////            if (active_count > 0) {
////                disaster_previously_detected = (next_state.drone_states[0].last_detected_disaster_position[0] > T(-1));
////            }
//
//            if (disaster_known_to_agents && next_state.disaster.active) {
//                // CASE: Disaster active but not currently detected, was detected before
//                // Encourage moving toward last known position and forming search pattern
//                T total_approach_reward = T(0);
//
//                // MODIFIED: Use first living drone's last known position (since all should be the same)
//                T last_pos_x = T(-1);
//                T last_pos_y = T(-1);
//                for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//                    if (!next_state.drone_states[i].dead) {
//                        last_pos_x = next_state.drone_states[i].last_detected_disaster_position[0];
//                        last_pos_y = next_state.drone_states[i].last_detected_disaster_position[1];
//                        break;
//                    }
//                }
//
//                for (TI i = 0; i < active_count; ++i) {
//                    const auto &d = next_state.drone_states[i];
//                    const auto &old_d = state.drone_states[i];
//                    if (d.dead) continue;
//
//                    T current_dist = magnitude(device,
//                                               d.position[0] - last_pos_x,
//                                               d.position[1] - last_pos_y);
//
//                    T old_dist = magnitude(device,
//                                           old_d.position[0] - last_pos_x,
//                                           old_d.position[1] - last_pos_y);
//
//                    // Reward for moving toward last known position
//                    if (current_dist < old_dist) {
//                        total_approach_reward += T(0.8);
//                    }
//
//                    // Reward for being spread around the last known position (search formation)
//                    for (TI j = i + 1; j < active_count; ++j) {
//                        T drone_distance = magnitude(device,
//                                                     d.position[0] - next_state.drone_states[j].position[0],
//                                                     d.position[1] - next_state.drone_states[j].position[1]);
//
//                        if (drone_distance >= parameters.SENSOR_RANGE * T(0.8) &&
//                            drone_distance <= parameters.SENSOR_RANGE * T(1.5)) {
//                            total_approach_reward += T(0.3);
//                        }
//                    }
//                }
//
//                total_reward += total_approach_reward;
//            } else {
//// CASE: No disaster detected and never detected, OR disaster not active
//// OPTIMIZED: More efficient exploration reward using sampling instead of full grid
//
//                const T cx = parameters.GRID_SIZE_X / T(2);
//                const T cy = parameters.GRID_SIZE_Y / T(2);
//
//                // Define high-priority areas with their boundaries
//                // Platform center
//                T platform_min_x = cx - parameters.PLATFORM_HALF_SIZE;
//                T platform_max_x = cx + parameters.PLATFORM_HALF_SIZE;
//                T platform_min_y = cy - parameters.PLATFORM_HALF_SIZE;
//                T platform_max_y = cy + parameters.PLATFORM_HALF_SIZE;
//
//                // Horizontal pipes
//                T pipe_h_min_y = cy - parameters.PIPE_WIDTH / T(2);
//                T pipe_h_max_y = cy + parameters.PIPE_WIDTH / T(2);
//                // Vertical pipes
//                T pipe_v_min_x = cx - parameters.PIPE_WIDTH / T(2);
//                T pipe_v_max_x = cx + parameters.PIPE_WIDTH / T(2);
//
//                // OPTIMIZED: Use strategic sampling instead of full grid enumeration
//                const TI num_sample_points = 50;  // Much more efficient than 400 grid cells
//                TI coverage_counts[5] = {0, 0, 0, 0, 0};  // platform, pipe_h_left, pipe_h_right, pipe_v_top, pipe_v_bottom
//                TI total_counts[5] = {0, 0, 0, 0, 0};
//
//                for (TI sample = 0; sample < num_sample_points; ++sample) {
//                    T sample_x, sample_y;
//                    TI area_type = sample % 5;  // Rotate through areas
//
//                    // Generate sample point based on area type
//                    switch (area_type) {
//                        case 0: // Platform
//                            sample_x = platform_min_x + (platform_max_x - platform_min_x) * T(sample % 10) / T(9);
//                            sample_y = platform_min_y + (platform_max_y - platform_min_y) * T((sample / 10) % 10) / T(9);
//                            break;
//                        case 1: // Horizontal pipe left
//                            sample_x = T(0) + (cx - parameters.PLATFORM_HALF_SIZE) * T(sample % 10) / T(9);
//                            sample_y = pipe_h_min_y + (pipe_h_max_y - pipe_h_min_y) * T((sample / 10) % 10) / T(9);
//                            break;
//                        case 2: // Horizontal pipe right
//                            sample_x = (cx + parameters.PLATFORM_HALF_SIZE) + (T(parameters.GRID_SIZE_X) - cx - parameters.PLATFORM_HALF_SIZE) * T(sample % 10) / T(9);
//                            sample_y = pipe_h_min_y + (pipe_h_max_y - pipe_h_min_y) * T((sample / 10) % 10) / T(9);
//                            break;
//                        case 3: // Vertical pipe top
//                            sample_x = pipe_v_min_x + (pipe_v_max_x - pipe_v_min_x) * T(sample % 10) / T(9);
//                            sample_y = (cy + parameters.PLATFORM_HALF_SIZE) + (T(parameters.GRID_SIZE_Y) - cy - parameters.PLATFORM_HALF_SIZE) * T((sample / 10) % 10) / T(9);
//                            break;
//                        case 4: // Vertical pipe bottom
//                            sample_x = pipe_v_min_x + (pipe_v_max_x - pipe_v_min_x) * T(sample % 10) / T(9);
//                            sample_y = T(0) + (cy - parameters.PLATFORM_HALF_SIZE) * T((sample / 10) % 10) / T(9);
//                            break;
//                    }
//
//                    total_counts[area_type]++;
//
//                    // Check if any drone covers this sample point
//                    bool covered = false;
//                    for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//                        if (next_state.drone_states[i].dead) continue;
//
//                        T dist = magnitude(device,
//                                           next_state.drone_states[i].position[0] - sample_x,
//                                           next_state.drone_states[i].position[1] - sample_y);
//
//                        if (dist < parameters.SENSOR_RANGE) {
//                            covered = true;
//                            break;
//                        }
//                    }
//
//                    if (covered) {
//                        coverage_counts[area_type]++;
//                    }
//                }
//
//                // Calculate coverage ratios for each area
//                T platform_coverage = (total_counts[0] > 0) ? T(coverage_counts[0]) / T(total_counts[0]) : T(0);
//                T pipe_h_left_coverage = (total_counts[1] > 0) ? T(coverage_counts[1]) / T(total_counts[1]) : T(0);
//                T pipe_h_right_coverage = (total_counts[2] > 0) ? T(coverage_counts[2]) / T(total_counts[2]) : T(0);
//                T pipe_v_top_coverage = (total_counts[3] > 0) ? T(coverage_counts[3]) / T(total_counts[3]) : T(0);
//                T pipe_v_bottom_coverage = (total_counts[4] > 0) ? T(coverage_counts[4]) / T(total_counts[4]) : T(0);
//
//                // Calculate total priority area coverage
//                TI total_priority_samples = total_counts[0] + total_counts[1] + total_counts[2] + total_counts[3] + total_counts[4];
//                TI total_covered_samples = coverage_counts[0] + coverage_counts[1] + coverage_counts[2] + coverage_counts[3] + coverage_counts[4];
//
//                T overall_coverage = (total_priority_samples > 0) ? T(total_covered_samples) / T(total_priority_samples) : T(0);
//
//                // Assign rewards based on coverage ratios
//                total_reward += T(3.0) * overall_coverage;  // Overall coverage reward
//
//                // Additional rewards for individual areas
//                total_reward += T(0.8) * platform_coverage;     // Platform is most important
//                total_reward += T(0.6) * pipe_h_left_coverage;  // Pipe sections each get 0.6
//                total_reward += T(0.6) * pipe_h_right_coverage;
//                total_reward += T(0.6) * pipe_v_top_coverage;
//                total_reward += T(0.6) * pipe_v_bottom_coverage;
//            }
//        }
//
//        // Termination penalties (consistent with before)
////        if (next_state.disaster.active && next_state.disaster_undetected_steps >= parameters.DISASTER_DETECTION_TIMEOUT) {
////            total_reward -= T(10.0);
////        }
//
//        if (next_state.disaster.active) {
//            if (next_state.disaster.position[0] < 0 ||
//                next_state.disaster.position[0] >= parameters.GRID_SIZE_X ||
//                next_state.disaster.position[1] < 0 ||
//                next_state.disaster.position[1] >= parameters.GRID_SIZE_Y) {
//
//                if (detecting_count > 0) {
//                    total_reward += T(2.0);
//                } else {
//                    total_reward -= T(5.0);
//                }
//            }
//        }
//
//        // KEPT: Return total reward without normalization to maintain "stay alive" incentive
//        return total_reward + death_penalty;
//    }


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

        T total_reward = T(0);
        TI living_drones = 0;
        TI detecting_drones = 0;
        TI appropriately_charging = 0;
        TI overcharging = 0;

        // Count living drones and their status
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            const auto &d = next_state.drone_states[i];

            if (!d.dead) {
                living_drones++;

                // Check if appropriately charging (low battery)
                if (d.is_charging && d.battery < T(90)) {
                    appropriately_charging++;
                }
                    // Check if overcharging (high battery while charging)
                else if (d.is_charging && d.battery >= T(90)) {
                    overcharging++;
                }
                    // Check disaster detection for non-charging drones
                else if (!d.is_charging && next_state.disaster.active) {
                    T dx = d.position[0] - next_state.disaster.position[0];
                    T dy = d.position[1] - next_state.disaster.position[1];
                    T dist = magnitude(device, dx, dy);

                    if (dist < parameters.SENSOR_RANGE) {
                        detecting_drones++;
                    }
                }
            }
        }

        // POSITIVE REWARDS: Reward good behavior

        // 1. Base survival reward - reward for keeping drones alive
        total_reward += T(living_drones) * T(0.1);  // Small positive reward per living drone

        // 2. Disaster detection rewards
        if (next_state.disaster.active && detecting_drones > 0) {
            // Base detection reward
            total_reward += T(detecting_drones) * T(2.0);

            // Bonus for multiple drones detecting (teamwork)
            if (detecting_drones > 1) {
                total_reward += T(detecting_drones - 1) * T(1.0);
            }

            // Extra bonus for full team detection
            TI non_charging_drones = living_drones - appropriately_charging - overcharging;
            if (detecting_drones == non_charging_drones && non_charging_drones > 0) {
                total_reward += T(3.0);
            }
        }

        // 3. Smart charging reward - encourage charging when battery is low
        total_reward += T(appropriately_charging) * T(0.5);

        // 4. Exploration reward - encourage area coverage when not detecting disaster
        if (detecting_drones == 0) {  // Only based on what agents can observe
            // Simple reward for drones being spread out (not clustering)
            T spread_reward = T(0);
            TI active_drones = living_drones - appropriately_charging - overcharging;

            if (active_drones > 1) {
                // Reward based on how spread out the active drones are
                T min_distance = std::numeric_limits<T>::max();
                for (TI i = 0; i < parameters.N_AGENTS; ++i) {
                    if (next_state.drone_states[i].dead ||
                        next_state.drone_states[i].is_charging) continue;

                    for (TI j = i + 1; j < parameters.N_AGENTS; ++j) {
                        if (next_state.drone_states[j].dead ||
                            next_state.drone_states[j].is_charging) continue;

                        T dx = next_state.drone_states[i].position[0] - next_state.drone_states[j].position[0];
                        T dy = next_state.drone_states[i].position[1] - next_state.drone_states[j].position[1];
                        T dist = magnitude(device, dx, dy);
                        min_distance = std::min(min_distance, dist);
                    }
                }

                // Reward if drones maintain good separation
                if (min_distance > parameters.SENSOR_RANGE * T(0.5)) {
                    spread_reward = T(1.0);
                }
            }
            total_reward += spread_reward;
        }

        // NEGATIVE PENALTIES: Only penalize clearly bad behavior

        // 1. Death penalty - only when drones actually die this step
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            if (!state.drone_states[i].dead && next_state.drone_states[i].dead) {
                total_reward -= T(5.0);  // One-time death penalty
            }
        }

        // 2. Overcharging penalty - wasting time charging when battery is full
        total_reward -= T(overcharging) * T(1.0);

        // 3. Mission failure penalty - disaster undetected for too long
        if (next_state.disaster.active && detecting_drones == 0) {
            // Escalating penalty for not detecting disaster
            T undetected_penalty = T(next_state.disaster_undetected_steps) * T(0.1);
            total_reward -= std::min(undetected_penalty, T(3.0));  // Cap the penalty
        }

        return total_reward;
    }

//    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
//    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(
//            DEVICE &device,
//            const rl::environments::multi_agent::OilPlatform<SPEC>& /*env*/,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& state,
//            const Matrix<ACTION_SPEC>& /*action*/,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& next_state,
//            RNG &/*rng*/
//    ) {
//        using T = typename SPEC::T;
//        using TI = typename SPEC::TI;
//
//        T total_reward = T(0);
//
//        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//            const auto &d = next_state.drone_states[i];
//
//            if (d.dead) {
//                // Dead drone: always -1 penalty (counts as "not detecting")
//                total_reward -= T(2);
//            } else if (!d.is_charging) {
//                if (next_state.disaster.active){
//                    // Living drone: check if detecting
//                    T dx = d.position[0] - next_state.disaster.position[0];
//                    T dy = d.position[1] - next_state.disaster.position[1];
//                    T dist = magnitude(device, dx, dy);
//
//                    if (dist >= parameters.SENSOR_RANGE) {
//                        // Not detecting: -1 penalty
//                        total_reward -= T(1);
//                    }
//                } else {
//                    total_reward -= T(1);
//                }
//            }
//            if (d.is_charging && d.battery >=90){
//                total_reward -= T(1);
//            }
//        }
//
//        // Optional: One-time death penalty for immediate death
//        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//            if (!state.drone_states[i].dead && next_state.drone_states[i].dead) {
//                total_reward -= T(2);  // Extra penalty for dying
//            }
//        }
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

        // Normalization constants - optimized for SAC + FAST_TANH
        const T pos_norm_x = T(2.0) / T(parameters.GRID_SIZE_X);  // [0, GRID_SIZE] -> [-1, 1]
        const T pos_norm_y = T(2.0) / T(parameters.GRID_SIZE_Y);
        const T vel_norm = T(1.0) / parameters.MAX_SPEED;  // Use parameter for velocity normalization

        // Charging station position (always at origin)
        const T charging_station_x_norm = (T(2.0) * T(0) / T(parameters.GRID_SIZE_X)) - T(1.0);  // = -1
        const T charging_station_y_norm = (T(2.0) * T(0) / T(parameters.GRID_SIZE_Y)) - T(1.0);  // = -1

        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            const auto &d = state.drone_states[i];

            if (d.dead) {
                // Dead drone observations - maintain position info but zero dynamics
                set(observation, 0, i * D + 0, (T(2.0) * d.position[0] / T(parameters.GRID_SIZE_X)) - T(1.0));
                set(observation, 0, i * D + 1, (T(2.0) * d.position[1] / T(parameters.GRID_SIZE_Y)) - T(1.0));
                set(observation, 0, i * D + 2, T(0));  // velocity x = 0
                set(observation, 0, i * D + 3, T(0));  // velocity y = 0
                set(observation, 0, i * D + 4, T(-1)); // battery = minimum (dead)
                set(observation, 0, i * D + 5, T(-1)); // disaster_detected = false
                set(observation, 0, i * D + 6, T(-2));  // no valid last detected position (out of [-1,1] range)
                set(observation, 0, i * D + 7, T(-2));
                set(observation, 0, i * D + 8, charging_station_x_norm);
                set(observation, 0, i * D + 9, charging_station_y_norm);
                set(observation, 0, i * D + 10, T(-1)); // is_charging = false
                set(observation, 0, i * D + 11, T(-1)); // dead = true
                continue;
            }

            // Living agents - all features normalized to [-1, 1] for optimal SAC performance

            // Positions: [0, GRID_SIZE] -> [-1, 1]
            set(observation, 0, i * D + 0, (pos_norm_x * d.position[0]) - T(1.0));
            set(observation, 0, i * D + 1, (pos_norm_y * d.position[1]) - T(1.0));

            // Velocities: [-2, 2] -> [-1, 1] (already symmetric)
            set(observation, 0, i * D + 2, d.velocity[0] * vel_norm);
            set(observation, 0, i * D + 3, d.velocity[1] * vel_norm);

            // Battery: [0, 100] -> [-1, 1]
            set(observation, 0, i * D + 4, (T(2.0) * d.battery / T(100.0)) - T(1.0));

            // Disaster detection: already optimal [-1, 1]
            bool is_detecting = false;
            if (state.disaster.active) {
                T dx = d.position[0] - state.disaster.position[0];
                T dy = d.position[1] - state.disaster.position[1];
                T dist = magnitude(device, dx, dy);
                is_detecting = (dist < parameters.SENSOR_RANGE);
            }
            set(observation, 0, i * D + 5, is_detecting ? T(1) : T(-1));

            // Last detected disaster position
            if (d.last_detected_disaster_position[0] > T(-1)) {
                // Valid position: normalize to [-1, 1]
                set(observation, 0, i * D + 6, (pos_norm_x * d.last_detected_disaster_position[0]) - T(1.0));
                set(observation, 0, i * D + 7, (pos_norm_y * d.last_detected_disaster_position[1]) - T(1.0));
            } else {
                // Invalid position: use value outside [-1,1] range to indicate "unknown"
                set(observation, 0, i * D + 6, T(-2));
                set(observation, 0, i * D + 7, T(-2));
            }

            // Charging station position: normalized to [-1, 1]
            set(observation, 0, i * D + 8, charging_station_x_norm);
            set(observation, 0, i * D + 9, charging_station_y_norm);

            // Boolean flags: already optimal [-1, 1]
            set(observation, 0, i * D + 10, d.is_charging ? T(1) : T(-1));
            set(observation, 0, i * D + 11, T(1)); // alive
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

        // Check if all drones are dead
        bool any_alive = false;
        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
            if (state.drone_states[i].battery > T(0)) {
                any_alive = true;
                break;
            }
        }
        if (!any_alive) {
            return true;
        }

        // Check if disaster has been undetected for too long
//        if (state.disaster.active && state.disaster_undetected_steps >= parameters.DISASTER_DETECTION_TIMEOUT) {
//            return true;
//        }

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