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

        // Create a vector of indices and shuffle it
        std::vector<TI> drone_indices(parameters.N_AGENTS);
        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            drone_indices[i] = i;
        }
        std::shuffle(drone_indices.begin(), drone_indices.end(), rng);

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
            d.last_detected_disaster_position[0] = T(-1);
            d.last_detected_disaster_position[1] = T(-1);
        }

        state.disaster.active = false;
        state.disaster.position[0] = T(0);
        state.disaster.position[1] = T(0);
        // ADDED: Initialize disaster velocity (will be set randomly when disaster spawns)
        state.disaster.velocity[0] = T(0);
        state.disaster.velocity[1] = T(0);
        state.step_count = 0;
        state.disaster_undetected_steps = 0;
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
//            d.mode = (i < parameters.ACTIVE_DRONES ? DroneMode::NORMAL : DroneMode::RECHARGING);
            d.battery = T(95);  // Start with higher battery level
            d.last_detected_disaster_position[0] = T(-1);
            d.last_detected_disaster_position[1] = T(-1);
        }

        state.disaster.active = false;
        state.disaster.position[0] = T(0);
        state.disaster.position[1] = T(0);
        // ADDED: Initialize disaster velocity (will be set randomly when disaster spawns)
        state.disaster.velocity[0] = T(0);
        state.disaster.velocity[1] = T(0);
        state.step_count = 0;
        state.disaster_undetected_steps = 0;
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

        // Local variable to track if any drone detects the disaster
        bool any_detection = false;

        // Maximum speed for drones
        const T MAX_SPEED = T(2.0);

        // (1) Disaster: 2% to start in any HIGH‑PRIORITY area, remains static once created
        {
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

                    // ADDED: Generate random direction for disaster movement
                    T angle = random::uniform_real_distribution(device.random, T(0), T(2) * T(M_PI), rng);
                    next_state.disaster.velocity[0] = parameters.DISASTER_MAX_SPEED * math::cos(device.math, angle);
                    next_state.disaster.velocity[1] = parameters.DISASTER_MAX_SPEED * math::sin(device.math, angle);
                } else {
                    next_state.disaster.active = false;
                    next_state.disaster.position[0] = state.disaster.position[0];
                    next_state.disaster.position[1] = state.disaster.position[1];
                    next_state.disaster.velocity[0] = state.disaster.velocity[0];
                    next_state.disaster.velocity[1] = state.disaster.velocity[1];
                }
            } else {
                // MODIFIED: Update disaster position based on velocity
                next_state.disaster.active = true;
                next_state.disaster.position[0] = state.disaster.position[0] + state.disaster.velocity[0] * parameters.DT;
                next_state.disaster.position[1] = state.disaster.position[1] + state.disaster.velocity[1] * parameters.DT;
                next_state.disaster.velocity[0] = state.disaster.velocity[0];
                next_state.disaster.velocity[1] = state.disaster.velocity[1];

            }

            // Check for disaster detection
            if (next_state.disaster.active) {
                for (TI i = 0; i < parameters.N_AGENTS; ++i) {
                    const auto &d = state.drone_states[i];
//                    if (d.mode == DroneMode::RECHARGING) continue;

                    // Calculate distance to disaster
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
                } else{
                    next_state.disaster_undetected_steps = state.disaster_undetected_steps + 1;
                }
            } else{
                next_state.disaster_undetected_steps = 0;
            }
        }

        // (4) Per‑drone update: battery, dynamics, flags
        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            const auto &old_d = state.drone_states[i];
            auto &new_d = next_state.drone_states[i];

            // Improved battery management
            // Switch to RECHARGING mode when battery gets low
//            if (old_d.mode != DroneMode::RECHARGING && old_d.battery < T(40)) {
//                new_d.mode = DroneMode::RECHARGING;
//            }

            // Handle recharging and battery updates
//            if (old_d.mode == DroneMode::RECHARGING) {
//                // Check if the drone is at the recharging station (center of the grid)
//                const T chargeX = T(0);
//                const T chargeY = T(0);
//                const T dx = old_d.position[0] - chargeX;
//                const T dy = old_d.position[1] - chargeY;
//                const T distance = math::sqrt(device.math, dx * dx + dy * dy);
//
//                // Only recharge if the drone is close enough to the center
//                if (distance < 1.0) {
//                    new_d.battery = std::min(T(100), old_d.battery + parameters.RECHARGE_RATE);
//                } else {
//                    // Still discharge but at a lower rate when in RECHARGING mode
//                    new_d.battery = std::max(T(0), old_d.battery - parameters.DISCHARGE_RATE * T(0.5));
//                }
//            } else {
//                new_d.battery = std::max(T(0), old_d.battery - parameters.DISCHARGE_RATE);
//            }
            new_d.battery = old_d.battery;
            // Calculate acceleration based on drone mode and current state
            T ax = T(0), ay = T(0);

//            if (old_d.mode == DroneMode::RECHARGING) {
//                // Get vector to base station (center of grid)
//                T chargeX = T(0);
//                T chargeY = T(0);
//                T dx = chargeX - old_d.position[0];
//                T dy = chargeY - old_d.position[1];
//                T dist = math::sqrt(device.math, dx * dx + dy * dy) + 1e-6f;
//
//                if (dist < 0.5) {
//                    // When very close to base, apply strong braking force
//                    new_d.velocity[0] = old_d.velocity[0] * T(0.3); // Damping factor
//                    new_d.velocity[1] = old_d.velocity[1] * T(0.3);
//
//                    if (math::sqrt(device.math, new_d.velocity[0]*new_d.velocity[0] + new_d.velocity[1]*new_d.velocity[1]) < T(0.1)) {
//                        new_d.velocity[0] = T(0);
//                        new_d.velocity[1] = T(0);
//                    }
//                }
//                else if (dist < 2.0) {
//                    T approach_speed = std::min(dist * T(0.8), T(1.0));
//                    T target_vx = dx / dist * approach_speed;
//                    T target_vy = dy / dist * approach_speed;
//                    ax = (target_vx - old_d.velocity[0]) / parameters.DT * T(0.7);
//                    ay = (target_vy - old_d.velocity[1]) / parameters.DT * T(0.7);
//                }
//                else {
//                    T approach_speed = std::min(T(2.0), dist * T(0.2));
//                    T target_vx = dx / dist * approach_speed;
//                    T target_vy = dy / dist * approach_speed;
//                    ax = (target_vx - old_d.velocity[0]) / parameters.DT * T(0.7);
//                    ay = (target_vy - old_d.velocity[1]) / parameters.DT * T(0.7);
//                }
//
//                // Limit acceleration magnitude to MAX_ACCELERATION
//                T accel_mag = magnitude(device, ax, ay);
//                if (accel_mag > parameters.MAX_ACCELERATION) {
//                    T scale = parameters.MAX_ACCELERATION / accel_mag;
//                    ax *= scale;
//                    ay *= scale;
//                }
//            } else {
            ax = get(action, 0, i * 2 + 0) * parameters.MAX_ACCELERATION;
            ay = get(action, 0, i * 2 + 1) * parameters.MAX_ACCELERATION;
//            }

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
//            new_d.mode = old_d.mode;

            // Copy last_detected_disaster_position if it wasn't updated above
            if (!any_detection) {
                new_d.last_detected_disaster_position[0] = old_d.last_detected_disaster_position[0];
                new_d.last_detected_disaster_position[1] = old_d.last_detected_disaster_position[1];
            }
        }

        // (5) Immediate swap: any drone that reaches 100% battery swaps with lowest-charge active drone
//        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//            const auto &old_d = state.drone_states[i];
//            if (old_d.mode != DroneMode::RECHARGING) continue;
//            const auto &new_d = next_state.drone_states[i];
//            if (new_d.battery >= T(100)) {
//                TI low_i = -1;
//                T low_b = std::numeric_limits<T>::infinity();
//                for (TI j = 0; j < parameters.N_AGENTS; ++j) {
//                    const auto &cand = next_state.drone_states[j];
//                    if (cand.mode == DroneMode::RECHARGING) continue;
//                    if (cand.battery < low_b) {
//                        low_b = cand.battery;
//                        low_i = j;
//                    }
//                }
//                if (low_i >= 0) {
//                    next_state.drone_states[i].mode = DroneMode::NORMAL;
//                    next_state.drone_states[low_i].mode = DroneMode::RECHARGING;
//                }
//            }
//        }

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

        TI active_count = parameters.N_AGENTS;
        T total_reward = T(0);

        // Check if any drone currently detects the disaster
        TI detecting_count = 0;
        std::vector<bool> detecting(active_count, false);
        std::vector<T> distances(active_count, std::numeric_limits<T>::max());
        std::vector<TI> detecting_indices;

        if (next_state.disaster.active) {
            for (TI i = 0; i < active_count; ++i) {
                const auto &d = next_state.drone_states[i];
                T dx = d.position[0] - next_state.disaster.position[0];
                T dy = d.position[1] - next_state.disaster.position[1];
                T dist = magnitude(device, dx, dy);
                distances[i] = dist;

                if (dist < parameters.SENSOR_RANGE) {
                    detecting[i] = true;
                    detecting_count++;
                    detecting_indices.push_back(i);
                }
            }
        }

        // DISASTER DETECTION MODE - when disaster is detected
        if (detecting_count > 0) {
            // Base reward for detection
            total_reward += T(1.0) * T(detecting_count);

            // Add the gradient improvement here:
            // Stronger rewards for being deeper in the detection zone
            for (TI i = 0; i < active_count; ++i) {
                if (detecting[i]) {
                    T norm_dist = distances[i] / parameters.SENSOR_RANGE;

                    // Previous proximity calculation
                    T proximity_factor = T(1.0) - norm_dist;
                    // Use cubic instead of square to create steeper gradient
                    total_reward += T(3.0) * proximity_factor * proximity_factor * proximity_factor;

                    // Add significant bonus for being at least halfway inside the detection radius
                    if (norm_dist < 0.5) {
                        total_reward += T(1.5);
                    }
                }
            }

            // Strong bonus for complete swarm detection
            if (detecting_count == active_count) {
                total_reward += T(5.0);
            } else {
                T missing_ratio = T(active_count - detecting_count) / T(active_count);
                total_reward -= T(1.0) * missing_ratio;
            }

            // Proximity rewards
            for (TI i = 0; i < active_count; ++i) {
                if (detecting[i]) {
                    T norm_dist = distances[i] / parameters.SENSOR_RANGE;
                    T proximity_factor = T(1.0) - norm_dist;
                    total_reward += T(2.0) * proximity_factor * proximity_factor;
                }
            }

            // Very close bonus
            for (TI i = 0; i < active_count; ++i) {
                if (distances[i] < parameters.SENSOR_RANGE * T(0.3)) {
                    total_reward += T(1.0);
                }
            }

            // ADDED: Formation reward for 360-degree coverage
            if (detecting_count >= 2) {
                T formation_reward = T(0);

                // Calculate angles of detecting drones relative to disaster
                std::vector<T> angles;
                for (TI idx : detecting_indices) {
                    const auto &d = next_state.drone_states[idx];
                    T dx = d.position[0] - next_state.disaster.position[0];
                    T dy = d.position[1] - next_state.disaster.position[1];
                    T angle = math::atan2(device.math, dy, dx);
                    // Normalize angle to [0, 2π)
                    if (angle < T(0)) {
                        angle += T(2) * T(M_PI);
                    }
                    angles.push_back(angle);
                }

                // Sort angles for better analysis
                std::sort(angles.begin(), angles.end());

                // Calculate angular separation between consecutive drones
                T ideal_separation = T(2) * T(M_PI) / T(detecting_count);
                T total_angle_error = T(0);

                for (size_t i = 0; i < angles.size(); ++i) {
                    size_t next_i = (i + 1) % angles.size();
                    T actual_separation = angles[next_i] - angles[i];

                    // Handle wraparound for the last drone
                    if (next_i == 0) {
                        actual_separation = (T(2) * T(M_PI) - angles[i]) + angles[0];
                    }

                    // Calculate error from ideal separation
                    T error = std::abs(actual_separation - ideal_separation);
                    total_angle_error += error;
                }

                // Reward for good angular distribution (lower error = higher reward)
                T max_error = T(2) * T(M_PI);  // Maximum possible error
                T normalized_error = total_angle_error / max_error;
                formation_reward = T(3.0) * (T(1.0) - normalized_error);

                // Additional reward for maintaining optimal distance while in formation
                T distance_consistency_reward = T(0);
                if (detecting_count >= 2) {
                    T ideal_radius = parameters.SENSOR_RANGE * T(0.75);  // Optimal monitoring distance
                    T distance_variance = T(0);

                    for (TI idx : detecting_indices) {
                        T deviation = std::abs(distances[idx] - ideal_radius);
                        distance_variance += deviation;
                    }

                    T avg_deviation = distance_variance / T(detecting_count);
                    T max_deviation = parameters.SENSOR_RANGE;
                    T normalized_deviation = avg_deviation / max_deviation;
                    distance_consistency_reward = T(1.5) * (T(1.0) - normalized_deviation);
                }

                total_reward += formation_reward + distance_consistency_reward;
            }
        }
            // EXPLORATION MODE - no disaster detected (regardless of disaster.active)
        else {
            const T cx = parameters.GRID_SIZE_X / T(2);
            const T cy = parameters.GRID_SIZE_Y / T(2);

            // Check if disaster has been detected before (shared knowledge)
            bool disaster_previously_detected = false;
            if (active_count > 0) {
                disaster_previously_detected = (next_state.drone_states[0].last_detected_disaster_position[0] > T(-1));
            }

            if (disaster_previously_detected && next_state.disaster.active) {
                // CASE: Disaster active but not currently detected, was detected before
                // Encourage moving toward last known position and forming search pattern
                T total_approach_reward = T(0);

                // Calculate center of mass around last known position
                T last_pos_x = next_state.drone_states[0].last_detected_disaster_position[0];
                T last_pos_y = next_state.drone_states[0].last_detected_disaster_position[1];

                for (TI i = 0; i < active_count; ++i) {
                    const auto &d = next_state.drone_states[i];
                    const auto &old_d = state.drone_states[i];

                    T current_dist = magnitude(device,
                                               d.position[0] - last_pos_x,
                                               d.position[1] - last_pos_y);

                    T old_dist = magnitude(device,
                                           old_d.position[0] - last_pos_x,
                                           old_d.position[1] - last_pos_y);

                    // Reward for moving toward last known position
                    if (current_dist < old_dist) {
                        total_approach_reward += T(0.8);
                    }

                    // Reward for being spread around the last known position (search formation)
                    for (TI j = i + 1; j < active_count; ++j) {
                        T drone_distance = magnitude(device,
                                                     d.position[0] - next_state.drone_states[j].position[0],
                                                     d.position[1] - next_state.drone_states[j].position[1]);

                        if (drone_distance >= parameters.SENSOR_RANGE * T(0.8) &&
                            drone_distance <= parameters.SENSOR_RANGE * T(1.5)) {
                            total_approach_reward += T(0.3);
                        }
                    }
                }

                total_reward += total_approach_reward;
            } else {
                // CASE: No disaster detected and never detected, OR disaster not active
                // General exploration rewards (increased values)

                // Zone coverage (increased rewards)
                bool platform_covered = false;
                bool pipe_h_left_covered = false;
                bool pipe_h_right_covered = false;
                bool pipe_v_top_covered = false;
                bool pipe_v_bottom_covered = false;

                T min_drone_distance = (active_count > 1) ? std::numeric_limits<T>::max() : T(0);

                for (TI i = 0; i < active_count; ++i) {
                    const auto &d = next_state.drone_states[i];
                    const auto &old_d = state.drone_states[i];

                    T x = d.position[0], y = d.position[1];
                    T adx = std::abs(x - cx);
                    T ady = std::abs(y - cy);

                    // Check coverage areas
                    if (adx <= parameters.PLATFORM_HALF_SIZE && ady <= parameters.PLATFORM_HALF_SIZE) {
                        platform_covered = true;
                    }
                    if (ady <= parameters.PIPE_WIDTH / 2) {
                        if (x < cx) pipe_h_left_covered = true;
                        if (x > cx) pipe_h_right_covered = true;
                    }
                    if (adx <= parameters.PIPE_WIDTH / 2) {
                        if (y < cy) pipe_v_bottom_covered = true;
                        if (y > cy) pipe_v_top_covered = true;
                    }

                    // Movement reward (increased)
                    T move_dist = magnitude(device,
                                            d.position[0] - old_d.position[0],
                                            d.position[1] - old_d.position[1]);
                    if (move_dist > T(0.2)) {
                        total_reward += T(0.3);  // Increased from 0.05
                    }

                    // Distance calculations for separation
                    for (TI j = i + 1; j < active_count; ++j) {
                        T dist = magnitude(device,
                                           d.position[0] - next_state.drone_states[j].position[0],
                                           d.position[1] - next_state.drone_states[j].position[1]);
                        min_drone_distance = std::min(min_drone_distance, dist);
                    }
                }

                // Coverage rewards (increased)
                if (platform_covered) total_reward += T(0.8);  // Increased from 0.3
                if (pipe_h_left_covered) total_reward += T(0.6);  // Increased from 0.2
                if (pipe_h_right_covered) total_reward += T(0.6);
                if (pipe_v_top_covered) total_reward += T(0.6);
                if (pipe_v_bottom_covered) total_reward += T(0.6);

                // Separation rewards (increased)
                if (active_count > 1) {
                    T optimal_distance = parameters.SENSOR_RANGE;
                    T distance_ratio = min_drone_distance / optimal_distance;

                    if (distance_ratio < T(0.5)) {
                        total_reward -= T(0.3);  // Increased penalty
                    } else if (distance_ratio <= T(1.5)) {
                        T factor = T(1.0) - std::abs(distance_ratio - T(1.0));
                        total_reward += T(1.0) * factor;  // Increased from 0.4
                    } else {
                        total_reward += T(0.3);  // Increased from 0.1
                    }
                }
            }
        }

        // Termination penalties (consistent with before)
        if (next_state.disaster.active && next_state.disaster_undetected_steps >= parameters.DISASTER_DETECTION_TIMEOUT) {
            total_reward -= T(10.0);
        }

        if (next_state.disaster.active) {
            if (next_state.disaster.position[0] < 0 ||
                next_state.disaster.position[0] >= parameters.GRID_SIZE_X ||
                next_state.disaster.position[1] < 0 ||
                next_state.disaster.position[1] >= parameters.GRID_SIZE_Y) {

                if (detecting_count > 0) {
                    total_reward += T(2.0);
                } else {
                    total_reward -= T(5.0);
                }
            }
        }

        return total_reward / T(active_count);
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
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI D = Observation<OBS_PARAMETERS>::PER_AGENT_DIM;

        for (TI i = 0; i < parameters.N_AGENTS; i++) {
            const auto &d = state.drone_states[i];

            // Position (2D)
            set(observation, 0, i * D + 0, d.position[0]);
            set(observation, 0, i * D + 1, d.position[1]);

            // Velocity (2D)
            set(observation, 0, i * D + 2, d.velocity[0]);
            set(observation, 0, i * D + 3, d.velocity[1]);

            // Acceleration (2D)
//            set(observation, 0, i * D + 4, d.acceleration[0]);
//            set(observation, 0, i * D + 5, d.acceleration[1]);

//            // Mode (NORMAL/EMERGENCY/RECHARGING)
//            set(observation, 0, i * D + 4, (T) d.mode);

            // Mode (one-hot encoded: NORMAL and RECHARGING)
//            set(observation, 0, i * D + 4, d.mode == DroneMode::NORMAL ? T(1) : T(0));
//            set(observation, 0, i * D + 5, d.mode == DroneMode::RECHARGING ? T(1) : T(0));

            // Calculate disaster_detected on-the-fly
            bool is_detecting = false;
//            if (state.disaster.active && d.mode != DroneMode::RECHARGING) {
            if (state.disaster.active) {
                T dx = d.position[0] - state.disaster.position[0];
                T dy = d.position[1] - state.disaster.position[1];
                T dist = magnitude(device, dx, dy);
                is_detecting = (dist < parameters.SENSOR_RANGE);
            }
            set(observation, 0, i * D + 4, is_detecting ? T(1) : T(0));

//            // Current disaster position (only if active)
//            if (state.disaster.active) {
//                set(observation, 0, i * D + 8, state.disaster.position[0]);
//                set(observation, 0, i * D + 9, state.disaster.position[1]);
//            } else {
//                set(observation, 0, i * D + 8, T(-1));
//                set(observation, 0, i * D + 9, T(-1));
//            }

            // Last detected disaster position (shared knowledge)
            set(observation, 0, i * D + 5, d.last_detected_disaster_position[0]);
            set(observation, 0, i * D + 6, d.last_detected_disaster_position[1]);
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

        // Check if disaster has been undetected for too long
        if (state.disaster.active && state.disaster_undetected_steps >= parameters.DISASTER_DETECTION_TIMEOUT) {
            return true;
        }

        // Check for critically low battery (below 5% instead of 0%)
//        for (TI i = 0; i < parameters.N_AGENTS; ++i) {
//            if (state.drone_states[i].battery <= T(5)) {
//                return true;
//            }
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