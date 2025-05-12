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

        // Count active drones
        TI active_count = parameters.N_AGENTS;  // All drones are active in your modified code

        // Initialize reward
        T total_reward = T(0);

        // DISASTER ACTIVE MODE
        if (next_state.disaster.active) {
            // Track individual drone detection and distances to disaster
            std::vector<bool> detecting(active_count, false);
            std::vector<T> distances(active_count, std::numeric_limits<T>::max());
            TI detecting_count = 0;

            // Calculate who's detecting and at what distance
            for (TI i = 0; i < active_count; ++i) {
                const auto &d = next_state.drone_states[i];

                // Calculate distance to disaster
                T dx = d.position[0] - next_state.disaster.position[0];
                T dy = d.position[1] - next_state.disaster.position[1];
                T dist = magnitude(device, dx, dy);
                distances[i] = dist;

                if (dist < parameters.SENSOR_RANGE) {
                    detecting[i] = true;
                    detecting_count++;
                }
            }

            if (detecting_count > 0) {
                // 1. Base team reward for detection
                total_reward += T(0.5) * T(detecting_count);

                // 2. Bonus for ALL drones detecting (swarm formation) - increased significantly
                if (detecting_count == active_count) {
                    total_reward += T(3.0);  // Increased from 1.5 to strongly drive complete detection
                }
                    // Add a small penalty when detection is incomplete
                else {
                    // Scale penalty based on how many drones aren't detecting
                    T missing_ratio = T(active_count - detecting_count) / T(active_count);
                    total_reward -= T(0.5) * missing_ratio;
                }

                // 3. Individual proximity rewards with GRADIENT - stronger incentive to get closer
                for (TI i = 0; i < active_count; ++i) {
                    if (detecting[i]) {
                        // Scaled proximity reward - higher reward the closer you get
                        T norm_dist = distances[i] / parameters.SENSOR_RANGE;
                        T proximity_factor = T(1.0) - norm_dist;

                        // Quadratic scaling to more strongly reward getting closer
                        total_reward += T(1.0) * proximity_factor * proximity_factor;
                    }
                }

                // 4. Extra reward for drones getting very close to disaster
                for (TI i = 0; i < active_count; ++i) {
                    if (distances[i] < parameters.SENSOR_RANGE * T(0.3)) {  // Very close
                        total_reward += T(0.5);
                    }
                }
            }

            if (next_state.disaster_undetected_steps >= parameters.DISASTER_DETECTION_TIMEOUT) {
                total_reward -= T(10.0);  // Heavy penalty
            }

            // Check if disaster left environment
//            if (next_state.disaster.active) {
            if (next_state.disaster.position[0] < 0 ||
                next_state.disaster.position[0] >= parameters.GRID_SIZE_X ||
                next_state.disaster.position[1] < 0 ||
                next_state.disaster.position[1] >= parameters.GRID_SIZE_Y) {

                if (detecting_count > 0) {
                    total_reward += T(2.0);   // Small bonus for successful tracking
                } else {
                    total_reward -= T(5.0);   // Moderate penalty
                }
            }
//            }

//            else {
//                // When disaster is active but not detected - stronger incentive to search
//                const T cx = parameters.GRID_SIZE_X / T(2);
//                const T cy = parameters.GRID_SIZE_Y / T(2);
//
//                // Track exploration coverage
//                bool explored_regions[4] = {false, false, false, false}; // quadrants
//
//                for (TI i = 0; i < active_count; ++i) {
//                    const auto &d = next_state.drone_states[i];
//                    const auto &old_d = state.drone_states[i];


                    // Calculate distance from last known position (or from default position)
//                    T explore_dx = T(0), explore_dy = T(0);
//
//                    // If we have a known disaster position from previous detection
//                    if (d.last_detected_disaster_position[0] > T(-1)) {
//                        // Calculate distance to last known disaster position
//                        T dx = d.position[0] - d.last_detected_disaster_position[0];
//                        T dy = d.position[1] - d.last_detected_disaster_position[1];
//                        T dist_to_last = magnitude(device, dx, dy);
//
//                        // Get previous distance (from state)
//                        const auto &old_d = state.drone_states[i];
//                        T old_dx = old_d.position[0] - old_d.last_detected_disaster_position[0];
//                        T old_dy = old_d.position[1] - old_d.last_detected_disaster_position[1];
//                        T old_dist_to_last = magnitude(device, old_dx, old_dy);
//
//                        // Reward for moving TOWARD the last known disaster position
//                        if (dist_to_last < old_dist_to_last) {
//                            total_reward += T(0.3);  // Higher reward for moving toward last known position
//                        }
//
////                        // Additional reward for being close to the last known position
////                        // (to encourage searching in that area)
////                        if (dist_to_last < parameters.SENSOR_RANGE) {
////                            total_reward += T(0.2);
////                        }
//                    }
//                    else {
//                        // Encourage movement from center if no known position
//                        explore_dx = d.position[0] - cx;
//                        explore_dy = d.position[1] - cy;
//
//                        T dist_from_center = magnitude(device, explore_dx, explore_dy);
//
//                        // Reward moving away from center to search when no disaster has been seen
//                        if (dist_from_center > T(5.0)) {
//                            total_reward += T(0.2);
//                        }
//                    }

//                    // Track quadrant coverage
//                    if (d.position[0] < cx && d.position[1] < cy) explored_regions[0] = true;
//                    if (d.position[0] >= cx && d.position[1] < cy) explored_regions[1] = true;
//                    if (d.position[0] < cx && d.position[1] >= cy) explored_regions[2] = true;
//                    if (d.position[0] >= cx && d.position[1] >= cy) explored_regions[3] = true;
//
//                    // High-priority area bonus
//                    T x = d.position[0], y = d.position[1];
//                    T adx = std::abs(x - cx);
//                    T ady = std::abs(y - cy);
//
//                    bool inPlat = (adx <= parameters.PLATFORM_HALF_SIZE && ady <= parameters.PLATFORM_HALF_SIZE);
//                    bool inPipeH = (ady <= parameters.PIPE_WIDTH / 2);
//                    bool inPipeV = (adx <= parameters.PIPE_WIDTH / 2);
//
//                    if (inPlat || inPipeH || inPipeV) {
//                        total_reward += T(0.15);  // Increased from 0.1
//                    }
//                }
//
//                // Bonus for covering multiple quadrants
//                TI regions_covered = 0;
//                for (bool covered : explored_regions) {
//                    if (covered) regions_covered++;
//                }
//
//                total_reward += T(0.1) * T(regions_covered);
//            }
        }
            // EXPLORATION MODE - no active disaster
        else {
            // Focus on coverage and continuous movement
            const T cx = parameters.GRID_SIZE_X / T(2);
            const T cy = parameters.GRID_SIZE_Y / T(2);

            // Track area coverage
            bool platform_covered = false;
            bool pipe_h_left_covered = false;
            bool pipe_h_right_covered = false;
            bool pipe_v_top_covered = false;
            bool pipe_v_bottom_covered = false;

            // Calculate minimum distance between any two drones
            T min_drone_distance = (active_count > 1) ? std::numeric_limits<T>::max() : T(0);

            for (TI i = 0; i < active_count; ++i) {
                const auto &d = next_state.drone_states[i];
                const auto &old_d = state.drone_states[i];

                // Check drone's position relative to high-priority areas
                T x = d.position[0], y = d.position[1];
                T adx = std::abs(x - cx);
                T ady = std::abs(y - cy);

                // Platform coverage
                if (adx <= parameters.PLATFORM_HALF_SIZE && ady <= parameters.PLATFORM_HALF_SIZE) {
                    platform_covered = true;
                }

                // Horizontal pipe coverage - split into left and right sections
                if (ady <= parameters.PIPE_WIDTH / 2) {
                    if (x < cx) pipe_h_left_covered = true;
                    if (x > cx) pipe_h_right_covered = true;
                }

                // Vertical pipe coverage - split into top and bottom sections
                if (adx <= parameters.PIPE_WIDTH / 2) {
                    if (y < cy) pipe_v_bottom_covered = true;
                    if (y > cy) pipe_v_top_covered = true;
                }

                // Calculate drone movement (encourage exploration)
                T move_dx = d.position[0] - old_d.position[0];
                T move_dy = d.position[1] - old_d.position[1];
                T move_dist = magnitude(device, move_dx, move_dy);

                // Reward for movement/exploration
                if (move_dist > T(0.2)) {
                    total_reward += T(0.05);
                }

                // Check distances to other drones for spatial distribution
                if (active_count > 1) {
                    for (TI j = i + 1; j < active_count; ++j) {
                        const auto &d2 = next_state.drone_states[j];

                        T dx = d.position[0] - d2.position[0];
                        T dy = d.position[1] - d2.position[1];
                        T dist = magnitude(device, dx, dy);

                        min_drone_distance = std::min(min_drone_distance, dist);
                    }
                }
            }

            // Reward for coverage - each area contributes individually
            if (platform_covered) total_reward += T(0.3);
            if (pipe_h_left_covered) total_reward += T(0.2);
            if (pipe_h_right_covered) total_reward += T(0.2);
            if (pipe_v_top_covered) total_reward += T(0.2);
            if (pipe_v_bottom_covered) total_reward += T(0.2);

            // Reward for spatial distribution (if multiple active drones)
            if (active_count > 1) {
                // Optimal separation is around sensor range
                T optimal_distance = parameters.SENSOR_RANGE;
                T distance_ratio = min_drone_distance / optimal_distance;

                // Modified distribution reward - stronger incentive to maintain optimal spacing
                if (distance_ratio < T(0.5)) {
                    // Too close - small penalty
                    total_reward -= T(0.1);
                }
                else if (distance_ratio <= T(1.5)) {
                    // Good range - stronger reward peak at optimal
                    T factor = T(1.0) - std::abs(distance_ratio - T(1.0));
                    total_reward += T(0.4) * factor;
                }
                else {
                    // Too far - small reward to avoid excessive spreading
                    total_reward += T(0.1);
                }
            }
        }

        // Divide total reward by active_count to get per-drone reward
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