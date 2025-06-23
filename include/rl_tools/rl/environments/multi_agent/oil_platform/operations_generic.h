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
    using rl::environments::multi_agent::oil_platform::Observation;

    // Helper function to compute the magnitude of a 2D vector
    template<typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static T magnitude(DEVICE& device, T x, T y) {
        return math::sqrt(device.math, x * x + y * y);
    }

    template<typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static T distance(DEVICE& device, T x1, T y1, T x2, T y2) {
        return magnitude(device, x1-x2, y1-y2);
    }


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

        // Initialize all drone states
        for (TI agent_i = 0; agent_i < SPEC::PARAMETERS::N_AGENTS; agent_i++) {
            auto &agent_state = state.drone_states[agent_i];
            agent_state.position[0] = SPEC::PARAMETERS::GRID_SIZE_X / 2;
            agent_state.position[1] = SPEC::PARAMETERS::GRID_SIZE_Y / 2;
            agent_state.velocity[0] = 0;
            agent_state.velocity[1] = 0;
            agent_state.battery = 70;
            agent_state.dead = false;
            agent_state.is_charging = false;
            agent_state.is_detecting = false;
        }

        state.disaster.active = false;
        state.disaster.position[0] = 0;
        state.disaster.position[1] = 0;
        state.disaster.velocity[0] = 0;
        state.disaster.velocity[1] = 0;

        state.disaster_detected_global = false;
        state.step_count = 0;
        state.disaster_undetected_steps = 0;
        state.last_detected_disaster_position[0] = 0;
        state.last_detected_disaster_position[1] = 0;

        // Metrics initialization
        state.metrics.total_coverage_ratio = 0;
        state.metrics.coverage_measurement_count = 0;
        state.metrics.disaster_active_steps = 0;
        state.metrics.total_charging_sessions = 0;
        state.metrics.appropriate_charging_count = 0;
        state.metrics.inappropriate_charging_count = 0;
        state.metrics.death_count = 0;

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

        // Initialize all drone states
        for (TI agent_i = 0; agent_i < SPEC::PARAMETERS::N_AGENTS; agent_i++) {
            auto &agent_state = state.drone_states[agent_i];
            agent_state.position[0] = SPEC::PARAMETERS::GRID_SIZE_X / 2;
            agent_state.position[1] = SPEC::PARAMETERS::GRID_SIZE_Y / 2;
            agent_state.velocity[0] = 0;
            agent_state.velocity[1] = 0;
            agent_state.battery = 70;
            agent_state.dead = false;
            agent_state.is_charging = false;
            agent_state.is_detecting = false;
        }

        // Disaster state
        state.disaster.active = false;
        state.disaster.position[0] = 0;
        state.disaster.position[1] = 0;
        state.disaster.velocity[0] = 0;
        state.disaster.velocity[1] = 0;

        state.disaster_detected_global = false;
        state.step_count = 0;
        state.disaster_undetected_steps = 0;
        state.last_detected_disaster_position[0] = 0;
        state.last_detected_disaster_position[1] = 0;

        // Metrics initialization
        state.metrics.total_coverage_ratio = 0;
        state.metrics.coverage_measurement_count = 0;
        state.metrics.disaster_active_steps = 0;
        state.metrics.total_charging_sessions = 0;
        state.metrics.appropriate_charging_count = 0;
        state.metrics.inappropriate_charging_count = 0;
        state.metrics.death_count = 0;
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

    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T step(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            const Matrix<ACTION_SPEC> &action,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State &next_state,
            RNG &rng
    ) {
        using ENV = rl::environments::multi_agent::OilPlatform<SPEC>;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using PARAMS = typename SPEC::PARAMETERS;
        constexpr TI N_AGENTS = ENV::PARAMETERS::N_AGENTS;

        utils::assert_exit(device, !is_nan(device, action), "Action is nan");


        const T cx = PARAMS::GRID_SIZE_X / T(2);
        const T cy = PARAMS::GRID_SIZE_Y / T(2);

        // (1) Disaster: 2% to start in any HIGH‑PRIORITY area, remains static once created
        if (!state.disaster.active) {
            if (state.step_count >= PARAMS::DISASTER_MINIMUM_SPAWN_STEP && random::uniform_real_distribution(device.random, T(0), T(1), rng) < T(0.02)) {
                T x, y;
                bool in_high_priority_zone = false;
                while (!in_high_priority_zone){
                    x = random::uniform_real_distribution(device.random, T(0), T(PARAMS::GRID_SIZE_X), rng);
                    y = random::uniform_real_distribution(device.random, T(0), T(PARAMS::GRID_SIZE_Y), rng);
                    T adx = std::abs(x - cx);
                    T ady = std::abs(y - cy);
                    bool inPlat = (adx <= PARAMS::PLATFORM_HALF_SIZE && ady <= PARAMS::PLATFORM_HALF_SIZE);
                    bool inPipeH = (ady <= PARAMS::PIPE_WIDTH / 2 && adx >= PARAMS::PLATFORM_HALF_SIZE);
                    bool inPipeV = (adx <= PARAMS::PIPE_WIDTH / 2 && ady >= PARAMS::PLATFORM_HALF_SIZE);
                    in_high_priority_zone = inPlat || inPipeH || inPipeV;
                }
                next_state.disaster.active = true;
                next_state.disaster.position[0] = x;
                next_state.disaster.position[1] = y;

                // Generate random velocity for the disaster
                T angle = random::uniform_real_distribution(device.random, T(0), T(2 * M_PI), rng);
                next_state.disaster.velocity[0] = PARAMS::DISASTER_MAX_SPEED * math::cos(device.math, angle);
                next_state.disaster.velocity[1] = PARAMS::DISASTER_MAX_SPEED * math::sin(device.math, angle);
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
            next_state.disaster.position[0] = state.disaster.position[0] + state.disaster.velocity[0] * PARAMS::DT;
            next_state.disaster.position[1] = state.disaster.position[1] + state.disaster.velocity[1] * PARAMS::DT;
            next_state.disaster.velocity[0] = state.disaster.velocity[0];
            next_state.disaster.velocity[1] = state.disaster.velocity[1];
        }

        // (2) Per-drone update with charging and death logic
        for (TI agent_i = 0; agent_i < N_AGENTS; agent_i++) {
            const auto &agent_state = state.drone_states[agent_i];
            auto &agent_next_state = next_state.drone_states[agent_i];
            if (!agent_state.dead) {
//                T desired_vx = get(action, 0, agent_i * 2 + 0) * PARAMS::MAX_SPEED;
//                T desired_vy = get(action, 0, agent_i * 2 + 1) * PARAMS::MAX_SPEED;
//
//                // Set velocity directly to desired values
//                agent_next_state.velocity[0] = desired_vx;
//                agent_next_state.velocity[1] = desired_vy;
//
//                // Clamp velocity magnitude to MAX_SPEED (redundant but safe)
//                T speed = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
//                if (speed > PARAMS::MAX_SPEED) {
//                    T scale = PARAMS::MAX_SPEED / speed;
//                    agent_next_state.velocity[0] *= scale;
//                    agent_next_state.velocity[1] *= scale;
//                }
                T ax = get(action, 0, agent_i * 2 + 0) * PARAMS::MAX_ACCELERATION;
                T ay = get(action, 0, agent_i * 2 + 1) * PARAMS::MAX_ACCELERATION;

                agent_next_state.velocity[0] = agent_state.velocity[0] + ax * PARAMS::DT;
                agent_next_state.velocity[1] = agent_state.velocity[1] + ay * PARAMS::DT;

                // clamp velocity magnitude to MAX_SPEED
                T speed = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
                if (speed > PARAMS::MAX_SPEED) {
                    T scale = PARAMS::MAX_SPEED / speed;
                    agent_next_state.velocity[0] *= scale;
                    agent_next_state.velocity[1] *= scale;
                }

                // integrate velocity to get new position
                agent_next_state.position[0] = agent_state.position[0] + agent_next_state.velocity[0] * PARAMS::DT;
                agent_next_state.position[1] = agent_state.position[1] + agent_next_state.velocity[1] * PARAMS::DT;

                // clamp position to environment boundaries
                T x_pre_clamping = agent_next_state.position[0];
                T y_pre_clamping = agent_next_state.position[1];
                agent_next_state.position[0] = math::clamp(device.math, T(x_pre_clamping), T(0), T(PARAMS::GRID_SIZE_X));
                agent_next_state.position[1] = math::clamp(device.math, T(y_pre_clamping), T(0), T(PARAMS::GRID_SIZE_Y));

                // if we clamped on the x‐axis, zero out x‐motion:
                if (agent_next_state.position[0] != x_pre_clamping) {
                    agent_next_state.velocity[0] = 0;
                }

                // same for y:
                if (agent_next_state.position[1] != y_pre_clamping) {
                    agent_next_state.velocity[1] = 0;
                }

            } else {
                agent_next_state = agent_state;
            }
        }

        for (TI agent_i = 0; agent_i < N_AGENTS; agent_i++) {
            auto& agent_next_state = next_state.drone_states[agent_i];
            const auto& agent_state = state.drone_states[agent_i];

            if(!agent_state.dead) {
                // Check if drone is at charging station
                T distance_to_charging_station = distance(device, agent_next_state.position[0], agent_next_state.position[1], PARAMS::CHARGING_STATION_POSITION_X, PARAMS::CHARGING_STATION_POSITION_Y);
                bool at_charging_station = (distance_to_charging_station < PARAMS::CHARGING_STATION_RANGE);
                T velocity_magnitude = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
                bool is_stationary = (velocity_magnitude < PARAMS::CHARGING_VELOCITY_THRESHOLD);
                agent_next_state.is_charging = (at_charging_station && is_stationary);

                // Update battery
                if (agent_next_state.is_charging) {
                    agent_next_state.battery = std::min(T(100), T(agent_state.battery + PARAMS::CHARGING_RATE));
                } else {
                    agent_next_state.battery = std::max(T(0), T(agent_state.battery - PARAMS::DISCHARGE_RATE));
                }

                // Check if drone dies
                if (agent_next_state.battery <= 0) {
                    agent_next_state.dead = true;
                    agent_next_state.velocity[0] = 0;
                    agent_next_state.velocity[1] = 0;
                    agent_next_state.battery = 0;
                    agent_next_state.is_charging = false;
                    agent_next_state.is_detecting = false;
                } else {
                    agent_next_state.dead = false;
                }
            }
        }


        // (3) Disaster detection logic
        bool detection = false;
        if (next_state.disaster.active) {
            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
                auto &agent_next_state = next_state.drone_states[agent_i];

                // Skip dead drones for disaster detection
                if (!agent_next_state.dead) {
                    if (distance(device, agent_next_state.position[0], agent_next_state.position[1], next_state.disaster.position[0], next_state.disaster.position[1]) < PARAMS::SENSOR_RANGE) {
                        detection = true;
                        agent_next_state.is_detecting = true;
                    } else {
                        agent_next_state.is_detecting = false;
                    }
                }
            }

            // Update all living drones' last detected position if any detection
            if (detection) {
                next_state.disaster_detected_global = true;
                next_state.disaster_undetected_steps = 0;
                next_state.last_detected_disaster_position[0] = next_state.disaster.position[0];
                next_state.last_detected_disaster_position[1] = next_state.disaster.position[1];
            } else {
                next_state.disaster_undetected_steps = state.disaster_undetected_steps + 1;
                next_state.last_detected_disaster_position[0] = state.last_detected_disaster_position[0];
                next_state.last_detected_disaster_position[1] = state.last_detected_disaster_position[1];
            }
        } else {
            next_state.disaster_detected_global = false;
            next_state.disaster_undetected_steps = 0;
            next_state.last_detected_disaster_position[0] = state.last_detected_disaster_position[0];
            next_state.last_detected_disaster_position[1] = state.last_detected_disaster_position[1];
        }

        next_state.metrics = state.metrics;

        // (4) Metrics update
        if (next_state.disaster.active) {
            next_state.metrics.disaster_active_steps = state.metrics.disaster_active_steps + 1;
        }

        // Update charging metrics
        TI charging_sessions = 0;
        TI appropriate_charging_sessions = 0;
        TI inappropriate_charging_sessions = 0;
        TI death_count = 0;

        for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
            const auto &agent_state = state.drone_states[agent_i];
            const auto &agent_next_state = next_state.drone_states[agent_i];

            // Count new charging sessions (started charging this step)
            if (agent_next_state.is_charging) {
                charging_sessions++;

                // Classify as appropriate or inappropriate
                if (agent_state.battery < T(50)) {
                    appropriate_charging_sessions++;
                } else {
                    inappropriate_charging_sessions++;
                }
            }

            // Count new deaths
            if (!agent_state.dead && agent_next_state.dead) {
                death_count++;
            }
        }

        next_state.metrics.total_charging_sessions = state.metrics.total_charging_sessions + charging_sessions;
        next_state.metrics.appropriate_charging_count = state.metrics.appropriate_charging_count + appropriate_charging_sessions;
        next_state.metrics.inappropriate_charging_count = state.metrics.inappropriate_charging_count + inappropriate_charging_sessions;
        next_state.metrics.death_count = state.metrics.death_count + death_count;

        // (5) Advance step count
        next_state.step_count = state.step_count + 1;

        return PARAMS::DT;
    }

    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC>& env,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& state,
            const Matrix<ACTION_SPEC>& action,
            typename rl::environments::multi_agent::OilPlatform<SPEC>::State& next_state,
            RNG & rng
    ) {
        using ENV = rl::environments::multi_agent::OilPlatform<SPEC>;
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using PARAMS = typename SPEC::PARAMETERS;
        constexpr TI N_AGENTS = ENV::PARAMETERS::N_AGENTS;

        const T cx = PARAMS::GRID_SIZE_X / 2;
        const T cy = PARAMS::GRID_SIZE_Y / 2;

        T total_reward = 0;
        T alive_agents_count = 0;

        // Death penalties
        T death_penalty = -10;
        T death_termination_penalty = -50;
//        T survival_bonus_per_agent = 0.1;

        // Charging and battery management penalties
        T necessary_charging_reward = 6;
        T convenient_charging_reward = 3;
        T unnecessary_charging_penalty = -2;
        T critical_battery_level_penalty = -4;
//        T critical_battery_level_penalty = -8;
        T low_battery_level_penalty = -2;
//        T low_battery_level_penalty = -4;
        T charging_station_approach_reward = 2.5;
//        T charging_station_approach_reward = 4;
        T charging_station_velocity_approach_reward = 0.5;
//        T charging_station_velocity_approach_reward = 2;

        // Detection rewards
        T single_agent_detection_reward = 1;
        T proximity_reward = 3;
        T separation_reward = 4;
        T disaster_approach_reward = 4;

        // Coverage rewards
        T coverage_reward_total = 3;
        T platform_coverage_reward = 0.8;
        T pipe_coverage_reward = 0.6;


        // Count living agents and apply death penalty
        for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
            const auto &agent_state = state.drone_states[agent_i];
            const auto &agent_next_state = next_state.drone_states[agent_i];

            if (!agent_state.dead) {
                alive_agents_count++;
            }

            // NEW: Death penalty - only when drone just died this step
            if (!agent_state.dead && agent_next_state.dead) {
                total_reward += death_penalty;  // Large penalty for dying
            }
        }

//        total_reward += survival_bonus_per_agent * alive_agents_count;


        // If all agents are dead, return large negative reward
        if (alive_agents_count == 0) {
            return death_termination_penalty;
        }

        // Charging and battery management rewards
        for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
            const auto &agent_next_state = next_state.drone_states[agent_i];
            const auto &agent_state = state.drone_states[agent_i];
            if (!agent_next_state.dead) {
                // Charging rewards
                if (agent_next_state.is_charging) {
                    if (agent_next_state.battery < 50) {
                        total_reward += necessary_charging_reward;  // High reward for necessary charging
                    } else if (agent_next_state.battery < 80) {
                        total_reward += convenient_charging_reward;  // Medium reward for preventive charging
                    } else {
                        total_reward += unnecessary_charging_penalty;  // Penalty for unnecessary charging
                    }
                } else {
                    // Battery level penalties
                    if (agent_next_state.battery < 20) {
                        total_reward += critical_battery_level_penalty;
                    } else if (agent_next_state.battery < 50) {
                        total_reward += low_battery_level_penalty;
                    }
                }

                // Low battery navigation incentive with incremental distance-based reward
                if (agent_next_state.battery < 50 && !agent_next_state.is_charging) {
                    T dist_to_charging = distance(device, agent_next_state.position[0], agent_next_state.position[1],
                                                  PARAMS::CHARGING_STATION_POSITION_X, PARAMS::CHARGING_STATION_POSITION_Y);
                    T prev_dist_to_charging = distance(device, agent_state.position[0], agent_state.position[1],
                                                       PARAMS::CHARGING_STATION_POSITION_X, PARAMS::CHARGING_STATION_POSITION_Y);

                    // Battery urgency multiplier (0.0 to 1.0, higher when battery is lower)
                    T battery_urgency_multiplier = (50.0 - agent_next_state.battery) / 50.0;

                    // Reward for getting closer to charging station
                    T distance_change = prev_dist_to_charging - dist_to_charging;  // positive = getting closer
                    T base_reward = charging_station_approach_reward * distance_change;
                    total_reward += base_reward * (1.0 + battery_urgency_multiplier);

                    // Small additional reward for being close AND stationary (preparation for charging)
                    // Keep this smaller than the actual charging reward (6)
                    if (dist_to_charging < PARAMS::CHARGING_STATION_RANGE) {
                        T velocity = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
                        if (velocity < PARAMS::CHARGING_VELOCITY_THRESHOLD) {
                            total_reward += charging_station_velocity_approach_reward * (1.0 + battery_urgency_multiplier);  // Max 4, always < 6
                        }
                    }
                }

                // Low battery navigation incentive with incremental distance-based reward
//                if (agent_next_state.battery < 50 && !agent_next_state.is_charging) {
//                    T dist_to_charging = distance(device, agent_next_state.position[0], agent_next_state.position[1],
//                                                  PARAMS::CHARGING_STATION_POSITION_X, PARAMS::CHARGING_STATION_POSITION_Y);
//                    T prev_dist_to_charging = distance(device, agent_state.position[0], agent_state.position[1],
//                                                       PARAMS::CHARGING_STATION_POSITION_X, PARAMS::CHARGING_STATION_POSITION_Y);
//
//                    // Battery urgency multiplier (0.0 to 1.0, higher when battery is lower)
//                    T battery_urgency_multiplier = (50.0 - agent_next_state.battery) / 50.0;
//
//                    if (dist_to_charging >= PARAMS::CHARGING_STATION_RANGE) {
//                        // Distance-based reward/penalty for movement toward/away from charging station
//                        T distance_change = prev_dist_to_charging - dist_to_charging;  // positive = getting closer
//                        T base_reward = charging_station_approach_reward * distance_change;
//                        total_reward += base_reward * (1.0 + battery_urgency_multiplier);
//                    } else {
//                        // Close to charging station - reward for being stationary (low velocity)
//                        T velocity = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
//                        T velocity_penalty = velocity / PARAMS::MAX_SPEED;  // 0.0 to 1.0, higher for faster movement
//                        T stationary_reward = charging_station_velocity_approach_reward * (1.0 - velocity_penalty);
//                        total_reward += stationary_reward * (1.0 + battery_urgency_multiplier);
//                    }
//                }

//                // Low battery navigation incentive
//                if (agent_next_state.battery < 50 && !agent_next_state.is_charging) {
//                    T dist_to_charging = distance(device, agent_next_state.position[0], agent_next_state.position[1], PARAMS::CHARGING_STATION_POSITION_X, PARAMS::CHARGING_STATION_POSITION_Y);
//                    if (dist_to_charging >= PARAMS::CHARGING_STATION_RANGE/2){
//                        T prev_dist_to_charging = distance(device, agent_state.position[0], agent_state.position[1], PARAMS::CHARGING_STATION_POSITION_X, PARAMS::CHARGING_STATION_POSITION_Y);
//                        if (dist_to_charging < prev_dist_to_charging) {
//                            total_reward += charging_station_approach_reward;  // Reward for approaching charging when needed
//                        }
//                    } else{
//                        T velocity = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
//                        T previous_velocity = magnitude(device, agent_state.velocity[0], agent_state.velocity[1]);
//                        if(velocity < previous_velocity){
//                            total_reward += charging_station_velocity_approach_reward;  // Reward for slowing down to charge in recharge range
//                        }
//                    }
//                }
            }
        }

        // Check if any drone currently detects the disaster
        T detecting_count = 0;

        std::vector<T> distances(N_AGENTS);
        std::vector<TI> detecting_indices;

        if (next_state.disaster.active) {
            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
                const auto &agent_next_state = next_state.drone_states[agent_i];
                if (!agent_next_state.dead && agent_next_state.is_detecting) {
                    distances[agent_i] = distance(device,
                                                  agent_next_state.position[0], agent_next_state.position[1],
                                                  next_state.disaster.position[0], next_state.disaster.position[1]);
                    detecting_indices.push_back(agent_i);
                    detecting_count++;
                }
            }
        }

        // DISASTER DETECTION MODE - when disaster is detected
        if (detecting_count > 0) {
            // Base reward for detection
            total_reward += single_agent_detection_reward * detecting_count;

            // Proximity rewards - closer agents get higher rewards
            for (TI agent_i : detecting_indices) {
                T normalized_distance = distances[agent_i] / PARAMS::SENSOR_RANGE;
                T proximity_factor = 1 - normalized_distance; // 1.0 at center, 0.0 at edge

                // Quadratic reward for being closer to disaster center
                total_reward += proximity_reward * proximity_factor * proximity_factor;
            }

            if (detecting_count >= 2) {
                T formation_reward = 0;
                for (size_t i = 0; i < detecting_indices.size(); ++i) {
                    for (size_t j = i + 1; j < detecting_indices.size(); ++j) {
                        TI agent_i = detecting_indices[i];
                        TI agent_j = detecting_indices[j];

                        T agent_distance = distance(device, next_state.drone_states[agent_i].position[0], next_state.drone_states[agent_i].position[1], next_state.drone_states[agent_j].position[0], next_state.drone_states[agent_j].position[1]);

                        // Dynamic ideal separation based on number of detecting agents and formation geometry
                        T detection_radius = PARAMS::SENSOR_RANGE * 0.75; // Assume agents position at 75% of sensor range
                        T ideal_separation;

                        if (detecting_count == 2) {
                            // Special case: 2 agents should be on opposite sides
                            ideal_separation = 2 * detection_radius; // Diameter = 7.5 units
                        } else {
                            // General case: evenly distributed around circumference
                            ideal_separation = 2 * M_PI * detection_radius / detecting_count;
                        }

                        // For close agents, encourage minimum separation to avoid overcrowding
                        T min_separation = PARAMS::SENSOR_RANGE * 0.3; // Minimum 30% of sensor range apart
                        T actual_ideal = math::max(device.math, ideal_separation, min_separation);

                        // Reward for maintaining good separation
                        T separation_error = std::abs(agent_distance - actual_ideal);
                        T max_error = PARAMS::SENSOR_RANGE;
                        formation_reward += separation_reward * (1 - separation_error / max_error);
                    }
                }
                total_reward += formation_reward;
            }

            // Strong bonus for complete swarm detection
            if (detecting_count == alive_agents_count) {
                total_reward += T(5.0);
            } else {
                T missing_ratio = T(alive_agents_count - detecting_count) / T(alive_agents_count);
                total_reward -= T(0.5) * missing_ratio;
            }
        } else if (next_state.disaster_detected_global && next_state.disaster.active) {
            // CASE: Disaster active but not currently detected, was detected before
            // Encourage moving toward the last known position

            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
                const auto &agent_next_state = next_state.drone_states[agent_i];
                if (!agent_next_state.dead) {
                    const auto &agent_state = state.drone_states[agent_i];
                    T current_dist = distance(device, agent_next_state.position[0], agent_next_state.position[1], next_state.last_detected_disaster_position[0], next_state.last_detected_disaster_position[1]);
                    T previous_dist = distance(device, agent_state.position[0], agent_state.position[1], state.last_detected_disaster_position[0], state.last_detected_disaster_position[1]);

                    // Reward for moving toward last known position
                    if (current_dist < previous_dist) {
                        total_reward += disaster_approach_reward;
                        // Alternative: quadratic scaling
//                        T distance_improvement = previous_dist - current_dist;
//                        T max_possible_improvement = math::sqrt(PARAMS::GRID_SIZE_X*PARAMS::GRID_SIZE_X + PARAMS::GRID_SIZE_Y * PARAMS::GRID_SIZE_Y);
//                        T normalized_improvement = distance_improvement / max_possible_improvement;
//                        total_reward += disaster_approach_reward * normalized_improvement * normalized_improvement; // Quadratic scaling
                    }
                }
            }
        } else {
            // CASE: No disaster detected and never detected, OR disaster not active
            // Exploration reward based on FOV coverage of high-priority areas

            // Define high-priority areas with their boundary
            // Platform center
            T platform_min_x = cx - PARAMS::PLATFORM_HALF_SIZE;
            T platform_max_x = cx + PARAMS::PLATFORM_HALF_SIZE;
            T platform_min_y = cy - PARAMS::PLATFORM_HALF_SIZE;
            T platform_max_y = cy + PARAMS::PLATFORM_HALF_SIZE;

            // Horizontal pipes
            T pipe_h_min_y = cy - PARAMS::PIPE_WIDTH / 2;
            T pipe_h_max_y = cy + PARAMS::PIPE_WIDTH / 2;

            // Vertical pipes
            T pipe_v_min_x = cx - PARAMS::PIPE_WIDTH / 2;
            T pipe_v_max_x = cx + PARAMS::PIPE_WIDTH / 2;

            // Create a grid to track which cells are covered by any drone's FOV
            const T grid_resolution = 20; // Higher number = more accurate but more computation
            const T cell_size_x = PARAMS::GRID_SIZE_X / grid_resolution;
            const T cell_size_y = PARAMS::GRID_SIZE_Y / grid_resolution;

            // Initialize counters for covered cells in each priority area
            TI platform_total_cells = 0;
            TI platform_covered_cells = 0;
            TI pipe_h_left_total_cells = 0;
            TI pipe_h_left_covered_cells = 0;
            TI pipe_h_right_total_cells = 0;
            TI pipe_h_right_covered_cells = 0;
            TI pipe_v_top_total_cells = 0;
            TI pipe_v_top_covered_cells = 0;
            TI pipe_v_bottom_total_cells = 0;
            TI pipe_v_bottom_covered_cells = 0;

            // Create a grid to track which cells are covered by FOV
            std::vector<std::vector<bool>> covered_grid(grid_resolution, std::vector<bool>(grid_resolution, false));

            // Mark cells covered by any drone's FOV
            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
                const auto &agent_next_state = next_state.drone_states[agent_i];
                if (!agent_next_state.dead) {
                    // Check all grid cells within the sensor range
                    for (TI cell_x = 0; cell_x < grid_resolution; ++cell_x) {
                        for (TI cell_y = 0; cell_y < grid_resolution; ++cell_y) {
                            if(!covered_grid[cell_x][cell_y]) {
                                // Calculate the center of this cell
                                T cell_center_x = cell_size_x * (cell_x + 0.5);
                                T cell_center_y = cell_size_y * (cell_y + 0.5);

                                // Check if this cell is within the drone's FOV
                                T distance_from_centre_of_cell = distance(device, cell_center_x, cell_center_y,
                                                                          agent_next_state.position[0],
                                                                          agent_next_state.position[1]);

                                if (distance_from_centre_of_cell < PARAMS::SENSOR_RANGE) {
                                    covered_grid[cell_x][cell_y] = true;
                                }
                            }
                        }
                    }
                }
            }

            // Count covered cells in each priority area
            for (TI cell_x = 0; cell_x < grid_resolution; ++cell_x) {
                for (TI cell_y = 0; cell_y < grid_resolution; ++cell_y) {
                    // Calculate the center of this cell
                    T cell_center_x = cell_size_x * (cell_x + 0.5);
                    T cell_center_y = cell_size_y * (cell_y + 0.5);

                    // Check which priority area this cell belongs to
                    bool in_platform = (cell_center_x >= platform_min_x && cell_center_x <= platform_max_x &&
                                        cell_center_y >= platform_min_y && cell_center_y <= platform_max_y);

                    bool in_pipe_h = (cell_center_y >= pipe_h_min_y && cell_center_y <= pipe_h_max_y);
                    bool in_pipe_h_left = in_pipe_h && (cell_center_x < cx);
                    bool in_pipe_h_right = in_pipe_h && (cell_center_x > cx);

                    bool in_pipe_v = (cell_center_x >= pipe_v_min_x && cell_center_x <= pipe_v_max_x);
                    bool in_pipe_v_top = in_pipe_v && (cell_center_y > cy);
                    bool in_pipe_v_bottom = in_pipe_v && (cell_center_y < cy);

                    // Update counters based on coverage
                    if (in_platform) {
                        platform_total_cells++;
                        if (covered_grid[cell_x][cell_y]) platform_covered_cells++;
                    }

                    if (in_pipe_h_left) {
                        pipe_h_left_total_cells++;
                        if (covered_grid[cell_x][cell_y]) pipe_h_left_covered_cells++;
                    }

                    if (in_pipe_h_right) {
                        pipe_h_right_total_cells++;
                        if (covered_grid[cell_x][cell_y]) pipe_h_right_covered_cells++;
                    }

                    if (in_pipe_v_top) {
                        pipe_v_top_total_cells++;
                        if (covered_grid[cell_x][cell_y]) pipe_v_top_covered_cells++;
                    }

                    if (in_pipe_v_bottom) {
                        pipe_v_bottom_total_cells++;
                        if (covered_grid[cell_x][cell_y]) pipe_v_bottom_covered_cells++;
                    }
                }
            }

            // Calculate coverage ratios for each area
            T platform_coverage = (platform_total_cells > 0) ?
                                  platform_covered_cells / platform_total_cells : 0;
            T pipe_h_left_coverage = (pipe_h_left_total_cells > 0) ?
                                     pipe_h_left_covered_cells / pipe_h_left_total_cells : 0;
            T pipe_h_right_coverage = (pipe_h_right_total_cells > 0) ?
                                      pipe_h_right_covered_cells / pipe_h_right_total_cells : 0;
            T pipe_v_top_coverage = (pipe_v_top_total_cells > 0) ?
                                    pipe_v_top_covered_cells / pipe_v_top_total_cells : 0;
            T pipe_v_bottom_coverage = (pipe_v_bottom_total_cells > 0) ?
                                       pipe_v_bottom_covered_cells / pipe_v_bottom_total_cells : 0;

            // Calculate total priority area coverage
            TI total_priority_cells = platform_total_cells + pipe_h_left_total_cells +
                                      pipe_h_right_total_cells + pipe_v_top_total_cells +
                                      pipe_v_bottom_total_cells;

            TI total_covered_cells = platform_covered_cells + pipe_h_left_covered_cells +
                                     pipe_h_right_covered_cells + pipe_v_top_covered_cells +
                                     pipe_v_bottom_covered_cells;

            T overall_coverage = (total_priority_cells > 0) ? T(total_covered_cells) / T(total_priority_cells) : 0;

            // Assign rewards based on coverage ratios
            total_reward += coverage_reward_total * overall_coverage;  // Overall coverage reward

            // Update coverage metrics in next_state (only during exploration mode)
            if (!detecting_count && !state.disaster_detected_global) {
                // Copy previous values and add current measurement
                next_state.metrics.total_coverage_ratio = state.metrics.total_coverage_ratio + overall_coverage;
                next_state.metrics.coverage_measurement_count = state.metrics.coverage_measurement_count + 1;
            } else {
                // Just copy previous values when not in exploration mode
                next_state.metrics.total_coverage_ratio = state.metrics.total_coverage_ratio;
                next_state.metrics.coverage_measurement_count = state.metrics.coverage_measurement_count;
            }

            // Additional rewards for individual areas
            total_reward += platform_coverage_reward * platform_coverage;     // Platform is most important
            total_reward += pipe_coverage_reward * pipe_h_left_coverage;  // Pipe sections each get 0.6
            total_reward += pipe_coverage_reward * pipe_h_right_coverage;
            total_reward += pipe_coverage_reward * pipe_v_top_coverage;
            total_reward += pipe_coverage_reward * pipe_v_bottom_coverage;
        }

        if (next_state.disaster.active) {
            if (next_state.disaster.position[0] < 0 ||
                next_state.disaster.position[0] >= PARAMS::GRID_SIZE_X ||
                next_state.disaster.position[1] < 0 ||
                next_state.disaster.position[1] >= PARAMS::GRID_SIZE_Y) {

                if (detecting_count > 0) {
                    total_reward += T(2.0);
                } else {
                    total_reward -= T(5.0);
                }
            }
        }
        utils::assert_exit(device, !math::is_nan(device.math, total_reward), "reward is nan");

        return total_reward;
    }

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
        using PARAMS = typename SPEC::PARAMETERS;
        for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; agent_i++) {
            const auto &agent_state = state.drone_states[agent_i];
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 0, 2 * (agent_state.position[0] / PARAMS::GRID_SIZE_X) - 1);  // Normalized position [-1,1]
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 1, 2 * (agent_state.position[1] / PARAMS::GRID_SIZE_Y) - 1);  // Normalized position [-1,1]
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 2, agent_state.dead ? 0 : agent_state.velocity[0] / PARAMS::MAX_SPEED);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 3, agent_state.dead ? 0 : agent_state.velocity[1] / PARAMS::MAX_SPEED);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 4, agent_state.dead ? -1 : (agent_state.is_detecting ? 1 : -1));
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 5, 2 * (agent_state.battery / 100) - 1);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 6, agent_state.dead ? 1 : -1);
            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 7, agent_state.dead ? -1 : agent_state.is_charging ? 1 : -1);
        }

        TI shared_offset = PARAMS::N_AGENTS * PER_AGENT_OBS_DIM;
        set(observation, 0, shared_offset + 0, state.disaster_detected_global ? 1 : -1);
        set(observation, 0, shared_offset + 1, !state.disaster_detected_global ? 0 : 2 * (state.last_detected_disaster_position[0]/PARAMS::GRID_SIZE_X) - 1);
        set(observation, 0, shared_offset + 2, !state.disaster_detected_global ? 0 : 2 * (state.last_detected_disaster_position[1]/PARAMS::GRID_SIZE_Y) - 1);
        set(observation, 0, shared_offset + 3, 2 * (PARAMS::CHARGING_STATION_POSITION_X / PARAMS::GRID_SIZE_X) - 1);
        set(observation, 0, shared_offset + 4, 2 * (PARAMS::CHARGING_STATION_POSITION_Y / PARAMS::GRID_SIZE_Y) - 1);

        utils::assert_exit(device, !is_nan(device, observation), "Observation is nan");

    }

    template<typename DEVICE, typename SPEC, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static bool terminated(
            DEVICE & device,
            const rl::environments::multi_agent::OilPlatform<SPEC> & env,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            RNG & rng
    ) {
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using PARAMS = typename SPEC::PARAMETERS;

        bool terminate = false;

        // Check if all drones are dead
        bool any_alive = false;
        for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; ++agent_i) {
            utils::assert_exit(device, (!state.drone_states[agent_i].dead == false) || (!state.drone_states[agent_i].dead == true), "alive is nan");
            if (!state.drone_states[agent_i].dead) {
                any_alive = true;
                break;
            }
        }

        if (!any_alive) {
            terminate = true;
        }

        // Check if disaster left the environment
        if (!terminate && state.disaster.active) {
            if (state.disaster.position[0] < 0 ||
                state.disaster.position[0] >= PARAMS::GRID_SIZE_X ||
                state.disaster.position[1] < 0 ||
                state.disaster.position[1] >= PARAMS::GRID_SIZE_Y) {
                terminate = true;
            }
        }

        // LOG METRICS WHEN EPISODE TERMINATES
        if (terminate) {
            // Basic episode metrics
            add_scalar(device, device.logger, "episode/total_steps", state.step_count);
            add_scalar(device, device.logger, "episode/final_deaths", state.metrics.death_count);

            // Coverage metrics
            if (state.metrics.coverage_measurement_count > 0) {
                T average_coverage = state.metrics.total_coverage_ratio / state.metrics.coverage_measurement_count;
                add_scalar(device, device.logger, "coverage/average_priority_area_coverage", average_coverage);
            }
            add_scalar(device, device.logger, "coverage/measurement_count", state.metrics.coverage_measurement_count);

            add_scalar(device, device.logger, "disaster/active_steps", state.metrics.disaster_active_steps);

            // Charging behavior metrics
            add_scalar(device, device.logger, "charging/total_sessions", state.metrics.total_charging_sessions);
            add_scalar(device, device.logger, "charging/appropriate_sessions", state.metrics.appropriate_charging_count);
            add_scalar(device, device.logger, "charging/inappropriate_sessions", state.metrics.inappropriate_charging_count);

            if (state.metrics.total_charging_sessions > 0) {
                T charging_efficiency = state.metrics.appropriate_charging_count / state.metrics.total_charging_sessions;
                add_scalar(device, device.logger, "charging/efficiency", charging_efficiency);
            }

            // Battery management metrics
            T total_battery = 0;
            T min_battery = 100;
            T max_battery = 0;
            TI alive_count = 0;

            for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; ++agent_i) {
                if (!state.drone_states[agent_i].dead) {
                    alive_count++;
                    T battery = state.drone_states[agent_i].battery;
                    total_battery += battery;
                    min_battery = math::min(device.math, min_battery, battery);
                    max_battery = math::max(device.math, max_battery, battery);
                }
            }

            if (alive_count > 0) {
                add_scalar(device, device.logger, "battery/average_final", total_battery / alive_count);
                add_scalar(device, device.logger, "battery/min_final", min_battery);
                add_scalar(device, device.logger, "battery/max_final", max_battery);
            }
            add_scalar(device, device.logger, "agents/alive_at_end", alive_count);

            // Survival rate
            T survival_rate = alive_count / PARAMS::N_AGENTS;
            add_scalar(device, device.logger, "agents/survival_rate", survival_rate);
        }

        return terminate;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif  // RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_GENERIC_H