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

    template<typename DEVICE, typename PARAMS, typename T, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_roi_position(
            DEVICE &device, T &x, T &y, RNG &rng) {
        using TI = typename PARAMS::TI;
        TI idx = random::uniform_int_distribution(device.random, TI(0),
                                                  PARAMS::ROI_SIZE - 1, rng);
        x = PARAMS::ROI_CATALOGUE[idx].x;
        y = PARAMS::ROI_CATALOGUE[idx].y;
    }

    template<typename DEVICE, typename PARAMS, typename T, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_charging_station_position(
            DEVICE &device, T &x, T &y, RNG &rng) {
        if (!PARAMS::RANDOMIZE_CHARGING_STATION_POSITION) {
            x = PARAMS::CHARGING_STATION_POSITION_X;
            y = PARAMS::CHARGING_STATION_POSITION_Y;
            return;
        }

        const T min_x = PARAMS::CHARGING_STATION_RANGE;
        const T max_x = T(PARAMS::GRID_SIZE_X) - PARAMS::CHARGING_STATION_RANGE;
        const T min_y = PARAMS::CHARGING_STATION_RANGE;
        const T max_y = T(PARAMS::GRID_SIZE_Y) - PARAMS::CHARGING_STATION_RANGE;

        if (max_x < min_x || max_y < min_y) {
            x = T(PARAMS::GRID_SIZE_X) * T(0.5);
            y = T(PARAMS::GRID_SIZE_Y) * T(0.5);
            return;
        }

        x = random::uniform_real_distribution(device.random, min_x, max_x, rng);
        y = random::uniform_real_distribution(device.random, min_y, max_y, rng);
    }

    template<typename DEVICE, typename PARAMS>
    RL_TOOLS_FUNCTION_PLACEMENT static
    typename PARAMS::T inbounds_fraction(DEVICE &device,
                                         typename PARAMS::T x,
                                         typename PARAMS::T y) {
        using T = typename PARAMS::T;
        const T R = PARAMS::SENSOR_RANGE;
        /* distance to each wall */
        T dxL = x;
        T dxR = PARAMS::GRID_SIZE_X - x;
        T dyB = y;
        T dyT = PARAMS::GRID_SIZE_Y - y;

        T f = T(1);               // full circle by default
        /* each wall cuts a circular segment if distance<R */
        if (dxL < R) f -= math::pow(device.math, (R - dxL) / R, 2);
        if (dxR < R) f -= math::pow(device.math, (R - dxR) / R, 2);
        if (dyB < R) f -= math::pow(device.math, (R - dyB) / R, 2);
        if (dyT < R) f -= math::pow(device.math, (R - dyT) / R, 2);
        return math::clamp(device.math, f, T(0), T(1));
    }

    // Helper function to get grid cell from position
    template<typename DEVICE, typename PARAMS, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static void get_grid_cell(
            DEVICE& device,
            T x, T y,
            typename PARAMS::TI& grid_x,
            typename PARAMS::TI& grid_y
    ) {
        using TI = typename PARAMS::TI;

        // Clamp to grid boundaries
        T norm_x = math::clamp(device.math, x / PARAMS::GRID_SIZE_X, T(0), T(0.999));
        T norm_y = math::clamp(device.math, y / PARAMS::GRID_SIZE_Y, T(0), T(0.999));

        grid_x = static_cast<TI>(norm_x * PARAMS::GRID_CELLS_X);
        grid_y = static_cast<TI>(norm_y * PARAMS::GRID_CELLS_Y);
    }


    template<typename DEVICE, typename PARAMS, typename T, typename TI>
    RL_TOOLS_FUNCTION_PLACEMENT
    static T multicenter_potential(DEVICE& device, T x, T y) {
        auto& m = device.math;

        // Gather Gaussian energies (log of the bumps without amplitude normalization)
        T e_max = -std::numeric_limits<T>::infinity();
        T inv_tau = T(0);
        if (PARAMS::MULTIGAUSS_SOFTMAX_TAU > T(0)) {
            inv_tau = T(1) / PARAMS::MULTIGAUSS_SOFTMAX_TAU;
        }

        // Hard-max accumulation
        T e_hard_max = e_max;

        // Softmax accumulation (if tau>0)
        T a_max = -std::numeric_limits<T>::infinity();
        T sum_exp = T(0);

        for (TI i = 0; i < PARAMS::N_GAUSS; ++i) {
            const auto& g = PARAMS::GAUSS_CATALOGUE[i];
            T qx = (x - g.cx) / g.sx;
            T qy = (y - g.cy) / g.sy;
            T e  = T(-0.5) * (qx*qx + qy*qy) + math::log(m, g.A);  // log(amplitude * exp(...))

            // hard max
            e_hard_max = math::max(m, e_hard_max, e);

            // softmax
            if (PARAMS::MULTIGAUSS_SOFTMAX_TAU > T(0)) {
                T a = e * inv_tau;
                a_max = math::max(m, a_max, a);
                sum_exp += math::exp(m, a); // We'll do the max-trick below for stability
            }
        }

        if (PARAMS::MULTIGAUSS_SOFTMAX_TAU == T(0)) {
            return math::exp(m, e_hard_max);   // in [0,1]
        } else {
            // Numerically-stable log-sum-exp
            // Recompute with max-trick for stability:
            sum_exp = T(0);
            for (TI i = 0; i < PARAMS::N_GAUSS; ++i) {
                const auto& g = PARAMS::GAUSS_CATALOGUE[i];
                T qx = (x - g.cx) / g.sx;
                T qy = (y - g.cy) / g.sy;
                T e  = T(-0.5) * (qx*qx + qy*qy) + math::log(m, g.A);
                sum_exp += math::exp(m, e * inv_tau - a_max);
            }
            T lse = a_max + math::log(m, sum_exp);
            return math::exp(m, PARAMS::MULTIGAUSS_SOFTMAX_TAU * lse);  // ≈ max_i exp(e_i) as tau→0
        }
    }


//    template<typename DEVICE, typename PARAMS, typename T>
//    RL_TOOLS_FUNCTION_PLACEMENT static T gaussian_potential(DEVICE &device,
//                                                            T px, T py,              // point to evaluate
//                                                            T cx, T cy,              // platform centre
//                                                            T sigma)                 // Gaussian width
//    {
//        T adx = math::abs(device.math, px - cx);
//        T ady = math::abs(device.math, py - cy);
//
//        /* square platform */
//        T dx_pl = math::max(device.math, adx - PARAMS::PLATFORM_HALF_SIZE, T(0));
//        T dy_pl = math::max(device.math, ady - PARAMS::PLATFORM_HALF_SIZE, T(0));
//        T d2_pl = dx_pl * dx_pl + dy_pl * dy_pl;
//
//        /* horizontal pipe */
//        T dy_ph = math::max(device.math, ady - PARAMS::PIPE_WIDTH / 2, T(0));
//        T d2_ph = dy_ph * dy_ph;
//
//        /* vertical pipe */
//        T dx_pv = math::max(device.math, adx - PARAMS::PIPE_WIDTH / 2, T(0));
//        T d2_pv = dx_pv * dx_pv;
//
//        T d2 = math::min(device.math, d2_pl,
//                         math::min(device.math, d2_ph, d2_pv));
//
//        return math::exp(device.math, -d2 / (T(2) * sigma * sigma));
//    }

    template<typename DEVICE, typename PARAMS>
    RL_TOOLS_FUNCTION_PLACEMENT static
    typename PARAMS::T calculate_overlap_penalty(DEVICE &device,
                                                 const typename PARAMS::T (&pos)[PARAMS::N_AGENTS][2],
                                                 const bool disaster_phase) {
        using T = typename PARAMS::T;
        using TI = typename PARAMS::TI;
        T sum = 0;
        for (TI i = 0; i < PARAMS::N_AGENTS; ++i) {
            for (TI j = i + 1; j < PARAMS::N_AGENTS; ++j) {
                T d = distance(device, pos[i][0], pos[i][1], pos[j][0], pos[j][1]);
                if (disaster_phase) {
                    sum += math::exp(device.math,
                                     -(d * d) / (PARAMS::OVERLAP_RHO_DETECTION * PARAMS::OVERLAP_RHO_DETECTION));
                } else {
                    sum += math::exp(device.math,
                                     -(d * d) / (PARAMS::OVERLAP_RHO_COVERAGE * PARAMS::OVERLAP_RHO_COVERAGE));
                }
            }
        }
        return sum;
    }

    /* --- Voronoi-based coverage reward ------------------------------------ */
    /* Computes coverage quality using Voronoi tessellation:
       - Each grid cell is assigned to its nearest agent
       - Agents are rewarded for covering ROI (platform + pipes) cells, weighted by battery state
       - Variance penalty encourages even distribution
       This replaces Gaussian attraction + repulsion when USE_VORONOI_COVERAGE = true */
    template<typename DEVICE, typename PARAMS>
    RL_TOOLS_FUNCTION_PLACEMENT static
    typename PARAMS::T calculate_voronoi_coverage_reward(
            DEVICE &device,
            const typename PARAMS::T (&pos)[PARAMS::N_AGENTS][2],
            const bool (&dead)[PARAMS::N_AGENTS],
            const typename PARAMS::T (&coverage_weights)[PARAMS::N_AGENTS]  // Per-agent battery-based weights
    ) {
        using T = typename PARAMS::T;
        using TI = typename PARAMS::TI;

        // Initialize per-agent ROI cell counters
        T voronoi_roi_value[PARAMS::N_AGENTS];
        for (TI i = 0; i < PARAMS::N_AGENTS; ++i) {
            voronoi_roi_value[i] = T(0);
        }

        // Count alive agents and sum their weights
        TI alive_count = 0;
        T total_weight = T(0);
        for (TI i = 0; i < PARAMS::N_AGENTS; ++i) {
            if (!dead[i]) {
                ++alive_count;
                total_weight += coverage_weights[i];
            }
        }

        if (alive_count == 0 || total_weight < T(1e-6)) return T(0);  // No alive agents or no coverage capacity

        // Grid parameters
        constexpr T DX = T(PARAMS::GRID_SIZE_X) / PARAMS::GRID_RES;
        constexpr T DY = T(PARAMS::GRID_SIZE_Y) / PARAMS::GRID_RES;
        constexpr T CX = PARAMS::GRID_SIZE_X * T(0.5);
        constexpr T CY = PARAMS::GRID_SIZE_Y * T(0.5);

        // Sweep through grid cells and assign to nearest agent
        for (TI gx = 0; gx < PARAMS::GRID_RES; ++gx) {
            for (TI gy = 0; gy < PARAMS::GRID_RES; ++gy) {
                T cx_cell = DX * (gx + T(0.5));
                T cy_cell = DY * (gy + T(0.5));

                // Check if cell is in ROI (platform or pipes)
                T adx = math::abs(device.math, cx_cell - CX);
                T ady = math::abs(device.math, cy_cell - CY);

                bool in_platform = (adx <= PARAMS::PLATFORM_HALF_SIZE &&
                                   ady <= PARAMS::PLATFORM_HALF_SIZE);
                bool in_pipe_h = (ady <= PARAMS::PIPE_WIDTH * T(0.5) &&
                                 adx >= PARAMS::PLATFORM_HALF_SIZE);
                bool in_pipe_v = (adx <= PARAMS::PIPE_WIDTH * T(0.5) &&
                                 ady >= PARAMS::PLATFORM_HALF_SIZE);

                if (!(in_platform || in_pipe_h || in_pipe_v)) {
                    continue;  // Skip non-ROI cells
                }

                // Find nearest alive agent to this cell (within sensor range)
                TI nearest = PARAMS::N_AGENTS;
                T min_dist_sq = std::numeric_limits<T>::infinity();
                constexpr T SENSOR_RANGE_SQ = PARAMS::SENSOR_RANGE * PARAMS::SENSOR_RANGE;

                for (TI i = 0; i < PARAMS::N_AGENTS; ++i) {
                    if (dead[i]) continue;

                    T dx = cx_cell - pos[i][0];
                    T dy = cy_cell - pos[i][1];
                    T dist_sq = dx*dx + dy*dy;

                    // Only consider agents within sensor range
                    if (dist_sq < min_dist_sq && dist_sq <= SENSOR_RANGE_SQ) {
                        min_dist_sq = dist_sq;
                        nearest = i;
                    }
                }

                // Assign cell to nearest agent ONLY if within sensor range
                // This ensures agents must actually be close to ROI cells to get credit
                // Similar to Gaussian approach where distance matters (exponential decay)
                if (nearest < PARAMS::N_AGENTS) {
                    voronoi_roi_value[nearest] += coverage_weights[nearest];
                }
                // If no agent is within sensor range of this cell, it remains uncovered (no reward)
            }
        }

        // Calculate total weighted coverage
        T total_coverage = T(0);

        for (TI i = 0; i < PARAMS::N_AGENTS; ++i) {
            total_coverage += voronoi_roi_value[i];
        }

        // Normalize coverage to [0,1] range
        // Removed agents_contributing multiplier to prevent infinite spreading incentive
        // Voronoi tessellation itself naturally encourages spreading (each agent wants their own cells)
        // but without the multiplier, spreading stops at optimal coverage distance
        T coverage_fraction = total_coverage / T(PARAMS::ROI_SIZE);

        return coverage_fraction;
    }

    /* --- helper: logistic sigmoid ----------------------------------------- */
    template<typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static T logistic(DEVICE &device, T x) {
        return T(1) / (T(1) + math::exp(device.math, -x));
    }

    /* ---------- helper: one-shot coverage ratio over high-priority areas --- */
    template<typename DEVICE, typename SPEC>
    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T
    priority_area_coverage(DEVICE &device,
                           const typename SPEC::STATE &s_next) {
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        using PARAMS = typename SPEC::PARAMETERS;
        constexpr TI N = PARAMS::N_AGENTS;
        constexpr TI RES = PARAMS::GRID_RES;          // 20
        constexpr T DX = T(PARAMS::GRID_SIZE_X) / RES;
        constexpr T DY = T(PARAMS::GRID_SIZE_Y) / RES;
        constexpr T CX = PARAMS::GRID_SIZE_X * T(0.5);
        constexpr T CY = PARAMS::GRID_SIZE_Y * T(0.5);

        /* platform & pipe masks pre-computed once */
        TI total_priority = 0, covered_priority = 0;
        for (TI gx = 0; gx < RES; ++gx) {
            for (TI gy = 0; gy < RES; ++gy) {
                T cx_cell = DX * (gx + T(0.5));
                T cy_cell = DY * (gy + T(0.5));

                bool in_platform = (std::abs(cx_cell - CX) <= PARAMS::PLATFORM_HALF_SIZE &&
                                    std::abs(cy_cell - CY) <= PARAMS::PLATFORM_HALF_SIZE);

                bool in_pipe_h = (std::abs(cy_cell - CY) <= PARAMS::PIPE_WIDTH * T(0.5) &&
                                  std::abs(cx_cell - CX) >= PARAMS::PLATFORM_HALF_SIZE);

                bool in_pipe_v = (std::abs(cx_cell - CX) <= PARAMS::PIPE_WIDTH * T(0.5) &&
                                  std::abs(cy_cell - CY) >= PARAMS::PLATFORM_HALF_SIZE);

                bool is_priority = in_platform || in_pipe_h || in_pipe_v;
                if (!is_priority) continue;
                ++total_priority;

                /* covered if ANY live drone has the cell centre within sensor range */
                for (TI i = 0; i < N; ++i) {
                    const auto &d = s_next.drone_states[i];
                    if (d.dead) continue;
                    T dist = (d.position[0] - cx_cell) * (d.position[0] - cx_cell)
                             + (d.position[1] - cy_cell) * (d.position[1] - cy_cell);
                    if (dist <= PARAMS::SENSOR_RANGE * PARAMS::SENSOR_RANGE) {
                        ++covered_priority;
                        break;
                    }
                }
            }
        }
        return (total_priority > 0) ? T(covered_priority) / T(total_priority) : T(0);
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

    template<typename DEVICE, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static T sigmoid(DEVICE& device, T z){
        return T(1) / (T(1) + math::exp(device.math, -z));
    }

// Smooth hysteresis weight h(b,latch) in [0,1]
    template<typename DEVICE, typename PARAMS>
    RL_TOOLS_FUNCTION_PLACEMENT static typename PARAMS::T
    hysteresis_weight(DEVICE& device,
                      typename PARAMS::T b01,      // battery in [0,1]
                      bool latch)
    {
        using T = typename PARAMS::T;
        // rises as battery drops (engage curve)
        T rise = sigmoid(device, (PARAMS::B_ON - b01) / PARAMS::B_SMOOTH_ON);
        // stays high and falls only near B_OFF (release curve)
        T fall = T(1) - sigmoid(device, (b01 - PARAMS::B_OFF) / PARAMS::B_SMOOTH_OFF);
        return latch ? fall : rise;
    }

    // Linear weight h(b) in [0,1]:
// h = 1 - b01  → 0 when full (no charging urge), 1 when empty (max urge)
    template<typename DEVICE, typename PARAMS>
    RL_TOOLS_FUNCTION_PLACEMENT static typename PARAMS::T
    linear_weight(DEVICE& device,
                      typename PARAMS::T b01)   // battery in [0,1]
    {
        using T = typename PARAMS::T;
        // Clamp b01 just in case, then return 1 - b01
        T b = math::clamp(device.math, b01, T(0), T(1));
        return T(1) - b;
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
        using PARAMS = typename SPEC::PARAMETERS;

        sample_charging_station_position<DEVICE, PARAMS>(device,
                                                         state.charging_station_position[0],
                                                         state.charging_station_position[1],
                                                         rng);

        // Initialize all drone states
        for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; ++agent_i) {
            auto &agent_state = state.drone_states[agent_i];
            // Spawn drones anywhere in the environment (not just center platform)
            // agent_state.position[0] = random::uniform_real_distribution(device.random, T(PARAMS::GRID_SIZE_X)/2 - 2, T(PARAMS::GRID_SIZE_X)/2 + 2, rng);
            // agent_state.position[1] = random::uniform_real_distribution(device.random, T(PARAMS::GRID_SIZE_Y)/2 - 2, T(PARAMS::GRID_SIZE_Y)/2 + 2, rng);
            agent_state.position[0] = random::uniform_real_distribution(device.random, T(0), T(PARAMS::GRID_SIZE_X), rng);
            agent_state.position[1] = random::uniform_real_distribution(device.random, T(0), T(PARAMS::GRID_SIZE_Y), rng);
            // agent_state.position[0] = T(PARAMS::GRID_SIZE_X/2);
            // agent_state.position[1] = T(PARAMS::GRID_SIZE_Y/2);

            agent_state.velocity[0] = 0;
            agent_state.velocity[1] = 0;
            if (PARAMS::BATTERY_ENABLED){
                agent_state.battery = random::uniform_real_distribution(device.random, T(50), T(100), rng);
            } else {
                agent_state.battery = 100;
            }
            agent_state.dead = false;
            agent_state.is_charging = false;
            agent_state.is_detecting = false;

            // Initialize grid tracking
//            get_grid_cell<DEVICE, PARAMS>(device,
//                                          agent_state.position[0],
//                                          agent_state.position[1],
//                                          agent_state.current_grid_x,
//                                          agent_state.current_grid_y);
//            agent_state.steps_in_current_cell = 0;
//            agent_state.charge_latch = false;
            agent_state.charge_hold_remaining = 0;
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
        state.metrics.cumulative_potential_reward = 0;
        state.metrics.potential_steps = 0;

        state.disaster_spawn_step = 0;
        state.metrics.cumulative_detection_latency = 0;
        state.metrics.detection_count = 0;

        state.metrics.total_disasters_spawned = 0;
        state.metrics.disasters_missed = 0;

        state.metrics.per_step_reward = 0;
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
        using PARAMS = typename SPEC::PARAMETERS;

        state.charging_station_position[0] = PARAMS::CHARGING_STATION_POSITION_X;
        state.charging_station_position[1] = PARAMS::CHARGING_STATION_POSITION_Y;

        // Initialize all drone states
        // Distribute drones evenly across the environment in a grid pattern
        TI agents_per_row = static_cast<TI>(math::sqrt(device.math, T(SPEC::PARAMETERS::N_AGENTS))) + 1;
        T spacing_x = T(SPEC::PARAMETERS::GRID_SIZE_X) / T(agents_per_row + 1);
        T spacing_y = T(SPEC::PARAMETERS::GRID_SIZE_Y) / T(agents_per_row + 1);
        
        for (TI agent_i = 0; agent_i < SPEC::PARAMETERS::N_AGENTS; ++agent_i) {
            auto &agent_state = state.drone_states[agent_i];
            // Distribute drones in a grid pattern across the environment
            TI row = agent_i / agents_per_row;
            TI col = agent_i % agents_per_row;
            agent_state.position[0] = (col + 1) * spacing_x;
            agent_state.position[1] = (row + 1) * spacing_y;
            agent_state.velocity[0] = 0;
            agent_state.velocity[1] = 0;
            if (PARAMS::BATTERY_ENABLED){
                agent_state.battery = 70;
            } else {
                agent_state.battery = 100;
            }
            agent_state.dead = false;
            agent_state.is_charging = false;
            agent_state.is_detecting = false;

            // Initialize grid tracking
//            get_grid_cell<DEVICE, PARAMS>(device,
//                                                    agent_state.position[0],
//                                                    agent_state.position[1],
//                                                    agent_state.current_grid_x,
//                                                    agent_state.current_grid_y);
//            agent_state.steps_in_current_cell = 0;
//            agent_state.charge_latch = false;
            agent_state.charge_hold_remaining = 0;
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
        state.metrics.cumulative_potential_reward = 0;
        state.metrics.potential_steps = 0;

        state.disaster_spawn_step = 0;
        state.metrics.cumulative_detection_latency = 0;
        state.metrics.detection_count = 0;

        state.metrics.total_disasters_spawned = 0;
        state.metrics.disasters_missed = 0;
        state.metrics.per_step_reward = 0;
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
            const rl::environments::multi_agent::OilPlatform<SPEC> &env,
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


//        const T cx = PARAMS::GRID_SIZE_X / T(2);
//        const T cy = PARAMS::GRID_SIZE_Y / T(2);

        next_state.metrics = state.metrics;
        next_state.charging_station_position[0] = state.charging_station_position[0];
        next_state.charging_station_position[1] = state.charging_station_position[1];


// (1) Disaster update
        if (!state.disaster.active) {
            if (state.step_count >= PARAMS::DISASTER_MINIMUM_SPAWN_STEP &&
                random::uniform_real_distribution(device.random, T(0), T(1), rng) <
                T(PARAMS::DISASTER_PROBABILITY_SPAWN)) {
                T x, y;
                sample_roi_position<DEVICE, PARAMS>(device, x, y, rng);
                next_state.disaster.active = true;
                next_state.disaster.position[0] = x;
                next_state.disaster.position[1] = y;

                // Generate random velocity for the disaster
                T angle = random::uniform_real_distribution(device.random, T(0), T(2 * M_PI), rng);
                T speed = random::uniform_real_distribution(device.random, T(0), T(PARAMS::DISASTER_MAX_SPEED), rng);
//                next_state.disaster.velocity[0] = PARAMS::DISASTER_MAX_SPEED * math::cos(device.math, angle);
//                next_state.disaster.velocity[1] = PARAMS::DISASTER_MAX_SPEED * math::sin(device.math, angle);
                next_state.disaster.velocity[0] = speed * math::cos(device.math, angle);
                next_state.disaster.velocity[1] = speed * math::sin(device.math, angle);
                next_state.disaster_spawn_step = state.step_count;

                // Increment disaster count
                next_state.metrics.total_disasters_spawned = state.metrics.total_disasters_spawned + 1;
            } else {
                next_state.disaster_spawn_step = state.disaster_spawn_step;
                next_state.disaster.active = false;
                next_state.disaster.position[0] = state.disaster.position[0];
                next_state.disaster.position[1] = state.disaster.position[1];
                next_state.disaster.velocity[0] = state.disaster.velocity[0];
                next_state.disaster.velocity[1] = state.disaster.velocity[1];
                next_state.metrics.total_disasters_spawned = state.metrics.total_disasters_spawned;
            }
        } else {
//            // MODIFIED: Update disaster position based on velocity
//            next_state.disaster.active = true;
//            next_state.disaster.position[0] = state.disaster.position[0] + state.disaster.velocity[0] * PARAMS::DT;
//            next_state.disaster.position[1] = state.disaster.position[1] + state.disaster.velocity[1] * PARAMS::DT;
//            next_state.disaster.velocity[0] = state.disaster.velocity[0];
//            next_state.disaster.velocity[1] = state.disaster.velocity[1];
//            next_state.disaster_spawn_step = state.disaster_spawn_step;

            /* ────────────────────────────────────────────────────────────────────
               Disaster wandering update
               - Start from previous velocity
               - Apply small random angular jitter
               - Apply small random speed jitter (fractional)
               - Clamp to [0, DISASTER_MAX_SPEED]
               - Integrate forward
               ──────────────────────────────────────────────────────────────────── */

            // Update disaster position with wandering logic
            next_state.disaster.active = true;
            next_state.disaster_spawn_step = state.disaster_spawn_step;
            next_state.metrics.total_disasters_spawned = state.metrics.total_disasters_spawned;

            // Current velocity
            T vx = state.disaster.velocity[0];
            T vy = state.disaster.velocity[1];

            // Current speed
            T vmag = magnitude(device, vx, vy);

            // If zero (can happen if spawned with 0 speed), nudge to tiny random dir
            if (vmag <= T(1e-6)) {
                T ang0 = random::uniform_real_distribution(device.random, T(0), T(2 * M_PI), rng);
                vx = math::cos(device.math, ang0) * PARAMS::DISASTER_MAX_SPEED * T(0.25);
                vy = math::sin(device.math, ang0) * PARAMS::DISASTER_MAX_SPEED * T(0.25);
                vmag = magnitude(device, vx, vy);
            }

            // Small random heading change
            T dtheta = random::uniform_real_distribution(device.random,
                                                         -PARAMS::DISASTER_TURN_MAX_RAD,
                                                         +PARAMS::DISASTER_TURN_MAX_RAD,
                                                         rng);
            T c = math::cos(device.math, dtheta);
            T s = math::sin(device.math, dtheta);
            T vx_r = vx * c - vy * s;
            T vy_r = vx * s + vy * c;

            // Small random speed jitter
            T dv_frac = random::uniform_real_distribution(device.random,
                                                          -PARAMS::DISASTER_SPEED_JITTER_FRAC,
                                                          +PARAMS::DISASTER_SPEED_JITTER_FRAC,
                                                          rng);
            T speed_target = vmag + dv_frac * PARAMS::DISASTER_MAX_SPEED;

            // Clamp to [0, MAX]
            speed_target = math::clamp(device.math, speed_target, T(0), PARAMS::DISASTER_MAX_SPEED);

            // Renormalize rotated velocity to speed_target
            T vmag_r = magnitude(device, vx_r, vy_r);
            if (vmag_r > T(1e-6)) {
                T scale = speed_target / vmag_r;
                vx_r *= scale;
                vy_r *= scale;
            } else {
                T ang_resamp = random::uniform_real_distribution(device.random, T(0), T(2 * M_PI), rng);
                vx_r = math::cos(device.math, ang_resamp) * speed_target;
                vy_r = math::sin(device.math, ang_resamp) * speed_target;
            }

            // Integrate forward
            T new_x = state.disaster.position[0] + vx_r * PARAMS::DT;
            T new_y = state.disaster.position[1] + vy_r * PARAMS::DT;

            // Check if disaster left the environment
            if (new_x < 0 || new_x >= PARAMS::GRID_SIZE_X ||
                new_y < 0 || new_y >= PARAMS::GRID_SIZE_Y) {
                // Disaster exits - deactivate it
                next_state.disaster.active = false;
                next_state.disaster.position[0] = 0;
                next_state.disaster.position[1] = 0;
                next_state.disaster.velocity[0] = 0;
                next_state.disaster.velocity[1] = 0;

                // Reset detection state
                next_state.disaster_detected_global = false;
                next_state.last_detected_disaster_position[0] = 0;
                next_state.last_detected_disaster_position[1] = 0;
                next_state.disaster_undetected_steps = 0;

                // Update missed disasters count if it was never detected
                if (!state.disaster_detected_global) {
                    next_state.metrics.disasters_missed = state.metrics.disasters_missed + 1;
                } else {
                    next_state.metrics.disasters_missed = state.metrics.disasters_missed;
                }
            } else {
                // Normal position update
                next_state.disaster.position[0] = new_x;
                next_state.disaster.position[1] = new_y;
                next_state.disaster.velocity[0] = vx_r;
                next_state.disaster.velocity[1] = vy_r;
                next_state.metrics.disasters_missed = state.metrics.disasters_missed;
            }
        }

//        // (2) Per-drone update for position and velocity, acceleration control
//        for (TI agent_i = 0; agent_i < N_AGENTS; agent_i++) {
//            const auto &agent_state = state.drone_states[agent_i];
//            auto &agent_next_state = next_state.drone_states[agent_i];
//            agent_next_state = agent_state;
//
//            if (!agent_state.dead) {
//                // ACCELERATION CONTROL
//                T ax = get(action, 0, agent_i * 2 + 0) * PARAMS::MAX_ACCELERATION;
//                T ay = get(action, 0, agent_i * 2 + 1) * PARAMS::MAX_ACCELERATION;
//
//                // Update velocity based on acceleration
//                agent_next_state.velocity[0] = agent_state.velocity[0] + ax * PARAMS::DT;
//                agent_next_state.velocity[1] = agent_state.velocity[1] + ay * PARAMS::DT;
//
//                // Clamp velocity magnitude to MAX_SPEED
//                T speed = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
//                if (speed > PARAMS::MAX_SPEED) {
//                    T scale = PARAMS::MAX_SPEED / speed;
//                    agent_next_state.velocity[0] *= scale;
//                    agent_next_state.velocity[1] *= scale;
//                }
//
//                // Integrate velocity to get new position
//                agent_next_state.position[0] = agent_state.position[0] + agent_next_state.velocity[0] * PARAMS::DT;
//                agent_next_state.position[1] = agent_state.position[1] + agent_next_state.velocity[1] * PARAMS::DT;
//
//                // Clamp position to environment boundaries
//                T x_pre_clamping = agent_next_state.position[0];
//                T y_pre_clamping = agent_next_state.position[1];
//                agent_next_state.position[0] = math::clamp(device.math, T(x_pre_clamping), T(0), T(PARAMS::GRID_SIZE_X));
//                agent_next_state.position[1] = math::clamp(device.math, T(y_pre_clamping), T(0), T(PARAMS::GRID_SIZE_Y));
//
//                // If we clamped on the x-axis, zero out x-motion:
//                if (agent_next_state.position[0] != x_pre_clamping) {
//                    agent_next_state.velocity[0] = 0;
//                }
//
//                // Same for y:
//                if (agent_next_state.position[1] != y_pre_clamping) {
//                    agent_next_state.velocity[1] = 0;
//                }
//            } else {
//                agent_next_state = agent_state;
//            }
//        }

        // (2) Per-drone update for position and velocity, velocity control
        for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {

            const auto &agent_state = state.drone_states[agent_i];
            auto &agent_next_state = next_state.drone_states[agent_i];
            agent_next_state = agent_state;
            if (!agent_state.dead) {

                T desired_vx = get(action, 0, agent_i * 2 + 0);   // expected ∈[-1,1]
                T desired_vy = get(action, 0, agent_i * 2 + 1);   // expected ∈[-1,1]

                // In the multi-agent wrapper (ppo implementation), the default activation function of the last layer is not in [-1,1]
                // so we clamp
                desired_vx = math::clamp(device.math, desired_vx, T(-1), T(1));
                desired_vy = math::clamp(device.math, desired_vy, T(-1), T(1));

                constexpr T ALPHA = 0.6;                  // 0<α≤1  → smaller α = more inertia
                desired_vx = agent_state.velocity[0] + ALPHA * (desired_vx * PARAMS::MAX_SPEED
                                                                - agent_state.velocity[0]);
                desired_vy = agent_state.velocity[1] + ALPHA * (desired_vy * PARAMS::MAX_SPEED
                                                                - agent_state.velocity[1]);


                // Set velocity directly to desired values
                agent_next_state.velocity[0] = desired_vx;
                agent_next_state.velocity[1] = desired_vy;

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
                agent_next_state.position[0] = math::clamp(device.math, T(x_pre_clamping), T(0),
                                                           T(PARAMS::GRID_SIZE_X));
                agent_next_state.position[1] = math::clamp(device.math, T(y_pre_clamping), T(0),
                                                           T(PARAMS::GRID_SIZE_Y));

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













//        // After updating positions, update grid tracking
//        for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; agent_i++) {
//            auto &agent_next_state = next_state.drone_states[agent_i];
//            const auto &agent_state = state.drone_states[agent_i];
//
//            if (!agent_next_state.dead) {
//                // Get new grid cell
//                TI new_grid_x, new_grid_y;
//                get_grid_cell<DEVICE, PARAMS>(device,
//                                              agent_next_state.position[0],
//                                              agent_next_state.position[1],
//                                              new_grid_x,
//                                              new_grid_y);
//
//                // Check if drone moved to a new cell
////                if (new_grid_x != agent_state.current_grid_x ||
////                    new_grid_y != agent_state.current_grid_y) {
////                    // Reset counter
////                    agent_next_state.current_grid_x = new_grid_x;
////                    agent_next_state.current_grid_y = new_grid_y;
////                    agent_next_state.steps_in_current_cell = 0;
////                } else {
////                    // Increment counter
////                    agent_next_state.current_grid_x = agent_state.current_grid_x;
////                    agent_next_state.current_grid_y = agent_state.current_grid_y;
////                    agent_next_state.steps_in_current_cell = agent_state.steps_in_current_cell + 1;
////                }
//            }
//        }

        // (3) Per-drone update for battery and charging
// (3) Per-drone update for battery and charging
        for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
            auto &agent_next_state = next_state.drone_states[agent_i];
            const auto &agent_state = state.drone_states[agent_i];

            if (!agent_state.dead) {
                if constexpr (PARAMS::BATTERY_ENABLED) {
                    // Check if drone is at charging station
                    T distance_to_charging_station = distance(device,
                                                              agent_state.position[0],  // Use CURRENT position, not next
                                                              agent_state.position[1],
                                                              state.charging_station_position[0],
                                                              state.charging_station_position[1]);
                    bool at_charging_station = (distance_to_charging_station < PARAMS::CHARGING_STATION_RANGE);

                    T velocity_magnitude = magnitude(device,
                                                     agent_next_state.velocity[0],
                                                     agent_next_state.velocity[1]);
                    bool is_stationary = (velocity_magnitude <= PARAMS::CHARGING_VELOCITY_THRESHOLD);
                    bool was_charging = agent_state.is_charging;

                    // Start charging only if: not charging, at pad, stationary, and battery < 100
                    bool start_charging = (!was_charging) && at_charging_station &&
                                          is_stationary && (agent_state.battery < PARAMS::MINIMUM_BATTERY_FOR_CHARGING);

                    // Determine if still charging
                    if (start_charging) {
                        // Start new charging session
                        agent_next_state.is_charging = true;
                        agent_next_state.charge_hold_remaining = PARAMS::MIN_CHARGE_STEPS;
                    } else if (was_charging) {
                        // Continue or stop charging
                        if (agent_state.charge_hold_remaining > 0 && agent_state.battery < T(100)) {
                            // Must continue charging (still in hold period and not full)
                            agent_next_state.is_charging = true;
                            agent_next_state.charge_hold_remaining = agent_state.charge_hold_remaining - 1;
                        } else {
                            // Can stop: either hold expired or battery full
                            agent_next_state.is_charging = false;
                            agent_next_state.charge_hold_remaining = 0;
                        }
                    } else {
                        // Not charging
                        agent_next_state.is_charging = false;
                        agent_next_state.charge_hold_remaining = 0;
                    }

                    // If charging, freeze the drone completely
                    if (agent_next_state.is_charging) {
                        // Override any movement - drone must stay frozen
                        agent_next_state.velocity[0] = 0;
                        agent_next_state.velocity[1] = 0;
                        agent_next_state.position[0] = agent_state.position[0];
                        agent_next_state.position[1] = agent_state.position[1];

                        // Update battery
                        agent_next_state.battery = math::min(device.math, T(100),
                                                             agent_state.battery + PARAMS::CHARGING_RATE);
                    } else {
                        // Not charging - normal discharge
                        agent_next_state.battery = math::max(device.math, T(0),
                                                             agent_state.battery - PARAMS::DISCHARGE_RATE);
                    }

                    // Check if drone dies: either battery = 0 OR can't reach charger in time
                    bool dies_from_empty_battery = (agent_next_state.battery <= 0);
                    
                    // Calculate if agent can reach charger before running out of battery
                    bool dies_from_point_of_no_return = false;
                    if (!agent_next_state.is_charging && agent_next_state.battery > 0) {
                        // Distance to charger
                        T dx_to_charger = agent_next_state.position[0] - state.charging_station_position[0];
                        T dy_to_charger = agent_next_state.position[1] - state.charging_station_position[1];
                        T dist_to_charger = math::sqrt(device.math, dx_to_charger * dx_to_charger + 
                                                                     dy_to_charger * dy_to_charger);
                        
                        // Account for inertia: with ALPHA=0.6 damping, agent can't instantly reach/stop at MAX_SPEED
                        // Conservative estimate: multiply naive time by factor accounting for:
                        // 1. Acceleration phase (~4-5 timesteps to reach 95% of max speed)
                        // 2. Deceleration phase (~4-5 timesteps to stop)
                        // 3. Current velocity (might not be aligned with charger direction)
                        constexpr T INERTIA_FACTOR = T(1.8); // Conservative multiplier for acceleration/deceleration
                        
                        // Time to reach charger (accounting for inertia)
                        T time_to_charger = (dist_to_charger / PARAMS::MAX_SPEED) * INERTIA_FACTOR;
                        
                        // Battery needed to reach charger (accounting for discharge during travel)
                        T battery_needed = PARAMS::DISCHARGE_RATE * time_to_charger;
                        
                        // Add safety margin for positioning at charger and unexpected delays
                        T safety_margin = PARAMS::DISCHARGE_RATE * T(3); // 3 timesteps worth
                        
                        // Die if can't make it to charger even with optimal acceleration
                        dies_from_point_of_no_return = (agent_next_state.battery < battery_needed + safety_margin);
                    }
                    
                    if (dies_from_empty_battery || dies_from_point_of_no_return) {
                        agent_next_state.dead = true;
                        agent_next_state.velocity[0] = 0;
                        agent_next_state.velocity[1] = 0;
                        agent_next_state.battery = 0;
                        agent_next_state.is_charging = false;
                        agent_next_state.is_detecting = false;
                        agent_next_state.charge_hold_remaining = 0;
                    }
                } else {
                    // Battery disabled
                    agent_next_state.battery = 100;
                    agent_next_state.is_charging = false;
                    agent_next_state.dead = false;
                }
            }
        }

        /* ───── (3)  Disaster detection logic  ───────────────────────────────── */

        if (next_state.disaster.active) {
            bool any_detection = false;
            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
                auto &agent_next_state = next_state.drone_states[agent_i];

                if (agent_next_state.dead || agent_next_state.is_charging) {
                    agent_next_state.is_detecting = false;
                    continue;
                }

                /* distance to disaster */
                T d = distance(device,
                               agent_next_state.position[0], agent_next_state.position[1],
                               next_state.disaster.position[0], next_state.disaster.position[1]);

                /* smooth detection probability */
                T p_det = logistic(device, PARAMS::DETECTION_BETA * (PARAMS::SENSOR_RANGE - d));

                /* Bernoulli draw */
                bool detects = random::uniform_real_distribution(device.random,
                                                                 T(0), T(1), rng) < p_det;

                agent_next_state.is_detecting = detects;
                if (detects) {
                    any_detection = true;
                }
            }

            /* -------- global bookkeeping ------------------------------------- */
            if (any_detection) {
                next_state.disaster_detected_global = true;
                next_state.disaster_undetected_steps = 0;
                next_state.last_detected_disaster_position[0] = next_state.disaster.position[0];
                next_state.last_detected_disaster_position[1] = next_state.disaster.position[1];
            } else {
                next_state.disaster_detected_global = state.disaster_detected_global;
                next_state.disaster_undetected_steps = state.disaster_undetected_steps + 1;
                next_state.last_detected_disaster_position[0] = state.last_detected_disaster_position[0];
                next_state.last_detected_disaster_position[1] = state.last_detected_disaster_position[1];
            }
        }
/* disaster inactive → reset flags exactly as before */
        else {
            next_state.disaster_detected_global = false;
            next_state.disaster_undetected_steps = 0;
            next_state.last_detected_disaster_position[0] = 0.0;
            next_state.last_detected_disaster_position[1] = 0.0;

            for (TI i = 0; i < N_AGENTS; ++i) {
                next_state.drone_states[i].is_detecting = false;
            }
        }



        // (4) Metrics update
        if (next_state.disaster.active) {
            next_state.metrics.disaster_active_steps = state.metrics.disaster_active_steps + 1;
        }

        // After the disaster detection logic, before metrics update:
        if (!next_state.disaster.active && state.disaster.active) {
            // Disaster just deactivated (exited), preserve metrics
            next_state.metrics.cumulative_detection_latency = state.metrics.cumulative_detection_latency;
            next_state.metrics.detection_count = state.metrics.detection_count;
        } else {
            if (next_state.disaster_detected_global && !state.disaster_detected_global) {
                TI latency = (state.step_count + 1)               // step *after* transition
                             - next_state.disaster_spawn_step;    // minus spawn step
                next_state.metrics.cumulative_detection_latency =
                        state.metrics.cumulative_detection_latency + latency;
                next_state.metrics.detection_count =
                        state.metrics.detection_count + 1;
            } else {
            /* plain carry-over */
            next_state.metrics.cumulative_detection_latency =
                    state.metrics.cumulative_detection_latency;
            next_state.metrics.detection_count =
                    state.metrics.detection_count;
            }
        }
        if (PARAMS::BATTERY_ENABLED){
            // Update charging metrics
            TI charging_sessions = 0;
            TI appropriate_charging_sessions = 0;
            TI inappropriate_charging_sessions = 0;
            TI death_count = 0;

            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
                const auto &agent_state = state.drone_states[agent_i];
                const auto &agent_next_state = next_state.drone_states[agent_i];

                // Count new charging sessions (started charging this step)
                if (!agent_state.is_charging && agent_next_state.is_charging) {
                    ++charging_sessions;

                    // Classify as appropriate or inappropriate
                    if (agent_state.battery < T(50)) {
                        ++appropriate_charging_sessions;
                    } else {
                        ++inappropriate_charging_sessions;
                    }
                }

                // Count new deaths
                if (!agent_state.dead && agent_next_state.dead) {
                    ++death_count;
                }
            }

            next_state.metrics.total_charging_sessions = state.metrics.total_charging_sessions + charging_sessions;
            next_state.metrics.appropriate_charging_count =
                    state.metrics.appropriate_charging_count + appropriate_charging_sessions;
            next_state.metrics.inappropriate_charging_count =
                    state.metrics.inappropriate_charging_count + inappropriate_charging_sessions;
            next_state.metrics.death_count = state.metrics.death_count + death_count;
        }

        // (5) Advance step count
        next_state.step_count = state.step_count + 1;

        return PARAMS::DT;
    }

//    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
//    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(
//            DEVICE &device,
//            const rl::environments::multi_agent::OilPlatform<SPEC>& env,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& state,
//            const Matrix<ACTION_SPEC>& action,
//            typename rl::environments::multi_agent::OilPlatform<SPEC>::State& next_state,
//            RNG & rng
//    ) {
//        using ENV = rl::environments::multi_agent::OilPlatform<SPEC>;
//        using T = typename SPEC::T;
//        using TI = typename SPEC::TI;
//        using PARAMS = typename SPEC::PARAMETERS;
//        constexpr TI N_AGENTS = ENV::PARAMETERS::N_AGENTS;
//
////        const T cx = PARAMS::GRID_SIZE_X / 2;
////        const T cy = PARAMS::GRID_SIZE_Y / 2;
//
//        T total_reward = 0;
///* ---------------------------------------------------------------
//   Phase selector
//   --------------------------------------------------------------- */
//        bool disaster_phase = next_state.disaster.active;
//
/////* ---------------------------------------------------------------
////   1.  Gaussian attraction
////   --------------------------------------------------------------- */
////        T sigma = disaster_phase ? PARAMS::GAUSS_SIGMA_EVENT
////                                 : PARAMS::GAUSS_SIGMA_COVER;
////
////        T phi_sum = 0;
////        for(TI agent_i=0; agent_i < N_AGENTS; ++agent_i){
////            const auto& agent_next_state = next_state.drone_states[agent_i];
////            if(agent_next_state.dead) {
////                continue;
////            }
////
////            T phi_raw = 0;
////            T frac_in = 1;
////
////            if(disaster_phase) {
////                // During disaster: simple distance-based reward to disaster location
////                if(agent_next_state.is_detecting) {
////                    T dist_to_disaster = distance(device,
////                                                  agent_next_state.position[0], agent_next_state.position[1],
////                                                  next_state.disaster.position[0], next_state.disaster.position[1]);
////                    phi_raw = math::exp(device.math, -(dist_to_disaster * dist_to_disaster) / (T(2) * sigma * sigma));
////                }
////                // No frac_in calculation needed during disaster
////            } else {
////                // During coverage: use platform+pipes structure (gaussian_potential)
////                phi_raw = gaussian_potential<DEVICE,PARAMS>(device,
////                                                            agent_next_state.position[0], agent_next_state.position[1], cx, cy, sigma);
////                frac_in = inbounds_fraction<DEVICE,PARAMS>(device,
////                                                           agent_next_state.position[0], agent_next_state.position[1]);
////            }
////
////            phi_sum += phi_raw * frac_in;
////        }
////
////        T reward_attr = PARAMS::GAUSS_BETA * phi_sum;
//
//        /* ---------------------------------------------------------------
//           1.  Gaussian attraction (scaled individually by each agent's battery)
//       --------------------------------------------------------------- */
//        T sigma = disaster_phase ? PARAMS::GAUSS_SIGMA_EVENT : PARAMS::GAUSS_SIGMA_COVER;
//        T beta = disaster_phase ? PARAMS::GAUSS_BETA_EVENT : PARAMS::GAUSS_BETA_COVER;
//
//        T phi_sum = 0;
//        for(TI agent_i=0; agent_i < N_AGENTS; ++agent_i){
//            const auto& agent_next_state = next_state.drone_states[agent_i];
//            if(agent_next_state.dead) {
//                continue;
//            }
//            T agent_coverage_weight = 1;
//
//            if (PARAMS::BATTERY_ENABLED){
//                // Individual battery-based weight for this agent
//                T agent_battery_criticality = math::pow(device.math,
//                                                        (100 - agent_next_state.battery) / 100, PARAMS::BATTERY_URGENCY_K);
//                agent_coverage_weight = 1 - agent_battery_criticality;
//            }
//
//            T phi_raw = 0;
//            T frac_in = 1;
//
//            if(disaster_phase) {
//                if(agent_next_state.is_detecting) {
//                    T dist_to_disaster = distance(device,
//                                                  agent_next_state.position[0], agent_next_state.position[1],
//                                                  next_state.disaster.position[0], next_state.disaster.position[1]);
//                    phi_raw = math::exp(device.math, -(dist_to_disaster * dist_to_disaster) / (T(2) * sigma * sigma));
//                }
//            } else {
//                phi_raw = multicenter_potential<DEVICE,PARAMS,T,TI>(device,
//                                                            agent_next_state.position[0], agent_next_state.position[1]);
//                frac_in = inbounds_fraction<DEVICE,PARAMS>(device,
//                                                           agent_next_state.position[0], agent_next_state.position[1]);
//            }
//
//            phi_sum += phi_raw * frac_in * agent_coverage_weight;  // Scale by individual agent's weight
//        }
//
//        T reward_attr = beta * phi_sum;
//
///* ---------------------------------------------------------------
//   2.  Overlap overlap_penalty (discourage redundant coverage)
//   --------------------------------------------------------------- */
//        typename PARAMS::T pos_arr[N_AGENTS][2];
//        for(TI i=0;i<N_AGENTS;++i){
//            pos_arr[i][0] = next_state.drone_states[i].position[0];
//            pos_arr[i][1] = next_state.drone_states[i].position[1];
//        }
//
//        //        T overlap_alpha = disaster_phase ? PARAMS::OVERLAP_ALPHA * 0.2 : PARAMS::OVERLAP_ALPHA;
//
//        if (PARAMS::OVERLAP_REPULSION_ACTIVE && reward_attr > 0){
//            T overlap_penalty = reward_attr * calculate_overlap_penalty<DEVICE, PARAMS>(device, pos_arr, disaster_phase) / T(N_AGENTS * (N_AGENTS - 1) / 2);
////            if (overlap_penalty > 0){
////                log(device, device.logger, "reward until now value: " + std::to_string(reward_attr));
////                log(device, device.logger, "overlap penalty value: " + std::to_string(overlap_penalty));
////            }
//            reward_attr -= overlap_penalty;
//        }
//
//        if(disaster_phase && !next_state.disaster_detected_global){
//            T temporal_penalty = (next_state.step_count - next_state.disaster_spawn_step)/(PARAMS::EPISODE_STEP_LIMIT - next_state.disaster_spawn_step);
//            reward_attr -= temporal_penalty;
//        }
//
//        /* ---------------------------------------------------------------
//            3.  Battery management rewards
//        --------------------------------------------------------------- */
//
//        if (PARAMS::BATTERY_ENABLED) {
//            // Death overlap_penalty
//            T death_penalty = 0;
//            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
//                if (!state.drone_states[agent_i].dead && next_state.drone_states[agent_i].dead) {
//                    death_penalty += PARAMS::DEATH_PENALTY;
//                }
//            }
//
////        // Charging attraction based on battery level
////        T charging_reward = 0;
////        for(TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
////            const auto& agent = next_state.drone_states[agent_i];
////            if(!agent.dead) {  // Only for alive, non-charging drones
////                T dist_to_charging = distance(device,
////                                              agent.position[0], agent.position[1],
////                                              PARAMS::CHARGING_STATION_POSITION_X,
////                                              PARAMS::CHARGING_STATION_POSITION_Y);
////
////                T gaussian = math::exp(device.math,
////                                       -(dist_to_charging * dist_to_charging) /
////                                       (2 * PARAMS::GAUSS_SIGMA_CHARGING * PARAMS::GAUSS_SIGMA_CHARGING));
////
////                T battery_urgency = math::pow(device.math,
////                                              (100 - agent.battery) / 100, PARAMS::BATTERY_URGENCY_K);
////
////                charging_reward += PARAMS::GAUSS_BETA_CHARGING * gaussian * battery_urgency;
////            }
////        }
//
//            // Individual battery-based charging attraction
//            T charging_reward = 0;
//            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
//                const auto &agent_next_state = next_state.drone_states[agent_i];
//                if (!agent_next_state.dead) {
//                    T dist_to_charging = distance(device,
//                                                  agent_next_state.position[0], agent_next_state.position[1],
//                                                  PARAMS::CHARGING_STATION_POSITION_X,
//                                                  PARAMS::CHARGING_STATION_POSITION_Y);
//
//                    T gaussian = math::exp(device.math,
//                                           -(dist_to_charging * dist_to_charging) /
//                                           (2 * PARAMS::GAUSS_SIGMA_CHARGING * PARAMS::GAUSS_SIGMA_CHARGING));
//
//                    // Just use the battery weight directly
//                    T agent_battery_weight = math::pow(device.math,
//                                                       (100 - agent_next_state.battery) / 100, PARAMS::BATTERY_URGENCY_K);
//
//                    charging_reward += PARAMS::GAUSS_BETA_CHARGING * gaussian * agent_battery_weight;
//                }
//            }
///* ---------------------------------------------------------------
//   4.  Final per-agent reward
//   --------------------------------------------------------------- */
//            total_reward = reward_attr + charging_reward + death_penalty;
//        } else{
//            total_reward = reward_attr;
//        }
//
//
//        utils::assert_exit(device, !math::is_nan(device.math, total_reward), "reward is nan");
//
//        /* ---------- coverage metric bookkeeping (no reward impact) ------------- */
//        if(!next_state.disaster_detected_global)
//        {
//            T cov = priority_area_coverage<DEVICE,SPEC>(device, next_state);
//
//            next_state.metrics.total_coverage_ratio =
//                    state.metrics.total_coverage_ratio + cov;
//
//            next_state.metrics.coverage_measurement_count =
//                    state.metrics.coverage_measurement_count + 1;
//        }
//        else
//        {
//            /* keep previous counters unchanged */
//            next_state.metrics.total_coverage_ratio     =
//                    state.metrics.total_coverage_ratio;
//            next_state.metrics.coverage_measurement_count =
//                    state.metrics.coverage_measurement_count;
//        }
///* ----------------------------------------------------------------------- */
//
//        return total_reward/N_AGENTS;
//    }

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
        using ENV    = rl::environments::multi_agent::OilPlatform<SPEC>;
        using T      = typename SPEC::T;
        using TI     = typename SPEC::TI;
        using PARAMS = typename SPEC::PARAMETERS;
        constexpr TI N_AGENTS = ENV::PARAMETERS::N_AGENTS;

        bool disaster_phase = next_state.disaster.active;
        if constexpr (PARAMS::DISASTER_REWARD_SWITCH_ON_DETECTION) {
            disaster_phase = next_state.disaster_detected_global;
        }
        T sigma = disaster_phase ? PARAMS::GAUSS_SIGMA_EVENT : PARAMS::GAUSS_SIGMA_COVER;
        T beta_phase = disaster_phase ? PARAMS::GAUSS_BETA_EVENT : PARAMS::GAUSS_BETA_COVER;

        // Accumulate fleet-wide reward contributions with symmetric battery weighting
        T fleet_coverage_value = 0;       // Sum of battery-weighted coverage contributions
        T fleet_charging_value = 0;       // Sum of battery-weighted charging contributions
        T fleet_coverage_capacity = 0;    // Sum of coverage weights: (1 - battery_urgency) per agent
        T fleet_charging_need = 0;        // Sum of charging weights: battery_urgency per agent

        for (TI i = 0; i < N_AGENTS; ++i) {
            const auto& agent_next_state = next_state.drone_states[i];
            if (agent_next_state.dead) {
                continue;
            }

            // Calculate battery urgency: 0 when full (100%), 1 when empty (0%)
            T battery_urgency = T(0);
            T coverage_weight = T(1);   // Weight for coverage contribution (high when battery full)
            T charging_weight = T(0);   // Weight for charging contribution (high when battery low)
            
            if constexpr (PARAMS::BATTERY_ENABLED) {
                T battery_normalized = agent_next_state.battery / T(100);
                battery_urgency = linear_weight<DEVICE, PARAMS>(device, battery_normalized);
                battery_urgency = math::clamp(device.math, battery_urgency, T(0), T(1));
                
                // Symmetric weights: coverage_weight + charging_weight = 1.0
                coverage_weight = T(1) - battery_urgency;
                charging_weight = battery_urgency;
                
                // Accumulate total weights across fleet
                fleet_coverage_capacity += coverage_weight;
                fleet_charging_need += charging_weight;
            } else {
                // When battery disabled, only coverage matters
                fleet_coverage_capacity += T(1);
            }

            // Calculate coverage/disaster-detection potential at agent's position
            T coverage_potential = 0;
            
            if (disaster_phase) {
                // During disaster: Gaussian attraction toward disaster location (always, regardless of coverage method)
                T dx_to_disaster = agent_next_state.position[0] - next_state.disaster.position[0];
                T dy_to_disaster = agent_next_state.position[1] - next_state.disaster.position[1];
                T dist_to_disaster_squared = dx_to_disaster*dx_to_disaster + dy_to_disaster*dy_to_disaster;
                coverage_potential = math::exp(device.math, -(dist_to_disaster_squared) / (T(2) * sigma * sigma));
                
                // Agent contributes to coverage proportional to battery level
                fleet_coverage_value += coverage_weight * coverage_potential;
            } else {
                // Coverage phase: choose between Voronoi or Gaussian based on compile-time flag
                if constexpr (PARAMS::USE_VORONOI_COVERAGE) {
                    // Voronoi coverage: computed collectively after loop, not per-agent
                    // Per-agent potential set to 0 here, will be overridden after loop
                    coverage_potential = T(0);
                } else {
                    // Original Gaussian coverage: Multi-Gaussian potential over platform+pipes
                    coverage_potential = multicenter_potential<DEVICE, PARAMS, T, TI>(device, 
                                                                                       agent_next_state.position[0], 
                                                                                       agent_next_state.position[1]);
                    // Agent contributes to coverage proportional to battery level
                    fleet_coverage_value += coverage_weight * coverage_potential;
                }
            }

            // Agent contributes to charging objective proportional to battery need
            // Low battery → high charging_weight → agent strongly encouraged to charge
            // Reward ACTUAL charging (is_charging = true), not just proximity to charger
            if constexpr (PARAMS::BATTERY_ENABLED) {
                if (agent_next_state.is_charging) {
                    // Agent is actively charging → full contribution
                    fleet_charging_value += charging_weight;
                } else {
                    // Agent not charging → use proximity as partial credit for moving toward charger
                    T dx_to_charger = agent_next_state.position[0] - state.charging_station_position[0];
                    T dy_to_charger = agent_next_state.position[1] - state.charging_station_position[1];
                    T dist_to_charger_squared = dx_to_charger*dx_to_charger + dy_to_charger*dy_to_charger;
                    T charging_proximity = math::exp(device.math, -(dist_to_charger_squared) / 
                                                     (T(2) * PARAMS::GAUSS_SIGMA_CHARGING * PARAMS::GAUSS_SIGMA_CHARGING));
                    
                    // Only partial credit for proximity (scale down to avoid hovering)
                    fleet_charging_value += charging_weight * charging_proximity * T(0.5);
                }
            }
        }

        // Voronoi coverage: compute battery-weighted coverage after processing all agents
        if constexpr (PARAMS::USE_VORONOI_COVERAGE) {
            if (!disaster_phase) {
                // Build position, dead, and coverage weight arrays for Voronoi computation
                typename PARAMS::T pos_arr[N_AGENTS][2];
                bool dead_arr[N_AGENTS];
                typename PARAMS::T coverage_weight_arr[N_AGENTS];

                for (TI i = 0; i < N_AGENTS; ++i) {
                    pos_arr[i][0] = next_state.drone_states[i].position[0];
                    pos_arr[i][1] = next_state.drone_states[i].position[1];
                    dead_arr[i] = next_state.drone_states[i].dead;

                    // Extract per-agent coverage weight (calculated in loop above)
                    if (next_state.drone_states[i].dead) {
                        coverage_weight_arr[i] = T(0);
                    } else {
                        if constexpr (PARAMS::BATTERY_ENABLED) {
                            T battery_normalized = next_state.drone_states[i].battery / T(100);
                            T battery_urgency = linear_weight<DEVICE, PARAMS>(device, battery_normalized);
                            battery_urgency = math::clamp(device.math, battery_urgency, T(0), T(1));
                            coverage_weight_arr[i] = T(1) - battery_urgency;
                        } else {
                            coverage_weight_arr[i] = T(1);
                        }
                    }
                }

                // Compute Voronoi coverage reward with per-agent battery weighting
                // Returns coverage fraction in [0,1]; scale by capacity to match penalty normalization
                // This maintains consistency with Gaussian approach where low-battery agents contribute less
                T coverage_fraction = calculate_voronoi_coverage_reward<DEVICE, PARAMS>(
                    device, pos_arr, dead_arr, coverage_weight_arr
                );
                fleet_coverage_value = coverage_fraction * fleet_coverage_capacity;
            }
        }

        // Convert coverage/disaster reward to penalty formulation (normalized by capacity)
        // Perfect: all agents at optimal positions given their battery states → penalty = 0
        // Poor: agents not covering despite having battery capacity → penalty = -beta_phase * fleet_coverage_capacity
        T coverage_penalty = -beta_phase * (fleet_coverage_capacity - fleet_coverage_value);
        if constexpr (PARAMS::CONST_PENALTY_WHILE_COVERAGE) {
            if (!disaster_phase) {
                coverage_penalty = -fleet_coverage_capacity;
            }
        }

        // Convert charging reward to penalty formulation (normalized by need)
        T charging_penalty = T(0);
        if constexpr (PARAMS::BATTERY_ENABLED) {
            // Perfect: agents charging exactly when needed → penalty = 0
            // Poor: agents with low battery not charging → penalty = -GAUSS_BETA_CHARGING * fleet_charging_need
            // Symmetric with coverage: both use actual weights, creating balanced trade-offs
            charging_penalty = -PARAMS::GAUSS_BETA_CHARGING * (fleet_charging_need - fleet_charging_value);
        }

        // Agent distribution penalty: penalize agents for clustering too close together
        // Encourages spatial distribution during coverage and around disaster
        T repulsion_penalty = T(0);
        if constexpr (PARAMS::OVERLAP_REPULSION_ACTIVE) {
            // Build position array for alive, non-charging agents
            typename PARAMS::T pos_arr[N_AGENTS][2];
            TI active_agent_count = 0;
            for (TI i = 0; i < N_AGENTS; ++i) {
                const auto& agent = next_state.drone_states[i];
                // Only apply repulsion to active agents (not dead, not charging)
                if (!agent.dead && !agent.is_charging) {
                    pos_arr[active_agent_count][0] = agent.position[0];
                    pos_arr[active_agent_count][1] = agent.position[1];
                    ++active_agent_count;
                }
            }
            
            // Calculate pairwise repulsion if we have at least 2 active agents
            if (active_agent_count >= 2) {
                T repulsion_sum = 0;
                T sigma_sq = disaster_phase ? PARAMS::OVERLAP_RHO_DETECTION * PARAMS::OVERLAP_RHO_DETECTION : PARAMS::OVERLAP_RHO_COVERAGE * PARAMS::OVERLAP_RHO_COVERAGE;
                
                for (TI i = 0; i < active_agent_count; ++i) {
                    for (TI j = i + 1; j < active_agent_count; ++j) {
                        T dx = pos_arr[i][0] - pos_arr[j][0];
                        T dy = pos_arr[i][1] - pos_arr[j][1];
                        T dist_sq = dx*dx + dy*dy;
                        // Gaussian repulsion: high penalty when agents are close
                        repulsion_sum += math::exp(device.math, -(dist_sq) / (T(2) * sigma_sq));
                    }
                }
                
                // Apply scaling directly to sum (no normalization by pairs)
                // This way penalty grows naturally with number of overlapping agents
                TI pairs = active_agent_count * (active_agent_count - 1) / 2;
                // if (pairs > 0) repulsion_penalty = -PARAMS::REPULSION_BETA * (repulsion_sum / T(pairs));
                repulsion_penalty = -PARAMS::REPULSION_BETA * repulsion_sum;
            }
        }

        // T temporal_penalty = T(0);
        // if (disaster_phase && !next_state.disaster_detected_global) {
        //     T num = next_state.step_count - T(next_state.disaster_spawn_step);
        //     T den = math::max(device.math, T(1), T(PARAMS::EPISODE_STEP_LIMIT) - T(next_state.disaster_spawn_step));
        //     temporal_penalty = -num / den;  // Already negative
        // }

        // Abandonment penalty: fixed penalty for not observing previously-detected disasters
        // Use state (not next_state) to ensure penalty reflects consequences of past abandonment
        T abandonment_penalty = T(0);
        // if (state.disaster.active &&
        //     state.disaster_detected_global &&
        //     state.disaster_undetected_steps > 0) {
        //     // Fixed penalty whenever disaster is unobserved after being detected
        //     abandonment_penalty = PARAMS::ABANDONMENT_PENALTY;  // Already negative
        // }

        T death_penalty = T(0);
        TI dead_count = 0;
        if constexpr (PARAMS::BATTERY_ENABLED) {
            for (TI i = 0; i < N_AGENTS; ++i) {
                if (!state.drone_states[i].dead && next_state.drone_states[i].dead) {
                    death_penalty += PARAMS::DEATH_PENALTY;  // One-time penalty (already negative)
                }
                if (next_state.drone_states[i].dead) {
                    ++dead_count;
                }
            }
        }

        // Ongoing penalty for being understaffed (per dead agent, per step)
        // This prevents the perverse incentive where death makes the task easier
        // IMPORTANT: Cast dead_count to T before negation to avoid unsigned integer overflow
        T ongoing_death_penalty = -T(dead_count);  // -1.0 penalty per dead agent per step

        // Movement cost as negative penalty
        T movement_penalty = T(0);
        for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
            const auto& agent_next_state = next_state.drone_states[agent_i];
            if (!agent_next_state.dead) {
                T speed = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
                movement_penalty -= PARAMS::MOVEMENT_COST_COEFFICIENT * speed*speed;  // Make negative
            }
        }

//        T total_reward = coverage_penalty + charging_penalty + temporal_penalty + death_penalty + ongoing_death_penalty + movement_penalty + repulsion_penalty;
        T total_reward = coverage_penalty + charging_penalty + repulsion_penalty + abandonment_penalty + death_penalty + ongoing_death_penalty + movement_penalty;

        utils::assert_exit(device, !math::is_nan(device.math, total_reward), "reward is nan");

        if(!next_state.disaster.active){
            T cov = priority_area_coverage<DEVICE,SPEC>(device, next_state);
            next_state.metrics.total_coverage_ratio =
                    state.metrics.total_coverage_ratio + cov;
            next_state.metrics.coverage_measurement_count =
                    state.metrics.coverage_measurement_count + 1;
        } else {
            next_state.metrics.total_coverage_ratio     = state.metrics.total_coverage_ratio;
            next_state.metrics.coverage_measurement_count = state.metrics.coverage_measurement_count;
        }

        next_state.metrics.per_step_reward = total_reward;

        return total_reward;
    }





//    template<typename DEVICE, typename SPEC, typename ACTION_SPEC, typename RNG>
//    RL_TOOLS_FUNCTION_PLACEMENT static typename SPEC::T reward(
//            DEVICE &device,
//            const rl::environments::multi_agent::OilPlatform<SPEC>& env,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters,
//            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State& state,
//            const Matrix<ACTION_SPEC>& action,
//            typename rl::environments::multi_agent::OilPlatform<SPEC>::State& next_state,
//            RNG & rng
//    ) {
//        using ENV    = rl::environments::multi_agent::OilPlatform<SPEC>;
//        using T      = typename SPEC::T;
//        using TI     = typename SPEC::TI;
//        using PARAMS = typename SPEC::PARAMETERS;
//        constexpr TI N_AGENTS = ENV::PARAMETERS::N_AGENTS;
//
//
//        bool disaster_phase = next_state.disaster.active;
//        T sigma = disaster_phase ? PARAMS::GAUSS_SIGMA_EVENT : PARAMS::GAUSS_SIGMA_COVER;
//        T beta_phase = disaster_phase ? PARAMS::GAUSS_BETA_EVENT : PARAMS::GAUSS_BETA_COVER;
//
//        T phi_sum = 0;
//        T g_sum   = 0;
//
//        for (TI i = 0; i < N_AGENTS; ++i) {
//            const auto& agent_next_state = next_state.drone_states[i];
//            if (agent_next_state.dead) {
//                continue;
//            }
//
//            T h = T(0);
//            if constexpr (PARAMS::BATTERY_ENABLED) {
//                T b01 = agent_next_state.battery / T(100);
//                h = linear_weight<DEVICE, PARAMS>(device, b01);
//                h = math::clamp(device.math, h, T(0), T(1));
//            }
//
//            T phi_raw = 0;
//            T frac_in = 1;
//            if(!agent_next_state.is_charging){
//                if (disaster_phase) {
//                    T dx = agent_next_state.position[0] - next_state.disaster.position[0];
//                    T dy = agent_next_state.position[1] - next_state.disaster.position[1];
//                    T d2 = dx*dx + dy*dy;
//                    phi_raw = math::exp(device.math, -(d2) / (T(2) * sigma * sigma));
//                } else {
//                    phi_raw = multicenter_potential<DEVICE, PARAMS, T, TI>(device, agent_next_state.position[0], agent_next_state.position[1]);
//                    frac_in = inbounds_fraction<DEVICE, PARAMS>(device, agent_next_state.position[0], agent_next_state.position[1]);
//                }
//            }
//
//            if constexpr (PARAMS::BATTERY_ENABLED) {
//                T dxp = agent_next_state.position[0] - PARAMS::CHARGING_STATION_POSITION_X;
//                T dyp = agent_next_state.position[1] - PARAMS::CHARGING_STATION_POSITION_Y;
//                T d2p = dxp*dxp + dyp*dyp;
//                T g = math::exp(device.math, -(d2p) / (T(2) * PARAMS::GAUSS_SIGMA_CHARGING * PARAMS::GAUSS_SIGMA_CHARGING));
//                g_sum += h * g;
//                phi_sum += (T(1) - h) * phi_raw * frac_in;
//            } else {
//                phi_sum += phi_raw * frac_in;
//            }
//        }
//
//        T reward_attr = beta_phase * phi_sum;
//        T charging_reward = T(0);
//        if constexpr (PARAMS::BATTERY_ENABLED) {
//            charging_reward = PARAMS::GAUSS_BETA_CHARGING * g_sum;
//        }
//
//        T total_primary = reward_attr + charging_reward;
//
//        T overlap_penalty = T(0);
//
//        if(PARAMS::OVERLAP_REPULSION_ACTIVE){
//            typename PARAMS::T pos_arr[N_AGENTS][2];
//            for (TI i = 0; i < N_AGENTS; ++i) {
//                pos_arr[i][0] = next_state.drone_states[i].position[0];
//                pos_arr[i][1] = next_state.drone_states[i].position[1];
//            }
//            T total_pairs = T(N_AGENTS * (N_AGENTS - 1) / 2);
//            T overlap_metric = (total_pairs > T(0)) ? calculate_overlap_penalty<DEVICE, PARAMS>(device, pos_arr, disaster_phase) / total_pairs : T(0);
//            overlap_penalty = total_primary * overlap_metric;
//        }
//
//        T temporal_penalty = T(0);
//        if (disaster_phase && !next_state.disaster_detected_global) {
//            T num = next_state.step_count - T(next_state.disaster_spawn_step);
//            T den = math::max(device.math, T(1), T(PARAMS::EPISODE_STEP_LIMIT) - T(next_state.disaster_spawn_step));
//            temporal_penalty = num / den;
//        }
//
//        T death_penalty = T(0);
//        if constexpr (PARAMS::BATTERY_ENABLED) {
//            for (TI i = 0; i < N_AGENTS; ++i) {
//                if (!state.drone_states[i].dead && next_state.drone_states[i].dead) {
//                    death_penalty += PARAMS::DEATH_PENALTY;
//                }
//            }
//        }
//
//        // Calculate movement cost penalty
//        T movement_cost = T(0);
//        for (TI agent_i = 0; agent_i < N_AGENTS; agent_i++) {
//            const auto& agent_next_state = next_state.drone_states[agent_i];
//            if (!agent_next_state.dead) {
//                T speed = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
//                movement_cost += PARAMS::MOVEMENT_COST_COEFFICIENT * speed;
//            }
//        }
//
//        T total_reward = total_primary/N_AGENTS - overlap_penalty - temporal_penalty + death_penalty - movement_cost;
//
//        utils::assert_exit(device, !math::is_nan(device.math, total_reward), "reward is nan");
//
//        if(!next_state.disaster_detected_global){
//            T cov = priority_area_coverage<DEVICE,SPEC>(device, next_state);
//            next_state.metrics.total_coverage_ratio =
//                    state.metrics.total_coverage_ratio + cov;
//            next_state.metrics.coverage_measurement_count =
//                    state.metrics.coverage_measurement_count + 1;
//        } else {
//            next_state.metrics.total_coverage_ratio     = state.metrics.total_coverage_ratio;
//            next_state.metrics.coverage_measurement_count = state.metrics.coverage_measurement_count;
//        }
//
//        next_state.metrics.per_step_reward = total_reward;
//
//        return total_reward;
//    }



#ifndef RL_TOOLS_USE_MULTI_AGENT_PPO
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
        constexpr TI PER_AGENT_DIM = OBS::PER_AGENT_DIM;
        constexpr TI SHARED_DIM = OBS::SHARED_DIM;
        using PARAMS = typename SPEC::PARAMETERS;
        
        // Pre-compute shared observations once
        T shared_obs[SHARED_DIM];
        shared_obs[0] = state.disaster_detected_global ? 1 : -1;
        shared_obs[1] = !state.disaster_detected_global ? 10 : 2 * (state.last_detected_disaster_position[0]/PARAMS::GRID_SIZE_X) - 1;
        shared_obs[2] = !state.disaster_detected_global ? 10 : 2 * (state.last_detected_disaster_position[1]/PARAMS::GRID_SIZE_Y) - 1;
        shared_obs[3] = 2 * (state.charging_station_position[0] / PARAMS::GRID_SIZE_X) - 1;
        shared_obs[4] = 2 * (state.charging_station_position[1] / PARAMS::GRID_SIZE_Y) - 1;
        
        for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; ++agent_i) {
            const auto &agent_state = state.drone_states[agent_i];
            TI offset = agent_i * PER_AGENT_DIM;
            
            // Per-agent own observations (8 dimensions)
            set(observation, 0, offset + 0, 2 * (agent_state.position[0] / PARAMS::GRID_SIZE_X) - 1);  // Normalized position [-1,1]
            set(observation, 0, offset + 1, 2 * (agent_state.position[1] / PARAMS::GRID_SIZE_Y) - 1);  // Normalized position [-1,1]
            set(observation, 0, offset + 2, agent_state.dead ? 0 : agent_state.velocity[0] / PARAMS::MAX_SPEED);
            set(observation, 0, offset + 3, agent_state.dead ? 0 : agent_state.velocity[1] / PARAMS::MAX_SPEED);
            set(observation, 0, offset + 4, agent_state.dead ? -1 : (agent_state.is_detecting ? 1 : -1));
            set(observation, 0, offset + 5, 2 * (agent_state.battery / 100) - 1);
            set(observation, 0, offset + 6, agent_state.dead ? 1 : -1);
            set(observation, 0, offset + 7, agent_state.dead ? -1 : agent_state.is_charging ? 1 : -1);
        }

        // Append shared observations once after all agent states
        TI shared_offset = PARAMS::N_AGENTS * PER_AGENT_DIM;
        for (TI shared_i = 0; shared_i < SHARED_DIM; ++shared_i) {
            set(observation, 0, shared_offset + shared_i, shared_obs[shared_i]);
        }

        utils::assert_exit(device, !is_nan(device, observation), "Observation is nan");

    }
#else
#include "operations_generic_ppo.h"
#endif

    template<typename DEVICE, typename SPEC, typename OBS_SPEC, typename OBS_PARAMETERS, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void observe(
            DEVICE &device,
            const rl::environments::multi_agent::OilPlatform<SPEC> &env,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters &parameters,
            const typename rl::environments::multi_agent::OilPlatform<SPEC>::State &state,
            const rl::environments::multi_agent::oil_platform::ObservationPrivileged<OBS_PARAMETERS> &,
            Matrix<OBS_SPEC> &observation,
            RNG &rng
    ) {
        using OBS = rl::environments::multi_agent::oil_platform::ObservationPrivileged<OBS_PARAMETERS>;
        static_assert(OBS_SPEC::ROWS == 1);
        static_assert(OBS_SPEC::COLS == OBS::DIM);
        using T = typename SPEC::T;
        using TI = typename SPEC::TI;
        constexpr TI PER_AGENT_DIM = OBS::PER_AGENT_DIM;
        using PARAMS = typename SPEC::PARAMETERS;
        (void)env;
        (void)parameters;
        (void)rng;

        // Per-agent features (shared critic view)
        for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; ++agent_i) {
            const auto &agent_state = state.drone_states[agent_i];
            TI base_offset = agent_i * PER_AGENT_DIM;

            set(observation, 0, base_offset + 0, 2 * (agent_state.position[0] / PARAMS::GRID_SIZE_X) - 1);  // pos_x
            set(observation, 0, base_offset + 1, 2 * (agent_state.position[1] / PARAMS::GRID_SIZE_Y) - 1);  // pos_y
            set(observation, 0, base_offset + 2, agent_state.dead ? 0 : agent_state.velocity[0] / PARAMS::MAX_SPEED);
            set(observation, 0, base_offset + 3, agent_state.dead ? 0 : agent_state.velocity[1] / PARAMS::MAX_SPEED);
            set(observation, 0, base_offset + 4, agent_state.dead ? -1 : agent_state.is_detecting ? 1 : -1);
            set(observation, 0, base_offset + 5, 2 * (agent_state.battery / 100) - 1);
            set(observation, 0, base_offset + 6, agent_state.dead ? 1 : -1);
            set(observation, 0, base_offset + 7, agent_state.dead ? -1 : agent_state.is_charging ? 1 : -1);
        }

        // Shared global features
        TI shared_offset = PARAMS::N_AGENTS * PER_AGENT_DIM;
        T disaster_active_flag = state.disaster.active ? T(1) : T(-1);
        T disaster_detected_flag = state.disaster_detected_global ? T(1) : T(-1);

        T disaster_pos_x = state.disaster.active ? T(2) * (state.disaster.position[0] / PARAMS::GRID_SIZE_X) - T(1) : T(0);
        T disaster_pos_y = state.disaster.active ? T(2) * (state.disaster.position[1] / PARAMS::GRID_SIZE_Y) - T(1) : T(0);
        T disaster_vel_x = state.disaster.active ? state.disaster.velocity[0] / PARAMS::DISASTER_MAX_SPEED : T(0);
        T disaster_vel_y = state.disaster.active ? state.disaster.velocity[1] / PARAMS::DISASTER_MAX_SPEED : T(0);
        T charger_pos_x = T(2) * (state.charging_station_position[0] / PARAMS::GRID_SIZE_X) - T(1);
        T charger_pos_y = T(2) * (state.charging_station_position[1] / PARAMS::GRID_SIZE_Y) - T(1);

        set(observation, 0, shared_offset + 0, disaster_active_flag);
        set(observation, 0, shared_offset + 1, disaster_detected_flag);
        set(observation, 0, shared_offset + 2, disaster_pos_x);
        set(observation, 0, shared_offset + 3, disaster_pos_y);
        set(observation, 0, shared_offset + 4, disaster_vel_x);
        set(observation, 0, shared_offset + 5, disaster_vel_y);
        set(observation, 0, shared_offset + 6, charger_pos_x);
        set(observation, 0, shared_offset + 7, charger_pos_y);

        utils::assert_exit(device, !is_nan(device, observation), "Privileged observation is nan");
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
        if (PARAMS::BATTERY_ENABLED){
            bool any_alive = false;
            for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; ++agent_i) {
                if (!state.drone_states[agent_i].dead) {
                    any_alive = true;
                    break;
                }
            }

            if (!any_alive) {
                terminate = true;
            }
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
        if (terminate || state.step_count == PARAMS::EPISODE_STEP_LIMIT) {

            // Basic episode metrics
            add_scalar(device, device.logger, "episode/total_steps", state.step_count);
            add_scalar(device, device.logger, "episode/final_deaths", state.metrics.death_count);

// Charging behavior metrics
            add_scalar(device, device.logger, "charging/total_sessions", state.metrics.total_charging_sessions);
            add_scalar(device, device.logger, "charging/appropriate_sessions", state.metrics.appropriate_charging_count);
            add_scalar(device, device.logger, "charging/inappropriate_sessions", state.metrics.inappropriate_charging_count);

            if (state.metrics.total_charging_sessions > 0) {
                T charging_efficiency = static_cast<T>(state.metrics.appropriate_charging_count) /
                                        static_cast<T>(state.metrics.total_charging_sessions);
                add_scalar(device, device.logger, "charging/efficiency", charging_efficiency);
            }

            // Basic episode metrics
//            add_scalar(device, device.logger, "episode/total_steps", state.step_count);
//            add_scalar(device, device.logger, "episode/final_deaths", state.metrics.death_count);

//            // Coverage metrics
//            if (state.metrics.coverage_measurement_count > 0) {
//                T average_coverage = state.metrics.total_coverage_ratio / state.metrics.coverage_measurement_count;
//                add_scalar(device, device.logger, "coverage/average_priority_area_coverage", average_coverage);
//            }
//            add_scalar(device, device.logger, "coverage/measurement_count", state.metrics.coverage_measurement_count);

            // Only log if disaster never spawned or spawned after minimum coverage time
            bool had_enough_coverage_time =
                    state.metrics.coverage_measurement_count >= PARAMS::MINIMUM_COVERAGE_STEPS;

            if (had_enough_coverage_time) {
                if (state.metrics.coverage_measurement_count > 0) {
                    T average_coverage = state.metrics.total_coverage_ratio / state.metrics.coverage_measurement_count;
                    add_scalar(device, device.logger, "coverage/average_priority_area_coverage", average_coverage);
                }
                add_scalar(device, device.logger, "coverage/measurement_count", state.metrics.coverage_measurement_count);
            }

            // Always log when disaster spawned (or episode length if no disaster)
            TI coverage_only_steps = state.metrics.coverage_measurement_count;
            add_scalar(device, device.logger, "coverage/coverage_only_steps", coverage_only_steps);

            /* --- add potential-coverage diagnostics --- */
            if (state.metrics.potential_steps > 0) {
                T avg_step_pot = state.metrics.cumulative_potential_reward /
                                 state.metrics.potential_steps;
                add_scalar(device, device.logger,
                           "potential/episode_cumulative",
                           state.metrics.cumulative_potential_reward);
                add_scalar(device, device.logger,
                           "potential/episode_avg_per_step",
                           avg_step_pot);
            }

            if (state.metrics.detection_count > 0) {
                T avg_latency = static_cast<T>(state.metrics.cumulative_detection_latency) /
                                state.metrics.detection_count;
                add_scalar(device, device.logger,
                           "disaster/avg_detection_latency",  avg_latency);
            } else{
                add_scalar(device, device.logger,
                           "disaster/avg_detection_latency",  300);
            }
            add_scalar(device, device.logger,
                       "disaster/events_detected", state.metrics.detection_count);



//            add_scalar(device, device.logger, "disaster/active_steps", state.metrics.disaster_active_steps);

            // Charging behavior metrics
//            add_scalar(device, device.logger, "charging/total_sessions", state.metrics.total_charging_sessions);
//            add_scalar(device, device.logger, "charging/appropriate_sessions", state.metrics.appropriate_charging_count);
//            add_scalar(device, device.logger, "charging/inappropriate_sessions", state.metrics.inappropriate_charging_count);

//            if (state.metrics.total_charging_sessions > 0) {
//                T charging_efficiency = state.metrics.appropriate_charging_count / state.metrics.total_charging_sessions;
//                add_scalar(device, device.logger, "charging/efficiency", charging_efficiency);
//            }

            // Battery management metrics
            T total_battery = 0;
            T min_battery = 100;
            T max_battery = 0;
            TI alive_count = 0;

            for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; ++agent_i) {
                if (!state.drone_states[agent_i].dead) {
                    ++alive_count;
                    T battery = state.drone_states[agent_i].battery;
                    total_battery += battery;
                    min_battery = math::min(device.math, min_battery, battery);
                    max_battery = math::max(device.math, max_battery, battery);
                }
            }

            // Add new disaster-related metrics
            add_scalar(device, device.logger, "disaster/total_spawned", state.metrics.total_disasters_spawned);
            add_scalar(device, device.logger, "disaster/total_missed", state.metrics.disasters_missed);

            if (state.metrics.total_disasters_spawned > 0) {
                T detection_rate = T(state.metrics.detection_count) / T(state.metrics.total_disasters_spawned);
                add_scalar(device, device.logger, "disaster/detection_rate", detection_rate);
            }

            if (alive_count > 0) {
                add_scalar(device, device.logger, "battery/average_final", total_battery / alive_count);
                add_scalar(device, device.logger, "battery/min_final", min_battery);
                add_scalar(device, device.logger, "battery/max_final", max_battery);
            }
            add_scalar(device, device.logger, "agents/alive_at_end", alive_count);

            // Survival rate
            T survival_rate = T(alive_count) / T(PARAMS::N_AGENTS);
            add_scalar(device, device.logger, "agents/survival_rate", survival_rate);
        }

        return terminate;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif  // RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_GENERIC_H
