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

    template<typename DEVICE, typename PARAMS, typename T, typename RNG>
    RL_TOOLS_FUNCTION_PLACEMENT static void sample_roi_position(
            DEVICE &device, T &x, T &y, RNG &rng) {
        using TI = typename PARAMS::TI;
        TI idx = random::uniform_int_distribution(device.random, TI(0),
                                                  PARAMS::ROI_SIZE - 1, rng);
        x = PARAMS::ROI_CATALOGUE[idx].x;
        y = PARAMS::ROI_CATALOGUE[idx].y;
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


    template<typename DEVICE, typename PARAMS, typename T>
    RL_TOOLS_FUNCTION_PLACEMENT static T gaussian_potential(DEVICE &device,
                                                            T px, T py,              // point to evaluate
                                                            T cx, T cy,              // platform centre
                                                            T sigma)                 // Gaussian width
    {
        T adx = math::abs(device.math, px - cx);
        T ady = math::abs(device.math, py - cy);

        /* square platform */
        T dx_pl = math::max(device.math, adx - PARAMS::PLATFORM_HALF_SIZE, T(0));
        T dy_pl = math::max(device.math, ady - PARAMS::PLATFORM_HALF_SIZE, T(0));
        T d2_pl = dx_pl * dx_pl + dy_pl * dy_pl;

        /* horizontal pipe */
        T dy_ph = math::max(device.math, ady - PARAMS::PIPE_WIDTH / 2, T(0));
        T d2_ph = dy_ph * dy_ph;

        /* vertical pipe */
        T dx_pv = math::max(device.math, adx - PARAMS::PIPE_WIDTH / 2, T(0));
        T d2_pv = dx_pv * dx_pv;

        T d2 = math::min(device.math, d2_pl,
                         math::min(device.math, d2_ph, d2_pv));

        return math::exp(device.math, -d2 / (T(2) * sigma * sigma));
    }

    template<typename DEVICE, typename PARAMS>
    RL_TOOLS_FUNCTION_PLACEMENT static
    typename PARAMS::T overlap_penalty(DEVICE &device,
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

            // Initialize grid tracking
            get_grid_cell<DEVICE, PARAMS>(device,
                                          agent_state.position[0],
                                          agent_state.position[1],
                                          agent_state.current_grid_x,
                                          agent_state.current_grid_y);
            agent_state.steps_in_current_cell = 0;
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

        /* ───── initialise coverage bookkeeping ───── */
//        state.metrics.total_covered_cells = 0;
//        for (int i = 0; i < SPEC::PARAMETERS::GRID_RES; i++) {
//            for (int j = 0; j < SPEC::PARAMETERS::GRID_RES; j++) {
//                state.metrics.covered_grid[i][j] = false;
//                state.metrics.visit_age_grid[i][j] = SPEC::PARAMETERS::NOVELTY_WINDOW + 1;
//            }
//        }
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

            // Initialize grid tracking
            get_grid_cell<DEVICE, PARAMS>(device,
                                                    agent_state.position[0],
                                                    agent_state.position[1],
                                                    agent_state.current_grid_x,
                                                    agent_state.current_grid_y);
            agent_state.steps_in_current_cell = 0;
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

//        /* ───── initialise coverage bookkeeping ───── */
//        state.metrics.total_covered_cells = 0;
//        for (int i = 0; i < SPEC::PARAMETERS::GRID_RES; i++) {
//            for (int j = 0; j < SPEC::PARAMETERS::GRID_RES; j++) {
//                state.metrics.covered_grid[i][j] = false;
//                state.metrics.visit_age_grid[i][j] = SPEC::PARAMETERS::NOVELTY_WINDOW + 1;
//            }
//        }
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


        const T cx = PARAMS::GRID_SIZE_X / T(2);
        const T cy = PARAMS::GRID_SIZE_Y / T(2);

        // (1) Disaster: 2% to start in any HIGH‑PRIORITY area, remains static once created
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
                next_state.disaster_spawn_step = state.step_count;   // mark this step
            } else {
                next_state.disaster_spawn_step = state.disaster_spawn_step;
                next_state.disaster.active = false;
                next_state.disaster.position[0] = state.disaster.position[0];
                next_state.disaster.position[1] = state.disaster.position[1];
                next_state.disaster.velocity[0] = state.disaster.velocity[0];
                next_state.disaster.velocity[1] = state.disaster.velocity[1];
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
            next_state.disaster.active = true;
            next_state.disaster_spawn_step = state.disaster_spawn_step;

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

            // ---- small random heading change ----
            T dtheta = random::uniform_real_distribution(device.random,
                                                         -PARAMS::DISASTER_TURN_MAX_RAD,
                                                         +PARAMS::DISASTER_TURN_MAX_RAD,
                                                         rng);
            T c = math::cos(device.math, dtheta);
            T s = math::sin(device.math, dtheta);
            T vx_r = vx * c - vy * s;
            T vy_r = vx * s + vy * c;

            // ---- small random speed jitter (fraction of current speed cap) ----
            // jitter range scaled to MAX_SPEED (not current speed) so a stopped disaster can re-accelerate a bit
            T dv_frac = random::uniform_real_distribution(device.random,
                                                          -PARAMS::DISASTER_SPEED_JITTER_FRAC,
                                                          +PARAMS::DISASTER_SPEED_JITTER_FRAC,
                                                          rng);
            T speed_target = vmag + dv_frac * PARAMS::DISASTER_MAX_SPEED;

            // clamp to [0, MAX]
            speed_target = math::clamp(device.math, speed_target, T(0), PARAMS::DISASTER_MAX_SPEED);

            // renormalize rotated velocity to speed_target
            T vmag_r = magnitude(device, vx_r, vy_r);
            if (vmag_r > T(1e-6)) {
                T scale = speed_target / vmag_r;
                vx_r *= scale;
                vy_r *= scale;
            } else {
                // degenerate (shouldn’t happen), just re-sample a dir at target speed
                T ang_resamp = random::uniform_real_distribution(device.random, T(0), T(2 * M_PI), rng);
                vx_r = math::cos(device.math, ang_resamp) * speed_target;
                vy_r = math::sin(device.math, ang_resamp) * speed_target;
            }

            // ---- integrate forward ----
            T new_x = state.disaster.position[0] + vx_r * PARAMS::DT;
            T new_y = state.disaster.position[1] + vy_r * PARAMS::DT;

            next_state.disaster.position[0] = new_x;
            next_state.disaster.position[1] = new_y;
            next_state.disaster.velocity[0] = vx_r;
            next_state.disaster.velocity[1] = vy_r;
        }

        // (2) Per-drone update for position and velocity
        for (TI agent_i = 0; agent_i < N_AGENTS; agent_i++) {

            const auto &agent_state = state.drone_states[agent_i];
            auto &agent_next_state = next_state.drone_states[agent_i];
            agent_next_state = agent_state;
            if (!agent_state.dead) {

                // TODO: REMOVE THIS WHEN REINTRODUCING BATTERY
                agent_next_state.dead = false;
                agent_next_state.is_charging = false;

                // VELOCITY CONTROL

//                T desired_vx = get(action, 0, agent_i * 2 + 0) * PARAMS::MAX_SPEED;
//                T desired_vy = get(action, 0, agent_i * 2 + 1) * PARAMS::MAX_SPEED;

                T desired_vx = get(action, 0, agent_i * 2 + 0);   // already ∈[-1,1]
                T desired_vy = get(action, 0, agent_i * 2 + 1);

// --- optional inertia (comment out if you don’t want it yet) ---
                constexpr T ALPHA = 0.6;                  // 0<α≤1  → smaller α = more inertia
                desired_vx = agent_state.velocity[0] + ALPHA * (desired_vx * PARAMS::MAX_SPEED
                                                                - agent_state.velocity[0]);
                desired_vy = agent_state.velocity[1] + ALPHA * (desired_vy * PARAMS::MAX_SPEED
                                                                - agent_state.velocity[1]);


                // Set velocity directly to desired values
                agent_next_state.velocity[0] = desired_vx;
                agent_next_state.velocity[1] = desired_vy;


                // ACCELERATION CONTROL
//                T ax = get(action, 0, agent_i * 2 + 0) * PARAMS::MAX_ACCELERATION;
//                T ay = get(action, 0, agent_i * 2 + 1) * PARAMS::MAX_ACCELERATION;
//
//                agent_next_state.velocity[0] = agent_state.velocity[0] + ax * PARAMS::DT;
//                agent_next_state.velocity[1] = agent_state.velocity[1] + ay * PARAMS::DT;

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

        // After updating positions, update grid tracking
        for (TI agent_i = 0; agent_i < PARAMS::N_AGENTS; agent_i++) {
            auto &agent_next_state = next_state.drone_states[agent_i];
            const auto &agent_state = state.drone_states[agent_i];

            if (!agent_next_state.dead) {
                // Get new grid cell
                TI new_grid_x, new_grid_y;
                get_grid_cell<DEVICE, PARAMS>(device,
                                              agent_next_state.position[0],
                                              agent_next_state.position[1],
                                              new_grid_x,
                                              new_grid_y);

                // Check if drone moved to a new cell
                if (new_grid_x != agent_state.current_grid_x ||
                    new_grid_y != agent_state.current_grid_y) {
                    // Reset counter
                    agent_next_state.current_grid_x = new_grid_x;
                    agent_next_state.current_grid_y = new_grid_y;
                    agent_next_state.steps_in_current_cell = 0;
                } else {
                    // Increment counter
                    agent_next_state.current_grid_x = agent_state.current_grid_x;
                    agent_next_state.current_grid_y = agent_state.current_grid_y;
                    agent_next_state.steps_in_current_cell = agent_state.steps_in_current_cell + 1;
                }
            }
        }

        // (2) Per-drone update for battery and charging
//        for (TI agent_i = 0; agent_i < N_AGENTS; agent_i++) {
//            auto& agent_next_state = next_state.drone_states[agent_i];
//            const auto& agent_state = state.drone_states[agent_i];
//
//            if(!agent_state.dead) {
//                // Check if drone is at charging station
//                T distance_to_charging_station = distance(device, agent_next_state.position[0], agent_next_state.position[1], PARAMS::CHARGING_STATION_POSITION_X, PARAMS::CHARGING_STATION_POSITION_Y);
//                bool at_charging_station = (distance_to_charging_station < PARAMS::CHARGING_STATION_RANGE);
//                T velocity_magnitude = magnitude(device, agent_next_state.velocity[0], agent_next_state.velocity[1]);
//                bool is_stationary = (velocity_magnitude < PARAMS::CHARGING_VELOCITY_THRESHOLD);
//                agent_next_state.is_charging = (at_charging_station && is_stationary);
//
//                // Update battery
//                if (agent_next_state.is_charging) {
//                    agent_next_state.battery = std::min(T(100), T(agent_state.battery + PARAMS::CHARGING_RATE));
//                } else {
//                    agent_next_state.battery = std::max(T(0), T(agent_state.battery - PARAMS::DISCHARGE_RATE));
//                }
//
//                // Check if drone dies
//                if (agent_next_state.battery <= 0) {
//                    agent_next_state.dead = true;
//                    agent_next_state.velocity[0] = 0;
//                    agent_next_state.velocity[1] = 0;
//                    agent_next_state.battery = 0;
//                    agent_next_state.is_charging = false;
//                    agent_next_state.is_detecting = false;
//                } else {
//                    agent_next_state.dead = false;
//                }
//            }
//        }


//        // (3) Disaster detection logic
//        bool detection = false;
//        if (next_state.disaster.active) {
//            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
//                auto &agent_next_state = next_state.drone_states[agent_i];
//
//                // Skip dead drones for disaster detection
//                if (!agent_next_state.dead) {
//                    if (distance(device, agent_next_state.position[0], agent_next_state.position[1], next_state.disaster.position[0], next_state.disaster.position[1]) < PARAMS::SENSOR_RANGE) {
//                        detection = true;
//                        agent_next_state.is_detecting = true;
//                    } else {
//                        agent_next_state.is_detecting = false;
//                    }
//                }
//            }
//
//            // Update all living drones' last detected position if any detection
//            if (detection) {
//                next_state.disaster_detected_global = true;
//                next_state.disaster_undetected_steps = 0;
//                next_state.last_detected_disaster_position[0] = next_state.disaster.position[0];
//                next_state.last_detected_disaster_position[1] = next_state.disaster.position[1];
//            } else {
//                next_state.disaster_undetected_steps = state.disaster_undetected_steps + 1;
//                next_state.last_detected_disaster_position[0] = state.last_detected_disaster_position[0];
//                next_state.last_detected_disaster_position[1] = state.last_detected_disaster_position[1];
//                next_state.disaster_detected_global = state.disaster_detected_global;
//            }
//        } else {
//            next_state.disaster_detected_global = false;
//            next_state.disaster_undetected_steps = 0;
//            next_state.last_detected_disaster_position[0] = state.last_detected_disaster_position[0];
//            next_state.last_detected_disaster_position[1] = state.last_detected_disaster_position[1];
//        }

        /* ───── (3)  Disaster detection logic  ───────────────────────────────── */
        constexpr T beta = PARAMS::DETECTION_BETA;


        bool any_detection = false;

        if (next_state.disaster.active) {
            for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
                auto&       ag_next = next_state.drone_states[agent_i];
                const auto& ag_prev = state.drone_states[agent_i];

                if (ag_next.dead) { ag_next.is_detecting = false; continue; }

                /* distance to disaster */
                T d = distance(device,
                               ag_next.position[0], ag_next.position[1],
                               next_state.disaster.position[0], next_state.disaster.position[1]);

                /* smooth detection probability */
                T p_det = logistic(device, beta * (PARAMS::SENSOR_RANGE - d));

                /* Bernoulli draw */
                bool detects = random::uniform_real_distribution(device.random,
                                                                 T(0), T(1), rng) < p_det;

                ag_next.is_detecting = detects;
                if (detects) {
                    any_detection = true;
                }
            }

            /* -------- global bookkeeping ------------------------------------- */
            if (any_detection) {
                next_state.disaster_detected_global  = true;
                next_state.disaster_undetected_steps = 0;
                next_state.last_detected_disaster_position[0] = next_state.disaster.position[0];
                next_state.last_detected_disaster_position[1] = next_state.disaster.position[1];
            } else {
                next_state.disaster_detected_global  = state.disaster_detected_global;
                next_state.disaster_undetected_steps = state.disaster_undetected_steps + 1;
                next_state.last_detected_disaster_position[0] = state.last_detected_disaster_position[0];
                next_state.last_detected_disaster_position[1] = state.last_detected_disaster_position[1];
            }
        }
/* disaster inactive → reset flags exactly as before */
        else {
            next_state.disaster_detected_global  = false;
            next_state.disaster_undetected_steps = 0;
            next_state.last_detected_disaster_position[0] = state.last_detected_disaster_position[0];
            next_state.last_detected_disaster_position[1] = state.last_detected_disaster_position[1];
        }



        next_state.metrics = state.metrics;

        // (4) Metrics update
        if (next_state.disaster.active) {
            next_state.metrics.disaster_active_steps = state.metrics.disaster_active_steps + 1;
        }

        /* ----- first-detection latency ------------------------------------ */
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

//        // Update charging metrics
//        TI charging_sessions = 0;
//        TI appropriate_charging_sessions = 0;
//        TI inappropriate_charging_sessions = 0;
//        TI death_count = 0;
//
//        for (TI agent_i = 0; agent_i < N_AGENTS; ++agent_i) {
//            const auto &agent_state = state.drone_states[agent_i];
//            const auto &agent_next_state = next_state.drone_states[agent_i];
//
//            // Count new charging sessions (started charging this step)
//            if (agent_next_state.is_charging) {
//                charging_sessions++;
//
//                // Classify as appropriate or inappropriate
//                if (agent_state.battery < T(50)) {
//                    appropriate_charging_sessions++;
//                } else {
//                    inappropriate_charging_sessions++;
//                }
//            }
//
//            // Count new deaths
//            if (!agent_state.dead && agent_next_state.dead) {
//                death_count++;
//            }
//        }
//
//        next_state.metrics.total_charging_sessions = state.metrics.total_charging_sessions + charging_sessions;
//        next_state.metrics.appropriate_charging_count = state.metrics.appropriate_charging_count + appropriate_charging_sessions;
//        next_state.metrics.inappropriate_charging_count = state.metrics.inappropriate_charging_count + inappropriate_charging_sessions;
//        next_state.metrics.death_count = state.metrics.death_count + death_count;

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
/* ---------------------------------------------------------------
   Phase selector
   --------------------------------------------------------------- */
        bool disaster_phase = next_state.disaster.active;

/* ---------------------------------------------------------------
   1.  Gaussian attraction
   --------------------------------------------------------------- */
        T sigma = disaster_phase ? PARAMS::GAUSS_SIGMA_EVENT
                                 : PARAMS::GAUSS_SIGMA_COVER;

        T phi_sum = 0;
        for(TI agent_i=0; agent_i < N_AGENTS; ++agent_i){
            const auto& agent_next_state = next_state.drone_states[agent_i];
            if(agent_next_state.dead) {
                continue;
            }

            T phi_raw = 0;
            T frac_in = 1;

            if(disaster_phase) {
                // During disaster: simple distance-based reward to disaster location
                if(agent_next_state.is_detecting) {
                    T dist_to_disaster = distance(device,
                                                  agent_next_state.position[0], agent_next_state.position[1],
                                                  next_state.disaster.position[0], next_state.disaster.position[1]);
                    phi_raw = math::exp(device.math, -(dist_to_disaster * dist_to_disaster) / (T(2) * sigma * sigma));
                }
                // No frac_in calculation needed during disaster
            } else {
                // During coverage: use platform+pipes structure (gaussian_potential)
                phi_raw = gaussian_potential<DEVICE,PARAMS>(device,
                                                            agent_next_state.position[0], agent_next_state.position[1], cx, cy, sigma);
                frac_in = inbounds_fraction<DEVICE,PARAMS>(device,
                                                           agent_next_state.position[0], agent_next_state.position[1]);
            }

            phi_sum += phi_raw * frac_in;
        }

        T reward_attr = PARAMS::GAUSS_BETA * phi_sum;

/* ---------------------------------------------------------------
   2.  Overlap penalty (discourage redundant coverage)
   --------------------------------------------------------------- */
        typename PARAMS::T pos_arr[N_AGENTS][2];
        for(TI i=0;i<N_AGENTS;++i){
            pos_arr[i][0] = next_state.drone_states[i].position[0];
            pos_arr[i][1] = next_state.drone_states[i].position[1];
        }
        T penalty = 0;
//        T overlap_alpha = disaster_phase ? PARAMS::OVERLAP_ALPHA * 0.2 : PARAMS::OVERLAP_ALPHA;
        if (!next_state.disaster_detected_global) {
//            if (!disaster_phase) {
            penalty = reward_attr *
                        overlap_penalty<DEVICE, PARAMS>(device, pos_arr, disaster_phase) /
                        T(N_AGENTS * (N_AGENTS - 1) / 2);
        }

        if(disaster_phase && !next_state.disaster_detected_global){
            T temporal_penalty = (next_state.step_count - next_state.disaster_spawn_step)/(PARAMS::EPISODE_STEP_LIMIT - next_state.disaster_spawn_step);
            reward_attr -= temporal_penalty;
        }
/* ---------------------------------------------------------------
   3.  Final per-agent reward
   --------------------------------------------------------------- */
        total_reward = reward_attr - penalty;

        utils::assert_exit(device, !math::is_nan(device.math, total_reward), "reward is nan");

        /* ---------- coverage metric bookkeeping (no reward impact) ------------- */
        if(!next_state.disaster_detected_global)
        {
            T cov = priority_area_coverage<DEVICE,SPEC>(device, next_state);

            next_state.metrics.total_coverage_ratio =
                    state.metrics.total_coverage_ratio + cov;

            next_state.metrics.coverage_measurement_count =
                    state.metrics.coverage_measurement_count + 1;
        }
        else
        {
            /* keep previous counters unchanged */
            next_state.metrics.total_coverage_ratio     =
                    state.metrics.total_coverage_ratio;
            next_state.metrics.coverage_measurement_count =
                    state.metrics.coverage_measurement_count;
        }
/* ----------------------------------------------------------------------- */

        return total_reward/N_AGENTS;
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

            // New: Normalized time in cell
            // Option 1: Linear with clipping to [-1, 1]
            T normalized_time = agent_state.dead ? T(-1) :
                                T(2) * math::min(device.math, T(agent_state.steps_in_current_cell) / T(PARAMS::MAX_STEPS_FOR_NORMALIZATION), T(1)) - T(1);

            set(observation, 0, agent_i * PER_AGENT_OBS_DIM + 8, normalized_time);
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
            log(device, device.logger, "All drones dead");
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
        if (terminate || state.step_count == PARAMS::EPISODE_STEP_LIMIT) {
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
            bool had_enough_coverage_time = !state.disaster.active ||
                                            (state.disaster_spawn_step >= PARAMS::MINIMUM_COVERAGE_STEPS);

            if (had_enough_coverage_time) {
                if (state.metrics.coverage_measurement_count > 0) {
                    T average_coverage = state.metrics.total_coverage_ratio / state.metrics.coverage_measurement_count;
                    add_scalar(device, device.logger, "coverage/average_priority_area_coverage", average_coverage);
                }
                add_scalar(device, device.logger, "coverage/measurement_count", state.metrics.coverage_measurement_count);
            }

            // Always log when disaster spawned (or episode length if no disaster)
            TI coverage_only_steps = state.disaster.active ? state.disaster_spawn_step : state.step_count;
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
                    alive_count++;
                    T battery = state.drone_states[agent_i].battery;
                    total_battery += battery;
                    min_battery = math::min(device.math, min_battery, battery);
                    max_battery = math::max(device.math, max_battery, battery);
                }
            }

//            if (alive_count > 0) {
//                add_scalar(device, device.logger, "battery/average_final", total_battery / alive_count);
//                add_scalar(device, device.logger, "battery/min_final", min_battery);
//                add_scalar(device, device.logger, "battery/max_final", max_battery);
//            }
//            add_scalar(device, device.logger, "agents/alive_at_end", alive_count);

            // Survival rate
            T survival_rate = alive_count / PARAMS::N_AGENTS;
//            add_scalar(device, device.logger, "agents/survival_rate", survival_rate);
        }

        return terminate;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif  // RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_GENERIC_H