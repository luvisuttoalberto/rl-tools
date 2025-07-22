#include "../../../../version.h"
#include "../../../../rl_tools.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OIL_PLATFORM_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OIL_PLATFORM_H

#include "../../../../containers/matrix/matrix.h"
#include "../../../../containers/matrix/operations_generic.h"
#include "../../../../math/operations_generic.h"
#include "../environments.h"

RL_TOOLS_NAMESPACE_WRAPPER_START

namespace rl_tools {
    namespace rl {
        namespace environments {
            namespace multi_agent {
                namespace oil_platform {
                    template <typename T_T, typename T_TI>
                    struct DefaultParameters {
                        using T = T_T;
                        using TI = T_TI;
                        // Number of drones
                        static constexpr TI N_AGENTS = 3;

                        // Sensing & motion
                        static constexpr T SENSOR_RANGE = 5.0;
                        static constexpr T DT = 0.05;
                        static constexpr T MAX_ACCELERATION = 2.0;
                        static constexpr T MAX_SPEED = 2.0;

                        // Geometry of platform & pipes
                        // Square platform at center, half-size
                        static constexpr T PLATFORM_HALF_SIZE = 2.0;
                        static constexpr T PIPE_WIDTH = 2.0;
                        static constexpr TI GRID_SIZE_X = 20;
                        static constexpr TI GRID_SIZE_Y = 20;

                        static constexpr TI GRID_RES = 20;


                        // Episode length
                        static constexpr TI EPISODE_STEP_LIMIT = 1000;

                        // Disaster parameters
                        static constexpr T DISASTER_MAX_SPEED = 1.0;
                        static constexpr TI DISASTER_MINIMUM_SPAWN_STEP = 0;
                        static constexpr T DISASTER_PROBABILITY_SPAWN = 0.005;

                        // --- new: stochastic drift controls -----------------------------------------
                        static constexpr T DISASTER_TURN_MAX_RAD       = T(0.10);   // ≤~5.7° yaw jitter/step
                        static constexpr T DISASTER_SPEED_JITTER_FRAC  = T(0.10);   // ±10% fractional speed jitter
// ----------------------------------------------------------------------------

                        // Charging parameters
                        static constexpr T CHARGING_RATE = 1.0;
//                        static constexpr T DISCHARGE_RATE = 0.15;
                        static constexpr T CHARGING_STATION_RANGE = 2.0;
                        static constexpr T CHARGING_VELOCITY_THRESHOLD = 0.5;
                        static constexpr T CHARGING_STATION_POSITION_X = 5.0;
                        static constexpr T CHARGING_STATION_POSITION_Y = 5.0;
                        // static constexpr T CHARGING_ACCELERATION_THRESHOLD = 0.5;

                        static constexpr TI NOVELTY_WINDOW = 50;              // freshness horizon

                        /* --- detection steepness constant ----------------------------------- */
                        static constexpr T EPSILON_INSIDE = T(0.10) * SENSOR_RANGE;     // 10 % inside rim
                        static constexpr T P0_DETECT      = T(0.95);                    // 95 % probability
                        static constexpr T DETECTION_BETA =
                                T( std::log(P0_DETECT / (T(1) - P0_DETECT)) ) / EPSILON_INSIDE;


                        static constexpr T GAUSS_SIGMA_COVER = SENSOR_RANGE/2;          // pre-disaster
                        static constexpr T GAUSS_SIGMA_EVENT = SENSOR_RANGE/2;    // post-spawn
                        static constexpr T GAUSS_BETA        = 3.0;   // positive scale
                        static constexpr T OVERLAP_RHO_COVERAGE       = SENSOR_RANGE * 0.5;
                        static constexpr T OVERLAP_RHO_DETECTION       = SENSOR_RANGE * 0.3;
                        static constexpr T OVERLAP_ALPHA     = 0.0;   // penalty scale


                        // For time in cell approx
                        static constexpr TI GRID_CELLS_X = 5; // Adjust based on your needs
                        static constexpr TI GRID_CELLS_Y = 5;

                        // Calculate expected time to cross a grid cell
                        static constexpr T CELL_SIZE_X = T(GRID_SIZE_X) / T(GRID_CELLS_X);
                        static constexpr T CELL_SIZE_Y = T(GRID_SIZE_Y) / T(GRID_CELLS_Y);
                        static constexpr T CELL_DIAGONAL = std::sqrt(CELL_SIZE_X * CELL_SIZE_X + CELL_SIZE_Y * CELL_SIZE_Y);
                        static constexpr T TIME_TO_CROSS_CELL = CELL_DIAGONAL / MAX_SPEED; // seconds
                        static constexpr TI STEPS_TO_CROSS_CELL = static_cast<TI>(TIME_TO_CROSS_CELL / DT);

                        // Set normalization based on "staying too long" - e.g., 3x the crossing time
                        static constexpr TI MAX_STEPS_FOR_NORMALIZATION = STEPS_TO_CROSS_CELL * 3;

                        static constexpr TI MINIMUM_COVERAGE_STEPS = 100;  // Minimum steps before disaster to count as valid coverage episode


                        struct Point { T x, y; };

                        /* Worst-case capacity (all grid cells) */
                        static constexpr TI ROI_CAP = GRID_RES * GRID_RES;

                        /* constexpr builder returns BOTH the array and the final count   */
                        static constexpr auto build_roi_catalogue()   // C++20 constexpr OK
                        {
                            std::array<Point, ROI_CAP> buf{};    // fixed-size storage
                            TI n = 0;

                            const T DX = T(GRID_SIZE_X) / GRID_RES;
                            const T DY = T(GRID_SIZE_Y) / GRID_RES;
                            const T CX = GRID_SIZE_X * T(0.5);
                            const T CY = GRID_SIZE_Y * T(0.5);

                            for (TI gx = 0; gx < GRID_RES; ++gx)
                                for (TI gy = 0; gy < GRID_RES; ++gy)
                                {
                                    const T x = DX * (gx + T(0.5));
                                    const T y = DY * (gy + T(0.5));

                                    const T adx = std::abs(x - CX);
                                    const T ady = std::abs(y - CY);

                                    const bool in_plat  = (adx <= PLATFORM_HALF_SIZE && ady <= PLATFORM_HALF_SIZE);
                                    const bool in_pipeh = (ady <= PIPE_WIDTH * 0.5 && adx >= PLATFORM_HALF_SIZE);
                                    const bool in_pipev = (adx <= PIPE_WIDTH * 0.5 && ady >= PLATFORM_HALF_SIZE);

                                    if (in_plat || in_pipeh || in_pipev)
                                        buf[n++] = {x, y};
                                }

                            /* Return both data and the true filled size */
                            return std::pair(buf, n);      // CTAD since C++17
                        }

                        /*  Store the result as two separate constexpr objects  */
                        static constexpr auto ROI_DATA = build_roi_catalogue();
                        static constexpr std::array<Point, ROI_CAP> ROI_CATALOGUE = ROI_DATA.first;
                        static constexpr TI ROI_SIZE  = ROI_DATA.second;
                    };

                    template <typename T_PARAMETERS>
                    struct Observation {
                        using PARAMETERS = T_PARAMETERS;
                        using T = typename PARAMETERS::T;
                        using TI = typename PARAMETERS::TI;
                        static constexpr TI PER_AGENT_DIM = 9; // pos(2), vel(2), battery (1), dead (1), is_charging (1), is_detecting(1), time_in_cell (1)
                        static constexpr TI SHARED_DIM = 5; // last_detected_disaster_pos(2), charging_station_pos(2), disaster_detected_global (1)
                        static constexpr TI DIM = PARAMETERS::N_AGENTS * PER_AGENT_DIM + SHARED_DIM;
                    };

                    template <typename T, typename TI>
                    struct DroneState {
                        T position[2];
                        T velocity[2];
                        T battery;
                        bool dead;
                        bool is_charging;
                        bool is_detecting;

                        // New grid tracking fields
                        TI current_grid_x;
                        TI current_grid_y;
                        TI steps_in_current_cell;
                    };

                    template <typename T>
                    struct DisasterState {
                        bool active;
                        T position[2];
                        T velocity[2];
                    };

                    template <typename T, typename TI, typename PARAMS>
                    struct Metrics {
                        T total_coverage_ratio;
                        TI coverage_measurement_count;
                        TI disaster_active_steps;
                        TI total_charging_sessions;
                        TI appropriate_charging_count;
                        TI inappropriate_charging_count;
                        TI death_count;
                        T  cumulative_potential_reward;                 // Σ r_t^{pot}
                        TI potential_steps;                             // #steps accumulated
                        TI cumulative_detection_latency;  // Σ first-detection latencies
                        TI detection_count;  // # disasters that were spotted

                        /* ───── new ───── */
//                        bool covered_grid[PARAMS::GRID_RES][PARAMS::GRID_RES];  // bitmap of visited hi-priority cells
//                        TI total_covered_cells;                       // scalar loggable metric
//
//                        TI visit_age_grid[PARAMS::GRID_RES][PARAMS::GRID_RES];   // steps since last visit

                    };


                    template <typename SPEC>
                    struct State {
                        using T = typename SPEC::T;
                        using TI = typename SPEC::TI;
                        using PARAMS = typename SPEC::PARAMETERS;

                        DroneState<T, TI> drone_states[SPEC::PARAMETERS::N_AGENTS];
                        DisasterState<T> disaster;
                        Metrics<T, TI, PARAMS> metrics;
                        bool disaster_detected_global;
                        T step_count;
                        T disaster_undetected_steps;
                        T last_detected_disaster_position[2];
                        TI disaster_spawn_step;   // episode step index at spawn time
                    };


                    template <typename T_T, typename T_TI,
                            typename T_PARAMETERS = DefaultParameters<T_T, T_TI>,
                            typename T_OBSERVATION = Observation<T_PARAMETERS>>
                    struct Specification {
                        using T = T_T;
                        using TI = T_TI;
                        using OBSERVATION = T_OBSERVATION;
                        using PARAMETERS = T_PARAMETERS;
                        using STATE = State<Specification<T_T, T_TI, T_PARAMETERS, T_OBSERVATION>>;
                        using ACTION = Matrix<
                                matrix::Specification<
                                        T, TI,
                                        /*ROWS=*/1,
                                        /*COLS=*/PARAMETERS::N_AGENTS * 2,
                                        /*RowMajor=*/true,
                                        matrix::layouts::Fixed<TI,1,PARAMETERS::N_AGENTS * 2>
                                >
                        >;
                    };

                } // namespace oil_platform

                // The OilPlatform environment itself wraps the oil_platform namespace:
                template <typename T_SPEC>
                struct OilPlatform : Environment<typename T_SPEC::T, typename T_SPEC::TI> {
                    using SPEC = T_SPEC;
                    using PARAMETERS = typename SPEC::PARAMETERS;
                    using T = typename SPEC::T;
                    using TI = typename SPEC::TI;
                    using State = typename SPEC::STATE;
                    using Parameters = typename SPEC::PARAMETERS;
                    using Observation = typename SPEC::OBSERVATION;
                    using ObservationPrivileged = Observation;
                    using Action = typename SPEC::ACTION;
                    static constexpr TI N_AGENTS = Parameters::N_AGENTS;
                    static constexpr TI PER_AGENT_ACTION_DIM = 2; // acceleration in x and y
                    static constexpr TI ACTION_DIM = N_AGENTS * PER_AGENT_ACTION_DIM;
                    static constexpr TI EPISODE_STEP_LIMIT = Parameters::EPISODE_STEP_LIMIT;
                };

            } // namespace multi_agent
        } // namespace environments
    } // namespace rl
} // namespace rl_tools

RL_TOOLS_NAMESPACE_WRAPPER_END

#endif