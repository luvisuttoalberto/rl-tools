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

                        static constexpr bool BATTERY_ENABLED = false;
                        static constexpr bool OVERLAP_REPULSION_ACTIVE = false;
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
                        // static constexpr T DISASTER_PROBABILITY_SPAWN = 0.01;
                        static constexpr T DISASTER_PROBABILITY_SPAWN = 0.;

                        // --- new: stochastic drift controls -----------------------------------------
                        static constexpr T DISASTER_TURN_MAX_RAD       = T(0.10);   // ≤~5.7° yaw jitter/step
                        static constexpr T DISASTER_SPEED_JITTER_FRAC  = T(0.10);   // ±10% fractional speed jitter
// ----------------------------------------------------------------------------

                        // Charging parameters
                        static constexpr T CHARGING_RATE = 1.0;
//                        static constexpr T DISCHARGE_RATE = 0.15;
                        static constexpr T CHARGING_STATION_RANGE = 2.0;
                        static constexpr T CHARGING_VELOCITY_THRESHOLD = 0.75;
                        static constexpr T CHARGING_STATION_POSITION_X = 5.0;
                        static constexpr T CHARGING_STATION_POSITION_Y = 5.0;
                        static constexpr T MINIMUM_BATTERY_FOR_CHARGING = 80.0;
                        // static constexpr T CHARGING_ACCELERATION_THRESHOLD = 0.5;

                        // Minimum number of consecutive steps a drone must remain charging
                        static constexpr TI MIN_CHARGE_STEPS = 70;

                        static constexpr TI NOVELTY_WINDOW = 50;              // freshness horizon

                        /* --- detection steepness constant ----------------------------------- */
                        static constexpr T EPSILON_INSIDE = T(0.10) * SENSOR_RANGE;     // 10 % inside rim
                        static constexpr T P0_DETECT      = T(0.95);                    // 95 % probability
                        static constexpr T DETECTION_BETA =
                                T( std::log(P0_DETECT / (T(1) - P0_DETECT)) ) / EPSILON_INSIDE;


                        static constexpr T GAUSS_SIGMA_COVER = SENSOR_RANGE/2;          // pre-disaster
                        static constexpr T GAUSS_SIGMA_EVENT = SENSOR_RANGE/2;    // post-spawn
                        static constexpr T GAUSS_BETA_COVER        = 1.0;   //Coverage reward
                        static constexpr T GAUSS_BETA_EVENT        = 1.0;  // Disaster detection reward
                        
                        // Agent distribution (repulsion) parameters
                        static constexpr T OVERLAP_RHO_COVERAGE       = SENSOR_RANGE;        // Larger spacing during coverage (5.0)
                        static constexpr T OVERLAP_RHO_DETECTION       = SENSOR_RANGE * 0.3;  // Tighter spacing around disaster (1.5)
                        static constexpr T REPULSION_BETA              = 0.675;                 // Penalty scale

                        // Voronoi coverage parameters (alternative to Gaussian + repulsion)
                        static constexpr bool USE_VORONOI_COVERAGE = true;  // Toggle: true=Voronoi, false=Gaussian (original)
                        static constexpr T VORONOI_VARIANCE_PENALTY_WEIGHT = T(0.3);  // Weight for distribution uniformity penalty

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

                        // Battery and charging parameters
                        static constexpr T DISCHARGE_RATE = 0.15;
                        static constexpr T GAUSS_SIGMA_CHARGING = CHARGING_STATION_RANGE*2;
                        static constexpr T GAUSS_BETA_CHARGING = 1.0;
                        static constexpr T BATTERY_URGENCY_K = 1.0;
                        static constexpr T DEATH_PENALTY = -10.0;
                        
                        // Movement cost parameters
                        static constexpr T MOVEMENT_COST_COEFFICIENT = 0.;  // Cost per unit of speed
//                        static constexpr T MOVEMENT_COST_COEFFICIENT = 0.03;  // Cost per unit of speed

                        // Abandonment penalty: penalize agents for not observing previously-detected disasters
                        static constexpr T ABANDONMENT_PENALTY = -0.5;  // Fixed penalty when disaster is unobserved

                        // ---- Multi-center Gaussian reward (platform + 4 arms) -----------------

// World half-sizes (derived from your 0..GRID_SIZE box)
                        static constexpr T WORLD_HALF_X = T(GRID_SIZE_X) * T(0.5);
                        static constexpr T WORLD_HALF_Y = T(GRID_SIZE_Y) * T(0.5);

// Placement of arm centers: distance from platform edge toward the boundary.
// κ = 0.5  ⇒ place arm centers roughly midway from platform edge to boundary.
                        static constexpr T PIPE_CENTER_FRACTION = T(0.5);

// Gaussian widths (tune as you like)
//                        static constexpr T SIGMA_PLATFORM     = T(0.9)  * PLATFORM_HALF_SIZE;             // round
//                        static constexpr T SIGMA_PIPE_LONG_X  = T(0.55) * (WORLD_HALF_X - PLATFORM_HALF_SIZE);
//                        static constexpr T SIGMA_PIPE_LONG_Y  = T(0.55) * (WORLD_HALF_Y - PLATFORM_HALF_SIZE);
//                        static constexpr T SIGMA_PIPE_SHORT   = T(0.35) * PIPE_WIDTH;                     // across pipe

                        // // Gaussian widths (tune as you like)
                        // static constexpr T SIGMA_PLATFORM     = T(1.4)  * PLATFORM_HALF_SIZE;             // round
                        // static constexpr T SIGMA_PIPE_LONG_X  = T(0.75) * (WORLD_HALF_X - PLATFORM_HALF_SIZE);
                        // static constexpr T SIGMA_PIPE_LONG_Y  = T(0.75) * (WORLD_HALF_Y - PLATFORM_HALF_SIZE);
                        // static constexpr T SIGMA_PIPE_SHORT   = T(0.60) * PIPE_WIDTH;                     // across pipe

                        // Gaussian widths (tune as you like)
                        static constexpr T SIGMA_PLATFORM     = T(0.8)  * PLATFORM_HALF_SIZE;             // round
                        static constexpr T SIGMA_PIPE_LONG_X  = T(0.4) * (WORLD_HALF_X - PLATFORM_HALF_SIZE);
                        static constexpr T SIGMA_PIPE_LONG_Y  = T(0.4) * (WORLD_HALF_Y - PLATFORM_HALF_SIZE);
                        static constexpr T SIGMA_PIPE_SHORT   = T(0.4) * PIPE_WIDTH;                     // across pipe



                        // Amplitude per bump (all equal ⇒ equal importance)
                        static constexpr T GAUSS_AMPLITUDE_PLATFORM = T(1.0);
                        static constexpr T GAUSS_AMPLITUDE_PIPE     = T(1.0);

// Aggregation: hard max by default (no overlap boost).
// If you want smoothness, set TAU > 0 and use softmax in the evaluator.
                        static constexpr T MULTIGAUSS_SOFTMAX_TAU = T(0.5);   // 0 ⇒ hard max, >0 ⇒ softmax

                        // Hysteresis thresholds on normalized battery b01 = battery/100
                        static constexpr T B_ON         = T(0.30);  // engage while at pad if b01 <= B_ON
                        static constexpr T B_OFF        = T(0.75);  // release when b01 >= B_OFF
                        static constexpr T B_SMOOTH_ON  = T(0.06);  // slope width around B_ON
                        static constexpr T B_SMOOTH_OFF = T(0.06);  // slope width around B_OFF



                        // Number of Gaussian bumps
                        static constexpr TI N_GAUSS = 5;

                        struct GaussSpec {
                            T cx, cy;   // center
                            T sx, sy;   // std dev along x/y (elliptical)
                            T A;        // amplitude
                        };

// Build the 5 Gaussians at compile time
                        static constexpr auto build_gauss_catalogue() {
                            std::array<GaussSpec, N_GAUSS> g{};

                            // World center
                            const T CX = T(GRID_SIZE_X) * T(0.5);
                            const T CY = T(GRID_SIZE_Y) * T(0.5);

                            // Offsets from center to arm centers (measured along axes)
                            const T offX = PLATFORM_HALF_SIZE + PIPE_CENTER_FRACTION * (WORLD_HALF_X - PLATFORM_HALF_SIZE);
                            const T offY = PLATFORM_HALF_SIZE + PIPE_CENTER_FRACTION * (WORLD_HALF_Y - PLATFORM_HALF_SIZE);

                            // 0) Platform (round)
                            g[0] = GaussSpec{ CX,           CY,           SIGMA_PLATFORM,    SIGMA_PLATFORM,    GAUSS_AMPLITUDE_PLATFORM };

                            // 1) Left arm  (long in x, short in y)
                            g[1] = GaussSpec{ CX - offX,    CY,           SIGMA_PIPE_LONG_X, SIGMA_PIPE_SHORT,  GAUSS_AMPLITUDE_PIPE };

                            // 2) Right arm (long in x, short in y)
                            g[2] = GaussSpec{ CX + offX,    CY,           SIGMA_PIPE_LONG_X, SIGMA_PIPE_SHORT,  GAUSS_AMPLITUDE_PIPE };

                            // 3) Top arm   (short in x, long in y)
                            g[3] = GaussSpec{ CX,           CY + offY,    SIGMA_PIPE_SHORT,  SIGMA_PIPE_LONG_Y, GAUSS_AMPLITUDE_PIPE };

                            // 4) Bottom arm(short in x, long in y)
                            g[4] = GaussSpec{ CX,           CY - offY,    SIGMA_PIPE_SHORT,  SIGMA_PIPE_LONG_Y, GAUSS_AMPLITUDE_PIPE };

                            return g;
                        }

                        static constexpr auto GAUSS_CATALOGUE = build_gauss_catalogue();



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
//                                    const T ady = std::abs(y - CY);
                                    const T ady = (y > CY) ? (y - CY) : (CY - y);

                                    const bool in_plat  = (adx <= PLATFORM_HALF_SIZE && ady <= PLATFORM_HALF_SIZE);
                                    const bool in_pipeh = (ady <= PIPE_WIDTH * 0.5 && adx >= PLATFORM_HALF_SIZE);
                                    const bool in_pipev = (adx <= PIPE_WIDTH * 0.5 && ady >= PLATFORM_HALF_SIZE);

//                                    const bool in_right_pipeh =
//                                            (ady <= PIPE_WIDTH * T(0.5)) &&
//                                            (x   >= CX + PLATFORM_HALF_SIZE);

                                    if (in_plat || in_pipeh || in_pipev)
//                                    if (in_right_pipeh)
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
                        static constexpr TI PER_AGENT_DIM = 8; // pos(2), vel(2), battery (1), dead (1), is_charging (1), is_detecting(1)
                        static constexpr TI PER_OTHER_AGENT_DIM = 6; // pos(2), vel(2), battery(1), is_charging(1) - per other agent
                        static constexpr TI OTHER_AGENTS_DIM = (PARAMETERS::N_AGENTS - 1) * PER_OTHER_AGENT_DIM; // observations of all other agents
                        static constexpr TI SHARED_DIM = 5; // last_detected_disaster_pos(2), charging_station_pos(2), disaster_detected_global (1)
                        // For multi-agent wrapper compatibility: each agent gets its own state + other agents' states + shared info
                        static constexpr TI PER_AGENT_TOTAL_DIM = PER_AGENT_DIM + OTHER_AGENTS_DIM + SHARED_DIM; // 8 + (N-1)*6 + 5 per agent
                        static constexpr TI DIM = PARAMETERS::N_AGENTS * PER_AGENT_TOTAL_DIM; // N * (8 + (N-1)*6 + 5) (divisible by N)
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
//                        TI current_grid_x;
//                        TI current_grid_y;
//                        TI steps_in_current_cell;
//                        bool charge_latch;   // committed-to-charging memory
                        TI charge_hold_remaining;  // counts down while locked to the charger

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
                        T  cumulative_potential_reward;
                        TI potential_steps;
                        TI cumulative_detection_latency;
                        TI detection_count;

                        // New metrics for multiple disasters
                        TI total_disasters_spawned;
                        TI disasters_missed;  // Disasters that left without detection

                        T per_step_reward;
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