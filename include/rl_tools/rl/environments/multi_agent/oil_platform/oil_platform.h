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

                    // Drone modes
//                    enum class DroneMode {
//                        NORMAL = 0,
//                        EMERGENCY = 1,
//                        RECHARGING = 2
//                    };

                    template <typename T_T, typename T_TI>
                    struct DefaultParameters {
                        using T = T_T;
                        using TI = T_TI;
                        // Number of drones total and deployed
                        static constexpr TI N_AGENTS           = 3;
//                        static constexpr TI ACTIVE_DRONES      = 3;

                        // Sensing & motion
                        static constexpr T SENSOR_RANGE        = 5.0;
                        static constexpr T DT                  = 0.05;
                        static constexpr T MAX_ACCELERATION    = 2.0;

                        // Geometry of platform & pipes
                        // Square platform at center, half-size
                        static constexpr T PLATFORM_HALF_SIZE  = 2.0;
                        // Pipes extending from platform, width
                        static constexpr T PIPE_WIDTH          = 2.0;  // Increased from 1.0 for better visibility

//                        static constexpr T EXPLORATION_BONUS   = 0.05;
//                        static constexpr T HIGH_PRIORITY_BONUS = 1.0f;
//
//                        // how many steps before a cell "ages out"
//                        static constexpr TI REVISIT_THRESHOLD  = 300;
//
//                        // small bonus for every non-priority cell
//                        static constexpr T GENERAL_AREA_BONUS  = 0.01f;
//
//                        // scales the proximity reward when disaster is active
//                        static constexpr T DISASTER_PRIORITY   = 8.0f;


                        // Grid discretization
//                        static constexpr T GRID_CELL_SIZE      = 1.0;
                        static constexpr TI GRID_SIZE_X        = 20;
                        static constexpr TI GRID_SIZE_Y        = 20;

                        // Episode length
                        static constexpr TI EPISODE_STEP_LIMIT = 1000;

                        static constexpr TI DISASTER_DETECTION_TIMEOUT = 200;

                        // Disaster parameters
                        static constexpr T DISASTER_MAX_SPEED = 0.5;  // ADDED: maximum disaster speed

                        // Battery & recharging parameters - improved values
//                        static constexpr T RECHARGE_RATE       = 1.5;   // Increased from 1.0 (% per step at base)
//                        static constexpr T DISCHARGE_RATE      = 0.15;  // Reduced from 0.5 (% per step in flight)
//                        static constexpr TI FULLY_CHARGED_STEPS = 3;    // Reduced from 5 (steps at 100% before swap)

                        // Grace period before disaster detection penalty
//                        static constexpr TI DISASTER_DETECTION_GRACE_PERIOD = 30;
//
//                        static constexpr T CRITICAL_BATTERY_PENALTY = 2.0f;  // Reduced from 3.0
                    };

                    template <typename T_PARAMETERS>
                    struct Observation {
                        using PARAMETERS = T_PARAMETERS;
                        using T = typename PARAMETERS::T;
                        using TI = typename PARAMETERS::TI;
                        static constexpr TI PER_AGENT_DIM = 7; // pos(2), vel(2), disaster_detected(1), last_detected_disaster_position(2)
                        static constexpr TI DIM = PARAMETERS::N_AGENTS * PER_AGENT_DIM;
                    };

                    template <typename T, typename TI>
                    struct DroneState {
                        T          position[2];
                        T          velocity[2];
                        T          acceleration[2];
//                        DroneMode  mode;
                        T          battery;         // [0–100]%
                        T          last_detected_disaster_position[2]; // (–1,–1) until first detection
                    };

                    template <typename T>
                    struct DisasterState {
                        bool active;
                        T position[2];
                        T velocity[2];  // ADDED: velocity vector for moving disaster
                    };



                    template <typename SPEC>
                    struct State {
                        using T  = typename SPEC::T;
                        using TI = typename SPEC::TI;

                        DroneState<T,TI>    drone_states[SPEC::PARAMETERS::N_AGENTS];
                        DisasterState<T>    disaster;

                        // store the last step when each cell was visited
//                        TI last_visit[SPEC::PARAMETERS::GRID_SIZE_X * SPEC::PARAMETERS::GRID_SIZE_Y];

                        TI step_count;

                        TI disaster_undetected_steps;

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