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
          enum class DroneMode {
              NORMAL = 0,
              EMERGENCY = 1,
              RECHARGING = 2
          };

          template <typename T_T, typename T_TI>
          struct DefaultParameters {
              using T = T_T;
              using TI = T_TI;
              // Number of drones total and deployed
              static constexpr TI N_AGENTS           = 5;
              static constexpr TI ACTIVE_DRONES      = 3;

              // Sensing & motion
              static constexpr T SENSOR_RANGE        = 5.0;
              static constexpr T DT                  = 0.1;
              static constexpr T MAX_ACCELERATION    = 1.0;

              // Geometry of platform & pipes
              // Square platform at center, half-size
              static constexpr T PLATFORM_HALF_SIZE  = 2.0;
              // Pipes extending from platform, width
              static constexpr T PIPE_WIDTH          = 1.0;

              // Exploration bonus (first‐visit)
              static constexpr T EXPLORATION_BONUS   = 0.05;
              static constexpr T HIGH_PRIORITY_BONUS = 1.0f;


              // Grid discretization
              static constexpr T GRID_CELL_SIZE      = 1.0;
              static constexpr TI GRID_SIZE_X        = 20;
              static constexpr TI GRID_SIZE_Y        = 20;

              // Episode length
              static constexpr TI EPISODE_STEP_LIMIT = 10000;

              // Battery & recharging parameters
              static constexpr T RECHARGE_RATE       = 1.0;   // % per step at base
              static constexpr T DISCHARGE_RATE      = 0.5;   // % per step in flight
              static constexpr TI FULLY_CHARGED_STEPS = 5;    // steps at 100% before swap

          };

          template <typename T_PARAMETERS>
          struct Observation {
              using PARAMETERS = T_PARAMETERS;
              using T = typename PARAMETERS::T;
              using TI = typename PARAMETERS::TI;
              static constexpr TI PER_AGENT_DIM = 10; // pos(2), vel(2), acc(2), mode(1), disaster_detected(1), disaster_pos(2)
              static constexpr TI DIM = PARAMETERS::N_AGENTS * PER_AGENT_DIM;
          };

            template <typename T, typename TI>
            struct DroneState {
                T          position[2];
                T          velocity[2];
                T          acceleration[2];
                DroneMode  mode;
                bool       disaster_detected;
                T          battery;         // [0–100]%
                TI         recharge_count;  // consecutive steps at 100%
            };

          template <typename T>
          struct DisasterState {
              bool active;
              T position[2];
          };

            template <typename SPEC>
            struct State {
                using T  = typename SPEC::T;
                using TI = typename SPEC::TI;

                DroneState<T,TI>    drone_states[SPEC::PARAMETERS::N_AGENTS];
                DisasterState<T>    disaster;
                bool                occupancy[SPEC::PARAMETERS::GRID_SIZE_X * SPEC::PARAMETERS::GRID_SIZE_Y];
                // Step counter to track how many steps have elapsed
                TI                  step_count;
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
