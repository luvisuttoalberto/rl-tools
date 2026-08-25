#pragma once
#include "environment.h"
#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1::ppo_multi_agent {
    namespace rlt = rl_tools;
    template <typename DEVICE, typename TYPE_POLICY, typename TI, typename RNG>
    struct FACTORY {
        using T = typename TYPE_POLICY::DEFAULT;
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI, true>::ENVIRONMENT;

        struct LOOP_CORE_PARAMETERS
                : rlt::rl::algorithms::ppo::loop::core::DefaultParameters<TYPE_POLICY, TI, ENVIRONMENT>
        {
            // Per-agent actor, centralised critic on the privileged observation. Sizes and
            // activations match sac.h so the comparison is between the algorithms and not
            // between two network architectures.
            static constexpr TI ACTOR_HIDDEN_DIM = 128;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;
            static constexpr TI CRITIC_HIDDEN_DIM = 256;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::RELU;

            static constexpr TI EPISODE_STEP_LIMIT = 1300;

            // Rollout shape is left at PPO's own operating point: an on-policy method needs a
            // wide rollout, and forcing N_ENVIRONMENTS down to SAC's 4 would handicap the
            // baseline in the direction of our own conclusion.
            static constexpr TI N_ENVIRONMENTS = 32;
            static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 128;
            static constexpr TI BATCH_SIZE = 256;

            // STEP_LIMIT counts loop steps; one loop step collects
            // N_ENVIRONMENTS * ON_POLICY_RUNNER_STEPS_PER_ENV = 4096 environment steps.
            // 14648 loop steps is 59,998,208 environment steps, matching SAC's 60M
            // (STEP_LIMIT 15e6 at N_ENVIRONMENTS 4). Recompute this if the rollout shape changes.
            static constexpr TI ENVIRONMENT_STEP_BUDGET = 60000000;
            static constexpr TI STEP_LIMIT = ENVIRONMENT_STEP_BUDGET / (N_ENVIRONMENTS * ON_POLICY_RUNNER_STEPS_PER_ENV);

            // The base class names these ACTOR_/CRITIC_OPTIMIZER_PARAMETERS; a plain
            // OPTIMIZER_PARAMETERS is not read by the approximator config.
            struct OPTIMIZER_PARAMETERS_COMMON: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<TYPE_POLICY>{
                static constexpr T ALPHA = 3e-4; // matching SAC's actor and critic learning rate
            };
            using ACTOR_OPTIMIZER_PARAMETERS = OPTIMIZER_PARAMETERS_COMMON;
            using CRITIC_OPTIMIZER_PARAMETERS = OPTIMIZER_PARAMETERS_COMMON;

            struct PPO_PARAMETERS: rl::algorithms::ppo::DefaultParameters<TYPE_POLICY, TI, BATCH_SIZE>{
                static constexpr T GAMMA = 0.99;
                static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.01;
                static constexpr TI N_EPOCHS = 1;
                // Bootstrap through death and step-limit transitions, as SAC does.
                static constexpr bool IGNORE_TERMINATION = true;
            };
            // LAMBDA, EPSILON_CLIP and the initial action standard deviation stay at the PPO
            // defaults.
        };

        using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<
                TYPE_POLICY, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS,
                rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequentialMultiAgent
        >;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
