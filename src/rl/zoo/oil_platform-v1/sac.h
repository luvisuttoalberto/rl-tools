#pragma once
#include "environment.h"
#include <rl_tools/rl/algorithms/sac/loop/core/config.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1::sac {
    namespace rlt = rl_tools;
    template <typename DEVICE, typename T, typename TI, typename RNG, bool DYNAMIC_ALLOCATION = true>
    struct FACTORY {
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, T, TI>::ENVIRONMENT;

        struct LOOP_CORE_PARAMETERS: rlt::rl::algorithms::sac::loop::core::DefaultParameters<T, TI, ENVIRONMENT>{
            struct SAC_PARAMETERS: rlt::rl::algorithms::sac::DefaultParameters<T, TI, ENVIRONMENT::ACTION_DIM>{
                static constexpr TI ACTOR_BATCH_SIZE = 512;
                static constexpr TI CRITIC_BATCH_SIZE = 512;
                static constexpr TI TRAINING_INTERVAL = 16;
                static constexpr TI CRITIC_TRAINING_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr TI ACTOR_TRAINING_INTERVAL = 2 * TRAINING_INTERVAL;
                static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 1 * TRAINING_INTERVAL;
                static constexpr T GAMMA = 0.99;
                static constexpr bool IGNORE_TERMINATION = false;
                static constexpr T TARGET_ENTROPY = -static_cast<T>(4);
                static constexpr TI SEQUENCE_LENGTH = 1;
                static constexpr bool ENTROPY_BONUS_NEXT_STEP = false;
            };
            static constexpr TI STEP_LIMIT = 10000000;
            static constexpr TI REPLAY_BUFFER_CAP = 10000000;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr TI ACTOR_HIDDEN_DIM = 128;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::RELU;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr TI CRITIC_HIDDEN_DIM = 256;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = rlt::nn::activation_functions::ActivationFunction::RELU;
            static constexpr TI EPISODE_STEP_LIMIT = 1000;
            static constexpr TI N_WARMUP_STEPS = 0;
            static constexpr TI N_WARMUP_STEPS_CRITIC = 10000;
            static constexpr TI N_WARMUP_STEPS_ACTOR = 10000;
            static constexpr bool SHARED_BATCH = true;
            static constexpr bool COLLECT_EPISODE_STATS = false;
            static constexpr TI N_ENVIRONMENTS = 1;
            struct OPTIMIZER_PARAMETERS_COMMON: rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>{
                static constexpr bool ENABLE_GRADIENT_CLIPPING = false;
                static constexpr T GRADIENT_CLIP_VALUE = 1;
                static constexpr bool ENABLE_WEIGHT_DECAY = false;
                static constexpr T WEIGHT_DECAY = 0.0001;
            };
            struct ACTOR_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                static constexpr T ALPHA = 3e-4;
            };
            struct CRITIC_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                static constexpr T ALPHA = 3e-4;
            };
            struct ALPHA_OPTIMIZER_PARAMETERS: OPTIMIZER_PARAMETERS_COMMON{
                static constexpr T ALPHA = 1e-4;
            };
            static constexpr bool SAMPLE_ENVIRONMENT_PARAMETERS = false;
        };

        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS, rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP, DYNAMIC_ALLOCATION>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
