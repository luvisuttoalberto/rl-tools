#include "environment.h"
#include <rl_tools/rl/algorithms/sac/loop/core/config.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1::sac {
    namespace rlt = rl_tools;
    template <typename DEVICE, typename T, typename TI, typename RNG>
    struct FACTORY {
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, T, TI>::ENVIRONMENT;

        struct LOOP_CORE_PARAMETERS
                : rlt::rl::algorithms::sac::loop::core::DefaultParameters<T, TI, ENVIRONMENT>
        {

            static constexpr TI ACTOR_HIDDEN_DIM = 64;
            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
            static constexpr TI CRITIC_HIDDEN_DIM = 128;
            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
//            static constexpr TI EPISODE_STEP_LIMIT = ENVIRONMENT::EPISODE_STEP_LIMIT;
//            static constexpr TI N_ENVIRONMENTS = 128;
//            static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 128;
//            static constexpr TI BATCH_SIZE = 1024;
            struct OPTIMIZER_PARAMETERS: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>{
                static constexpr T ALPHA = 1e-3;
            };
            static constexpr bool NORMALIZE_OBSERVATIONS = true;

            struct SAC_PARAMETERS
                    : rl::algorithms::sac::DefaultParameters<T, TI, ENVIRONMENT::ACTION_DIM>
            {
                static constexpr T GAMMA = 0.98;
//                static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.01;
                static constexpr TI N_EPOCHS = 1;
//                static constexpr bool IGNORE_TERMINATION = true;
//                static constexpr bool ADAPTIVE_LEARNING_RATE = true;
            };
//                static constexpr TI ACTOR_BATCH_SIZE  = 256;
//                static constexpr TI CRITIC_BATCH_SIZE = 256;
//                static constexpr bool ADAPTIVE_ALPHA    = true;
                static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.02;
//            };

//            struct ACTOR_OPTIMIZER_PARAMETERS
//                    : rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>
//            {
//                static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
//                static constexpr T GRADIENT_CLIP_VALUE     = 0.5;
//                static constexpr bool ENABLE_WEIGHT_DECAY   = true;
//                static constexpr T WEIGHT_DECAY             = 1e-4;
//            };

//            struct CRITIC_OPTIMIZER_PARAMETERS
//                    : rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>
//            {
//                static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
//                static constexpr T GRADIENT_CLIP_VALUE     = 0.5;
//                static constexpr bool ENABLE_WEIGHT_DECAY   = true;
//                static constexpr T WEIGHT_DECAY             = 1e-4;
//            };

            static constexpr TI STEP_LIMIT        = 100000;
            static constexpr TI REPLAY_BUFFER_CAP = 100000;
//            static constexpr TI ACTOR_NUM_LAYERS  = 3;
//            static constexpr TI ACTOR_HIDDEN_DIM  = 128;
//            static constexpr TI CRITIC_NUM_LAYERS = 3;
//            static constexpr TI CRITIC_HIDDEN_DIM = 128;
//            static constexpr T   ALPHA            = 1.0;

            // Exploration warm-up
//            static constexpr TI N_WARMUP_STEPS        = 1000;
//            static constexpr TI N_WARMUP_STEPS_CRITIC = 1000;
//            static constexpr TI N_WARMUP_STEPS_ACTOR  = 2000;

            // Discount factor
//            static constexpr T GAMMA = 0.995;

        };



        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<
                T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS,
                rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP
        >;

        struct LOOP_EVAL_PARAMETERS
                : rlt::rl::loop::steps::evaluation::Parameters<T,TI,LOOP_CORE_CONFIG>
        {
            static constexpr TI EVALUATION_INTERVAL = 10000;
        };
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
