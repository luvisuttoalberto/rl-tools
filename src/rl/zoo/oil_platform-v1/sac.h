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

//            static constexpr TI ACTOR_HIDDEN_DIM = 64;
//            static constexpr TI ACTOR_NUM_LAYERS = 3;
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
//            static constexpr TI CRITIC_HIDDEN_DIM = 128;
//            static constexpr TI CRITIC_NUM_LAYERS = 3;
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;


//            struct OPTIMIZER_PARAMETERS: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>{
//                static constexpr T ALPHA = 1e-3;
//            };
//
//            struct ACTOR_OPTIMIZER_PARAMETERS: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>{
//                static constexpr T ALPHA = 3e-4;
//                static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
//                static constexpr T GRADIENT_CLIP_VALUE = 5.0;
//            };
//
//            struct CRITIC_OPTIMIZER_PARAMETERS: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>{
//                static constexpr T ALPHA = 3e-4;
//                static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
//                static constexpr T GRADIENT_CLIP_VALUE = 5.0;
//            };
//
//            struct ALPHA_OPTIMIZER_PARAMETERS: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>{
//                static constexpr T ALPHA = 3e-4;
//                static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
//                static constexpr T GRADIENT_CLIP_VALUE = 1.0;
//            };
//
//            // Increase warmup steps
//            static constexpr TI N_WARMUP_STEPS = 10000;
//            static constexpr TI N_WARMUP_STEPS_CRITIC = 10000;
//            static constexpr TI N_WARMUP_STEPS_ACTOR = 10000;

            struct SAC_PARAMETERS
                    : rl::algorithms::sac::DefaultParameters<T, TI, ENVIRONMENT::ACTION_DIM>
            {
                static constexpr T GAMMA = 0.98;
//                static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.03; // Increased from default to encourage exploration
                static constexpr TI N_EPOCHS = 1;
            };

            static constexpr TI STEP_LIMIT        = 2000000;
            static constexpr TI REPLAY_BUFFER_CAP = 2000000;
        };

        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<
                T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS,
                rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP
        >;

        struct LOOP_EVAL_PARAMETERS
                : rlt::rl::loop::steps::evaluation::Parameters<T,TI,LOOP_CORE_CONFIG>
        {
            static constexpr TI EVALUATION_INTERVAL = 200000;
        };
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END