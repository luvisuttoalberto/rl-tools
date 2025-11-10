#include "environment.h"
#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/algorithms/sac/loop/core/approximators_hierarchical.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1::sac_hierarchical {
    namespace rlt = rl_tools;
    template <typename DEVICE, typename T, typename TI, typename RNG>
    struct FACTORY {
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, T, TI>::ENVIRONMENT;

        struct LOOP_CORE_PARAMETERS
                : rlt::rl::algorithms::sac::loop::core::DefaultParameters<T, TI, ENVIRONMENT>
        {
            // Hierarchical architecture parameters
            // These are larger than standard because we're processing hierarchical features
            static constexpr TI ACTOR_HIDDEN_DIM = 128;   // Increased for hierarchical processing
            static constexpr TI ACTOR_NUM_LAYERS = 3;     // Deep enough for hierarchical learning
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
            
            static constexpr TI CRITIC_HIDDEN_DIM = 128;   // Increased for hierarchical processing
            static constexpr TI CRITIC_NUM_LAYERS = 3;     // Deep enough for hierarchical learning
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;

            // Custom hierarchical architecture parameters (optional overrides)
            // If not specified, the approximator will use defaults based on environment
            // static constexpr TI AGENT_HEAD_DIM = 24;         // Override default (16) 
            // static constexpr TI SHARED_PROCESSOR_DIM = 16;   // Override default (10)

            struct SAC_PARAMETERS
                    : rl::algorithms::sac::DefaultParameters<T, TI, ENVIRONMENT::ACTION_DIM>
            {
                static constexpr T GAMMA = 0.98;
                static constexpr TI N_EPOCHS = 1;
                
                // Slightly larger batch sizes for better hierarchical learning
                static constexpr TI ACTOR_BATCH_SIZE = 256;
                static constexpr TI CRITIC_BATCH_SIZE = 256;
            };

            static constexpr TI STEP_LIMIT        = 10000000;
            static constexpr TI REPLAY_BUFFER_CAP = 10000000;
        };

        // Use our custom hierarchical approximator instead of the standard MLP
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<
                T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS,
                rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsHierarchical
        >;

        struct LOOP_EVAL_PARAMETERS
                : rlt::rl::loop::steps::evaluation::Parameters<T,TI,LOOP_CORE_CONFIG>
        {
            static constexpr TI EVALUATION_INTERVAL = 200000;
        };
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
