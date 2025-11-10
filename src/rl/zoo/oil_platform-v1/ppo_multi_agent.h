#include "environment.h"
#include <rl_tools/rl/algorithms/ppo/loop/core/config.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1::ppo_multi_agent {
    namespace rlt = rl_tools;
    template <typename DEVICE, typename T, typename TI, typename RNG>
    struct FACTORY {
        using ENVIRONMENT = typename ENVIRONMENT_FACTORY<DEVICE, T, TI>::ENVIRONMENT;

        struct LOOP_CORE_PARAMETERS
                : rlt::rl::algorithms::ppo::loop::core::DefaultParameters<T, TI, ENVIRONMENT>
        {
            // Multi-agent specific network parameters
            // Smaller networks per agent since each processes less information
            static constexpr TI ACTOR_HIDDEN_DIM = 64;    // Per-agent network size
            static constexpr TI ACTOR_NUM_LAYERS = 3;     // Deep enough for agent learning
            static constexpr auto ACTOR_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;
            
            // Critic processes full state, so keep it reasonably sized
            static constexpr TI CRITIC_HIDDEN_DIM = 128;   // Full state processing
            static constexpr TI CRITIC_NUM_LAYERS = 3;     
            static constexpr auto CRITIC_ACTIVATION_FUNCTION = nn::activation_functions::ActivationFunction::FAST_TANH;

            // PPO-specific parameters for multi-agent
            static constexpr T GAMMA = 0.98;
//            static constexpr T LAMBDA = 0.95;  // GAE lambda
//            static constexpr T EPSILON_CLIP = 0.2;  // PPO clipping
//            static constexpr T INITIAL_ACTION_STD = 0.5;  // For continuous actions
//            static constexpr T LEARNING_RATE_ACTOR = 3e-4;
//            static constexpr T LEARNING_RATE_CRITIC = 1e-3;
            
            // Training parameters (declare first so BATCH_SIZE can be used below)
            // You can easily adjust N_ENVIRONMENTS: 1 (single env), 16, 32, 64, 128...
            static constexpr TI N_ENVIRONMENTS = 32;  // Start with moderate parallelism
            static constexpr TI ON_POLICY_RUNNER_STEPS_PER_ENV = 128;  // Match bottleneck
            static constexpr TI BATCH_SIZE = 1024;  // Fixed batch size like bottleneck (not calculated)
            static constexpr TI STEP_LIMIT = 10000;
            
            // Optimizer parameters
            struct OPTIMIZER_PARAMETERS: nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>{
                static constexpr T ALPHA = 1e-3;
            };
            
            // PPO algorithm parameters
            struct PPO_PARAMETERS: rl::algorithms::ppo::DefaultParameters<T, TI, BATCH_SIZE>{
                static constexpr T GAMMA = 0.98;
                static constexpr T ACTION_ENTROPY_COEFFICIENT = 0.01;  // Reduced exploration
                static constexpr TI N_EPOCHS = 1;  // Match bottleneck for stability
                static constexpr bool IGNORE_TERMINATION = true;
            };
        };

        // Use multi-agent approximator (same as bottleneck)
        using LOOP_CORE_CONFIG = rlt::rl::algorithms::ppo::loop::core::Config<
                T, TI, RNG, ENVIRONMENT, LOOP_CORE_PARAMETERS,
                rlt::rl::algorithms::ppo::loop::core::ConfigApproximatorsSequentialMultiAgent
        >;

        struct LOOP_EVAL_PARAMETERS
                : rlt::rl::loop::steps::evaluation::Parameters<T,TI,LOOP_CORE_CONFIG>
        {
            static constexpr TI EVALUATION_INTERVAL = 200000;
        };
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
