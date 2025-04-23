#pragma once

#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/rl/environments/multi_agent/oil_platform/operations_cpu.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>

#include <rl_tools/rl/algorithms/sac/loop/core/config.h>
#include <rl_tools/rl/loop/steps/evaluation/config.h>
#include <rl_tools/rl/loop/steps/timing/config.h>
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <rl_tools/rl/loop/steps/evaluation/operations_generic.h>
#include <rl_tools/rl/loop/steps/timing/operations_cpu.h>

namespace rlt = rl_tools;

// 1) Device & RNG typedefs
using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG    = decltype(rlt::random::default_engine(typename DEVICE::SPEC::RANDOM{}));
using T      = float;
using TI     = typename DEVICE::index_t;

// 2) Environment alias
using PLATFORM_SPEC = rlt::rl::environments::multi_agent::oil_platform::Specification<T,TI>;
using ENVIRONMENT  = rlt::rl::environments::multi_agent::OilPlatform<PLATFORM_SPEC>;

// 3) SAC core parameters
struct LOOP_CORE_PARAMETERS
        : rlt::rl::algorithms::sac::loop::core::DefaultParameters<T,TI,ENVIRONMENT>
{
    struct SAC_PARAMETERS
            : rlt::rl::algorithms::sac::DefaultParameters<T,TI,ENVIRONMENT::ACTION_DIM>
    {
        static constexpr TI ACTOR_BATCH_SIZE  = 256;
        static constexpr TI CRITIC_BATCH_SIZE = 256;
    };

    // Warmup steps (increased for larger replay buffer)
    static constexpr TI N_WARMUP_STEPS = 10000;        // Increased from 100 for better initial exploration
    static constexpr TI N_WARMUP_STEPS_CRITIC = 5000;  // Start critic training earlier
    static constexpr TI N_WARMUP_STEPS_ACTOR = 10000;  // Wait for critic to stabilize

    // Core parameters
    static constexpr TI STEP_LIMIT        = ENVIRONMENT::EPISODE_STEP_LIMIT;
    static constexpr TI REPLAY_BUFFER_CAP = 1000000;
    static constexpr TI ACTOR_NUM_LAYERS  = 4;
    static constexpr TI ACTOR_HIDDEN_DIM  = 256;
    static constexpr TI CRITIC_NUM_LAYERS = 4;
    static constexpr TI CRITIC_HIDDEN_DIM = 256;
    static constexpr T   ALPHA            = 0.2;

    // Custom optimizer parameters for the multi-agent task
    struct ACTOR_OPTIMIZER_PARAMETERS : rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T> {
        static constexpr T ALPHA = 0.0001;  // Reduced from 0.001 for stability
        static constexpr T EPSILON = 1e-5;  // Increased from 1e-7 for better numerical stability
    };

    struct CRITIC_OPTIMIZER_PARAMETERS : rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T> {
        static constexpr T ALPHA = 0.0001;  // Reduced from 0.001 for stability
        static constexpr T EPSILON = 1e-5;  // Increased from 1e-7 for better numerical stability
    };

    struct ALPHA_OPTIMIZER_PARAMETERS : rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T> {
        static constexpr T ALPHA = 0.0001;  // Reduced from 0.001 for stability
    };
};

// 4) Build the loop config — two modes: BENCHMARK or with evaluation
#ifdef BENCHMARK

using LOOP_CORE_CONFIG   = rlt::rl::algorithms::sac::loop::core::Config<T,TI,RNG,ENVIRONMENT,LOOP_CORE_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_CORE_CONFIG>;
using LOOP_CONFIG        = LOOP_TIMING_CONFIG;

#else  // ───────── evaluation‑enabled build ────────────

using LOOP_CORE_CONFIG = rlt::rl::algorithms::sac::loop::core::Config<
        T,TI,RNG,ENVIRONMENT,LOOP_CORE_PARAMETERS,
        rlt::rl::algorithms::sac::loop::core::ConfigApproximatorsMLP
>;

// **HERE**: define how many episodes & how often to evaluate
struct LOOP_EVAL_PARAMETERS
        : rlt::rl::loop::steps::evaluation::Parameters<T,TI,LOOP_CORE_CONFIG>
{
    static constexpr TI EVALUATION_EPISODES = 100;
};

using LOOP_EVAL_CONFIG   = rlt::rl::loop::steps::evaluation::Config<LOOP_CORE_CONFIG,LOOP_EVAL_PARAMETERS>;
using LOOP_TIMING_CONFIG = rlt::rl::loop::steps::timing::Config<LOOP_EVAL_CONFIG>;
using LOOP_CONFIG        = LOOP_TIMING_CONFIG;

#endif  // BENCHMARK

// 5) The full state
using LOOP_STATE = typename LOOP_CONFIG::template State<LOOP_CONFIG>;

// 6) run( seed ): malloc, init, step, free
inline void run(TI seed = 0) {
    DEVICE    device;
    LOOP_STATE ts;

    rlt::malloc(device, ts);
    rlt::init   (device, ts, seed);

    while (!rlt::step(device, ts)) {
#ifndef BENCHMARK
        if (ts.step == LOOP_CORE_PARAMETERS::STEP_LIMIT/2) {
            std::cout << "Step: "
                      << ts.step << "/"
                      << LOOP_CORE_PARAMETERS::STEP_LIMIT
                      << "\n";
        }
#endif
    }

    rlt::free(device, ts);
}
