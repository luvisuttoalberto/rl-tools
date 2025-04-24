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
    // Episode & buffer length
    static constexpr TI STEP_LIMIT        = 1000000;
    static constexpr TI REPLAY_BUFFER_CAP = 1000000;

    // Exploration warm-up
    static constexpr TI N_WARMUP_STEPS        = 10000;
    static constexpr TI N_WARMUP_STEPS_CRITIC = 10000;
    static constexpr TI N_WARMUP_STEPS_ACTOR  = 20000;

    // Discount factor
    static constexpr T GAMMA = 0.995;

    // Network size
    static constexpr TI ACTOR_NUM_LAYERS  = 2;
    static constexpr TI ACTOR_HIDDEN_DIM  = 128;
    static constexpr TI CRITIC_NUM_LAYERS = 2;
    static constexpr TI CRITIC_HIDDEN_DIM = 128;

    struct SAC_PARAMETERS
            : rlt::rl::algorithms::sac::DefaultParameters<T,TI,ENVIRONMENT::ACTION_DIM>
    {
        // Batch sizes
        static constexpr TI ACTOR_BATCH_SIZE  = 256;
        static constexpr TI CRITIC_BATCH_SIZE = 256;

        // Update frequencies
        static constexpr TI ACTOR_TRAINING_INTERVAL       = 2;
        static constexpr TI CRITIC_TARGET_UPDATE_INTERVAL = 8;

        // Discount & soft updates (τ = 0.0005)
        static constexpr T GAMMA                        = 0.995;
        static constexpr T ACTOR_POLYAK                 = 1.0 - 0.0005;
        static constexpr T CRITIC_POLYAK                = 1.0 - 0.0005;

        // Entropy tuning
        static constexpr T TARGET_ENTROPY       = -0.5 * ((T)ENVIRONMENT::ACTION_DIM);
        static constexpr T ALPHA                = 0.01;
        static constexpr bool ADAPTIVE_ALPHA    = true;
    };

    struct ACTOR_OPTIMIZER_PARAMETERS
            : rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>
    {
        static constexpr T ALPHA                   = 3e-5;
        static constexpr T EPSILON                 = 1e-6;
        static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
        static constexpr T GRADIENT_CLIP_VALUE     = 0.5;
        static constexpr bool ENABLE_WEIGHT_DECAY   = true;
        static constexpr T WEIGHT_DECAY             = 1e-4;
    };

    struct CRITIC_OPTIMIZER_PARAMETERS
            : rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>
    {
        static constexpr T ALPHA                   = 3e-5;
        static constexpr T EPSILON                 = 1e-6;
        static constexpr bool ENABLE_GRADIENT_CLIPPING = true;
        static constexpr T GRADIENT_CLIP_VALUE     = 0.5;
        static constexpr bool ENABLE_WEIGHT_DECAY   = true;
        static constexpr T WEIGHT_DECAY             = 1e-4;
    };

    struct ALPHA_OPTIMIZER_PARAMETERS
            : rlt::nn::optimizers::adam::DEFAULT_PARAMETERS_TENSORFLOW<T>
    {
        static constexpr T ALPHA = 1e-4;
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
