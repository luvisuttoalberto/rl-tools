// evaluate_oil_platform.cpp
// Loads a trained oil-platform multi-agent actor from an HDF5 checkpoint
// and evaluates it through the shared zoo-style evaluation runner (see
// oil_platform-v1/evaluation_runner.h), emitting the same artifacts as the
// scripted baseline (TensorBoard episode metrics with x = episode index,
// return.json, trajectories.json.gz, ui.esm.js). This produces the
// evaluation-episode metrics of a *final* policy, directly comparable to
// the scripted baseline run — unlike the training TensorBoard logs, which
// span the whole training process.
// The run is placed at:
//   experiments/<timestamp>/<commit>_zoo_environment_algorithm/oil_platform-v1_<label>/<seed>/
//
// Usage:
//   evaluate_oil_platform -c path/to/checkpoint.h5 [-s seed] [--label sac-eval]
//                         [-e extrack_base] [--extrack-experiment ts]
//
// Compiled twice: without RL_TOOLS_USE_MULTI_AGENT_PPO it loads the SAC actor
// (per-agent MLP + sample_and_squash), with it the PPO actor (per-agent
// mlp_unconditional_stddev). The two actors have different HDF5 group layouts, so
// the checkpoint has to be read by the binary built for the algorithm that wrote it.
// Both are deterministic under Mode<Evaluation<>>: sample_and_squash emits the mean,
// and the PPO actor's forward pass is the mean with the standard deviation held in a
// separate parameter that evaluation does not touch.

#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/operations_cpu_mux.h>
// The container templates (sequential, multi_agent_wrapper) call malloc/forward/load on
// whatever they wrap through dependent lookup, and the overloads for the wrapped pieces
// live in namespace rl_tools, which ADL does not reach from a type nested deeper (e.g.
// rl_tools::nn::layers::standardize). So every layer and leaf-model header has to be
// included *before* the two container headers, exactly as zoo.cpp orders them.
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#ifdef RL_TOOLS_USE_MULTI_AGENT_PPO
#include <rl_tools/nn/layers/standardize/operations_generic.h>
#endif
#include <rl_tools/nn_models/mlp/operations_generic.h>
#ifdef RL_TOOLS_USE_MULTI_AGENT_PPO
#include <rl_tools/nn_models/mlp_unconditional_stddev/operations_generic.h>
#endif
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/multi_agent_wrapper/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/numeric_types/policy.h>

#include <rl_tools/persist/backends/hdf5/operations_cpu.h>
#include <rl_tools/nn/layers/sample_and_squash/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#ifdef RL_TOOLS_USE_MULTI_AGENT_PPO
#include <rl_tools/nn/layers/standardize/persist.h>
#endif
#include <rl_tools/nn_models/mlp/persist.h>
#ifdef RL_TOOLS_USE_MULTI_AGENT_PPO
#include <rl_tools/nn_models/mlp_unconditional_stddev/persist.h>
#endif
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist.h>

#include "oil_platform-v1/environment.h"
#ifdef RL_TOOLS_USE_MULTI_AGENT_PPO
#include "oil_platform-v1/ppo_multi_agent.h"
#else
#include "oil_platform-v1/per_agent_actor.h"
#include "oil_platform-v1/sac.h"
#endif
#include "oil_platform-v1/evaluation_runner.h"

#include <highfive/H5File.hpp>
#include <CLI/CLI.hpp>

#include <iostream>
#include <string>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG    = DEVICE::SPEC::RANDOM::ENGINE<>;
using TI     = typename DEVICE::index_t;

using PARAMETER_POLICY = rlt::numeric_types::UseCase<rlt::numeric_types::categories::Parameter, float>;
using TYPE_POLICY      = rlt::numeric_types::Policy<float, PARAMETER_POLICY>;

#ifdef RL_TOOLS_USE_MULTI_AGENT_PPO
using FACTORY          = rlt::rl::zoo::oil_platform_v1::ppo_multi_agent::FACTORY<DEVICE, TYPE_POLICY, TI, RNG>;
constexpr const char* DEFAULT_LABEL = "ppo-eval";
#else
using FACTORY          = rlt::rl::zoo::oil_platform_v1::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG>;
constexpr const char* DEFAULT_LABEL = "sac-eval";
#endif
using LOOP_CORE_CONFIG = typename FACTORY::LOOP_CORE_CONFIG;
using ENVIRONMENT      = typename FACTORY::ENVIRONMENT;
using ACTOR            = typename LOOP_CORE_CONFIG::NN::ACTOR_TYPE::template CHANGE_CAPABILITY<rlt::nn::capability::Forward<>>;

int main(int argc, char** argv){
    CLI::App app{"evaluate_oil_platform — evaluate a trained checkpoint, emitting zoo-compatible artifacts"};
    std::string checkpoint_path;
    std::string label = DEFAULT_LABEL;
    TI seed = 0;
    std::string extrack_base_path = "experiments";
    std::string extrack_experiment;
    app.add_option("-c,--checkpoint", checkpoint_path, "Path to HDF5 checkpoint (.h5)")->required();
    app.add_option("--label", label, std::string("Algorithm label for the run directory (default: ") + DEFAULT_LABEL + ")");
    app.add_option("-s,--seed", seed, "Seed (default: 0)");
    app.add_option("-e,--extrack", extrack_base_path, "Extrack base path (default: experiments)");
    app.add_option("--ee,--extrack-experiment", extrack_experiment, "Extrack experiment timestamp (default: now / $RL_TOOLS_EXTRACK_EXPERIMENT)");
    CLI11_PARSE(app, argc, argv);

    DEVICE device;
    rlt::malloc(device);
    rlt::init(device);
    RNG rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, seed);

    ACTOR actor;
    // The SAC actor's operations live in its own namespace and are found via ADL, so the call
    // has to stay unqualified; the PPO actor is a multi_agent_wrapper whose operations are in
    // rl_tools, which ADL does not reach from a type nested below it. These using-declarations
    // make both sets visible and hide the ::malloc/::free that <cstdlib> would otherwise win with.
    using rlt::malloc;
    using rlt::free;
    malloc(device, actor);
    {
        auto hdf5_file = HighFive::File(checkpoint_path, HighFive::File::ReadOnly);
        auto group = rlt::get_group(device, hdf5_file, "actor");
        bool ok = rlt::load(device, actor, group);
        if(!ok){
            std::cerr << "Failed to load actor from: " << checkpoint_path << std::endl;
            return 1;
        }
    }
    std::cerr << "Loaded actor from: " << checkpoint_path << std::endl;

    ENVIRONMENT env;

    int result = rlt::rl::zoo::oil_platform_v1::evaluation_runner::run<DEVICE, TYPE_POLICY>(device, env, actor, rng, seed, label, extrack_base_path, extrack_experiment);

    free(device, actor);
    rlt::free(device, rng);
    return result;
}
