// baseline_oil_platform.cpp
// Runs the scripted (non-learning) baseline policy on the oil-platform
// environment through the shared zoo-style evaluation runner (see
// oil_platform-v1/evaluation_runner.h for the emitted artifacts).
// The run is placed at:
//   experiments/<timestamp>/<commit>_zoo_environment_algorithm/oil_platform-v1_scripted/<seed>/
//
// Usage:
//   baseline_oil_platform [-s seed] [-e extrack_base] [--extrack-experiment ts]

#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/multi_agent_wrapper/operations_generic.h>
#include <rl_tools/numeric_types/policy.h>

#include "oil_platform-v1/environment.h"
#include "oil_platform-v1/scripted_policy.h"
#include "oil_platform-v1/evaluation_runner.h"

#include <CLI/CLI.hpp>

#include <string>

namespace rlt = rl_tools;

using DEVICE = rlt::devices::DEVICE_FACTORY<>;
using RNG    = DEVICE::SPEC::RANDOM::ENGINE<>;
using TI     = typename DEVICE::index_t;

using PARAMETER_POLICY = rlt::numeric_types::UseCase<rlt::numeric_types::categories::Parameter, float>;
using TYPE_POLICY      = rlt::numeric_types::Policy<float, PARAMETER_POLICY>;

using ENVIRONMENT = typename rlt::rl::zoo::oil_platform_v1::ENVIRONMENT_FACTORY<DEVICE, TYPE_POLICY, TI, true>::ENVIRONMENT;
using POLICY      = rlt::rl::zoo::oil_platform_v1::scripted::ScriptedPolicy<TYPE_POLICY, TI, ENVIRONMENT, 1>;

int main(int argc, char** argv){
    CLI::App app{"baseline_oil_platform — evaluate the scripted baseline policy, emitting zoo-compatible artifacts"};
    TI seed = 0;
    std::string extrack_base_path = "experiments";
    std::string extrack_experiment;
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

    ENVIRONMENT env;
    POLICY policy;

    int result = rlt::rl::zoo::oil_platform_v1::evaluation_runner::run<DEVICE, TYPE_POLICY>(device, env, policy, rng, seed, "scripted", extrack_base_path, extrack_experiment);

    rlt::free(device, rng);
    return result;
}
