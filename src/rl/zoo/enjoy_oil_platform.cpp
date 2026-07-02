// enjoy_oil_platform.cpp
// Loads a trained oil-platform multi-agent SAC policy from an HDF5 checkpoint
// and runs evaluation rollouts, writing per-step data to a CSV file.
//
// Usage:
//   enjoy_oil_platform -c path/to/checkpoint.h5 [-o output.csv] [-n 20] [-s 42] [--stochastic]
//
// Output columns:
//   episode, step, reward, terminated, disaster_active, dis_x, dis_y,
//   agent<i>_x, agent<i>_y, agent<i>_vx, agent<i>_vy,
//   agent<i>_battery, agent<i>_dead, agent<i>_charging, agent<i>_detecting

#include <rl_tools/operations/cpu_mux.h>
#include <rl_tools/nn/operations_cpu_mux.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/multi_agent_wrapper/operations_generic.h>
#include <rl_tools/numeric_types/policy.h>

#include <rl_tools/nn/layers/sample_and_squash/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/mlp/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist.h>

#include "oil_platform-v1/environment.h"
#include "oil_platform-v1/per_agent_actor.h"
#include "oil_platform-v1/sac.h"

#include <HighFive/HighFive.hpp>
#include <CLI/CLI.hpp>

#include <iostream>
#include <fstream>
#include <string>

namespace rlt = rl_tools;

using DEVICE          = rlt::devices::DEVICE_FACTORY<>;
using RNG             = DEVICE::SPEC::RANDOM::ENGINE<>;
using T               = float;
using TI              = typename DEVICE::index_t;

using PARAMETER_POLICY = rlt::numeric_types::UseCase<rlt::numeric_types::categories::Parameter, float>;
using TYPE_POLICY      = rlt::numeric_types::Policy<float, PARAMETER_POLICY>;

using FACTORY          = rlt::rl::zoo::oil_platform_v1::sac::FACTORY<DEVICE, TYPE_POLICY, TI, RNG>;
using LOOP_CORE_CONFIG = typename FACTORY::LOOP_CORE_CONFIG;
using ENVIRONMENT      = typename FACTORY::ENVIRONMENT;
using ACTOR            = typename LOOP_CORE_CONFIG::NN::ACTOR_TYPE::template CHANGE_CAPABILITY<rlt::nn::capability::Forward>;

static constexpr TI N_AGENTS   = ENVIRONMENT::N_AGENTS;
static constexpr TI OBS_DIM    = ENVIRONMENT::Observation::DIM;
static constexpr TI ACTION_DIM = ENVIRONMENT::ACTION_DIM;

// Write CSV header
void write_header(std::ostream& out) {
    out << "episode,step,reward,terminated,disaster_active,dis_x,dis_y";
    for (TI i = 0; i < N_AGENTS; ++i) {
        out << ",a" << i << "_x"
            << ",a" << i << "_y"
            << ",a" << i << "_vx"
            << ",a" << i << "_vy"
            << ",a" << i << "_battery"
            << ",a" << i << "_dead"
            << ",a" << i << "_charging"
            << ",a" << i << "_detecting";
    }
    out << "\n";
}

// Write one step row
void write_step(std::ostream& out, TI episode, TI step, T reward, bool terminated,
                const ENVIRONMENT::State& state) {
    out << episode << "," << step << "," << reward << ","
        << (terminated ? 1 : 0) << ","
        << (state.disaster.active ? 1 : 0) << ","
        << state.disaster.position[0] << "," << state.disaster.position[1];
    for (TI i = 0; i < N_AGENTS; ++i) {
        const auto& d = state.drone_states[i];
        out << "," << d.position[0]
            << "," << d.position[1]
            << "," << d.velocity[0]
            << "," << d.velocity[1]
            << "," << d.battery
            << "," << (d.dead ? 1 : 0)
            << "," << (d.is_charging ? 1 : 0)
            << "," << (d.is_detecting ? 1 : 0);
    }
    out << "\n";
}

int main(int argc, char** argv) {
    DEVICE device;
    RNG rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);

    CLI::App app{"enjoy_oil_platform — evaluate a trained oil-platform SAC policy"};
    std::string checkpoint_path;
    std::string output_path = "trajectories.csv";
    TI n_episodes = 20;
    TI seed = 0;
    bool stochastic = false;

    app.add_option("-c,--checkpoint", checkpoint_path, "Path to HDF5 checkpoint (.h5)")->required();
    app.add_option("-o,--output",     output_path,     "Output CSV file (default: trajectories.csv)");
    app.add_option("-n,--episodes",   n_episodes,      "Number of episodes to run (default: 20)");
    app.add_option("-s,--seed",       seed,            "RNG seed (default: 0)");
    app.add_flag("--stochastic",      stochastic,      "Use stochastic (randomized) episode initialization");
    CLI11_PARSE(app, argc, argv);

    rlt::init(device, rng, seed);

    // Load actor
    ACTOR actor;
    typename ACTOR::template Buffer<1> actor_buffer;
    rlt::malloc(device, actor);
    rlt::malloc(device, actor_buffer);

    auto hdf5_file = HighFive::File(checkpoint_path, HighFive::File::ReadOnly);
    bool ok = rlt::load(device, actor, hdf5_file.getGroup("actor"));
    if (!ok) {
        std::cerr << "Failed to load actor from: " << checkpoint_path << "\n";
        return 1;
    }
    std::cerr << "Loaded actor from: " << checkpoint_path << "\n";

    // Observation / action matrices
    using OBS_SPEC = rlt::matrix::Specification<T, TI, 1, OBS_DIM>;
    using ACT_SPEC = rlt::matrix::Specification<T, TI, 1, ACTION_DIM>;
    rlt::Matrix<OBS_SPEC> observation;
    rlt::Matrix<ACT_SPEC> action;
    rlt::malloc(device, observation);
    rlt::malloc(device, action);

    typename ACTOR::template State<false> actor_state;
    rlt::malloc(device, actor_state);

    // Open output
    std::ofstream out_file;
    std::ostream* out = &std::cout;
    if (output_path != "-") {
        out_file.open(output_path);
        if (!out_file.is_open()) {
            std::cerr << "Cannot open output file: " << output_path << "\n";
            return 1;
        }
        out = &out_file;
        std::cerr << "Writing trajectories to: " << output_path << "\n";
    }
    write_header(*out);

    ENVIRONMENT env;
    ENVIRONMENT::Parameters parameters;
    ENVIRONMENT::State state, next_state;

    T total_return = 0;
    TI total_steps = 0;

    for (TI ep = 0; ep < n_episodes; ++ep) {
        // Initialize episode
        if (stochastic) {
            rlt::sample_initial_parameters(device, env, parameters, rng);
            rlt::sample_initial_state(device, env, parameters, state, rng);
        } else {
            rlt::initial_parameters(device, env, parameters);
            rlt::initial_state(device, env, parameters, state);
        }

        rlt::reset(device, actor, actor_state, rng);

        T ep_return = 0;
        TI ep_steps = 0;
        bool done = false;

        // Run episode up to max episode step limit
        const TI step_limit = ENVIRONMENT::PARAMETERS::EPISODE_STEP_LIMIT_MAX + 10;
        for (TI s = 0; s < step_limit && !done; ++s) {
            // Observe
            rlt::observe(device, env, parameters, state, typename ENVIRONMENT::Observation{}, observation, rng);

            // Convert to tensor for evaluate_step
            auto obs_tensor = rlt::to_tensor(device, observation);
            auto act_tensor = rlt::to_tensor(device, action);
            rlt::evaluate_step(device, actor, obs_tensor, actor_state, act_tensor,
                               actor_buffer, rng, rlt::Mode<rlt::mode::Evaluation<>>{});

            // Step
            rlt::step(device, env, parameters, state, action, next_state, rng);

            // Reward
            T r = rlt::reward(device, env, parameters, state, action, next_state, rng);

            // Terminated
            done = rlt::terminated(device, env, parameters, next_state, rng);

            write_step(*out, ep, s, r, done, state);

            ep_return += r;
            ep_steps++;
            state = next_state;
        }

        total_return += ep_return;
        total_steps += ep_steps;
        std::cerr << "Episode " << ep << ": return=" << ep_return << " steps=" << ep_steps << "\n";
    }

    std::cerr << "Mean return: " << (total_return / n_episodes)
              << "  Mean steps: " << (static_cast<T>(total_steps) / n_episodes) << "\n";

    rlt::free(device, actor);
    rlt::free(device, actor_buffer);
    rlt::free(device, observation);
    rlt::free(device, action);
    rlt::free(device, actor_state);

    return 0;
}
