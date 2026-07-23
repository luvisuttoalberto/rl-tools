#pragma once
// Shared zoo-style evaluation runner for the oil-platform environment.
//
// Runs N_EPISODES single-episode evaluations of an arbitrary policy (neural
// actor or scripted baseline) and emits the same artifacts as a zoo training
// run so different policies can be compared 1:1:
//   - TensorBoard logs (logs.tfevents/...): the environment logs its episode
//     metrics (coverage/*, charging/*, disaster/*, battery/*, agents/*)
//     inside terminated(); the logger step is set to the episode index, so
//     runs evaluated with this runner align on a common x-axis.
//   - return.json / return.json.set: aggregated returns in the zoo format.
//   - steps/000000000000000/trajectories.json.gz + ui.esm.js for the ExTrack
//     UI (./tools/serve.sh).
// Artifacts are placed in the standard extrack layout:
//   <base>/<timestamp>/<commit>_zoo_environment_algorithm/oil_platform-v1_<label>/<seed>/

#include <rl_tools/version.h>
#include <rl_tools/rl/utils/evaluation/operations_cpu.h>
#include <rl_tools/rl/loop/steps/save_trajectories/operations_cpu.h>
#include <rl_tools/utils/extrack/operations_cpu.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1::evaluation_runner {
    namespace rlt = rl_tools;

    template <typename DEVICE, typename TYPE_POLICY, typename ENVIRONMENT, typename POLICY, typename RNG>
    int run(DEVICE& device, ENVIRONMENT& env, POLICY& policy, RNG& rng,
            typename DEVICE::index_t seed,
            const std::string& algorithm_label,
            const std::string& extrack_base_path,
            const std::string& extrack_experiment){
        using TI = typename DEVICE::index_t;
        using T = typename TYPE_POLICY::DEFAULT;

        // Same evaluation setup as the zoo training loop (100 episodes, stochastic initial state)
        constexpr TI N_EPISODES = 400;
        // Trajectories are for the browser UI only; it decompresses the file into a single JS
        // string, which hard-fails ("Invalid string length") above ~0.5-1 GB decompressed.
        // Cap the dump at the proven-loadable size; metrics still cover all N_EPISODES.
        constexpr TI TRAJECTORY_EPISODES = N_EPISODES < 100 ? N_EPISODES : 100;
        constexpr TI STEP_LIMIT = ENVIRONMENT::EPISODE_STEP_LIMIT;
        using EVAL_SPEC = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, ENVIRONMENT, 1, STEP_LIMIT, false>;
        using DATA_SPEC = rlt::rl::utils::evaluation::DataSpecification<EVAL_SPEC>;
        using AGG_SPEC  = rlt::rl::utils::evaluation::Specification<TYPE_POLICY, TI, ENVIRONMENT, N_EPISODES, STEP_LIMIT, false>;

        rlt::utils::extrack::Config<TI> extrack_config;
        rlt::utils::extrack::Paths extrack_paths;
        extrack_config.name = "zoo";
        extrack_config.base_path = extrack_base_path;
        if(!extrack_experiment.empty()){
            extrack_config.experiment = extrack_experiment;
        }
        extrack_config.population_variates = "environment_algorithm";
        extrack_config.population_values = "oil_platform-v1_" + algorithm_label;
        rlt::init(device, extrack_config, extrack_paths, seed);
        std::filesystem::create_directories(extrack_paths.seed);

        {
            std::ofstream ui_file(extrack_paths.seed / "ui.esm.js");
            ui_file << rlt::get_ui(device, env);
        }

        rlt::rl::environments::DummyUI ui;
        rlt::rl::utils::evaluation::Result<EVAL_SPEC> result;
        rlt::rl::utils::evaluation::Data<DATA_SPEC> data;
        rlt::malloc(device, data);

        rlt::rl::utils::evaluation::Result<AGG_SPEC> aggregate;
        aggregate.returns_mean = 0;
        aggregate.returns_std = 0;
        aggregate.episode_length_mean = 0;
        aggregate.episode_length_std = 0;
        aggregate.num_terminated = 0;

        std::string episodes_json = "[";
        for(TI episode_i = 0; episode_i < N_EPISODES; episode_i++){
            rlt::set_step(device, device.logger, episode_i);
            rlt::evaluate(device, env, ui, policy, result, data, rng, rlt::Mode<rlt::mode::Evaluation<>>{});
            aggregate.returns[episode_i] = result.returns[0];
            aggregate.episode_length[episode_i] = result.episode_length[0];
            aggregate.num_terminated += result.num_terminated;
            aggregate.returns_mean += result.returns[0];
            aggregate.returns_std += result.returns[0] * result.returns[0];
            aggregate.episode_length_mean += result.episode_length[0];
            aggregate.episode_length_std += (T)result.episode_length[0] * (T)result.episode_length[0];

            // Per-episode metrics under eval/* tags (the environment's own episode logging is
            // left untouched for backwards compatibility with the training-time graphs).
            // Coverage: same definition as the environment's average_priority_area_coverage,
            // read from the accumulated metrics in the final state and logged unconditionally.
            {
                auto& final_state = get_ref(device, data.states, 0, STEP_LIMIT - 1);
                if(final_state.metrics.coverage_measurement_count > 0){
                    T average_coverage = final_state.metrics.total_coverage_ratio / final_state.metrics.coverage_measurement_count;
                    rlt::add_scalar(device, device.logger, "eval/average_priority_area_coverage", average_coverage);
                }
                rlt::add_scalar(device, device.logger, "eval/coverage_measurement_count", final_state.metrics.coverage_measurement_count);
                rlt::add_scalar(device, device.logger, "eval/return", result.returns[0]);
                rlt::add_scalar(device, device.logger, "eval/episode_length", result.episode_length[0]);
            }

            if(episode_i < TRAJECTORY_EPISODES){
                std::string episode_json = rlt::rl::loop::steps::save_trajectories::to_string(device, env, data);
                // to_string returns a one-element JSON array; splice its content into the combined array
                episodes_json += episode_json.substr(1, episode_json.size() - 2);
                if(episode_i < TRAJECTORY_EPISODES - 1){
                    episodes_json += ",\n";
                }
            }
            std::cerr << "Episode " << episode_i << ": return=" << result.returns[0] << " steps=" << result.episode_length[0] << std::endl;
        }
        episodes_json += "]";

        aggregate.returns_mean /= N_EPISODES;
        aggregate.returns_std = rlt::math::sqrt(device.math, rlt::math::max(device.math, (T)0, aggregate.returns_std / N_EPISODES - aggregate.returns_mean * aggregate.returns_mean));
        aggregate.episode_length_mean /= N_EPISODES;
        aggregate.episode_length_std = rlt::math::sqrt(device.math, rlt::math::max(device.math, (T)0, aggregate.episode_length_std / N_EPISODES - aggregate.episode_length_mean * aggregate.episode_length_mean));
        aggregate.share_terminated = aggregate.num_terminated / (T)N_EPISODES;

        {
            std::ofstream return_file(extrack_paths.seed / "return.json");
            return_file << "[" << rlt::json(device, aggregate, (TI)0) << "]";
        }
        {
            std::ofstream return_file_confirmation(extrack_paths.seed / "return.json.set");
        }
        {
            auto step_folder = rlt::get_step_folder(device, extrack_config, extrack_paths, 0);
            rlt::rl::loop::steps::save_trajectories::write_to_file(device, episodes_json, step_folder, "trajectories");
        }

        std::cerr << "Mean return: " << aggregate.returns_mean << " (std " << aggregate.returns_std << ")"
                  << "  Mean episode length: " << aggregate.episode_length_mean << std::endl;
        std::cerr << "Run directory: " << extrack_paths.seed << std::endl;

        rlt::free(device, data);
        return 0;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
