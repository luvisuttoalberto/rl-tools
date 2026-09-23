#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/optimizers/adam/instance/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/persist/backends/tar/operations_cpu.h>
#include <rl_tools/persist/backends/tar/operations_generic.h>
#include <rl_tools/nn_models/mean_embedding/operations_generic.h>
#include <rl_tools/nn_models/mean_embedding/persist.h>
#include <rl_tools/nn_models/mean_embedding/persist_code.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/multi_agent_wrapper/operations_generic.h>
#include <rl_tools/nn_models/random_uniform/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/operations_generic.h>
#include <rl_tools/rl/environments/operations_generic.h>
#include <rl_tools/nn_models/sequential/persist.h>
#include <rl_tools/nn_models/sequential/persist_code.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist.h>
#include <rl_tools/nn_models/multi_agent_wrapper/persist_code.h>
#include <rl_tools/nn/layers/sample_and_squash/persist.h>
#include <rl_tools/nn/layers/sample_and_squash/persist_code.h>
#include "../../../../src/rl/zoo/oil_platform-v1/sac.h"
#include <rl_tools/rl/algorithms/sac/loop/core/operations_generic.h>
#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>

namespace rlt = rl_tools;
namespace me = rlt::nn_models::mean_embedding;
using DEVICE = rlt::devices::DefaultCPU;
using TI = DEVICE::index_t;
using T = double;
using TP = rlt::numeric_types::Policy<T>;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;
constexpr auto TANH = rlt::nn::activation_functions::TANH;
constexpr auto IDENTITY = rlt::nn::activation_functions::IDENTITY;
using Encoder = rlt::nn_models::mlp::Configuration<TP, TI, 4, 3, 5, TANH, IDENTITY>;
using Embedding = me::Configuration<Encoder, 2, 3, 2, 1>;
using Head = rlt::nn_models::mlp::Configuration<TP, TI, 2, 2, 6, TANH, IDENTITY>;
using Chain = rlt::nn_models::sequential::Module<me::BindConfiguration<Embedding>,
    rlt::nn_models::sequential::Module<rlt::nn_models::mlp::BindConfiguration<Head>>>;
using Capability = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam>;
using Model = rlt::nn_models::sequential::Build<Capability, Chain, rlt::tensor::Shape<TI, 1, 2, 9>>;
template<TI COLS> using Tensor = rlt::Tensor<rlt::tensor::Specification<T, TI, rlt::tensor::Shape<TI, 1, 2, COLS>>>;

class MeanEmbedding: public ::testing::Test {
protected:
    DEVICE device;
    RNG rng;
    Model model;
    Model::Buffer<> buffer;
    Tensor<9> input, d_input;
    Tensor<2> result, d_output;
    void SetUp() override {
        rlt::init(device);
        rlt::malloc(device, rng);
        rlt::init(device, rng, 17);
        rlt::malloc(device, model);
        rlt::malloc(device, buffer);
        rlt::malloc(device, input);
        rlt::malloc(device, d_input);
        rlt::malloc(device, result);
        rlt::malloc(device, d_output);
        rlt::init_weights(device, model, rng);
        auto in = rlt::matrix_view(device, input);
        auto dout = rlt::matrix_view(device, d_output);
        for(TI r = 0; r < 2; ++r) {
            for(TI c = 0; c < 9; ++c) rlt::set(in, r, c, T(0.13) * T(1 + r * 9 + c) - 1);
            for(TI c = 0; c < 2; ++c) rlt::set(dout, r, c, T(0.3) + T(r) - T(c) * 0.7);
        }
    }
    void TearDown() override {
        rlt::free(device, d_output);
        rlt::free(device, result);
        rlt::free(device, d_input);
        rlt::free(device, input);
        rlt::free(device, buffer);
        rlt::free(device, model);
        rlt::free(device, rng);
    }
    T loss() {
        rlt::evaluate(device, model, input, result, buffer, rng);
        auto out = rlt::matrix_view(device, result);
        auto dout = rlt::matrix_view(device, d_output);
        T sum = 0;
        for(TI r = 0; r < 2; ++r) for(TI c = 0; c < 2; ++c) sum += rlt::get(out, r, c) * rlt::get(dout, r, c);
        return sum;
    }
    template<typename PARAMETER>
    void check_weights(PARAMETER& parameter) {
        auto weights = rlt::matrix_view(device, parameter.parameters);
        auto gradients = rlt::matrix_view(device, parameter.gradient);
        for(TI r = 0; r < decltype(weights)::ROWS; ++r) for(TI c = 0; c < decltype(weights)::COLS; ++c) {
            T original = rlt::get(weights, r, c);
            rlt::set(weights, r, c, original + 1e-6);
            T plus = loss();
            rlt::set(weights, r, c, original - 1e-6);
            T minus = loss();
            rlt::set(weights, r, c, original);
            EXPECT_NEAR(rlt::get(gradients, r, c), (plus - minus) / 2e-6, 2e-8) << r << ", " << c;
        }
    }
};

TEST_F(MeanEmbedding, FiniteDifferenceParametersAndInputs) {
    rlt::zero_gradient(device, model);
    rlt::forward(device, model, input, buffer, rng);
    rlt::backward_full(device, model, input, d_output, d_input, buffer);
    check_weights(model.content.encoder.input_layer.weights);
    check_weights(model.content.encoder.hidden_layers[0].weights);
    check_weights(model.content.encoder.output_layer.weights);
    check_weights(model.next_module.content.input_layer.weights);
    check_weights(model.next_module.content.output_layer.weights);
    auto in = rlt::matrix_view(device, input);
    auto din = rlt::matrix_view(device, d_input);
    for(TI r = 0; r < 2; ++r) for(TI c = 0; c < 9; ++c) {
        T original = rlt::get(in, r, c);
        rlt::set(in, r, c, original + 1e-6);
        T plus = loss();
        rlt::set(in, r, c, original - 1e-6);
        T minus = loss();
        rlt::set(in, r, c, original);
        EXPECT_NEAR(rlt::get(din, r, c), (plus - minus) / 2e-6, 2e-8);
    }
}

TEST_F(MeanEmbedding, PermutationAndIdentityInvariance) {
    T reference = loss();
    auto in = rlt::matrix_view(device, input);
    for(TI r = 0; r < 2; ++r) {
        for(TI c = 0; c < 3; ++c) {
            T old = rlt::get(in, r, 2 + c);
            rlt::set(in, r, 2 + c, rlt::get(in, r, 5 + c));
            rlt::set(in, r, 5 + c, old);
        }
        rlt::set(in, r, 8, 1234);
    }
    EXPECT_NEAR(loss(), reference, 1e-14);
}

TEST_F(MeanEmbedding, CheckpointAndInferenceCapability) {
    T reference = loss();
    rlt::persist::backends::tar::Writer writer;
    rlt::persist::backends::tar::WriterGroup<rlt::persist::backends::tar::WriterGroupSpecification<TI, decltype(writer)>> group{"", &writer};
    rlt::save(device, model, group);
    rlt::persist::backends::tar::finalize(device, writer);
    rlt::init_weights(device, model, rng);
    EXPECT_GT(std::abs(loss() - reference), 1e-6);
    rlt::persist::backends::tar::ReaderGroup<rlt::persist::backends::tar::ReaderGroupSpecification<TI>> reader;
    reader.data.data = writer.buffer.data();
    reader.data.size = writer.buffer.size();
    ASSERT_TRUE(rlt::load(device, model, reader));
    EXPECT_NEAR(loss(), reference, 1e-14);
    using Inference = Model::CHANGE_CAPABILITY<rlt::nn::capability::Forward<>>;
    Inference inference;
    rlt::malloc(device, inference);
    rlt::copy(device, device, model, inference);
    rlt::evaluate(device, inference, input, result, buffer, rng);
    auto out = rlt::matrix_view(device, result);
    auto dout = rlt::matrix_view(device, d_output);
    T inference_loss = 0;
    for(TI r = 0; r < 2; ++r) for(TI c = 0; c < 2; ++c) inference_loss += rlt::get(out, r, c) * rlt::get(dout, r, c);
    EXPECT_NEAR(inference_loss, reference, 1e-14);
    // Optionally emit a header for a separate compilation of the generated model.
    if(const char* path = std::getenv("RL_TOOLS_MEAN_EMBEDDING_EXPORT")) {
        std::ofstream file(path);
        file << rlt::save_code(device, inference, "mean_embedding_test");
        file << "\nnamespace mean_embedding_expected { constexpr double output[] = {" << std::setprecision(std::numeric_limits<T>::max_digits10);
        for(TI r = 0; r < 2; ++r) for(TI c = 0; c < 2; ++c) file << rlt::get(out, r, c) << ",";
        file << "}; }\n";
        ASSERT_TRUE(file.good());
    }
    rlt::free(device, inference);
}

using Factory = rlt::rl::zoo::oil_platform_v1::sac::FACTORY<DEVICE, TP, TI, RNG>;
struct SmokeParameters: Factory::LOOP_CORE_PARAMETERS {
    struct SAC_PARAMETERS: Factory::LOOP_CORE_PARAMETERS::SAC_PARAMETERS {
        static constexpr TI ACTOR_BATCH_SIZE = 4, CRITIC_BATCH_SIZE = 4;
        static constexpr TI TRAINING_INTERVAL = 1, ACTOR_TRAINING_INTERVAL = 1, CRITIC_TRAINING_INTERVAL = 1, CRITIC_TARGET_UPDATE_INTERVAL = 1;
    };
    static constexpr TI ACTOR_HIDDEN_DIM = 8, CRITIC_HIDDEN_DIM = 8;
    static constexpr TI TEAMMATE_ENCODER_HIDDEN_DIM = 5, TEAMMATE_EMBEDDING_DIM = 4;
    static constexpr TI STEP_LIMIT = 64, REPLAY_BUFFER_CAP = 128, N_ENVIRONMENTS = 2;
    static constexpr TI EPISODE_STEP_LIMIT = 4;
    static constexpr TI N_WARMUP_STEPS = 4, N_WARMUP_STEPS_CRITIC = 4, N_WARMUP_STEPS_ACTOR = 4;
};
struct FiveAgentParameters: Factory::ENVIRONMENT::PARAMETERS {
    static constexpr TI MAX_AGENTS = 5, N_AGENTS = MAX_AGENTS;
};
using FiveAgentSpec = rlt::rl::environments::multi_agent::oil_platform::Specification<T, TI, FiveAgentParameters,
    rlt::rl::environments::multi_agent::oil_platform::Observation<FiveAgentParameters>,
    rlt::rl::environments::multi_agent::oil_platform::ObservationPrivileged<FiveAgentParameters>>;
using FiveAgentEnv = rlt::rl::environments::multi_agent::OilPlatform<FiveAgentSpec>;
using FiveAgentActor = rlt::rl::zoo::oil_platform_v1::multi_agent_sac::ConfigApproximatorsMLPMultiAgent<TP, TI, FiveAgentEnv, SmokeParameters, true>::ACTOR_TYPE;

using SmokeConfig = rlt::rl::algorithms::sac::loop::core::Config<TP, TI, RNG, Factory::ENVIRONMENT, SmokeParameters,
    rlt::rl::zoo::oil_platform_v1::multi_agent_sac::ConfigApproximatorsMLPMultiAgent>;

TEST(MeanEmbeddingSAC, TrainingAndActorCheckpoint) {
    DEVICE device;
    rlt::init(device);
    rlt::rl::algorithms::sac::loop::core::State<SmokeConfig> state;
    rlt::malloc(device, state);
    rlt::init(device, state, 5);
    auto& actor = state.actor_critic.actor;
    auto& weights = actor.wrapper.content.content.encoder.input_layer.weights.parameters;
    auto& head_weights = actor.wrapper.content.next_module.content.input_layer.weights.parameters;
    T initial = rlt::get(device, weights, 0, 0);
    auto head_matrix = rlt::matrix_view(device, head_weights);
    T initial_head[decltype(head_matrix)::ROWS * decltype(head_matrix)::COLS];
    for(TI r = 0; r < decltype(head_matrix)::ROWS; ++r) for(TI c = 0; c < decltype(head_matrix)::COLS; ++c)
        initial_head[r * decltype(head_matrix)::COLS + c] = rlt::get(head_matrix, r, c);
    for(TI step = 0; step < SmokeParameters::STEP_LIMIT; ++step) rlt::step(device, state);
    EXPECT_FALSE(is_nan(device, actor));
    EXPECT_NE(rlt::get(device, weights, 0, 0), initial);
    T head_change = 0;
    for(TI r = 0; r < decltype(head_matrix)::ROWS; ++r) for(TI c = 0; c < decltype(head_matrix)::COLS; ++c)
        head_change += std::abs(rlt::get(head_matrix, r, c) - initial_head[r * decltype(head_matrix)::COLS + c]);
    EXPECT_GT(head_change, 1e-8);
    bool sizes[Factory::ENVIRONMENT::N_AGENTS + 1] = {};
    for(TI e = 0; e < SmokeParameters::N_ENVIRONMENTS; ++e) {
        auto& replay = rlt::get(state.off_policy_runner.replay_buffers, 0, e);
        for(TI i = 0; i < SmokeParameters::STEP_LIMIT; ++i) {
            TI present = 0;
            using Obs = Factory::ENVIRONMENT::Observation;
            for(TI a = 0; a < Factory::ENVIRONMENT::N_AGENTS; ++a)
                present += rlt::get(replay.observations, i, a * Obs::PER_AGENT_DIM + Obs::BASE_PER_AGENT_DIM + Obs::RELATIVE_EXTRA_DIM) > 0;
            ASSERT_GE(present, 2);
            ASSERT_LE(present, Factory::ENVIRONMENT::N_AGENTS);
            sizes[present] = true;
            for(TI a = present; a < Factory::ENVIRONMENT::N_AGENTS; ++a)
                for(TI k = 0; k < 2; ++k) EXPECT_EQ(rlt::get(replay.actions, i, a * 2 + k), 0);
        }
    }
    TI different_sizes = 0;
    for(TI n = 2; n <= Factory::ENVIRONMENT::N_AGENTS; ++n) different_sizes += sizes[n];
    EXPECT_GE(different_sizes, 3);
    EXPECT_EQ(rlt::get(device, state.actor_critic.actor_optimizer.age, 0), 1 + SmokeParameters::STEP_LIMIT - SmokeParameters::N_WARMUP_STEPS_ACTOR);
    T trained = rlt::get(device, weights, 0, 0);
    rlt::persist::backends::tar::Writer writer;
    rlt::persist::backends::tar::WriterGroup<rlt::persist::backends::tar::WriterGroupSpecification<TI, decltype(writer)>> group{"", &writer};
    rlt::save(device, actor, group);
    rlt::persist::backends::tar::finalize(device, writer);
    rlt::set(device, weights, 999, 0, 0);
    rlt::persist::backends::tar::ReaderGroup<rlt::persist::backends::tar::ReaderGroupSpecification<TI>> reader;
    reader.data.data = writer.buffer.data();
    reader.data.size = writer.buffer.size();
    ASSERT_TRUE(rlt::load(device, actor, reader));
    EXPECT_EQ(rlt::get(device, weights, 0, 0), trained);

    // Load the training checkpoint into the batch-one, forward-only actor used
    // at deployment, then check the complete squashed policy's invariance.
    using Forward = SmokeConfig::NN::ACTOR_TYPE::CHANGE_CAPABILITY<rlt::nn::capability::Forward<>>;
    using Inference = Forward::CHANGE_BATCH_SIZE<TI, 1>;
    Inference inference;
    Inference::Buffer<> inference_buffer;
    Inference::State<> inference_state;
    malloc(device, inference);
    malloc(device, inference_buffer);
    reader.path[0] = '\0';
    ASSERT_TRUE(rlt::load(device, inference, reader));
    // Reject the old count-input schema before reading incompatible MLP tensors.
    rlt::persist::backends::tar::Writer legacy_writer;
    decltype(group) legacy_group{"", &legacy_writer};
    auto legacy_policy = rlt::create_group(device, legacy_group, "swarm2x6");
    rlt::save(device, actor.wrapper.content, legacy_policy);
    rlt::persist::backends::tar::finalize(device, legacy_writer);
    decltype(reader) legacy_reader;
    legacy_reader.data.data = legacy_writer.buffer.data();
    legacy_reader.data.size = legacy_writer.buffer.size();
    EXPECT_FALSE(rlt::load(device, inference, legacy_reader));
    FiveAgentActor wrong_capacity;
    reader.path[0] = '\0';
    EXPECT_FALSE(rlt::load(device, wrong_capacity, reader));
    using Obs = Factory::ENVIRONMENT::Observation;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, Obs::DIM>> observations;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, Factory::ENVIRONMENT::ACTION_DIM>> actions;
    rlt::malloc(device, observations);
    rlt::malloc(device, actions);
    Factory::ENVIRONMENT env;
    env.fixed_n_agents = Factory::ENVIRONMENT::N_AGENTS;
    Factory::ENVIRONMENT::Parameters parameters;
    Factory::ENVIRONMENT::State env_state;
    rlt::initial_state(device, env, parameters, env_state);
    rlt::observe(device, env, parameters, env_state, Obs{}, observations, state.rng);
    auto obs_tensor = rlt::to_tensor(device, observations);
    auto action_tensor = rlt::to_tensor(device, actions);
    evaluate_step(device, inference, obs_tensor, inference_state, action_tensor, inference_buffer, state.rng, rlt::Mode<rlt::mode::Evaluation<>>{});
    T reference[Factory::ENVIRONMENT::ACTION_DIM];
    for(TI k = 0; k < Factory::ENVIRONMENT::ACTION_DIM; ++k) reference[k] = rlt::get(actions, 0, k);
    for(TI i = 0; i < Factory::ENVIRONMENT::N_AGENTS; ++i) {
        TI neighbors = i * Obs::PER_AGENT_DIM + Obs::PREFIX_DIM;
        for(TI k = 0; k < Obs::PER_OTHER_AGENT_DIM; ++k) {
            T old = rlt::get(observations, 0, neighbors + k);
            rlt::set(observations, 0, neighbors + k, rlt::get(observations, 0, neighbors + Obs::PER_OTHER_AGENT_DIM + k));
            rlt::set(observations, 0, neighbors + Obs::PER_OTHER_AGENT_DIM + k, old);
        }
        for(TI k = 0; k < Obs::AGENT_ID_DIM; ++k) rlt::set(observations, 0, neighbors + Obs::OTHER_AGENTS_DIM + k, 42);
    }
    evaluate_step(device, inference, obs_tensor, inference_state, action_tensor, inference_buffer, state.rng, rlt::Mode<rlt::mode::Evaluation<>>{});
    for(TI k = 0; k < Factory::ENVIRONMENT::ACTION_DIM; ++k) EXPECT_NEAR(rlt::get(actions, 0, k), reference[k], 1e-14);
    if(const char* path = std::getenv("RL_TOOLS_MEAN_EMBEDDING_ACTOR_EXPORT")) {
        std::ofstream file(path);
        file << rlt::save_code(device, inference, "actor_test");
        file << "\nnamespace actor_expected { constexpr double input[] = {" << std::setprecision(std::numeric_limits<T>::max_digits10);
        for(TI k = 0; k < Obs::DIM; ++k) file << rlt::get(observations, 0, k) << ",";
        file << "}; constexpr double output[] = {";
        for(TI k = 0; k < Factory::ENVIRONMENT::ACTION_DIM; ++k) file << reference[k] << ",";
        file << "}; }\n";
        // Also export an observation with padding and a present dead drone.
        env.fixed_n_agents = 2;
        rlt::initial_state(device, env, parameters, env_state);
        env_state.drone_states[0].dead = true;
        rlt::observe(device, env, parameters, env_state, Obs{}, observations, state.rng);
        evaluate_step(device, inference, obs_tensor, inference_state, action_tensor, inference_buffer, state.rng, rlt::Mode<rlt::mode::Evaluation<>>{});
        file << "namespace actor_expected { constexpr double masked_input[] = {";
        for(TI k = 0; k < Obs::DIM; ++k) file << rlt::get(observations, 0, k) << ",";
        file << "}; constexpr double masked_output[] = {";
        for(TI k = 0; k < Factory::ENVIRONMENT::ACTION_DIM; ++k) file << rlt::get(actions, 0, k) << ",";
        file << "}; }\n";
        ASSERT_TRUE(file.good());
    }
    rlt::free(device, actions);
    rlt::free(device, observations);
    free(device, inference_buffer);
    free(device, inference);
    rlt::free(device, state);
}

TEST(VariableSwarm, MaskedMeanGradientsAndEmptySet) {
    using Config = me::Configuration<Encoder, 2, 3, 3, 0, true>;
    using Network = me::Module<Config, Capability, rlt::tensor::Shape<TI, 1, 2, 14>>;
    DEVICE device;
    RNG rng;
    rlt::init(device, rng, 23);
    Network network;
    Network::Buffer<> buffer;
    Tensor<14> input, din;
    Tensor<6> output, dout;
    malloc(device, network); malloc(device, buffer);
    rlt::malloc(device, input); rlt::malloc(device, din);
    rlt::malloc(device, output); rlt::malloc(device, dout);
    init_weights(device, network, rng);
    auto in = rlt::matrix_view(device, input);
    auto out = rlt::matrix_view(device, output);
    auto grad = rlt::matrix_view(device, din);
    rlt::set_all(device, input, T(0.2));
    rlt::set_all(device, dout, T(1));
    // Row 0 has two present elements; row 1 is empty. Nonzero encoder biases
    // ensure masking after encoding is necessary (zero padding alone is wrong).
    rlt::set_all(device, network.encoder.output_layer.biases.parameters, T(0.7));
    for(TI row = 0; row < 2; ++row) for(TI j = 0; j < 3; ++j)
        rlt::set(in, row, 2 + j * 4 + 3, row == 0 && j != 1 ? 1 : 0);
    zero_gradient(device, network);
    forward(device, network, input, buffer, rng);
    backward_full(device, network, input, dout, din, buffer);
    auto loss = [&]() {
        evaluate(device, network, input, output, buffer, rng);
        return rlt::sum(device, output);
    };
    T reference = loss();
    for(TI k = 2; k < 6; ++k) EXPECT_EQ(rlt::get(out, 1, k), 0);
    // Padding contents must not affect either the mean or its gradients.
    for(TI k = 0; k < 3; ++k) rlt::set(in, 0, 6 + k, 1234);
    EXPECT_DOUBLE_EQ(loss(), reference);
    for(TI row = 0; row < 2; ++row) for(TI k = 0; k < 14; ++k) {
        if(k >= 2 && (k - 2) % 4 == 3) { EXPECT_EQ(rlt::get(grad, row, k), 0); continue; }
        T old = rlt::get(in, row, k);
        rlt::set(in, row, k, old + 1e-6); T plus = loss();
        rlt::set(in, row, k, old - 1e-6); T minus = loss();
        rlt::set(in, row, k, old);
        EXPECT_NEAR(rlt::get(grad, row, k), (plus - minus) / 2e-6, 2e-8);
    }
    auto weights = rlt::matrix_view(device, network.encoder.input_layer.weights.parameters);
    auto dw = rlt::matrix_view(device, network.encoder.input_layer.weights.gradient);
    for(TI r = 0; r < decltype(weights)::ROWS; ++r) for(TI c = 0; c < decltype(weights)::COLS; ++c) {
        T old = rlt::get(weights, r, c);
        rlt::set(weights, r, c, old + 1e-6); T plus = loss();
        rlt::set(weights, r, c, old - 1e-6); T minus = loss();
        rlt::set(weights, r, c, old);
        EXPECT_NEAR(rlt::get(dw, r, c), (plus - minus) / 2e-6, 2e-8);
    }
    free(device, buffer); free(device, network);
    rlt::free(device, input); rlt::free(device, din); rlt::free(device, output); rlt::free(device, dout);
}

struct MaskedSASParameters: rlt::nn::layers::sample_and_squash::DefaultParameters<TP> {
    static constexpr T TARGET_ENTROPY = -6;
};
TEST(VariableSwarm, MaskedSamplingEntropyAndGradients) {
    using Config = rlt::nn::layers::sample_and_squash::Configuration<TP, TI, MaskedSASParameters, true>;
    using SAS = rlt::nn::layers::sample_and_squash::Layer<Config, Capability, rlt::tensor::Shape<TI, 1, 4, 12>>;
    DEVICE device;
    RNG rng;
    rlt::init(device, rng, 13);
    SAS sas;
    SAS::Buffer<> buffer;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 4, 12>> input, din;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 4, 6>> output, dout;
    rlt::malloc(device, sas); rlt::malloc(device, buffer);
    rlt::malloc(device, input); rlt::malloc(device, din);
    rlt::malloc(device, output); rlt::malloc(device, dout);
    rlt::init_weights(device, sas, rng); rlt::zero_gradient(device, sas);
    rlt::set_all(device, buffer.noise, T(0.3));
    rlt::set_all(device, dout, T(0.4));
    const TI active[4] = {2, 4, 6, 0};
    for(TI row = 0; row < 4; ++row) for(TI k = 0; k < 6; ++k) {
        rlt::set(input, row, k, 0.1); rlt::set(input, row, 6 + k, -0.2);
        rlt::set(buffer.action_mask, row, k, k < active[row] ? 1 : 0);
    }
    using Mode = rlt::Mode<rlt::nn::layers::sample_and_squash::mode::ExternalNoise<rlt::mode::Default<>>>;
    rlt::forward(device, sas, input, buffer, rng, Mode{});
    rlt::backward_full(device, sas, input, dout, din, buffer, Mode{});
    for(TI row = 0; row < 4; ++row) {
        EXPECT_NEAR(rlt::get(buffer.d_log_alpha, 0, row), -rlt::get(sas.log_probabilities, 0, row) + T(active[row]), 1e-12);
        for(TI k = active[row]; k < 6; ++k) {
            EXPECT_EQ(rlt::get(sas.output, row, k), 0);
            EXPECT_EQ(rlt::get(din, row, k), 0);
            EXPECT_EQ(rlt::get(din, row, k + 6), 0);
        }
    }
    auto loss = [&]() {
        rlt::evaluate(device, sas, input, output, buffer, rng, Mode{});
        return rlt::sum(device, buffer.log_probabilities) / T(4) + T(0.4) * rlt::sum(device, output);
    };
    for(TI row = 0; row < 4; ++row) for(TI k = 0; k < 12; ++k) {
        T old = rlt::get(input, row, k);
        rlt::set(input, row, k, old + 1e-6); T plus = loss();
        rlt::set(input, row, k, old - 1e-6); T minus = loss();
        rlt::set(input, row, k, old);
        // The existing SAS derivative approximates the epsilon in the tanh Jacobian.
        EXPECT_NEAR(rlt::get(din, row, k), (plus - minus) / 2e-6, 2e-6);
    }
    rlt::free(device, sas); rlt::free(device, buffer);
    rlt::free(device, input); rlt::free(device, din); rlt::free(device, output); rlt::free(device, dout);
}

TEST(VariableSwarm, ReplicatedTeammatesDoNotExposeGlobalCount) {
    using Env = Factory::ENVIRONMENT;
    using Obs = Env::Observation;
    using Forward = SmokeConfig::NN::ACTOR_TYPE::CHANGE_CAPABILITY<rlt::nn::capability::Forward<>>;
    using Actor = Forward::CHANGE_BATCH_SIZE<TI, 1>;
    DEVICE device;
    RNG rng;
    rlt::init(device, rng, 37);
    Env env;
    env.fixed_n_agents = 2;
    Env::Parameters parameters;
    Env::State state;
    rlt::initial_state(device, env, parameters, state);
    // Repeat the same neighbor features without changing agent 0 or the task.
    // This is an observation test; coincident neighbors are never stepped.
    for(TI a = 2; a < Env::N_AGENTS; ++a) state.drone_states[a] = state.drone_states[1];
    Actor actor;
    Actor::Buffer<> buffer;
    Actor::State<> actor_state;
    malloc(device, actor); malloc(device, buffer); init_weights(device, actor, rng);
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, Obs::DIM>> obs;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, Env::ACTION_DIM>> actions;
    rlt::malloc(device, obs); rlt::malloc(device, actions);
    T reference_prefix[Obs::PREFIX_DIM];
    T reference_actions[2];
    for(TI n = 2; n <= Env::N_AGENTS; ++n) {
        state.n_agents = n;
        rlt::set_all(device, obs, std::numeric_limits<T>::quiet_NaN());
        rlt::observe(device, env, parameters, state, Obs{}, obs, rng);
        for(TI k = 0; k < Obs::DIM; ++k) EXPECT_TRUE(std::isfinite(rlt::get(obs, 0, k)));
        for(TI k = 0; k < Obs::PREFIX_DIM; ++k) {
            if(n == 2) reference_prefix[k] = rlt::get(obs, 0, k);
            else EXPECT_EQ(rlt::get(obs, 0, k), reference_prefix[k]);
        }
        auto ot = rlt::to_tensor(device, obs); auto at = rlt::to_tensor(device, actions);
        evaluate_step(device, actor, ot, actor_state, at, buffer, rng, rlt::Mode<rlt::mode::Evaluation<>>{});
        for(TI k = 0; k < 2; ++k) {
            if(n == 2) reference_actions[k] = rlt::get(actions, 0, k);
            else EXPECT_NEAR(rlt::get(actions, 0, k), reference_actions[k], 1e-14);
        }
    }
    free(device, actor); free(device, buffer);
    rlt::free(device, obs); rlt::free(device, actions);
}

TEST(VariableSwarm, ResetsPaddingDynamicsRewardsAndInference) {
    using Env = Factory::ENVIRONMENT;
    using Obs = Env::Observation;
    using Priv = Env::ObservationPrivileged;
    using Forward = SmokeConfig::NN::ACTOR_TYPE::CHANGE_CAPABILITY<rlt::nn::capability::Forward<>>;
    using Actor = Forward::CHANGE_BATCH_SIZE<TI, 1>;
    DEVICE device;
    RNG rng;
    rlt::init(device, rng, 29);
    Env env;
    Env::Parameters parameters;
    Env::State state, next;
    Actor actor;
    Actor::Buffer<> buffer;
    Actor::State<> actor_state;
    malloc(device, actor); malloc(device, buffer); init_weights(device, actor, rng);
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, Obs::DIM>> obs;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, Priv::DIM>> priv;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, Env::ACTION_DIM>> action;
    rlt::malloc(device, obs); rlt::malloc(device, priv); rlt::malloc(device, action);
    bool seen[Env::N_AGENTS + 1] = {};
    for(TI i = 0; i < 200; ++i) {
        rlt::sample_initial_state(device, env, parameters, state, rng);
        ASSERT_GE(state.n_agents, 2); ASSERT_LE(state.n_agents, Env::N_AGENTS);
        seen[state.n_agents] = true;
    }
    for(TI n = 2; n <= Env::N_AGENTS; ++n) {
        EXPECT_TRUE(seen[n]);
        env.fixed_n_agents = n;
        rlt::sample_initial_state(device, env, parameters, state, rng);
        ASSERT_EQ(state.n_agents, n);
        rlt::observe(device, env, parameters, state, Obs{}, obs, rng);
        rlt::observe(device, env, parameters, state, Priv{}, priv, rng);
        for(TI a = 0; a < Env::N_AGENTS; ++a) {
            EXPECT_EQ(rlt::get(priv, 0, a * Priv::PER_AGENT_DIM + 8), a < n ? 1 : 0);
            EXPECT_EQ(rlt::get(obs, 0, a * Obs::PER_AGENT_DIM + 13), a < n ? 1 : 0);
            if(a >= n) {
                for(TI k = 0; k < Obs::PER_AGENT_DIM; ++k) EXPECT_EQ(rlt::get(obs, 0, a * Obs::PER_AGENT_DIM + k), 0);
                for(TI k = 0; k < Priv::PER_AGENT_DIM; ++k) EXPECT_EQ(rlt::get(priv, 0, a * Priv::PER_AGENT_DIM + k), 0);
            }
        }
        auto ot = rlt::to_tensor(device, obs); auto at = rlt::to_tensor(device, action);
        evaluate_step(device, actor, ot, actor_state, at, buffer, rng, rlt::Mode<rlt::mode::Evaluation<>>{});
        for(TI a = n; a < Env::N_AGENTS; ++a) for(TI k = 0; k < 2; ++k) EXPECT_EQ(rlt::get(action, 0, a * 2 + k), 0);
        // A present dead drone stays in the set, but has no action or entropy.
        state.drone_states[0].dead = true;
        rlt::observe(device, env, parameters, state, Obs{}, obs, rng);
        evaluate_step(device, actor, ot, actor_state, at, buffer, rng, rlt::Mode<rlt::mode::Evaluation<>>{});
        EXPECT_EQ(rlt::get(action, 0, 0), 0); EXPECT_EQ(rlt::get(action, 0, 1), 0);
        // Dead agent 0 is the first teammate in agent 1's observation.
        EXPECT_EQ(rlt::get(obs, 0, Obs::PER_AGENT_DIM + Obs::PREFIX_DIM + 8), 1);
        EXPECT_EQ(rlt::get(obs, 0, Obs::PER_AGENT_DIM + Obs::PREFIX_DIM + 5), 1);
        rlt::step(device, env, parameters, state, action, next, rng);
        T reward = rlt::reward(device, env, parameters, state, action, next, rng);
        EXPECT_TRUE(std::isfinite(reward)); EXPECT_EQ(next.n_agents, n);
        EXPECT_NEAR(next.metrics.ongoing_death_penalty, -T(3) / T(n), 1e-12);
        EXPECT_EQ(next.metrics.death_penalty, 0);
        for(TI a = n; a < Env::N_AGENTS; ++a) EXPECT_EQ(next.drone_states[a].battery, 0);
        // Arbitrary padding state and actions cannot affect the physical task or reward.
        auto dirty = state;
        for(TI a = n; a < Env::N_AGENTS; ++a) {
            dirty.drone_states[a] = state.drone_states[1];
            dirty.drone_states[a].is_charging = true;
            rlt::set(action, 0, a * 2, 99); rlt::set(action, 0, a * 2 + 1, -99);
        }
        auto dirty_next = next;
        for(TI a = n; a < Env::N_AGENTS; ++a) dirty_next.drone_states[a] = dirty.drone_states[a];
        EXPECT_DOUBLE_EQ(rlt::reward(device, env, parameters, dirty, action, dirty_next, rng), reward);
        for(TI a = 0; a < n; ++a) dirty_next.drone_states[a].dead = true;
        EXPECT_TRUE(rlt::terminated(device, env, parameters, dirty_next, rng));
        auto serialized = rlt::json(device, env, parameters, state);
        EXPECT_NE(serialized.find("\"n_agents\": " + std::to_string(n)), std::string::npos);
    }
    free(device, actor); free(device, buffer);
    rlt::free(device, obs); rlt::free(device, priv); rlt::free(device, action);
}

TEST(VariableSwarm, ThreeAgentRewardAndDynamicsMatchFixedEnvironment) {
    using VariableEnv = Factory::ENVIRONMENT;
    using FixedEnv = rlt::rl::zoo::oil_platform_v1::ENVIRONMENT_FACTORY<DEVICE, TP, TI, true>::ENVIRONMENT;
    DEVICE device;
    RNG rng_fixed, rng_variable;
    rlt::init(device, rng_fixed, 123);
    rlt::init(device, rng_variable, 123);
    FixedEnv fixed;
    VariableEnv variable;
    variable.fixed_n_agents = 3;
    FixedEnv::Parameters fp;
    VariableEnv::Parameters vp;
    FixedEnv::State fs, fn;
    VariableEnv::State vs, vn;
    rlt::initial_state(device, fixed, fp, fs);
    rlt::initial_state(device, variable, vp, vs);
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, FixedEnv::ACTION_DIM, false>> fa;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, VariableEnv::ACTION_DIM, false>> va;
    // Cover ordinary movement, charging, and death penalties at reference size.
    fs.drone_states[0].battery = vs.drone_states[0].battery = 0.01;
    fs.drone_states[1].position[0] = vs.drone_states[1].position[0] = 50;
    fs.drone_states[1].position[1] = vs.drone_states[1].position[1] = 50;
    fs.drone_states[1].battery = vs.drone_states[1].battery = 20;
    for(TI step = 0; step < 32; ++step) {
        rlt::set_all(device, va, T(0));
        for(TI k = 0; k < FixedEnv::ACTION_DIM; ++k) {
            const T action = k == 2 || k == 3 ? T(0) : T(0.1) * T(int((step + k) % 5) - 2);
            rlt::set(fa, 0, k, action); rlt::set(va, 0, k, action);
        }
        rlt::step(device, fixed, fp, fs, fa, fn, rng_fixed);
        rlt::step(device, variable, vp, vs, va, vn, rng_variable);
        const T fr = rlt::reward(device, fixed, fp, fs, fa, fn, rng_fixed);
        const T vr = rlt::reward(device, variable, vp, vs, va, vn, rng_variable);
        EXPECT_DOUBLE_EQ(fr, vr);
        for(TI a = 0; a < 3; ++a) {
            EXPECT_DOUBLE_EQ(fn.drone_states[a].position[0], vn.drone_states[a].position[0]);
            EXPECT_DOUBLE_EQ(fn.drone_states[a].position[1], vn.drone_states[a].position[1]);
            EXPECT_DOUBLE_EQ(fn.drone_states[a].battery, vn.drone_states[a].battery);
            EXPECT_EQ(fn.drone_states[a].dead, vn.drone_states[a].dead);
        }
        fs = fn; vs = vn;
    }
}
