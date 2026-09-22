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
    static constexpr TI STEP_LIMIT = 16, REPLAY_BUFFER_CAP = 32, N_ENVIRONMENTS = 1;
    static constexpr TI N_WARMUP_STEPS = 4, N_WARMUP_STEPS_CRITIC = 4, N_WARMUP_STEPS_ACTOR = 4;
};
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
    T initial_head = rlt::get(device, head_weights, 0, 0);
    for(TI step = 0; step < SmokeParameters::STEP_LIMIT; ++step) rlt::step(device, state);
    EXPECT_FALSE(is_nan(device, actor));
    EXPECT_NE(rlt::get(device, weights, 0, 0), initial);
    EXPECT_NE(rlt::get(device, head_weights, 0, 0), initial_head);
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
    using Obs = Factory::ENVIRONMENT::Observation;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, Obs::DIM>> observations;
    rlt::Matrix<rlt::matrix::Specification<T, TI, 1, Factory::ENVIRONMENT::ACTION_DIM>> actions;
    rlt::malloc(device, observations);
    rlt::malloc(device, actions);
    for(TI k = 0; k < Obs::DIM; ++k) rlt::set(observations, 0, k, T(k % 11) / 11 - 0.5);
    auto obs_tensor = rlt::to_tensor(device, observations);
    auto action_tensor = rlt::to_tensor(device, actions);
    evaluate_step(device, inference, obs_tensor, inference_state, action_tensor, inference_buffer, state.rng, rlt::Mode<rlt::mode::Evaluation<>>{});
    T reference[Factory::ENVIRONMENT::ACTION_DIM];
    for(TI k = 0; k < Factory::ENVIRONMENT::ACTION_DIM; ++k) reference[k] = rlt::get(actions, 0, k);
    for(TI i = 0; i < Factory::ENVIRONMENT::N_AGENTS; ++i) {
        TI neighbors = i * Obs::PER_AGENT_DIM + Obs::BASE_PER_AGENT_DIM + Obs::RELATIVE_EXTRA_DIM;
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
        ASSERT_TRUE(file.good());
    }
    rlt::free(device, actions);
    rlt::free(device, observations);
    free(device, inference_buffer);
    free(device, inference);
    rlt::free(device, state);
}
