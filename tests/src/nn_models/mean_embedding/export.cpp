#include <rl_tools/operations/cpu.h>
#include <rl_tools/nn/layers/dense/operations_generic.h>
#include <rl_tools/nn_models/mean_embedding/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn_models/multi_agent_wrapper/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include "mean_embedding_export.h"
#include "mean_embedding_actor_export.h"
#include <gtest/gtest.h>
#include <cmath>

namespace rlt = rl_tools;
using DEVICE = rlt::devices::DefaultCPU;
using TI = DEVICE::index_t;
using RNG = DEVICE::SPEC::RANDOM::ENGINE<>;

TEST(MeanEmbeddingExport, CompiledPolicyMatchesOriginal) {
    DEVICE device;
    RNG rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    mean_embedding_test::TYPE::Buffer<> buffer;
    rlt::malloc(device, buffer);
    rlt::Tensor<rlt::tensor::Specification<double, TI, rlt::tensor::Shape<TI, 1, 2, 9>>> input;
    rlt::Tensor<rlt::tensor::Specification<double, TI, rlt::tensor::Shape<TI, 1, 2, 2>>> output;
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    auto in = rlt::matrix_view(device, input);
    auto out = rlt::matrix_view(device, output);
    for(TI r = 0; r < 2; ++r) for(TI c = 0; c < 9; ++c) rlt::set(in, r, c, 0.13 * double(1 + r * 9 + c) - 1);
    rlt::evaluate(device, mean_embedding_test::module, input, output, buffer, rng);
    for(TI r = 0; r < 2; ++r) for(TI c = 0; c < 2; ++c) EXPECT_NEAR(rlt::get(out, r, c), mean_embedding_expected::output[r * 2 + c], 1e-14);
    rlt::free(device, buffer);
    rlt::free(device, output);
    rlt::free(device, input);
    rlt::free(device, rng);
}

TEST(MeanEmbeddingExport, CompiledSACWrapperMatchesOriginalActions) {
    DEVICE device;
    RNG rng;
    rlt::malloc(device, rng);
    rlt::init(device, rng, 0);
    using Wrapper = actor_test_wrapper::TYPE;
    Wrapper::Buffer<> buffer;
    rlt::malloc(device, buffer);
    rlt::Tensor<rlt::tensor::Specification<double, TI, Wrapper::INPUT_SHAPE>> input;
    rlt::Tensor<rlt::tensor::Specification<double, TI, Wrapper::OUTPUT_SHAPE>> output;
    rlt::malloc(device, input);
    rlt::malloc(device, output);
    auto in = rlt::matrix_view(device, input);
    auto out = rlt::matrix_view(device, output);
    for(TI k = 0; k < decltype(in)::COLS; ++k) rlt::set(in, 0, k, actor_expected::input[k]);
    rlt::evaluate(device, actor_test_wrapper::module, input, output, buffer, rng);
    for(TI i = 0; i < 3; ++i) for(TI k = 0; k < 2; ++k)
        EXPECT_NEAR(std::tanh(rlt::get(out, 0, i * 4 + k)), actor_expected::output[i * 2 + k], 1e-14);
    rlt::free(device, output);
    rlt::free(device, input);
    rlt::free(device, buffer);
    rlt::free(device, rng);
}
