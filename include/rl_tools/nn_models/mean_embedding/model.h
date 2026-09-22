#pragma once
#include <rl_tools/nn_models/mlp/network.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn_models::mean_embedding {
    // Input: [prefix, N_ELEMENTS * ELEMENT_DIM, ignored suffix].
    // Output: [prefix, masked mean(shared_encoder(element))].
    // Optional final feature in each element is a binary presence mask, not an encoder input.
    template<typename T_ENCODER_CONFIG, auto T_PREFIX_DIM, auto T_ELEMENT_DIM,
             auto T_N_ELEMENTS, auto T_IGNORED_SUFFIX_DIM = 0, bool T_MASKED = false>
    struct Configuration {
        using ENCODER_CONFIG = T_ENCODER_CONFIG;
        using TYPE_POLICY = typename ENCODER_CONFIG::TYPE_POLICY;
        using TI = typename ENCODER_CONFIG::TI;
        static constexpr TI PREFIX_DIM = T_PREFIX_DIM;
        static constexpr TI ELEMENT_DIM = T_ELEMENT_DIM;
        static constexpr bool MASKED = T_MASKED;
        static constexpr TI ELEMENT_STRIDE = ELEMENT_DIM + (MASKED ? 1 : 0);
        static constexpr TI N_ELEMENTS = T_N_ELEMENTS;
        static constexpr TI IGNORED_SUFFIX_DIM = T_IGNORED_SUFFIX_DIM;
        static constexpr TI EMBEDDING_DIM = ENCODER_CONFIG::OUTPUT_DIM;
        static_assert(N_ELEMENTS > 0, "Mean embedding requires at least one element");
    };

    template<typename T_CONFIG, typename T_CAPABILITY, typename T_INPUT_SHAPE>
    struct Specification: T_CONFIG {
        using CONFIG = T_CONFIG;
        using CAPABILITY = T_CAPABILITY;
        using TI = typename CONFIG::TI;
        using INPUT_SHAPE = T_INPUT_SHAPE;
        static constexpr TI INPUT_DIM = get_last(INPUT_SHAPE{});
        static constexpr TI OUTPUT_DIM = CONFIG::PREFIX_DIM + CONFIG::EMBEDDING_DIM;
        static constexpr TI INTERNAL_BATCH_SIZE = get<0>(tensor::CumulativeProduct<tensor::PopBack<INPUT_SHAPE>>{});
        static_assert(INPUT_DIM == CONFIG::PREFIX_DIM + CONFIG::N_ELEMENTS * CONFIG::ELEMENT_STRIDE + CONFIG::IGNORED_SUFFIX_DIM);
        template<typename SHAPE>
        using OUTPUT_SHAPE_FACTORY = tensor::Replace<SHAPE, OUTPUT_DIM, length(SHAPE{}) - 1>;
        using OUTPUT_SHAPE = OUTPUT_SHAPE_FACTORY<INPUT_SHAPE>;
        using ENCODER_INPUT_SHAPE = tensor::Shape<TI, 1, INTERNAL_BATCH_SIZE * CONFIG::N_ELEMENTS, CONFIG::ELEMENT_DIM>;
        using ENCODER = mlp::NeuralNetwork<typename CONFIG::ENCODER_CONFIG, CAPABILITY, ENCODER_INPUT_SHAPE>;
    };

    struct State {};
    template<typename T_SPEC, bool DYNAMIC_ALLOCATION>
    struct Buffer {
        using SPEC = T_SPEC;
        using TI = typename SPEC::TI;
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
        template<TI COLS>
        using MatrixType = Matrix<matrix::Specification<T, TI, SPEC::INTERNAL_BATCH_SIZE * SPEC::N_ELEMENTS, COLS, DYNAMIC_ALLOCATION>>;
        MatrixType<SPEC::ELEMENT_DIM> packed, d_packed;
        MatrixType<SPEC::EMBEDDING_DIM> embedded, d_embedded;
        typename SPEC::ENCODER::template Buffer<DYNAMIC_ALLOCATION> encoder;
    };

    template<typename T_SPEC>
    struct ModuleForward {
        using SPEC = T_SPEC;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;
        using TI = typename SPEC::TI;
        using INPUT_SHAPE = typename SPEC::INPUT_SHAPE;
        using OUTPUT_SHAPE = typename SPEC::OUTPUT_SHAPE;
        template<typename SHAPE>
        using OUTPUT_SHAPE_FACTORY = typename SPEC::template OUTPUT_SHAPE_FACTORY<SHAPE>;
        static constexpr TI INPUT_DIM = SPEC::INPUT_DIM;
        static constexpr TI OUTPUT_DIM = SPEC::OUTPUT_DIM;
        static constexpr TI NUM_WEIGHTS = SPEC::ENCODER::NUM_WEIGHTS;
        typename SPEC::ENCODER encoder;
        template<bool DA = true> using Buffer = mean_embedding::Buffer<SPEC, DA>;
        template<bool = true> using State = mean_embedding::State;
    };
    template<typename SPEC>
    struct ModuleBackward: ModuleForward<SPEC> {};
    template<typename SPEC>
    struct ModuleGradient: ModuleBackward<SPEC> {
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
        Matrix<matrix::Specification<T, typename SPEC::TI, SPEC::INTERNAL_BATCH_SIZE, SPEC::OUTPUT_DIM,
                                    SPEC::CAPABILITY::DYNAMIC_ALLOCATION>> output;
    };
    template<typename CONFIG, typename CAPABILITY, typename INPUT_SHAPE>
    using Module = utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Forward,
        ModuleForward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
        utils::typing::conditional_t<CAPABILITY::TAG == nn::LayerCapability::Backward,
            ModuleBackward<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>,
            ModuleGradient<Specification<CONFIG, CAPABILITY, INPUT_SHAPE>>>>;
    template<typename CONFIG>
    struct BindConfiguration {
        template<typename CAPABILITY, typename INPUT_SHAPE>
        using Layer = Module<CONFIG, CAPABILITY, INPUT_SHAPE>;
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END
