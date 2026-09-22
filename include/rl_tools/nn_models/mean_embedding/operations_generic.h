#pragma once
#include "model.h"
#include <rl_tools/nn_models/mlp/operations_generic.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
// Operations live beside the model so sequential's dependent calls find them
// through ADL regardless of header inclusion order.
namespace rl_tools::nn_models::mean_embedding {
    template<typename DEVICE, typename SPEC>
    void malloc(DEVICE& device, ModuleForward<SPEC>& model) { rl_tools::malloc(device, model.encoder); }
    template<typename DEVICE, typename SPEC>
    void free(DEVICE& device, ModuleForward<SPEC>& model) { rl_tools::free(device, model.encoder); }
    template<typename DEVICE, typename SPEC>
    void malloc(DEVICE& device, ModuleGradient<SPEC>& model) {
        rl_tools::malloc(device, model.encoder);
        rl_tools::malloc(device, model.output);
    }
    template<typename DEVICE, typename SPEC>
    void free(DEVICE& device, ModuleGradient<SPEC>& model) {
        rl_tools::free(device, model.encoder);
        rl_tools::free(device, model.output);
    }
    template<typename DEVICE, typename SPEC, bool DA>
    void malloc(DEVICE& device, Buffer<SPEC, DA>& buffer) {
        rl_tools::malloc(device, buffer.packed);
        rl_tools::malloc(device, buffer.d_packed);
        rl_tools::malloc(device, buffer.embedded);
        rl_tools::malloc(device, buffer.d_embedded);
        rl_tools::malloc(device, buffer.encoder);
    }
    template<typename DEVICE, typename SPEC, bool DA>
    void free(DEVICE& device, Buffer<SPEC, DA>& buffer) {
        rl_tools::free(device, buffer.packed);
        rl_tools::free(device, buffer.d_packed);
        rl_tools::free(device, buffer.embedded);
        rl_tools::free(device, buffer.d_embedded);
        rl_tools::free(device, buffer.encoder);
    }
    template<typename DEVICE> void malloc(DEVICE&, State&) {}
    template<typename DEVICE> void free(DEVICE&, State&) {}
    template<typename SD, typename TD> void copy(SD&, TD&, const State&, State&) {}
    template<typename DEVICE, typename SPEC, typename RNG, typename MODE>
    void reset(DEVICE&, const ModuleForward<SPEC>&, State&, RNG&, const Mode<MODE>&) {}

    template<typename DEVICE, typename SPEC, typename RNG>
    void init_weights(DEVICE& device, ModuleForward<SPEC>& model, RNG& rng) { rl_tools::init_weights(device, model.encoder, rng); }
    template<typename DEVICE, typename SPEC>
    void zero_gradient(DEVICE& device, ModuleGradient<SPEC>& model) { rl_tools::zero_gradient(device, model.encoder); }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    void update(DEVICE& device, ModuleGradient<SPEC>& model, OPTIMIZER& optimizer) { rl_tools::update(device, model.encoder, optimizer); }
    template<typename DEVICE, typename SPEC, typename OPTIMIZER>
    void _reset_optimizer_state(DEVICE& device, ModuleGradient<SPEC>& model, OPTIMIZER& optimizer) { rl_tools::_reset_optimizer_state(device, model.encoder, optimizer); }
    template<typename SD, typename TD, typename S, typename T>
    void copy(SD& sd, TD& td, const ModuleForward<S>& source, ModuleForward<T>& target) { rl_tools::copy(sd, td, source.encoder, target.encoder); }
    template<typename SD, typename TD, typename S, typename T, bool SDA, bool TDA>
    void copy(SD& sd, TD& td, const Buffer<S, SDA>& source, Buffer<T, TDA>& target) {
        rl_tools::copy(sd, td, source.packed, target.packed);
        rl_tools::copy(sd, td, source.d_packed, target.d_packed);
        rl_tools::copy(sd, td, source.embedded, target.embedded);
        rl_tools::copy(sd, td, source.d_embedded, target.d_embedded);
        rl_tools::copy(sd, td, source.encoder, target.encoder);
    }
    template<typename DEVICE, typename SPEC, typename MODE = mode::Default<>>
    bool is_nan(DEVICE& device, const ModuleForward<SPEC>& model, const Mode<MODE>& mode = Mode<mode::Default<>>{}) { return rl_tools::is_nan(device, model.encoder, mode); }
    template<typename DEVICE, typename SPEC>
    auto output(DEVICE& device, ModuleGradient<SPEC>& model) {
        return view(device, model.output, matrix::ViewSpec<SPEC::INTERNAL_BATCH_SIZE, SPEC::OUTPUT_DIM>{});
    }
    template<typename DEVICE, typename SPEC>
    auto output(DEVICE& device, const ModuleGradient<SPEC>& model) {
        return view(device, model.output, matrix::ViewSpec<SPEC::INTERNAL_BATCH_SIZE, SPEC::OUTPUT_DIM>{});
    }

    template<typename SPEC, typename INPUT, typename PACKED>
    void pack(const INPUT& input, PACKED& packed) {
        using TI = typename SPEC::TI;
        static_assert(INPUT::COLS == SPEC::INPUT_DIM);
        for(TI row = 0; row < INPUT::ROWS; ++row)
            for(TI j = 0; j < SPEC::N_ELEMENTS; ++j)
                for(TI k = 0; k < SPEC::ELEMENT_DIM; ++k)
                    set(packed, row * SPEC::N_ELEMENTS + j, k, get(input, row, SPEC::PREFIX_DIM + j * SPEC::ELEMENT_DIM + k));
    }
    template<typename SPEC, typename INPUT, typename EMBEDDED, typename OUTPUT>
    void pool(const INPUT& input, const EMBEDDED& embedded, OUTPUT& output) {
        using TI = typename SPEC::TI;
        using T = typename SPEC::TYPE_POLICY::template GET<numeric_types::categories::Accumulator>;
        static_assert(OUTPUT::ROWS == INPUT::ROWS && OUTPUT::COLS == SPEC::OUTPUT_DIM);
        for(TI row = 0; row < INPUT::ROWS; ++row) {
            for(TI k = 0; k < SPEC::PREFIX_DIM; ++k) set(output, row, k, get(input, row, k));
            for(TI k = 0; k < SPEC::EMBEDDING_DIM; ++k) {
                T sum = 0;
                for(TI j = 0; j < SPEC::N_ELEMENTS; ++j) sum += get(embedded, row * SPEC::N_ELEMENTS + j, k);
                set(output, row, SPEC::PREFIX_DIM + k, sum / T(SPEC::N_ELEMENTS));
            }
        }
    }

    template<typename DEVICE, typename SPEC, typename INPUT, typename OUTPUT, typename BS, bool DA, typename RNG, typename MODE = mode::Default<>>
    void evaluate(DEVICE& device, const ModuleForward<SPEC>& model, const INPUT& input, OUTPUT& output, Buffer<BS, DA>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto in = matrix_view(device, input);
        auto out = matrix_view(device, output);
        constexpr auto ROWS = decltype(in)::ROWS;
        static_assert(ROWS <= BS::INTERNAL_BATCH_SIZE);
        auto packed = view(device, buffer.packed, matrix::ViewSpec<ROWS * SPEC::N_ELEMENTS, SPEC::ELEMENT_DIM>{});
        auto embedded = view(device, buffer.embedded, matrix::ViewSpec<ROWS * SPEC::N_ELEMENTS, SPEC::EMBEDDING_DIM>{});
        pack<SPEC>(in, packed);
        rl_tools::evaluate(device, model.encoder, packed, embedded, buffer.encoder, rng, mode);
        pool<SPEC>(in, embedded, out);
    }
    template<typename DEVICE, typename SPEC, typename INPUT, typename OUTPUT, typename BS, bool DA, typename RNG, typename MODE = mode::Default<>>
    void evaluate_step(DEVICE& device, const ModuleForward<SPEC>& model, const INPUT& input, State&, OUTPUT& output, Buffer<BS, DA>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        evaluate(device, model, input, output, buffer, rng, mode);
    }
    template<typename DEVICE, typename SPEC, typename INPUT, typename BS, bool DA, typename RNG, typename MODE = mode::Default<>>
    void forward(DEVICE& device, ModuleGradient<SPEC>& model, const INPUT& input, Buffer<BS, DA>& buffer, RNG& rng, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        auto in = matrix_view(device, input);
        static_assert(decltype(in)::ROWS == SPEC::INTERNAL_BATCH_SIZE && BS::INTERNAL_BATCH_SIZE == SPEC::INTERNAL_BATCH_SIZE);
        pack<SPEC>(in, buffer.packed);
        rl_tools::forward(device, model.encoder, buffer.packed, buffer.encoder, rng, mode);
        auto embedded_tensor = rl_tools::output(device, model.encoder);
        auto embedded = matrix_view(device, embedded_tensor);
        pool<SPEC>(in, embedded, model.output);
    }
    template<typename DEVICE, typename SPEC, typename INPUT, typename D_OUTPUT, typename BS, bool DA, typename MODE = mode::Default<>>
    void backward(DEVICE& device, ModuleGradient<SPEC>& model, const INPUT& input, D_OUTPUT& d_output, Buffer<BS, DA>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        using TI = typename SPEC::TI;
        using T = typename SPEC::TYPE_POLICY::DEFAULT;
        auto in = matrix_view(device, input);
        auto dout = matrix_view(device, d_output);
        static_assert(decltype(in)::ROWS == SPEC::INTERNAL_BATCH_SIZE);
        pack<SPEC>(in, buffer.packed);
        for(TI row = 0; row < SPEC::INTERNAL_BATCH_SIZE; ++row)
            for(TI j = 0; j < SPEC::N_ELEMENTS; ++j)
                for(TI k = 0; k < SPEC::EMBEDDING_DIM; ++k)
                    set(buffer.d_embedded, row * SPEC::N_ELEMENTS + j, k, get(dout, row, SPEC::PREFIX_DIM + k) / T(SPEC::N_ELEMENTS));
        rl_tools::backward_full(device, model.encoder, buffer.packed, buffer.d_embedded, buffer.d_packed, buffer.encoder, mode);
    }
    template<typename DEVICE, typename SPEC, typename INPUT, typename D_OUTPUT, typename D_INPUT, typename BS, bool DA, typename MODE = mode::Default<>>
    void backward_full(DEVICE& device, ModuleGradient<SPEC>& model, const INPUT& input, D_OUTPUT& d_output, D_INPUT& d_input, Buffer<BS, DA>& buffer, const Mode<MODE>& mode = Mode<mode::Default<>>{}) {
        backward(device, model, input, d_output, buffer, mode);
        using TI = typename SPEC::TI;
        auto dout = matrix_view(device, d_output);
        auto din = matrix_view(device, d_input);
        set_all(device, din, 0);
        for(TI row = 0; row < SPEC::INTERNAL_BATCH_SIZE; ++row) {
            for(TI k = 0; k < SPEC::PREFIX_DIM; ++k) set(din, row, k, get(dout, row, k));
            for(TI j = 0; j < SPEC::N_ELEMENTS; ++j)
                for(TI k = 0; k < SPEC::ELEMENT_DIM; ++k)
                    set(din, row, SPEC::PREFIX_DIM + j * SPEC::ELEMENT_DIM + k, get(buffer.d_packed, row * SPEC::N_ELEMENTS + j, k));
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
