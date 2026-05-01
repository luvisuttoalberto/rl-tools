#pragma once
#include <rl_tools/version.h>
#include <rl_tools/nn_models/multi_agent_wrapper/model.h>
#include <rl_tools/nn_models/multi_agent_wrapper/operations_generic.h>
#include <rl_tools/nn/layers/sample_and_squash/layer.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn_models/mlp/network.h>
#include <rl_tools/nn_models/sequential/model.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>
#include <rl_tools/nn/optimizers/adam/adam.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1::multi_agent_sac {
    namespace rlt = rl_tools;

    // -----------------------------------------------------------------------
    // ActorSpec: bundles all type information.  Rebind helpers allow
    // CHANGE_BATCH_SIZE / CHANGE_SEQUENCE_LENGTH to work transparently.
    // -----------------------------------------------------------------------
    template<
        typename T_TYPE_POLICY, typename T_TI,
        typename T_WRAPPER_CONFIG,
        typename T_SAS_CONFIG,
        typename T_CAPABILITY,
        T_TI T_N_AGENTS,
        T_TI T_PER_AGENT_ACTION_DIM,
        T_TI T_BATCH_SIZE,
        T_TI T_SEQUENCE_LENGTH,
        T_TI T_OBS_DIM,
        T_TI T_ACTION_DIM,
        bool  T_DYNAMIC_ALLOCATION
    >
    struct ActorSpec {
        using TYPE_POLICY = T_TYPE_POLICY;
        using TI          = T_TI;
        using WRAPPER_CONFIG = T_WRAPPER_CONFIG;
        using SAS_CONFIG     = T_SAS_CONFIG;
        using CAPABILITY     = T_CAPABILITY;
        static constexpr TI N_AGENTS              = T_N_AGENTS;
        static constexpr TI PER_AGENT_ACTION_DIM  = T_PER_AGENT_ACTION_DIM;
        static constexpr TI BATCH_SIZE            = T_BATCH_SIZE;
        static constexpr TI SEQUENCE_LENGTH       = T_SEQUENCE_LENGTH;
        static constexpr TI OBS_DIM               = T_OBS_DIM;
        static constexpr TI ACTION_DIM            = T_ACTION_DIM;
        static constexpr TI WRAPPER_OUTPUT_DIM    = N_AGENTS * 2 * PER_AGENT_ACTION_DIM;
        static constexpr bool DYNAMIC_ALLOCATION  = T_DYNAMIC_ALLOCATION;

        using INPUT_SHAPE          = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, OBS_DIM>;
        using OUTPUT_SHAPE         = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, ACTION_DIM>;
        using WRAPPER_OUTPUT_SHAPE = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, WRAPPER_OUTPUT_DIM>;
        using SAS_INPUT_SHAPE      = rlt::tensor::Shape<TI, SEQUENCE_LENGTH, BATCH_SIZE, 2 * ACTION_DIM>;

        using WRAPPER_TYPE = rlt::nn_models::multi_agent_wrapper::Build<CAPABILITY, WRAPPER_CONFIG, INPUT_SHAPE>;
        using SAS_TYPE     = rlt::nn::layers::sample_and_squash::Layer<SAS_CONFIG, CAPABILITY, SAS_INPUT_SHAPE>;

        template<typename NEW_TI, NEW_TI NEW_BS>
        using RebindBatchSize = ActorSpec<
            TYPE_POLICY, TI, WRAPPER_CONFIG, SAS_CONFIG, CAPABILITY,
            N_AGENTS, PER_AGENT_ACTION_DIM, (TI)NEW_BS, SEQUENCE_LENGTH,
            OBS_DIM, ACTION_DIM, DYNAMIC_ALLOCATION>;

        template<typename NEW_TI, NEW_TI NEW_SL>
        using RebindSeqLen = ActorSpec<
            TYPE_POLICY, TI, WRAPPER_CONFIG, SAS_CONFIG, CAPABILITY,
            N_AGENTS, PER_AGENT_ACTION_DIM, BATCH_SIZE, (TI)NEW_SL,
            OBS_DIM, ACTION_DIM, DYNAMIC_ALLOCATION>;

        template<typename NEW_CAPABILITY>
        using RebindCapability = ActorSpec<
            TYPE_POLICY, TI, WRAPPER_CONFIG, SAS_CONFIG, NEW_CAPABILITY,
            N_AGENTS, PER_AGENT_ACTION_DIM, BATCH_SIZE, SEQUENCE_LENGTH,
            OBS_DIM, ACTION_DIM, DYNAMIC_ALLOCATION>;
    };

    // -----------------------------------------------------------------------
    // ActorBuffer
    // -----------------------------------------------------------------------
    template<typename T_SPEC, bool T_DYNAMIC_ALLOCATION>
    struct ActorBuffer {
        using SPEC = T_SPEC;
        using TI   = typename SPEC::TI;
        using T_BUF = typename SPEC::TYPE_POLICY::template GET<rlt::numeric_types::categories::Buffer>;
        static constexpr TI BATCH_SIZE         = SPEC::BATCH_SIZE;
        static constexpr TI ACTION_DIM         = SPEC::ACTION_DIM;
        static constexpr TI WRAPPER_OUTPUT_DIM = SPEC::WRAPPER_OUTPUT_DIM;

        // Exposed as "last buffer" to SAC:
        //   .noise             Matrix[BATCH, ACTION_DIM]  — SAC copies action_noise here
        //   .log_probabilities Matrix[1, BATCH]           — SAC reads for entropy backup
        typename SPEC::SAS_TYPE::template Buffer<T_DYNAMIC_ALLOCATION> sas_buffer;

        // Multi-agent-wrapper buffer
        typename SPEC::WRAPPER_TYPE::template Buffer<T_DYNAMIC_ALLOCATION> wrapper_buffer;

        // Permuted wrapper output used as SAS input:  Matrix[BATCH, WRAPPER_OUTPUT_DIM=12]
        // Gradient of permuted input (backward pass): Matrix[BATCH, WRAPPER_OUTPUT_DIM=12]
        using PERM_MAT_SPEC = rlt::matrix::Specification<T_BUF, TI, BATCH_SIZE, WRAPPER_OUTPUT_DIM, T_DYNAMIC_ALLOCATION>;
        rlt::Matrix<PERM_MAT_SPEC> perm_buf;
        rlt::Matrix<PERM_MAT_SPEC> d_perm_buf;

        // Gradient of wrapper output (backward) / temp storage for wrapper output (evaluate):
        // Tensor[SEQUENCE_LENGTH, BATCH, WRAPPER_OUTPUT_DIM]
        using WRAPPER_OUT_TENSOR_SPEC = rlt::tensor::Specification<T_BUF, TI, typename SPEC::WRAPPER_OUTPUT_SHAPE, T_DYNAMIC_ALLOCATION>;
        rlt::Tensor<WRAPPER_OUT_TENSOR_SPEC> d_wrapper_output;
    };

    // -----------------------------------------------------------------------
    // ActorState (empty — MLP has no recurrent state)
    // -----------------------------------------------------------------------
    template<typename T_SPEC, bool T_DA = true>
    struct ActorState {};

    // -----------------------------------------------------------------------
    // ActorGradient — the custom per-agent SAC actor
    // -----------------------------------------------------------------------
    template<typename T_SPEC>
    struct ActorGradient {
        using SPEC        = T_SPEC;
        using TI          = typename SPEC::TI;
        using TYPE_POLICY = typename SPEC::TYPE_POLICY;

        typename SPEC::WRAPPER_TYPE wrapper;
        typename SPEC::SAS_TYPE     sas;

        using INPUT_SHAPE  = typename SPEC::INPUT_SHAPE;
        using OUTPUT_SHAPE = typename SPEC::OUTPUT_SHAPE;

        template<typename NEW_TI, NEW_TI N>
        using CHANGE_BATCH_SIZE = ActorGradient<typename SPEC::template RebindBatchSize<NEW_TI, N>>;

        template<typename NEW_TI, NEW_TI N>
        using CHANGE_SEQUENCE_LENGTH = ActorGradient<typename SPEC::template RebindSeqLen<NEW_TI, N>>;

        template<typename NEW_CAP>
        using CHANGE_CAPABILITY = ActorGradient<typename SPEC::template RebindCapability<NEW_CAP>>;

        template<bool DA = true>
        using Buffer = ActorBuffer<SPEC, DA>;

        template<bool DA = true>
        using State = ActorState<SPEC, DA>;
    };

    // -----------------------------------------------------------------------
    // Permutation helpers
    //
    // The multi_agent_wrapper inner MLP outputs per-agent blocks:
    //   [mean0_a0, mean1_a0, logstd0_a0, logstd1_a0,  (agent 0)
    //    mean0_a1, mean1_a1, logstd0_a1, logstd1_a1,  (agent 1)
    //    mean0_a2, mean1_a2, logstd0_a2, logstd1_a2]  (agent 2)
    //
    // SAS expects grouped layout:
    //   [mean0_a0, mean1_a0, mean0_a1, mean1_a1, mean0_a2, mean1_a2,       (all means)
    //    logstd0_a0, logstd1_a0, logstd0_a1, logstd1_a1, logstd0_a2, logstd1_a2] (all log_stds)
    //
    // Forward P:  perm_buf[row, j]  = wrapper_out[row, P[j]]
    // Inverse Q:  d_wrapper[row, i] = d_perm_buf[row, Q[i]]
    // -----------------------------------------------------------------------
    template<typename DEVICE, typename SPEC, typename SRC_MAT, typename DST_MAT>
    void apply_permutation(DEVICE&, const SRC_MAT& src, DST_MAT& dst) {
        using TI = typename SPEC::TI;
        constexpr TI N    = SPEC::N_AGENTS;
        constexpr TI K    = SPEC::PER_AGENT_ACTION_DIM;
        constexpr TI ROWS = SRC_MAT::ROWS;
        for (TI row = 0; row < ROWS; ++row) {
            for (TI a = 0; a < N; ++a) {
                for (TI k = 0; k < K; ++k) {
                    rlt::set(dst, row, a*K+k,     rlt::get(src, row, a*2*K+k));
                    rlt::set(dst, row, N*K+a*K+k, rlt::get(src, row, a*2*K+K+k));
                }
            }
        }
    }

    template<typename DEVICE, typename SPEC, typename SRC_MAT, typename DST_MAT>
    void apply_inverse_permutation(DEVICE&, const SRC_MAT& src, DST_MAT& dst) {
        using TI = typename SPEC::TI;
        constexpr TI N    = SPEC::N_AGENTS;
        constexpr TI K    = SPEC::PER_AGENT_ACTION_DIM;
        constexpr TI ROWS = SRC_MAT::ROWS;
        for (TI row = 0; row < ROWS; ++row) {
            for (TI a = 0; a < N; ++a) {
                for (TI k = 0; k < K; ++k) {
                    rlt::set(dst, row, a*2*K+k,   rlt::get(src, row, a*K+k));
                    rlt::set(dst, row, a*2*K+K+k, rlt::get(src, row, N*K+a*K+k));
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // malloc / free — actor
    // -----------------------------------------------------------------------
    template<typename DEVICE, typename SPEC>
    void malloc(DEVICE& device, ActorGradient<SPEC>& actor) {
        rlt::malloc(device, actor.wrapper);
        rlt::malloc(device, actor.sas);
    }
    template<typename DEVICE, typename SPEC>
    void free(DEVICE& device, ActorGradient<SPEC>& actor) {
        rlt::free(device, actor.wrapper);
        rlt::free(device, actor.sas);
    }

    // malloc / free — buffer
    template<typename DEVICE, typename SPEC, bool DA>
    void malloc(DEVICE& device, ActorBuffer<SPEC, DA>& b) {
        rlt::malloc(device, b.sas_buffer);
        rlt::malloc(device, b.wrapper_buffer);
        rlt::malloc(device, b.perm_buf);
        rlt::malloc(device, b.d_perm_buf);
        rlt::malloc(device, b.d_wrapper_output);
    }
    template<typename DEVICE, typename SPEC, bool DA>
    void free(DEVICE& device, ActorBuffer<SPEC, DA>& b) {
        rlt::free(device, b.sas_buffer);
        rlt::free(device, b.wrapper_buffer);
        rlt::free(device, b.perm_buf);
        rlt::free(device, b.d_perm_buf);
        rlt::free(device, b.d_wrapper_output);
    }

    // malloc / free — state (no-op)
    template<typename DEVICE, typename SPEC, bool DA>
    void malloc(DEVICE&, ActorState<SPEC, DA>&) {}
    template<typename DEVICE, typename SPEC, bool DA>
    void free(DEVICE&, ActorState<SPEC, DA>&) {}

    // -----------------------------------------------------------------------
    // init / zero_gradient / reset
    // -----------------------------------------------------------------------
    template<typename DEVICE, typename SPEC, typename RNG>
    void init_weights(DEVICE& device, ActorGradient<SPEC>& actor, RNG& rng) {
        rlt::init_weights(device, actor.wrapper, rng);
        rlt::init_weights(device, actor.sas, rng);
    }
    template<typename DEVICE, typename SPEC>
    void zero_gradient(DEVICE& device, ActorGradient<SPEC>& actor) {
        rlt::zero_gradient(device, actor.wrapper);
        rlt::zero_gradient(device, actor.sas);
    }
    template<typename DEVICE, typename SPEC, typename STATE, typename RNG, typename MODE>
    void reset(DEVICE&, const ActorGradient<SPEC>&, STATE&, RNG&, const rlt::Mode<MODE>&) {}

    template<typename DEVICE, typename SPEC, typename STATE, typename RNG>
    void reset(DEVICE&, const ActorGradient<SPEC>&, STATE&, RNG&) {}

    // -----------------------------------------------------------------------
    // get_last_layer / get_last_buffer
    // -----------------------------------------------------------------------
    template<typename SPEC>
    auto& get_last_layer(ActorGradient<SPEC>& actor) { return actor.sas; }
    template<typename SPEC>
    const auto& get_last_layer(const ActorGradient<SPEC>& actor) { return actor.sas; }

    template<typename SPEC, bool DA>
    auto& get_last_buffer(ActorBuffer<SPEC, DA>& b) { return b.sas_buffer; }
    template<typename SPEC, bool DA>
    const auto& get_last_buffer(const ActorBuffer<SPEC, DA>& b) { return b.sas_buffer; }

    // -----------------------------------------------------------------------
    // forward  (gradient mode — stores intermediate state for backward)
    // -----------------------------------------------------------------------
    template<typename DEVICE, typename SPEC, typename INPUT, typename OUTPUT, typename BUFFER, typename RNG, typename MODE>
    void forward(DEVICE& device, ActorGradient<SPEC>& actor,
                 const INPUT& input, OUTPUT& output,
                 BUFFER& buffer, RNG& rng, const rlt::Mode<MODE>& mode)
    {
        // 1. Wrapper forward: output stored inside wrapper.content
        rlt::forward(device, actor.wrapper, input, buffer.wrapper_buffer, rng, mode);
        // 2. Get wrapper output as Matrix[BATCH, WRAPPER_OUTPUT_DIM]
        auto wrapper_out_t = rlt::output(device, actor.wrapper);
        auto wrapper_out_m = rlt::matrix_view(device, wrapper_out_t);
        // 3. Permute to [all_means | all_log_stds] layout
        apply_permutation<DEVICE, SPEC>(device, wrapper_out_m, buffer.perm_buf);
        // 4. SAS forward (stores output in sas.output, log_probs in buffer.sas_buffer)
        rlt::forward(device, actor.sas, buffer.perm_buf, buffer.sas_buffer, rng, mode);
        // 5. Copy sas.output → output tensor
        auto out_m = rlt::matrix_view(device, output);
        rlt::copy(device, device, actor.sas.output, out_m);
    }

    // -----------------------------------------------------------------------
    // evaluate  (const — no gradient state stored)
    // -----------------------------------------------------------------------
    template<typename DEVICE, typename SPEC, typename INPUT, typename OUTPUT, typename BUFFER, typename RNG, typename MODE>
    void evaluate(DEVICE& device, const ActorGradient<SPEC>& actor,
                  const INPUT& input, OUTPUT& output,
                  BUFFER& buffer, RNG& rng, const rlt::Mode<MODE>& mode)
    {
        // Reuse d_wrapper_output as temp for wrapper output (safe: backward not called in evaluate)
        rlt::evaluate(device, actor.wrapper, input, buffer.d_wrapper_output, buffer.wrapper_buffer, rng, mode);
        auto wrapper_out_m = rlt::matrix_view(device, buffer.d_wrapper_output);
        apply_permutation<DEVICE, SPEC>(device, wrapper_out_m, buffer.perm_buf);
        auto out_m = rlt::matrix_view(device, output);
        rlt::evaluate(device, actor.sas, buffer.perm_buf, out_m, buffer.sas_buffer, rng, mode);
    }

    // evaluate_step — called with 2D [BATCH, OBS_DIM] input/output from the eval loop.
    // Uses wrapper::evaluate_step (2D interface) to avoid the 3D requirement of evaluate.
    template<typename DEVICE, typename SPEC, typename INPUT, typename OUTPUT, typename STATE, typename BUFFER, typename RNG, typename MODE>
    void evaluate_step(DEVICE& device, const ActorGradient<SPEC>& actor,
                       const INPUT& input, STATE&, OUTPUT& output,
                       BUFFER& buffer, RNG& rng, const rlt::Mode<MODE>& mode)
    {
        // wrapper::evaluate_step writes to wrapper_buffer.output[0] slice (2D)
        typename SPEC::WRAPPER_TYPE::template State<> wrapper_state;
        auto wrapper_out_step = rlt::view(device, buffer.wrapper_buffer.output, 0); // [BATCH, WRAPPER_OUTPUT_DIM]
        rlt::evaluate_step(device, actor.wrapper, input, wrapper_state, wrapper_out_step, buffer.wrapper_buffer, rng, mode);
        auto wrapper_out_m = rlt::matrix_view(device, wrapper_out_step);
        apply_permutation<DEVICE, SPEC>(device, wrapper_out_m, buffer.perm_buf);
        auto out_m = rlt::matrix_view(device, output);
        rlt::evaluate(device, actor.sas, buffer.perm_buf, out_m, buffer.sas_buffer, rng, mode);
    }

    // -----------------------------------------------------------------------
    // backward
    // -----------------------------------------------------------------------
    template<typename DEVICE, typename SPEC, typename INPUT, typename D_OUTPUT, typename BUFFER, typename MODE>
    void backward(DEVICE& device, ActorGradient<SPEC>& actor,
                  const INPUT& input, D_OUTPUT& d_output,
                  BUFFER& buffer, const rlt::Mode<MODE>& mode)
    {
        // 1. d_output (Tensor[1,BATCH,ACTION_DIM]) → Matrix view
        auto d_out_m = rlt::matrix_view(device, d_output);
        // 2. SAS backward_full: fills d_perm_buf (gradient w.r.t. perm_buf)
        //    also accumulates gradient into sas.log_alpha.gradient
        rlt::backward_full(device, actor.sas, buffer.perm_buf, d_out_m, buffer.d_perm_buf, buffer.sas_buffer, mode);
        // 3. Inverse permutation: d_perm_buf → d_wrapper_output tensor
        auto d_wrap_m = rlt::matrix_view(device, buffer.d_wrapper_output);
        apply_inverse_permutation<DEVICE, SPEC>(device, buffer.d_perm_buf, d_wrap_m);
        // 4. Wrapper backward: accumulates gradients into shared MLP weights
        rlt::backward(device, actor.wrapper, input, buffer.d_wrapper_output, buffer.wrapper_buffer, mode);
    }

    // -----------------------------------------------------------------------
    // step / _reset_optimizer_state / reset_optimizer_state
    // -----------------------------------------------------------------------
    template<typename DEVICE, typename SPEC, typename OPT_SPEC>
    void step(DEVICE& device, rlt::nn::optimizers::Adam<OPT_SPEC>& optimizer, ActorGradient<SPEC>& actor) {
        rlt::step(device, optimizer, actor.wrapper);
        rlt::step(device, optimizer, actor.sas);
    }
    template<typename DEVICE, typename SPEC, typename OPT_SPEC>
    void _reset_optimizer_state(DEVICE& device, ActorGradient<SPEC>& actor, rlt::nn::optimizers::Adam<OPT_SPEC>& optimizer) {
        rlt::_reset_optimizer_state(device, actor.wrapper, optimizer);
        rlt::_reset_optimizer_state(device, actor.sas, optimizer);
    }
    template<typename DEVICE, typename SPEC, typename OPT_SPEC>
    void reset_optimizer_state(DEVICE& device, rlt::nn::optimizers::Adam<OPT_SPEC>& optimizer, ActorGradient<SPEC>& actor) {
        rlt::reset_optimizer_state(device, optimizer, actor.wrapper);
        // sas.log_alpha optimizer state is reset separately (via alpha_optimizer on log_alpha)
    }

    // -----------------------------------------------------------------------
    // copy / is_nan
    // -----------------------------------------------------------------------
    template<typename SD, typename TD, typename SPEC_SRC, typename SPEC_TGT>
    void copy(SD& sd, TD& td, const ActorGradient<SPEC_SRC>& src, ActorGradient<SPEC_TGT>& tgt) {
        rlt::copy(sd, td, src.wrapper, tgt.wrapper);
        rlt::copy(sd, td, src.sas, tgt.sas);
    }
    template<typename DEVICE, typename SPEC>
    bool is_nan(DEVICE& device, const ActorGradient<SPEC>& actor) {
        return rlt::is_nan(device, actor.wrapper);
    }

    // -----------------------------------------------------------------------
    // ConfigApproximatorsMLPMultiAgent
    //
    // Same template signature as ConfigApproximatorsMLP so it can be passed
    // as the APPROXIMATOR_CONFIG template argument to sac::loop::core::Config.
    // -----------------------------------------------------------------------
    template<typename TYPE_POLICY, typename TI, typename ENVIRONMENT, typename PARAMETERS, bool DYNAMIC_ALLOCATION>
    struct ConfigApproximatorsMLPMultiAgent {
        using SAC_PARAMETERS = typename PARAMETERS::SAC_PARAMETERS;

        static constexpr TI N_AGENTS             = ENVIRONMENT::Parameters::N_AGENTS;
        static constexpr TI PER_AGENT_OBS_DIM    = ENVIRONMENT::Observation::PER_AGENT_DIM;
        static constexpr TI PER_AGENT_ACTION_DIM = ENVIRONMENT::ACTION_DIM / N_AGENTS;
        static constexpr TI OBS_DIM              = ENVIRONMENT::Observation::DIM;
        static constexpr TI ACTION_DIM           = ENVIRONMENT::ACTION_DIM;

        template<typename CAPABILITY>
        struct Actor {
            // Inner MLP: PER_AGENT_OBS_DIM → 2*PER_AGENT_ACTION_DIM (means + log_stds per agent)
            using INNER_MLP_CONFIG = rlt::nn_models::mlp::Configuration<
                TYPE_POLICY, TI,
                2 * PER_AGENT_ACTION_DIM,
                PARAMETERS::ACTOR_NUM_LAYERS,
                PARAMETERS::ACTOR_HIDDEN_DIM,
                PARAMETERS::ACTOR_ACTIVATION_FUNCTION,
                rlt::nn::activation_functions::IDENTITY,
                typename PARAMETERS::INITIALIZER
            >;
            using INNER_MLP_BOUND = rlt::nn_models::mlp::BindConfiguration<INNER_MLP_CONFIG>;
            template<typename T_CONTENT, typename T_NEXT = rlt::nn_models::sequential::OutputModule>
            using SeqMod = rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT>;
            using INNER_MODULE_CHAIN = SeqMod<INNER_MLP_BOUND>;

            using WRAPPER_CONFIG = rlt::nn_models::multi_agent_wrapper::Configuration<
                TYPE_POLICY, TI, N_AGENTS, INNER_MODULE_CHAIN>;

            struct SAS_PARAMETERS_STRUCT {
                using T = typename TYPE_POLICY::DEFAULT;
                static constexpr T   LOG_STD_LOWER_BOUND      = SAC_PARAMETERS::LOG_STD_LOWER_BOUND;
                static constexpr T   LOG_STD_UPPER_BOUND      = SAC_PARAMETERS::LOG_STD_UPPER_BOUND;
                static constexpr T   LOG_PROBABILITY_EPSILON  = SAC_PARAMETERS::LOG_PROBABILITY_EPSILON;
                static constexpr bool ADAPTIVE_ALPHA          = SAC_PARAMETERS::ADAPTIVE_ALPHA;
                static constexpr bool UPDATE_ALPHA_WITH_ACTOR = false;
                static constexpr T   ALPHA                    = SAC_PARAMETERS::ALPHA;
                static constexpr T   TARGET_ENTROPY           = SAC_PARAMETERS::TARGET_ENTROPY;
            };
            using SAS_CONFIG = rlt::nn::layers::sample_and_squash::Configuration<
                TYPE_POLICY, TI, SAS_PARAMETERS_STRUCT>;

            using ACTOR_SPEC = ActorSpec<
                TYPE_POLICY, TI,
                WRAPPER_CONFIG, SAS_CONFIG,
                CAPABILITY,
                N_AGENTS, PER_AGENT_ACTION_DIM,
                (TI)SAC_PARAMETERS::ACTOR_BATCH_SIZE,
                (TI)SAC_PARAMETERS::SEQUENCE_LENGTH,
                OBS_DIM, ACTION_DIM,
                DYNAMIC_ALLOCATION
            >;
            using MODEL = ActorGradient<ACTOR_SPEC>;
        };

        template<typename CAPABILITY>
        struct Critic {
            static constexpr TI INPUT_DIM = ENVIRONMENT::ObservationPrivileged::DIM + ENVIRONMENT::ACTION_DIM;
            using INPUT_SHAPE = rlt::tensor::Shape<TI, (TI)SAC_PARAMETERS::SEQUENCE_LENGTH, (TI)SAC_PARAMETERS::CRITIC_BATCH_SIZE, INPUT_DIM>;
            using MLP_CONFIG = rlt::nn_models::mlp::Configuration<
                TYPE_POLICY, TI, 1,
                PARAMETERS::CRITIC_NUM_LAYERS,
                PARAMETERS::CRITIC_HIDDEN_DIM,
                PARAMETERS::CRITIC_ACTIVATION_FUNCTION,
                rlt::nn::activation_functions::IDENTITY,
                typename PARAMETERS::INITIALIZER
            >;
            using MLP_BOUND = rlt::nn_models::mlp::BindConfiguration<MLP_CONFIG>;
            template<typename T_CONTENT, typename T_NEXT = rlt::nn_models::sequential::OutputModule>
            using SeqMod = rlt::nn_models::sequential::Module<T_CONTENT, T_NEXT>;
            using MODULE_CHAIN = SeqMod<MLP_BOUND>;
            using MODEL = rlt::nn_models::sequential::Build<CAPABILITY, MODULE_CHAIN, INPUT_SHAPE>;
        };

        using ACTOR_OPTIMIZER_SPEC  = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI, typename PARAMETERS::ACTOR_OPTIMIZER_PARAMETERS,  DYNAMIC_ALLOCATION>;
        using CRITIC_OPTIMIZER_SPEC = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI, typename PARAMETERS::CRITIC_OPTIMIZER_PARAMETERS, DYNAMIC_ALLOCATION>;
        using ALPHA_OPTIMIZER_SPEC  = rlt::nn::optimizers::adam::Specification<TYPE_POLICY, TI, typename PARAMETERS::ALPHA_OPTIMIZER_PARAMETERS,  DYNAMIC_ALLOCATION>;
        using ACTOR_OPTIMIZER  = rlt::nn::optimizers::Adam<ACTOR_OPTIMIZER_SPEC>;
        using CRITIC_OPTIMIZER = rlt::nn::optimizers::Adam<CRITIC_OPTIMIZER_SPEC>;
        using ALPHA_OPTIMIZER  = rlt::nn::optimizers::Adam<ALPHA_OPTIMIZER_SPEC>;

        using CAPABILITY_ACTOR  = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, DYNAMIC_ALLOCATION>;
        using CAPABILITY_CRITIC = rlt::nn::capability::Gradient<rlt::nn::parameters::Adam, DYNAMIC_ALLOCATION>;

        using ACTOR_TYPE         = typename Actor<CAPABILITY_ACTOR>::MODEL;
        using CRITIC_TYPE        = typename Critic<CAPABILITY_CRITIC>::MODEL;
        using CRITIC_TARGET_TYPE = typename Critic<rlt::nn::capability::Forward<DYNAMIC_ALLOCATION>>::MODEL;
    };

} // namespace rl_tools::rl::zoo::oil_platform_v1::multi_agent_sac

// Persist / analytics overloads — must be in the rl_tools namespace so explicitly-qualified
// calls (rl_tools::save, rl_tools::load, rl_tools::save_code, rl_tools::nn_analytics)
// from the loop steps resolve correctly.
namespace rl_tools {
    template <typename DEVICE, typename SPEC>
    std::string nn_analytics(DEVICE& device,
        rl::zoo::oil_platform_v1::multi_agent_sac::ActorGradient<SPEC>& actor)
    {
        std::string data = "{\"wrapper\": ";
        data += nn_analytics(device, actor.wrapper);
        data += ", \"sas\": ";
        data += nn_analytics(device, actor.sas);
        data += "}";
        return data;
    }

    // save / load (HDF5 or tar backend — GROUP is deduced)
    template <typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device,
              rl::zoo::oil_platform_v1::multi_agent_sac::ActorGradient<SPEC>& actor,
              GROUP& group)
    {
        auto wrapper_group = create_group(device, group, "wrapper");
        save(device, actor.wrapper, wrapper_group);
        auto sas_group = create_group(device, group, "sas");
        save(device, actor.sas, sas_group);
    }

    template <typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device,
              rl::zoo::oil_platform_v1::multi_agent_sac::ActorGradient<SPEC>& actor,
              GROUP& group)
    {
        auto wrapper_group = get_group(device, group, "wrapper");
        bool ok = load(device, actor.wrapper, wrapper_group);
        auto sas_group = get_group(device, group, "sas");
        ok &= load(device, actor.sas, sas_group);
        return ok;
    }

    // save_code — produces embeddable C++ constant from actor weights
    template <typename DEVICE, typename SPEC>
    std::string save_code(DEVICE& device,
                          rl::zoo::oil_platform_v1::multi_agent_sac::ActorGradient<SPEC>& actor,
                          std::string name,
                          bool const_declaration = true,
                          typename DEVICE::index_t indent = 0)
    {
        std::string code;
        code += save_code(device, actor.wrapper, name + "_wrapper", const_declaration, indent);
        code += "\n";
        code += save_code(device, actor.sas,     name + "_sas",     const_declaration, indent);
        return code;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
