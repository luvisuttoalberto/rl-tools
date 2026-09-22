#pragma once
#include "model.h"
#include <rl_tools/numeric_types/persist_code.h>
#include <rl_tools/containers/matrix/persist_code.h>
#include <rl_tools/containers/tensor/persist_code.h>
#include <rl_tools/nn/optimizers/adam/instance/persist_code.h>
#include <rl_tools/nn/layers/dense/persist_code.h>
#include <rl_tools/nn_models/mlp/persist_code.h>
#include <utility>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn_models::mean_embedding {
    template<typename SHAPE, std::size_t... I>
    void write_shape(std::stringstream& body, std::index_sequence<I...>) {
        ((body << ", " << get<I>(SHAPE{})), ...);
    }
    template<typename DEVICE, typename SPEC>
    persist::Code save_code_split(DEVICE& device, ModuleForward<SPEC>& model, std::string name, bool const_declaration = true, typename DEVICE::index_t indent = 0) {
        auto encoder = rl_tools::save_code_split(device, model.encoder, "encoder", const_declaration, indent + 1);
        const std::string ind(indent * 4, ' ');
        const std::string ns = "RL_TOOLS_NAMESPACE_WRAPPER ::rl_tools::";
        const std::string qualifier = const_declaration ? "constexpr " : "";
        std::stringstream body;
        body << ind << "namespace " << name << " {\n" << encoder.body;
        body << ind << "using CONFIG = " << ns << "nn_models::mean_embedding::Configuration<encoder::CONFIG, "
             << SPEC::PREFIX_DIM << ", " << SPEC::ELEMENT_DIM << ", " << SPEC::N_ELEMENTS << ", " << SPEC::IGNORED_SUFFIX_DIM << ">;\n";
        body << ind << "using TEMPLATE = " << ns << "nn_models::mean_embedding::BindConfiguration<CONFIG>;\n";
        body << ind << "using INPUT_SHAPE = " << ns << "tensor::Shape<" << containers::persist::get_type_string<typename SPEC::TI>();
        write_shape<typename SPEC::INPUT_SHAPE>(body, std::make_index_sequence<length(typename SPEC::INPUT_SHAPE{})>{});
        body << ">;\n";
        body << ind << "using CAPABILITY = " << to_string(typename SPEC::CAPABILITY::template CHANGE_PARAMETERS<true, true>{}) << ";\n";
        body << ind << "using TYPE = " << ns << "nn_models::mean_embedding::Module<CONFIG, CAPABILITY, INPUT_SHAPE>;\n";
        body << ind << "template<typename T_TYPE = TYPE> " << qualifier << "T_TYPE factory_function() {\n";
        body << ind << "    if constexpr(T_TYPE::SPEC::CAPABILITY::TAG == " << ns << "nn::LayerCapability::Forward)\n"
             << ind << "        return {encoder::factory_function<typename T_TYPE::SPEC::ENCODER>()};\n";
        body << ind << "    else if constexpr(T_TYPE::SPEC::CAPABILITY::TAG == " << ns << "nn::LayerCapability::Backward)\n"
             << ind << "        return {{encoder::factory_function<typename T_TYPE::SPEC::ENCODER>()}};\n";
        body << ind << "    else return {{{encoder::factory_function<typename T_TYPE::SPEC::ENCODER>()}}, {}};\n";
        body << ind << "}\n";
        body << ind << "template<typename T_TYPE = TYPE> " << qualifier << "T_TYPE factory = factory_function<T_TYPE>();\n";
        body << ind << qualifier << "TYPE module = factory_function<>();\n" << ind << "}\n";
        return {encoder.header + "#include <rl_tools/nn_models/mean_embedding/model.h>\n", body.str()};
    }
    template<typename DEVICE, typename SPEC>
    std::string nn_analytics(DEVICE& device, ModuleGradient<SPEC>& model) {
        return "{\"type\":\"mean_embedding\",\"elements\":" + std::to_string(SPEC::N_ELEMENTS)
            + ",\"encoder\":" + rl_tools::nn_analytics(device, model.encoder) + "}";
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
