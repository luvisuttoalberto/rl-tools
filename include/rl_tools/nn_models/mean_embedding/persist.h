#pragma once
#include "model.h"
#include <rl_tools/nn_models/mlp/persist.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::nn_models::mean_embedding {
    template<typename DEVICE, typename SPEC, typename GROUP>
    void save(DEVICE& device, ModuleForward<SPEC>& model, GROUP& group) {
        set_attribute(device, group, "type", "mean_embedding");
        write_attributes(device, group);
        auto encoder = create_group(device, group, "encoder");
        rl_tools::save(device, model.encoder, encoder);
    }
    template<typename DEVICE, typename SPEC, typename GROUP>
    bool load(DEVICE& device, ModuleForward<SPEC>& model, GROUP& group) {
        auto encoder = get_group(device, group, "encoder");
        return rl_tools::load(device, model.encoder, encoder);
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
