add_executable(test_nn_models_mean_embedding mean_embedding.cpp)
target_link_libraries(test_nn_models_mean_embedding PRIVATE RLtools::Core RLtools::Test)
target_compile_definitions(test_nn_models_mean_embedding PRIVATE RL_TOOLS_BACKEND_DISABLE_BLAS RL_TOOLS_DISABLE_HDF5 RL_TOOLS_DISABLE_TENSORBOARD RL_TOOLS_DEBUG_CONTAINER_CHECK_BOUNDS)
gtest_discover_tests(test_nn_models_mean_embedding)

if(NOT CMAKE_CROSSCOMPILING)
    set(EMBEDDING_EXPORT "${CMAKE_CURRENT_BINARY_DIR}/mean_embedding_export.h")
    set(ACTOR_EXPORT "${CMAKE_CURRENT_BINARY_DIR}/mean_embedding_actor_export.h")
    add_custom_command(OUTPUT "${EMBEDDING_EXPORT}" "${ACTOR_EXPORT}"
        COMMAND ${CMAKE_COMMAND} -E env
            "RL_TOOLS_MEAN_EMBEDDING_EXPORT=${EMBEDDING_EXPORT}"
            "RL_TOOLS_MEAN_EMBEDDING_ACTOR_EXPORT=${ACTOR_EXPORT}"
            $<TARGET_FILE:test_nn_models_mean_embedding>
            --gtest_filter=MeanEmbedding.CheckpointAndInferenceCapability:MeanEmbeddingSAC.TrainingAndActorCheckpoint
        DEPENDS test_nn_models_mean_embedding VERBATIM)
    add_executable(test_nn_models_mean_embedding_export export.cpp "${EMBEDDING_EXPORT}" "${ACTOR_EXPORT}")
    target_include_directories(test_nn_models_mean_embedding_export PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
    target_link_libraries(test_nn_models_mean_embedding_export PRIVATE RLtools::Core RLtools::Test)
    target_compile_definitions(test_nn_models_mean_embedding_export PRIVATE RL_TOOLS_BACKEND_DISABLE_BLAS)
    gtest_discover_tests(test_nn_models_mean_embedding_export)
endif()
