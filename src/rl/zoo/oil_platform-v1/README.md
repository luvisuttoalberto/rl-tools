# SAC with variable swarm sizes

The SAC configuration trains one shared MLP policy on swarms of **2–6 agents**.
Each episode independently samples its size uniformly at reset. The size stays
constant during the episode; a drone dying does not remove its slot from the
swarm. PPO and the scripted baseline retain their fixed three-agent configuration.

`VariableSwarmParameters` in `environment.h` defines `MIN_AGENTS=2` and
`MAX_AGENTS=6`. `N_AGENTS` remains an alias for storage capacity in the existing
library interfaces. Change `MAX_AGENTS` before building/training if a different
upper bound is needed. A trained checkpoint supports every size from 2 to that
bound without rebuilding or retraining. It does not support fleets above that
bound. Changing the bound requires a new training run.

## Policy, critic, and replay

The environment uses fixed arrays with present drones in `[0, state.n_agents)`.
Unused slots have zero observations and a zero presence mask. Dynamics,
coverage, collisions, charging, deaths, termination, and metrics consider only
present drones. Present dead drones retain their observations and are included
in teammate pooling; presence and death are different features.

The actor uses only MLPs and arithmetic pooling:

```text
Teammate features: 8 -> 32 (ReLU) -> 32 (linear), shared across slots
Presence-masked mean of teammate embeddings: 32 features
Own/task features (13) + own presence (1) + fleet size / MAX_AGENTS (1)
    + mean embedding (32) -> 128 -> 128 -> 4
Four outputs -> two sampled and squashed actions per agent
```

Each teammate block stores eight features followed by its presence mask. The
mask is not passed to the encoder: it controls the mean and its backward pass.
An empty set produces a zero embedding with zero encoder gradient. One-hot ID
slots remain in the raw observation for compatibility with the layout, but the
actor ignores them. Teammate permutations do not change the policy distribution.

At capacity six, actor observations contain 396 values, privileged critic
observations 63, and actions 12. The centralized critics remain ordinary MLPs,
with a presence flag appended to each agent's eight privileged features.
Replay stores masks and count features in its fixed-size observations, so a
training batch can contain different swarm sizes without changing shapes.

Absent **and dead** drones have zero actions. They contribute no log probability,
entropy, temperature gradient, or action gradient. SAC's target entropy is
`-2 * number_of_present_alive_drones` for each sample. The sampling layer's
mask support is opt-in; other SAC configurations retain the unmasked path.
Random warmup actions are also masked before insertion into replay.

## Reward scaling

Variable-swarm SAC enables `NORMALIZE_SWARM_REWARD`, with
`REWARD_REFERENCE_AGENTS=3`. For episode size `n`:

- Agent-summed coverage, charging, death, movement, and charging-potential terms
  are multiplied by `3/n`.
- Pairwise repulsion is multiplied by `3*(3-1)/(n*(n-1))`.
- Excess charger-occupancy penalties are multiplied by `(3-1)/(n-1)`.
- Global abandonment and undetected-disaster penalties are unchanged.

These factors preserve the existing three-agent reward and reduce changes in
reward scale caused only by fleet size. Denominators use the original present
count, not the surviving count, so death cannot shrink the normalization
population. This is a training objective choice, not a claim that every fleet
size has the same achievable return. Log metrics retain their physical units;
`agents/swarm_size` and `eval/swarm_size` identify the episode's population.

## Build and train

Use the same configured build directory and targets as before:

```sh
cmake --build build --target rl_zoo_oil_platform-v1_sac evaluate_oil_platform enjoy_oil_platform -j2
MKL_NUM_THREADS=1 OMP_NUM_THREADS=1 ./build/src/rl/zoo/rl_zoo_oil_platform-v1_sac -s 4 --ee VARIABLE_SWARM_2_TO_6
```

Replace `build` if your CMake directory has a different name. Start a **new**
experiment. Stage-1 three-agent checkpoints are incompatible with the new
count/presence inputs. A stage-1 executable that is already running continues
its original experiment; its eventual checkpoint cannot be loaded by step 2.

The replay capacity remains 1,000,000 transitions **per environment**, with four
parallel environments inside one seed. Storage is sized for MAX_AGENTS even for
two-agent episodes, so larger bounds increase memory usage. At capacity six,
the replay allocation is approximately **18.54 GB (17.27 GiB) per seed**, plus
networks and temporary buffers; reservation and resident memory differ. The unused second
training state in `zoo.cpp` has been removed; it previously reserved another
complete replay buffer. This removes a duplicate allocation, not necessarily
an equivalent amount of resident RAM. No convergence-length training is run by
the implementation tests.

## Evaluate a single checkpoint at different sizes

`--agents` selects a fixed episode population without changing model shapes:

```sh
./build/src/rl/zoo/evaluate_oil_platform -c path/to/checkpoint.h5 --agents 2 --label sac-variable --ee VARIABLE_SWARM_EVAL
./build/src/rl/zoo/evaluate_oil_platform -c path/to/checkpoint.h5 --agents 6 --label sac-variable --ee VARIABLE_SWARM_EVAL
./build/src/rl/zoo/enjoy_oil_platform -c path/to/checkpoint.h5 --agents 4 --stochastic -n 20 -o trajectories_n4.csv
```

Both tools accept sizes 2–6 (default: MAX_AGENTS). The evaluation runner uses
400 episodes and appends `-n<size>` to the run label to separate results. The
training loop's periodic evaluations sample episode sizes using the training
configuration, so their aggregate return mixes sizes. Use the fixed-size tools
for per-size comparisons.

JSON trajectories include `n_agents` and only present drones, so the browser
renders the actual swarm. CSV output keeps fixed columns up to capacity and
adds `n_agents` and `a<i>_present`; use the presence flags when processing it.

## Checkpoints and C++ exports

Actor checkpoint groups are `swarm2x6` (version 2, capacity six) and `sas`.
The schema rejects stage-1 checkpoints and checkpoints built with a different
capacity, since the fleet-size feature uses that capacity for normalization.
Training-to-forward-only loading and batch-one inference are supported.

C++ exports retain separate `<name>_wrapper` and `<name>_sas` modules. Evaluate
the wrapper, permute its per-agent `[means, log_stds]` blocks into SAC's
`[all_means, all_log_stds]` layout, call the generated
`<name>::set_action_mask(observation_matrix, sas_buffer)`, then evaluate the
sampling layer. The mask must be refreshed for every observation. The exported
sampling configuration also retains its masking flag.

## Validation

With repository tests enabled:

```sh
cmake --build <test-build-dir> --target test_nn_models_mean_embedding_export
ctest --test-dir <test-build-dir> -R 'MeanEmbedding|VariableSwarm' --output-on-failure
```

Tests cover finite-difference encoder/input and masked-SAC gradients, an empty
teammate set, padding-content invariance, teammate permutations, all reset sizes,
dead-versus-absent semantics, zero padded replay actions (including random
warmup), mixed-size SAC updates, reference-size reward/dynamics equivalence,
checkpoint loading, and compiled C++ exports with padded/dead actions.
The implementation and tests exercise CPU execution; CUDA pooling is not
implemented. Learning quality across swarm sizes remains an experiment.
