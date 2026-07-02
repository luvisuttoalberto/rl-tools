# Scientific Article — Outline and Writing Guide

**Provisional title:**
*Multi-Agent Soft Actor-Critic for Battery-Constrained UAV Surveillance via Sensor-Range Coverage Shaping on a High-Performance RL Framework*

**Target venues (in order of preference):**
- IEEE Robotics and Automation Letters (RA-L)
- International Conference on Robotics and Automation (ICRA)
- Conference on Robot Learning (CoRL)
- International Joint Conference on Autonomous Agents and Multi-Agent Systems (AAMAS)

**Target length:** 8 pages (RA-L/ICRA format) or 10–12 pages (journal)

---

## Abstract (150–200 words)

**What to write:**
- One sentence on the application domain: persistent aerial surveillance of critical infrastructure (oil platforms) with battery-constrained UAV swarms
- One sentence on the problem difficulty: multi-objective reward (coverage + disaster detection + battery management), partial observability, non-stationarity
- One sentence on the technical approach: centralized-critic / decentralized-actor SAC with shared-weight per-agent actor, sensor-range coverage reward, and a systematic reward shaping methodology
- One sentence on the engineering contribution: extension of the rl-tools C++ RL framework with a multi-agent actor architecture (multi_agent_wrapper + permutation buffer + SampleAndSquash)
- Two sentences on key results: quantitative metrics (coverage %, disaster detection rate, detection latency, survival rate) from best configuration
- One sentence on availability: code is implemented as an open-source extension to rl-tools

**Key numbers to include:** coverage %, disaster detection rate, average detection latency, survival rate, number of training steps.

---

## 1. Introduction (≈ 1 page)

### 1.1 Motivation
- Oil platforms and critical infrastructure require persistent 24/7 aerial surveillance
- Stochastic, rare disaster events (leaks, fires) can occur anywhere on the structure and must be detected rapidly
- Battery-limited UAV swarms are the natural solution: cheap, redundant, deployable
- The core difficulty: agents must simultaneously (a) cover the ROI to enable rapid disaster detection, (b) respond to detected disasters by converging on them, and (c) manage individual battery levels by visiting a single charging station
- These three sub-objectives are in constant tension: covering the ROI requires spreading out; detecting disasters requires converging; charging requires leaving both tasks temporarily

### 1.2 Connection to prior work
- Explicitly reference the published underwater monitoring paper:
  *"In prior work [cite], we demonstrated tabular MARL for AUV pipe inspection under limited visibility. That work established that individual rewards with shared observations suffice to produce cooperative behavior, and identified energy management as a key open problem for real deployment. The present work addresses both limitations: we scale to continuous deep MARL and introduce explicit battery management with a single shared charging station."*
- This frames the article as a logical extension, not an isolated contribution

### 1.3 Contributions (bullet list, exactly 4)
1. **Multi-agent SAC actor extension for rl-tools:** a shared-weight per-agent actor architecture (multi_agent_wrapper → permutation buffer → SampleAndSquash) that integrates transparently with the existing SAC training loop via a drop-in config template
2. **Sensor-range coverage formulation:** a nearest-agent assignment reward with hard sensor-range cutoff that outperforms multi-Gaussian coverage shaping by eliminating overlap incentives and providing natural spatial spreading
3. **Systematic reward shaping methodology:** a 10-configuration ablation protocol (A0–A9) that identifies and resolves behavioral pathologies (charger clustering, group charging, hovering exploit, terminal-state battery gaming) through principled root-cause analysis
4. **Asymmetric CTDE with randomized episode length:** privileged critic observations (ground-truth disaster state + remaining episode time) that stabilize Q-value learning under variable episode horizons, following the centralized-training / decentralized-execution paradigm

### 1.4 Paper organization
One sentence per section.

---

## 2. Related Work (≈ 1 page)

### 2.1 Multi-agent reinforcement learning
- **Dec-POMDP framework** [Oliehoek & Amato, 2016]: cooperative multi-agent setting where agents share a reward function but observe only local observations; the non-stationarity of each agent's environment (other agents' policies change during training) is the central difficulty
- **CTDE family:** the standard response to non-stationarity is Centralized Training, Decentralized Execution — a centralized critic conditions on global state and joint actions during training; at deployment each actor sees only its local observation
  - MADDPG [Lowe et al., 2017]: per-agent deterministic actor + centralized critic; continuous actions; no parameter sharing; N separate critics; unstable for N > 3 or sparse rewards
  - QMIX [Rashid et al., 2018]: monotone mixing network over per-agent Q-values; decentralized argmax preserved; discrete actions only — inapplicable to continuous UAV velocity control
  - MAPPO [Yu et al., 2022]: shared actor + centralized value function; on-policy; strong on StarCraft benchmarks; less sample-efficient than SAC in continuous control; already supported in rl-tools
  - MASAC [Iqbal & Sha, 2019]: multi-agent SAC with attention critic; off-policy; continuous actions; Python-only; no high-performance C++ implementation
- **Strictly privileged CTDE (this work):** the critic additionally receives information that is physically unavailable at deployment — ground-truth disaster position/velocity (actors can only observe stochastic detections) and remaining episode time (undefined for real-world missions of unknown duration); this is a stronger asymmetry than standard CTDE where privileged information is at least observable in principle
- **Parameter sharing** [Gupta et al., 2017]: all N agents share one network, differentiated by one-hot agent ID; reduces parameters from O(N·P) to O(P); combined here with relative observation frames for permutation equivariance; agents can learn context-dependent roles (e.g., "I am agent 0, teammate 1 is charging → I should cover") without separate networks

### 2.2 Coverage and patrol with multi-robot systems
- **Lloyd's algorithm** [Cortés et al., 2004]: iterative Voronoi partition moving each agent to the weighted centroid of its cell; provably minimizes the locational coverage cost H = ∫∫φ(q)‖q − pᵢ(q)‖²dq; requires a known density function φ, continuous-time control, and no resource constraints — all violated in this setting
- **Persistent surveillance** [Pasqualetti et al., 2012]: cyclic patrol strategies with worst-case idleness guarantees; optimal for deterministic environments; the Poisson disaster process in this work invalidates any fixed cycle (the optimal response — converging on a detected disaster — breaks the patrol pattern)
- **RL-based coverage:** recent work [cite 2–3 papers] typically uses discrete grids, perfect sensing (no range cutoff), and no resource constraints; reward = fraction of cells visited, which allows overlap exploitation where two agents near the same cell each collect positive reward
- **Sensor-range coverage (this work):** hard sensor-range cutoff enforced in the reward; nearest-agent cell assignment eliminates the overlap incentive; battery-urgency weighting embeds charging coordination directly in the coverage metric; the result is a single shaped reward that naturally produces spatial repulsion, sensor-physics grounding, and charging turn-taking without separate auxiliary terms

### 2.3 Energy-aware multi-UAV coordination
- **Single-robot battery planning:** most approaches treat battery as a hard termination condition or use deterministic return-to-base planning [Nigam & Bieniawski, 2012]; neither approach generalizes to cooperative settings where charging must be coordinated across agents competing for one charger
- **Multi-robot charging coordination:** queuing-theory approaches (M/M/1 models) optimize average wait time but require a fixed arrival distribution — circular when the arrival policy is itself being learned; optimization-based slot scheduling is not adaptive to real-time battery states or disaster events; RL-based charging coordination exists for multi-charger settings but not for single-charger turn-taking under a simultaneous coverage objective [cite]
- **Behavioral pathologies (this work):** charger clustering, group charging, hovering exploit, and terminal-state battery gaming are emergent failure modes specific to the combination of a shared physical resource, multi-objective reward, and finite episode horizon; none of these are documented in non-energy-constrained MARL, and their systematic resolution via a principled ablation methodology is a contribution of this paper

### 2.4 RL frameworks for robotics
- **Python-based (RLlib, SB3, CleanRL):** general-purpose; high per-step overhead from Python interpreter and framework abstractions; inference latency of 1–10 ms limits real-time embedded deployment; RLlib supports MARL but adds significant dependency weight
- **GPU-parallelized simulation (Isaac Gym, MuJoCo MJX):** excellent for massively parallel training; Python interface; not designed for inference on embedded MCUs
- **rl-tools** [Eschmann et al., 2023]: header-only C++17/20; all dispatch resolved at compile time via template structs (zero virtual function overhead); `static constexpr` hyperparameters guarantee binary reproducibility; CPU (MKL/BLAS) and CUDA backends; actor inference < 10 µs on embedded ARM; multi-agent PPO already supported via `multi_agent_wrapper`; SAC loop present but only for single-agent use (`ConfigApproximatorsMLP` treats all observations concatenated as a single vector — no per-agent structure; `SampleAndSquash` layout `[all_μ | all_log σ]` incompatible with `multi_agent_wrapper` per-agent block output `[μᵢ, log σᵢ | ...]` without an intermediate permutation step)
- **This work:** the first multi-agent SAC extension to rl-tools: the permutation buffer that bridges `multi_agent_wrapper` output to `SampleAndSquash` input, `ConfigApproximatorsMLPMultiAgent` as a drop-in config replacement, and the `ActorGradient` forward/backward with gradient routing through the inverse permutation to shared inner MLP weights

---

## 3. The rl-tools Multi-Agent SAC Extension (≈ 1 page)

**Key message:** This section documents the engineering contribution. Other researchers can reproduce and reuse this extension.

### 3.1 rl-tools background
- Header-only C++17/20, policy-typed via template structs (no virtual dispatch, no heap allocation in policy-critical paths)
- Compile-time parameter structs: every hyperparameter is a `static constexpr` value — changes require recompilation, guaranteeing reproducibility
- Single-agent SAC loop: replay buffer → environment step → critic training → actor training → target update; parameterized by `ConfigApproximatorsMLP` which binds network architecture to the loop
- No prior support for multi-agent actors: the existing actor is a single MLP mapping the full concatenated observation to all actions — no per-agent structure

### 3.2 The shared-weight per-agent actor
- Problem: for parameter-sharing cooperative MARL, each agent should run the same network on its own observation block, not a single large network on all observations concatenated
- Solution: `multi_agent_wrapper` — a module that contains one inner MLP and applies it to each agent's observation block (block = `Observation::DIM / N_AGENTS`, already containing the other agents' states under unlimited communication)
- Output: N blocks of dimension `2 * PER_AGENT_ACTION_DIM` (mean and log-std for each agent's action dimensions)
- Weight sharing mechanism: NOT N separate forward passes — the per-agent blocks are reshaped into the batch dimension and the inner MLP runs ONCE over a batch enlarged by N (`INTERNAL_BATCH_SIZE = BATCH * N_AGENTS`); one set of weights processes all agents as batch elements, and the backward pass accumulates the gradient over the enlarged batch into the shared weights

### 3.3 Bridging the wrapper and SampleAndSquash layouts
- Right-size: SampleAndSquash is used UNMODIFIED; what is added is an index permutation (and its exact inverse for the gradient), not a new layer or a clever trick — present it as a reshape/permutation
- SampleAndSquash (SAS) expects input layout: `[μ₁, μ₂, ..., μₙ | log σ₁, log σ₂, ..., log σₙ]`
- multi_agent_wrapper outputs: `[μ₁, log σ₁ | μ₂, log σ₂ | ... | μₙ, log σₙ]` (per-agent blocks)
- Forward permutation P maps wrapper output indices to SAS input indices:
  - `perm_buf[row, a·K + k] = wrapper_out[row, a·2K + k]` (means)
  - `perm_buf[row, N·K + a·K + k] = wrapper_out[row, a·2K + K + k]` (log-stds)
  where K = PER_AGENT_ACTION_DIM, a = agent index
- Backward: inverse permutation Q (exact transpose) routes SAS gradients back to wrapper output gradients
- Implemented in `apply_permutation` / `apply_inverse_permutation` in `per_agent_actor.h`
- The contribution to emphasize is the drop-in `ActorGradient` (§3.4) that composes the unmodified wrapper + SAS into a SAC-compatible actor, plus the deployability (trained per-agent network = the network each drone runs), not the permutation itself

### 3.4 Drop-in integration: ConfigApproximatorsMLPMultiAgent
- Template signature matches `ConfigApproximatorsMLP` exactly: same `Actor<CAPABILITY>` and `Critic<CAPABILITY>` nested types
- Can be passed as `APPROXIMATOR_CONFIG` to `sac::loop::core::Config` without modifying the training loop
- Critic remains a standard single MLP mapping `[ObservationPrivileged ∥ all_actions]` to scalar Q-value

### 3.5 Visualization: the browser-based render function
- Implemented as a JavaScript canvas render function embedded in `operations_cpu.h`
- Rendered elements: 20×20 grid background, ROI outline (platform + 4 pipe arms), drone positions as colored circles with velocity vectors and battery bars, charging/detecting/dead state icons, disaster position + velocity arrow, last-detected position marker, per-step reward display
- Integration: rl-tools emits a WebSocket JSON frame per environment step; the JS function redraws the canvas at ≥30 fps during training
- Role in this work: every behavioral pathology described in Section 5 was first identified visually through this interface before being confirmed with logged metrics — it was the primary debugging tool across 140+ training runs

---

## 4. Environment and Problem Formulation (≈ 1 page)

### 4.1 Physical setup
- 20×20 continuous 2D world (meters), 4 UAV agents (extendable)
- ROI: central platform (4×4 m square) + 4 pipe arms (2 m wide, extending to world boundary) forming a cross/plus shape
- Single charging station at fixed position (5, 5); agents must visit it when battery is low
- Stochastic disaster event: Poisson process (p = 0.01/step, disabled for first DISASTER_MINIMUM_SPAWN_STEP steps); spawns at a random ROI cell with random initial velocity; performs a bounded random walk (±5.7° heading jitter, ±10% speed jitter per step); exits the world if it crosses the boundary

### 4.2 Agent kinematics
- Action: 2D target velocity in [-1, 1]² (scaled to MAX_SPEED = 2 m/s)
- Inertia model: first-order filter α = 0.6 on velocity: `v_next = v + 0.6 * (a * MAX_SPEED - v)`
- Position integrated at DT = 0.05 s; clamped to world boundaries (velocity zeroed on wall contact)

### 4.3 Battery model
- Starts at U[50, 100]% per episode; discharges at 0.15%/step during flight
- Charging: agent must be within CHARGING_STATION_RANGE = 2 m and nearly stationary (|v| ≤ 0.75 m/s) and below MINIMUM_BATTERY_FOR_CHARGING = 80%; charges at 1%/step; locked for MIN_CHARGE_STEPS = 70 steps minimum
- Death: battery reaches 0% → agent is permanently removed; one-time death penalty + ongoing per-step penalty per dead agent

### 4.4 Disaster detection
- Probabilistic: logistic detection probability p_det = σ(β · (SENSOR_RANGE - d)) where d is distance to disaster, β set so p_det = 95% at SENSOR_RANGE - ε_inside
- Per-step Bernoulli draw; any detection sets disaster_detected_global = true and updates last_detected_position
- Disaster is considered detected globally once any agent detects it; reward phase switches from coverage to disaster-detection mode

### 4.5 Observation spaces
**Actor observation (decentralized, per agent):**
- Own state: absolute position (2), velocity (2), is_detecting (1), battery (1), dead (1), is_charging (1) = 8 dims
- Relative disaster: (disaster_pos - own_pos) (2), relative charger: (charger_pos - own_pos) (2), disaster_detected_global flag (1) = 5 dims
- Other agents (N-1 agents × 8 dims each): relative position, relative velocity, battery, dead, is_charging, is_detecting
- One-hot agent ID (N dims) for symmetry breaking
- Total per agent: 8 + 5 + (N-1)×8 + N dims; full observation: N × PER_AGENT_DIM

**Critic observation (privileged, centralized):**
- Per agent: absolute position (2), velocity (2), is_detecting (1), battery (1), dead (1), is_charging (1) = 8 dims × N
- Shared: disaster_active (1), disaster_detected_global (1), disaster absolute position (2), disaster velocity (2), charger absolute position (2), remaining_normalized (1) = 9 dims
- Total: N×8 + 9 dims

**Key design choices:**
- Relative positions for actor: translation invariance, works with randomized charger position
- Relative velocity for other agents: consistent frame with relative position encoding
- One-hot agent ID: breaks the weight-sharing symmetry so agents can learn differentiated roles (first-to-charge, etc.)
- Privileged remaining_normalized for critic only: prevents end-of-episode battery gaming without exposing episode length to the deployed policy

### 4.6 Episode structure
- Randomized length: U[700, 1300] steps per episode (DT=0.05 s → 35–65 s simulated time)
- Terminated (IGNORE_TERMINATION=false): when all agents are dead
- Truncated at episode_step_limit (handled by rl-tools loop with EPISODE_STEP_LIMIT=1300)
- 4 parallel environments (N_ENVIRONMENTS=4)

---

## 5. Reward Design (≈ 1.5 pages)

**Framing:** All reward terms are non-positive (penalty formulation). The maximum achievable reward is 0, corresponding to perfect coverage with all agents alive and fully charged.

### 5.1 Coverage reward (pre-disaster phase)

**Sensor-range coverage formulation:**
- Sweep all GRID_RES² = 400 grid cells; for each ROI cell (platform or pipe), find the nearest alive, non-charging agent within SENSOR_RANGE
- If found, that agent "owns" the cell and contributes `coverage_weight_i` to the total coverage value
- Coverage fraction = total_weighted_coverage / ROI_SIZE (number of ROI cells)
- Fleet coverage value = coverage_fraction × fleet_coverage_capacity (sum of coverage weights)
- Coverage penalty = -β_cover × (fleet_coverage_capacity - fleet_coverage_value)

**Battery-urgency weighting:**
- For each alive agent: `coverage_weight_i = 1 - battery_urgency_i`
- Battery urgency (gated): 0 when battery ≥ 80%; linear ramp from 0→1 as battery drops from 80%→0%
- This means full-battery agents contribute fully to coverage; critically-low agents contribute little (they should be at the charger)
- Charging agents: coverage_value_weight = 0 (excluded from coverage value), but still counted in coverage_capacity (capacity is penalized for agents who are charging)

**Why sensor-range coverage over Gaussian:**
- Hard sensor range cutoff: an agent 6 m from an ROI cell gets zero credit (sensor range = 5 m); a Gaussian gives positive reward even at 10 m
- No overlap boost: with Gaussians, two agents near the same cell each get positive reward; sensor-range coverage assigns the cell to only one, incentivizing spatial separation naturally
- Battery-weighted sensor-range coverage preserves the intended tradeoff: low-battery agents should charge, not cover

### 5.2 Disaster-detection reward (post-detection phase)

- Triggers when disaster_detected_global becomes true (DISASTER_REWARD_SWITCH_ON_DETECTION = true)
- Per agent: Gaussian attraction toward disaster position with σ_event = 10.0 m
  `coverage_potential_i = exp(-dist_to_disaster² / (2 · σ_event²))`
- Fleet disaster value = Σ coverage_value_weight_i × coverage_potential_i
- Disaster penalty = -β_event × (fleet_coverage_capacity - fleet_disaster_value)
- During disaster phase, abandonment penalty = -0.5/step when disaster is known but no agent is currently detecting it

**Why the reward switches on detection, not on disaster spawn:**
- Before detection, agents cannot know where the disaster is
- Switching on global detection (any agent detects) allows all agents to benefit from the signal, even those far away who cannot yet detect it themselves

### 5.3 Charging reward

**Gated spatial shaping (CHARGING_OBJECTIVE_VARIANT = 0):**
- Battery urgency gates the charging term: no pull when battery ≥ 80% threshold
- Below threshold: urgency = (0.80 - battery_normalized) / 0.80 (linear ramp)
- Charging weight = battery_urgency; coverage weight = 1 - battery_urgency (symmetric decomposition summing to 1)
- If agent is actively charging: fleet_charging_value += charging_weight (full contribution)
- If agent is not charging but urgency > 0 and not in disaster phase: spatial proximity shaping toward charger
  `fleet_charging_value += charging_weight × Gaussian(dist_to_charger, σ_charging=5.0) × 0.3`
- Charging penalty = -β_charging × (fleet_charging_need - fleet_charging_value)

**Key design rationale:**
- Symmetric weights: coverage and charging share the same "budget"; an agent cannot be counted fully for both simultaneously
- Gate prevents constant pull toward charger during coverage phase (charger clustering pathology)
- CHARGING_SHAPING_SCALE = 0.3 creates a gap between hovering outside the charging zone and actually docking (prevents hovering exploit)
- σ_charging = 5.0 < σ_event = 10.0: intentional asymmetry; at critical battery (urgency ≈ 0.75–0.9) the charging pull still dominates even near the disaster

### 5.4 Coordination penalties

**Charger occupancy penalty:**
- If more than 1 agent is simultaneously charging: penalty = -0.3 × (charging_count - 1) per step
- Encourages turn-taking at the charger rather than group visits
- Beta = 0.3: calibrated so that emergency charging (battery < 45%) overrides the penalty but convenient early charging does not

**Abandonment penalty:**
- Active when: disaster is known (detected_global = true) AND no agent is currently detecting it AND disaster is still active
- Penalty = -0.5/step
- Prevents agents from "abandoning" a detected disaster — they must maintain at least one observer

**Death penalties:**
- One-time death penalty = -10.0 per agent that dies
- Ongoing staffing penalty = -1.0/step per dead agent (prevents strategic self-sacrifice)

**Overlap repulsion (inactive in final config):**
- Gaussian repulsion between agent pairs: exp(-dist²/ρ²); inactive (OVERLAP_REPULSION_ACTIVE = false)
- Sensor-range coverage provides natural repulsion; explicit repulsion was redundant

### 5.5 Total reward
```
R = coverage_penalty + charging_penalty + charger_occupancy_penalty
  + abandonment_penalty + death_penalty + ongoing_death_penalty
```
All terms ≤ 0; optimal = 0.

### 5.6 Reward shaping development (brief)
Reference the ablation table in Section 7. Key steps:
1. Battery-urgency gate (A2): resolved charger clustering
2. Occupancy penalty + scale reduction (A3): resolved group charging and hovering exploit
3. Equal-sigma expansion → asymmetric sigma (A4 → A9): resolved agents unable to leave disaster when battery critical
4. Relative inter-agent velocity (A5): improved inter-agent coordination
5. Randomized episode length + privileged remaining time (A6 + A7): resolved end-of-episode battery gaming and critic instability

---

## 6. Learning Architecture (≈ 0.75 page)

### 6.1 SAC configuration
- Batch size: 512 (actor and critic)
- Replay buffer: 1M transitions
- Training interval: every 4 environment steps (critic: 1×, actor: 2×, target update: 1×)
- Discount: γ = 0.99
- Target entropy: -6 (= -ACTION_DIM/2 approximately)
- Warmup: 5000 steps for critic, 5000 steps for actor

### 6.2 Actor network
- Inner MLP: per-agent observation block → 128 → 128 → 2 × PER_AGENT_ACTION_DIM, ReLU activations, identity output
- Wrapped by multi_agent_wrapper: per-agent blocks reshaped into the batch dimension, inner MLP run once over the batch enlarged by N_AGENTS
- Output routed through permutation buffer → SampleAndSquash (tanh squash + adaptive temperature α)
- Adam optimizer: α_lr = 3×10⁻⁴
- Parameter count is independent of N_AGENTS (one shared inner MLP, reused over the enlarged batch rather than replicated)

### 6.3 Critic network
- Single MLP: (ObservationPrivileged::DIM + ACTION_DIM) → 256 → 256 → 1, ReLU, identity output
- Twin critics (standard SAC); target network with soft update
- Adam optimizer: α_lr = 3×10⁻⁴
- Input: concatenation of privileged observation (absolute positions, ground-truth disaster state, remaining_normalized) and all agents' actions

### 6.4 CTDE guarantees
- Actor is deployed without the critic; it only observes its own local observation block at execution time
- remaining_normalized is in privileged observation only → deployed policy is oblivious to episode length
- Disaster ground-truth position and velocity are in privileged observation only → deployed policy relies solely on detection signal

---

## 7. Experiments (≈ 2 pages)

### 7.1 Training setup
- Total training: 15M loop steps = 60M environment steps (4 parallel envs)
- Hardware: [fill in: CPU/GPU, wall-clock time]
- Seeds: [fill in number of seeds] per configuration
- Evaluation: fixed 1300-step episodes, no training updates, deterministic initial state

### 7.2 Metrics
| Metric | Definition | Notes |
|---|---|---|
| `return/mean` | Mean episode return | Primary policy quality metric |
| `coverage/priority_area` | % of ROI cells covered at any step (averaged over episode, coverage phase only) | |
| `disaster/detection_rate` | Fraction of disasters detected before exit | |
| `disaster/avg_detection_latency` | Mean steps from spawn to first detection | Lower is better |
| `agents/survival_rate` | 1 - death_count / (N_AGENTS × episodes) | |
| `charging/efficiency` | appropriate_sessions / total_sessions | appropriate = battery < 50% at start |
| `share_terminated` | Fraction of episodes ending in death (vs step limit) | Lower is better |

### 7.3 Ablation table
One row per configuration, columns = metrics. Rows to include:

| Config | Coverage | Detection Rate | Latency | Survival | Efficiency |
|---|---|---|---|---|---|
| No battery (baseline) | | | N/A | 100% | N/A |
| Battery, no gate (clustering) | | | | | |
| A2: + Battery-urgency gate | | | | | |
| A3: + Occupancy penalty + scale | | | | | |
| A4: + Equal-sigma expansion | | | | | |
| A5: + Relative inter-agent velocity | | | | | |
| A6: + Randomized episode length | | | | | |
| A7: + remaining_normalized in critic | | | | | |
| A8: + Full privileged critic | | | | | |
| **A9: + Asymmetric sigma (best config)** | | | | | |

### 7.4 Behavioral analysis
- Show representative trajectory plots from render function for best config:
  - Coverage phase: agents spread across ROI, one at charger at any time
  - Disaster detection: agents converge toward disaster, one observer maintained
  - Charging during disaster: low-battery agent leaves disaster, returns after full charge
- Show trajectory plots for key pathologies (before fix):
  - Charger clustering: all agents hover near charger
  - Group charging: all agents visit charger simultaneously
  - End-of-episode battery gaming: agents stop charging near episode end

### 7.5 Comparison with PPO baseline
- PPO with multi-agent wrapper and identical observation/reward (config. B1)
- Compare on: convergence speed, final coverage, detection rate, stability
- Expected finding: SAC shows better sample efficiency for continuous action spaces

### 7.6 Scaling analysis (optional, if space allows)
- N = 2, 3, 4 agents on same ROI
- How does coverage scale? How does detection latency change?

---

## 8. Results and Discussion (≈ 0.75 page)

### 8.1 Key findings
- Best configuration achieves X% ROI coverage, Y% disaster detection rate, Z steps average detection latency, W% survival rate
- The battery-urgency gate (A2) is the single most important reward change: without it, charger clustering prevents the coverage objective from being met at all
- The privileged critic with remaining_normalized (A7–A8) eliminates critic loss spikes and stabilizes training; without it, Q-values are inconsistent across episodes of different length
- Sensor-range coverage is strictly better than multi-Gaussian: it provides a natural repulsion mechanism without overlap incentives, and the hard sensor-range cutoff prevents "phantom coverage" from agents far from the ROI

### 8.2 Limitations
- Single charger: with multiple chargers the turn-taking problem changes qualitatively; the occupancy penalty and charging gate would need retuning
- Shared-weight actor: agents cannot learn permanently differentiated roles (e.g., a dedicated scout); one-hot ID partially mitigates this but full role differentiation requires separate networks
- Fixed ROI geometry: the multi-Gaussian catalogue and ROI mask are compile-time constants; generalizing to arbitrary geometries requires runtime computation
- Unlimited communication range assumed: all agents' states are fully observable by all other agents at all times; real deployments would require explicit bandwidth-limited communication protocols; some coordination failures (simultaneous critical battery) could be mitigated with explicit signaling (e.g. "I'm going to charge") even under constrained bandwidth

### 8.3 Future work
- Recurrent actor (LSTM/GRU) for memory of last-known disaster position
- Multi-charger coordination with dynamic charger assignment
- Sim-to-real: the continuous action space and inertia model are designed to be physically plausible; direct transfer experiments are a natural next step
- Heterogeneous roles: separate actor networks per role (scout, responder, charger)

---

## 9. Conclusion (≈ 0.5 page)

- Summarize the three contributions: multi-agent SAC extension to rl-tools, sensor-range coverage reward, systematic shaping methodology
- State the key empirical finding in one sentence
- Connect back to the prior underwater work: the energy management challenge flagged in [cite] is now addressed in a fundamentally harder setting
- Point to open-source availability

---

## Figures Checklist

- [ ] Fig. 1: Environment overview — annotated screenshot from render function showing ROI (platform + 4 pipes), charging station, 4 drones, disaster, grid
- [ ] Fig. 2: Multi-agent actor architecture diagram — multi_agent_wrapper → permutation buffer → SampleAndSquash, showing data flow for N=3 agents
- [ ] Fig. 3: Reward function diagram — visual decomposition of coverage penalty, charging penalty, disaster penalty, showing how battery urgency gates/weights them
- [ ] Fig. 4: Ablation learning curves — return/mean and coverage vs. training steps for all ablation configs (or a subset showing key steps)
- [ ] Fig. 5: Trajectory visualization — 4 panels: (a) best-config coverage phase, (b) best-config disaster response, (c) charger clustering pathology, (d) group charging pathology
- [ ] Fig. 6: Behavioral metrics table or bar chart — final performance of all ablation configs on all metrics

---

## Key Equations to Include

1. Sensor-range coverage reward (sum over ROI cells, nearest-agent assignment within sensor range, battery weighting)
2. Battery urgency and symmetric weight decomposition
3. Coverage penalty formulation (fleet_capacity - fleet_value)
4. Logistic detection probability
5. Disaster Gaussian attraction (coverage_potential during disaster phase)
6. Permutation forward/backward (P and Q matrices)
7. SAC objective (standard, just cite Haarnoja et al.)

---

## References (target ≈ 30–40)

**Must cite:**
- Haarnoja et al. 2018 (SAC)
- Lowe et al. 2017 (MADDPG)
- Rashid et al. 2018 (QMIX)
- Yu et al. 2022 (MAPPO)
- Iqbal & Sha 2019 (MASAC)
- Sunehag et al. 2018 (VDN — for completeness in value-based CTDE survey)
- Cortés et al. 2004 (Lloyd's algorithm / Voronoi coverage control)
- Du et al. 1999 (centroidal Voronoi tessellations — theoretical basis for Lloyd's)
- Pasqualetti et al. 2012 (persistent surveillance / cyclic patrol)
- Nigam & Bieniawski 2012 (energy-aware UAV patrol)
- Gupta et al. 2017 (parameter sharing in cooperative MARL)
- Oliehoek & Amato 2016 (Dec-POMDP textbook)
- rl-tools paper/repo [Eschmann et al. 2023]
- Your own underwater paper [Luvisutto et al. 2025, Expert Systems With Applications]
- Sutton & Barto (RL textbook)

**Fill in from related work survey:**
- RL-based coverage (2–3 recent papers without sensor-range constraints)
- Multi-UAV target search with RL (Ye et al. 2020 or equivalent)
- Multi-robot charging coordination (queuing-theory or RL, 1–2 papers)
- CTDE with privileged information beyond standard CTDE (1–2 papers if found)
- Reward shaping in MARL (Mannion et al. 2017 already in bibliography; add 1 more if available)
