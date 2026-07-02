# PhD Thesis — Outline and Writing Guide

**Provisional title:**
*From Tabular to Deep Multi-Agent Reinforcement Learning for Autonomous Monitoring: Battery-Constrained UAV Surveillance of Critical Infrastructure*

**Thesis type:** PhD thesis (Cotutelle or single institution, to be confirmed)
**Estimated length:** 100–130 pages

---

## Terminology Constraints (Hard Rules — Do Not Deviate)

These decisions are final and must be applied consistently in all writing, figures, and future prompts.

### Coverage reward naming
**The coverage reward implemented in this thesis is called "sensor-range coverage" — NOT "Voronoi coverage", "sensing-constrained Voronoi", or any other Voronoi-based term.**

Rationale: the implementation sweeps ROI cells and assigns each to the nearest alive agent *within SENSOR_RANGE*; cells beyond all agents' sensor range are simply uncovered. This is **not a Voronoi partition** — a true Voronoi partition assigns every point in space to its nearest agent with no distance cutoff, guaranteeing full coverage. The hard sensor-range cutoff breaks the partition property: cells can be unowned. Using "Voronoi" would mislead any reader familiar with Voronoi coverage theory.

Correct usage:
- **"sensor-range coverage reward"** — the named contribution
- **"nearest-agent assignment within SENSOR_RANGE"** — when describing the mechanism precisely
- **"Lloyd's algorithm / Voronoi coverage control"** — only when referring to the classical geometric method (Cortés et al., 2004) in §2.2 for comparison; never for our implementation

Incorrect usage (never use for our implementation):
- ~~sensing-constrained Voronoi~~
- ~~Voronoi coverage~~
- ~~Voronoi-based reward~~
- ~~sensing-constrained coverage~~ (ambiguous)

Code parameters (`USE_VORONOI_COVERAGE`, `VORONOI_VARIANCE_PENALTY_WEIGHT`) are legacy identifiers in the C++ source and may appear as such when referencing the code, but the reward itself is always called "sensor-range coverage" in prose.

---

## Writing Order and Practical Guide

### Current status (June 2026)

| Chapter / Section | Status |
|---|---|
| Ch. 5 §5.1–5.8 (environment + reward) | **Done** (~11 pp) |
| Ch. 5 §5.9 (metrics tracking) | **Next** (~1–2 pp) |
| Ch. 4 — rl-tools Extension | Not started |
| Appendices A, B, C, D | Not started |
| Ch. 3 — Preparatory Works | Not started |
| Ch. 6 §6.1–6.2 (protocol + configs) | Can be written now |
| Ch. 6 §6.3–6.10 (results + discussion) | Blocked on experiments |
| Ch. 2 — State of the Art | Not started |
| Ch. 7 — Conclusion | Not started |
| Ch. 1 — Introduction | Not started |
| Abstract + Front Matter | Not started |

### Pending edits to already-written chapters (do not lose)

Corrections to text that is already drafted (Ch. 3 §3.1–3.2, Ch. 4, Ch. 5). Verify each against the current code before editing.

- **[Ch. 4 / cross-refs] Deep-model learning algorithm asymmetry.** Ch. 4 and Ch. 5 name the SAC replay buffer, entropy objective, and adaptive temperature but give no update equations or hyperparameter values, whereas Ch. 3 §3.2.3 fully specifies Expected-SARSA. Once §2.1.6 (SAC mechanics) and §6.1.1 (hyperparameter table) are written, add cross-references from Ch. 4 so the deep model is specified at the same altitude.
- ~~**[Ch. 5 §5.2–5.6] ×10 spatial/velocity rescaling.**~~ Done. Updated world size (200 m), platform side (40 m) / pipe width (20 m), grid cell size (10 m), charger position (50, 50), docking radius (20 m), docking speed threshold (7.5 m/s), sensor range $R$ (50 m) and detection margin $\varepsilon$ (5 m, so $R-\varepsilon=45$ m), σ_chg (50 m), σ_dis (100 m), disaster $v_{\max}$ (10 m/s). Note: `oil_platform.h` currently has `GAUSS_SIGMA_EVENT = 50.0`, inconsistent with the other ×10 factors and with the 100 m reported in the text; per instruction the text reports 100 m regardless — recheck against the code once that value is settled. Ch. 4 has no numeric spatial/velocity values, so nothing there needed changing.
- **[Ch. 5 §5.4] New battery discharge model — code corrected, retrain before writing.** Discharge is no longer a constant 0.15%/step. It first went through a velocity-proportional variant (rate scaled with current speed) that turned out not to match intent: cruising at high speed drained fastest and hovering drained slowest, whereas the goal is that only accelerating/braking/turning should cost extra energy, not maintaining speed. Code now has both: `VELOCITY_PROPORTIONAL_DISCHARGE` (kept, disabled, for ablations) and the new `ACCELERATION_PROPORTIONAL_DISCHARGE` (enabled by default), which scales the rate by `|v_next - v_prev|` normalized by `2 * VELOCITY_FILTER_ALPHA * MAX_SPEED`; see the comments in `oil_platform.h` for the exact formula. The two flags are mutually exclusive (`static_assert`-checked). No policy has been trained with this model yet — rerun training before writing the Ch. 5 §5.4 discharge description or citing any resulting behavior/metrics, and drop the "velocity-dependent discharge is future work" caveat since a form of it is now implemented.
- **[Ch. 5, new subsection] Collision model.** A collision model is being tested and appears to work well. If adopted, add a subsection (e.g., §5.x) describing it, and reflect it in the observation and/or reward sections as needed. The current draft states the environment has no collisions.

### Recommended chapter order

Write in this order — not chapter order. The logic: start with factual/descriptive chapters grounded in code, build toward argumentative chapters, leave framing chapters for last.

| Priority | Chapter | Why this position |
|---|---|---|
| **Done** | ~~Ch. 5 §5.1–5.8 — Environment and Reward~~ | ~~Purely descriptive; grounded in code~~ |
| **Next** | **Ch. 5 §5.9 — Metrics tracking** | One short section to close Ch. 5; write while the reward design is fresh |
| 1st | **Ch. 4 — rl-tools Extension** | Code-grounded; write 4.1–4.4 (architecture) while Ch. 5 is fresh; 4.5 (render function) is narrative and can come after |
| 2nd | **Appendices A, B, C, D** | Write immediately alongside Ch. 4 — they are the reference versions of the same content; Appendix D documents discarded reward formulations already described in Ch. 5; costs almost nothing while the code is open |
| 3rd | **Ch. 3 — Preparatory Works** | Mostly adapting the published paper; write after you have momentum; the bridge sections (3.3, 3.4) are the hardest and require Ch. 4 and Ch. 5 to already exist in draft |
| 4th | **Ch. 6 §6.1–6.2** | Protocol and ablation configurations can be written now without results |
| 5th | **Ch. 6 §6.3–6.10** | Results and discussion; write once experiments are complete; use `[FILL: X%]` placeholders until then |
| 6th | **Ch. 2 — State of the Art** | Write after the contribution chapters exist in draft; easier to know what to position against once you know exactly what you contributed; each section ends with a gap statement derived from the actual contribution |
| 7th | **Ch. 7 — Conclusion** | §7.2 (RQ answers) requires Ch. 6 results; §7.1 and §7.3–7.4 can be drafted earlier |
| 8th | **Ch. 1 — Introduction** | Write last; the contributions list, research questions, and roadmap can only be final once all chapters exist |
| Last | **Abstract + Front Matter** | After everything; one focused session |

### Within each chapter

- **Write your own rough draft first**, even bullet points. Do not ask Claude to generate the section.
- Use a new Claude session per section (not per chapter). Bring: the relevant outline subsection + any code snippets needed + your draft.
- **Sections to write in a single sitting** (cohesive arguments that lose quality if interrupted): 5.3, 5.4, 5.5, 5.8 (reward function — done), 2.6 (research gap synthesis), 7.2 (RQ answers), 7.1 (energy thread).
- **Sections that can be written in fragments** (factual enumeration): 4.3, 4.4, 4.5, 3.2.1, 3.2.2, Appendices A–D.

### Dependencies (what blocks what)

- Ch. 5 is done; it unblocks Ch. 4, Appendix D, Ch. 3, and Ch. 6
- Ch. 4 unblocks Appendices A, B, C and completes the dependency for Ch. 3
- Ch. 3 requires Ch. 4 and Ch. 5 in draft (bridge sections)
- Ch. 6 requires Ch. 5 and completed experiments
- Ch. 7 §7.2 requires Ch. 6 results
- Ch. 2 and Ch. 1 require all contribution chapters in draft
- Abstract requires everything

### What to do if experimental results are not yet final

Do not block writing on unfinished results. Write everything except the tables and plots in Ch. 6 (Results). Use `[FILL: X%]` placeholders for numbers. The prose structure of Ch. 6 can be complete and correct before the final values are known.

---

## Front Matter

- Title page
- Abstract (English + Italian/French depending on institution requirements): 300–400 words
- Acknowledgements
- Table of contents
- List of figures
- List of tables
- List of acronyms/abbreviations: RL, MARL, CTDE, SAC, PPO, UAV, AUV, ROI, POMDP, Dec-POMDP, MDP, CTDE, etc.

---

## Chapter 1 — Introduction (≈ 10–12 pages)

### 1.1 The monitoring problem in safety-critical infrastructure (≈ 2 pages)
- Industrial context: oil platforms, chemical plants, infrastructure require 24/7 monitoring for safety events (leaks, fires, structural failures)
- Current practice: fixed sensor networks + periodic human inspection; both have coverage gaps and high cost
- The opportunity: autonomous aerial vehicles (UAVs) as persistent, mobile, low-cost sensor platforms
- The key challenge: unlike fixed sensors, UAVs have finite energy and must manage battery constraints while performing their monitoring task
- This thesis addresses: how to design and train a fleet of battery-constrained UAVs to jointly cover a structured region of interest and detect stochastic rare events, using multi-agent reinforcement learning

### 1.2 Why multi-agent reinforcement learning (≈ 2 pages)
- Classical control and optimization: require explicit system models; poor robustness to environment uncertainty and stochastic events
- Single-agent RL: cannot scale to multi-robot coordination without combinatorial action spaces
- MARL: allows agents to learn cooperative strategies through interaction; naturally handles partial observability and non-stationarity
- The specific difficulty of this problem: three competing objectives (coverage, disaster response, battery management) that are not easily decomposable; reward shaping is non-trivial
- RL allows reward design to encode the objectives, with agents discovering coordination strategies that no explicit algorithm could easily specify

### 1.3 Research questions (≈ 1 page)
Explicitly state the research questions the thesis answers:
1. **RQ1:** Can tabular MARL produce cooperative monitoring behavior from individual rewards alone, without explicit cooperation incentives?
2. **RQ2:** How should a single-agent deep RL framework (rl-tools) be extended to support multi-agent continuous control with parameter sharing?
3. **RQ3:** What reward formulation allows a fleet of battery-constrained UAVs to jointly optimize spatial coverage, stochastic event detection, and energy management?
4. **RQ4:** What behavioral pathologies arise in multi-objective MARL, and how can they be systematically diagnosed and resolved through reward shaping?
5. **RQ5:** How does asymmetric information between actor and critic (CTDE paradigm) affect learning stability in environments with variable episode length?

### 1.4 Contributions summary (≈ 1.5 pages)
Bulleted list, one paragraph per contribution, grouped into two phases:

**Phase 1 — Field scoping (survey contributions, Section 2.1):**
- **C0a:** Survey of underwater robotic swarm projects, challenges, and perspectives: taxonomy of current platforms, communication modalities, and open problems; identifies energy autonomy and recharging coordination as the primary unsolved challenge for real-world deployment [Luvisutto et al., OCEANS/IEEE 2022]
- **C0b:** Survey of mission planning algorithms for multi-AUV systems: taxonomy of centralized (optimization, algebraic geometry) vs. decentralized (RL, GNN, evolutionary computation) approaches; identifies CTDE RL as the most promising paradigm for dynamic, communication-constrained underwater environments [Luvisutto et al., OCEANS 2022]

**Phase 2 — Technical contributions (Chapters 3–6):**
- **C1 (Chapter 3):** Tabular MARL framework for AUV pipe inspection; first application of MARL to underwater pipeline monitoring; demonstration that cooperation emerges from individual rewards without explicit cooperation incentives; published in Expert Systems With Applications [cite]
- **C2 (Chapter 4):** Multi-agent SAC extension for rl-tools: shared-weight per-agent actor architecture (multi_agent_wrapper + permutation buffer + SampleAndSquash), drop-in config template, browser-based trajectory visualization system
- **C3 (Chapter 5, §5.1–5.8):** Oil platform surveillance environment: physically plausible UAV kinematics, stochastic disaster dynamics, battery model with charging station, sensor-range coverage reward, compile-time parameterization
- **C4 (Chapter 5, §5.8):** Systematic reward engineering methodology: 10-configuration ablation protocol identifying and resolving behavioral pathologies (charger clustering, group charging, hovering exploit, terminal-state battery gaming); asymmetric CTDE with privileged remaining episode time
- **C5 (Chapter 6):** Experimental validation: comparative evaluation against ablated configurations; behavioral analysis linking learned policies to reward design decisions

### 1.5 Thesis roadmap (≈ 0.5 pages)
One paragraph per chapter, explaining what it contains and why it appears in that position.

---

## Chapter 2 — Background and State of the Art (≈ 12–15 pages)

**Purpose:** Establish the technical context for the thesis contributions. Each section ends with a positioning paragraph that identifies what existing work does not provide and what this thesis contributes in that gap. This chapter is distinct from Chapter 3 (which covers the two survey papers C0a, C0b as first-person research contributions, not as background).

---

### 2.1 Multi-Agent Reinforcement Learning (≈ 4 pages)

#### 2.1.1 The Dec-POMDP framework
- Formal definition: tuple (N, S, {Aᵢ}, {Oᵢ}, T, R, Z, γ)
  - N agents; joint state S; per-agent action sets Aᵢ; per-agent observation sets Oᵢ; transition function T: S × A → Δ(S); shared reward R: S × A → ℝ (cooperative setting); observation function Z: S × A → Δ(O); discount γ ∈ [0,1)
- Three structural difficulties:
  1. **Partial observability:** agent i sees oᵢ ~ Z(·|s,a) only; no agent observes the full joint state s at execution time
  2. **Non-stationarity:** from agent i's perspective the environment is non-stationary because other agents' policies π_{-i} change during training
  3. **Exponential joint space:** |A| = Πᵢ|Aᵢ|; joint planning is intractable for large N
- Cooperative setting (this thesis): all agents maximize the same shared cumulative discounted return ΣₜγᵗR(sₜ,aₜ); no competitive term; the reward function is a scalar evaluated over the entire joint state and joint action, not per-agent decomposed
- This thesis: the Dec-POMDP is not used as a solved formulation but as the problem statement that motivates CTDE; the solution approach is SAC with a centralized critic

#### 2.1.2 Centralized Training, Decentralized Execution (CTDE)
- Origin: Lowe et al. (MADDPG, 2017); formalized in Oliehoek & Amato (Dec-POMDP textbook, 2016)
- Core idea: allow the critic (or mixing network) to condition on global state s and joint actions a₁,...,aₙ during training; at deployment each actor πᵢ observes only its local observation oᵢ
- Why CTDE resolves non-stationarity: the centralized critic receives all agents' actions simultaneously → Bellman target is a function of (s, a₁,...,aₙ) which is stationary given other policies; the actor gradient is computed through the centralized Q function, giving a correct signal even under joint non-stationarity
- Strictly privileged CTDE (this thesis, §5.7.2): the critic additionally receives ground-truth disaster position and velocity (unavailable to deployed actors who must rely on stochastic detection) and remaining episode time (undefined for real-world deployments of unknown duration); this is a stronger asymmetry than standard CTDE where privileged information is at least observable in principle

#### 2.1.3 Value-based CTDE methods
- **VDN** (Sunehag et al., 2018): Q_total = Σᵢ Qᵢ; linear utility decomposition; decentralized execution via per-agent argmax; weak representational capacity — cannot model agents whose optimal action depends on other agents' actions in a non-linear way
- **QMIX** (Rashid et al., 2018): Q_total = f(Q₁,...,Qₙ; s) with monotone constraint ∂Q_total/∂Qᵢ ≥ 0; allows richer interaction patterns while preserving decentralized argmax; state-of-the-art on StarCraft Multi-Agent Challenge; **discrete actions only**
- **QTRAN** (Son et al., 2019): drops monotonicity constraint for full generality; higher computational cost; rarely used in practice; still discrete actions
- Limitation for this thesis: all value-based methods require discrete action spaces; UAV velocity control requires continuous 2D actions → value-based CTDE is inapplicable without action-space discretization, which would coarsen the control

#### 2.1.4 Policy-gradient CTDE methods
- **MADDPG** (Lowe et al., 2017): per-agent deterministic actor πᵢ(oᵢ) → aᵢ; per-agent centralized critic Qᵢ(s, a₁,...,aₙ) → ℝ; continuous actions; N separate critic networks; no entropy regularization; pioneering work but unstable in practice for N > 3 or sparse rewards
- **MAPPO** (Yu et al., 2022): shared actor policy, centralized value function V(s); on-policy (PPO clip objective); strong on discrete-action tasks (SMAC); rl-tools already supports MAPPO via `multi_agent_wrapper`; **limitation:** on-policy → less sample-efficient than off-policy SAC in continuous control; known sensitivity to PPO hyperparameters (clip ratio, GAE λ) in continuous settings
- **MASAC** (Iqbal & Sha, 2019): multi-agent soft actor-critic with attention over other agents' observations in the critic; off-policy → reuse experience; continuous actions; entropy-regularized objective → stable exploration; conceptually the closest prior work to this thesis; **not available in any high-performance C++ framework**; designed for competitive/mixed settings; attention adds complexity not needed in fully cooperative homogeneous settings

#### 2.1.5 Parameter sharing
- Standard technique for cooperative homogeneous MARL: all N agents share one network, differentiated by a per-agent ID input (one-hot encoding or learned embedding) [Gupta et al., 2017; Terry et al., 2021]
- Benefits: parameter count O(P) instead of O(N·P); natural permutation equivariance when combined with relative observation frames; easier generalization to different N at test time without retraining
- Mechanism in this thesis (§4.2): `multi_agent_wrapper` applies the inner MLP independently to each agent's observation block and accumulates gradients into shared weights — equivalent to batching N agents but without changing the inner module's interface
- Limitation: cannot learn permanently differentiated roles; one-hot ID partially compensates by allowing the network to condition on identity (e.g., "I am agent 0, and the observation suggests agent 1 is charging → I should not charge now"); full role differentiation requires separate networks (future work, §7.4)

#### 2.1.6 Soft Actor-Critic (the base algorithm)
Purpose: give single-agent SAC at the altitude of §3.2.3's Expected-SARSA, so the thesis specifies the learning rule for both the tabular and the deep model. The multi-agent extension is Chapter 4; the privileged centralized critic is Chapter 5 §5.7.2; hyperparameter values are Chapter 6 §6.1.1. (Consider placing this before §2.1.4, since MASAC builds on it.)
- Entropy-regularized objective: maximize E[Σₜ γᵗ (rₜ + α·H(π(·|sₜ)))]; the temperature α trades reward against policy entropy
- Soft critic target: y = r + γ (min_{j=1,2} Q̄ⱼ(s′,ã′) − α log π(ã′|s′)), with ã′ ~ π(·|s′); twin critics (clipped double-Q) reduce overestimation; target networks Q̄ updated by Polyak averaging (τ)
- Actor loss: minimize E[α log π(a|s) − min_j Qⱼ(s,a)] with a reparameterized, tanh-squashed Gaussian (the SampleAndSquash layer, §4.1.2)
- Automatic temperature tuning: α adjusted toward a target entropy
- Off-policy: a replay buffer stores past transitions; updates sample minibatches uniformly, giving the sample efficiency that motivates SAC over on-policy MAPPO (§4.1.3)

**Positioning:** Existing CTDE algorithms either (a) require discrete actions (QMIX, VDN), (b) use N separate critics with no privileged asymmetry (MADDPG), (c) are available only in Python with no embedded deployment path (MASAC, MAPPO), or (d) support MARL but not SAC in C++ (rl-tools pre-this-work). Chapter 4 fills the gap: a multi-agent SAC with strictly privileged critic observations, implemented in rl-tools for high-performance training and embedded deployment.

---

### 2.2 Multi-Robot Coverage Control (≈ 3 pages)

#### 2.2.1 Classical geometric coverage
- **Lloyd's algorithm** (Du et al., 1999; Cortés et al., 2004): iterative Voronoi partition where each agent moves to the weighted centroid of its Voronoi cell; minimizes the locational cost H = ∫∫_Q φ(q)·‖q − pᵢ(q)‖² dq where pᵢ(q) is the nearest agent to point q and φ(q) is a known density function
  - Convergence: provably converges to a local minimum of H under mild conditions on φ
  - Assumptions violated in this thesis: known density function (φ is implicit in ROI geometry); continuous-time control law (discrete 20 Hz agents); no resource constraints (battery forces agents to temporarily leave coverage); centralized Voronoi computation or inter-agent boundary negotiation
- **Art Gallery theorem** (Chvátal, 1975): minimum static coverage of a polygon requires ⌊n/3⌋ guards; bounds static guard placement, not dynamic control
- This thesis uses sensor-range coverage as a *reward*, not as a control law — agents learn to approximate Lloyd's optimum implicitly through gradient descent on the shaped reward, without computing Voronoi boundaries explicitly

#### 2.2.2 Persistent surveillance and patrol scheduling
- **Cyclic strategies** (Pasqualetti et al., 2012): graph abstraction of the environment; agents follow cyclic routes; worst-case idleness minimized; proved optimal for deterministic environments; centralized
  - Limitation: optimal cycle assumes no stochastic events; a Poisson disaster process invalidates any fixed patrol schedule because the optimal response (converging on a detected disaster) breaks the cycle
- **Multi-robot patrol** (Chevaleyre, 2004; Portugal & Rocha, 2011): idleness minimization; surveyed strategies include cyclic, random, and partitioned approaches; assumes equal robot speeds and no energy constraints; no response to rare events
- **Energy-aware patrol** (Nigam & Bieniawski, 2012): single-UAV battery-limited patrol; deterministic point-to-base return planning to reserve fuel for return trip; no multi-agent coordination; no stochastic event response; the per-agent planning approach does not generalize to cooperative multi-agent settings

#### 2.2.3 RL-based coverage
- Recent RL coverage work typically assumes: discrete grid world with perfect sensing, reward = fraction of unique cells visited, single agent or small N without resource constraints, no dynamic events requiring response [cite 2–3 recent papers]
- **What sensor-range coverage adds:** hard sensor-range cutoff enforced in the reward (an agent beyond SENSOR_RANGE = 5 m gets zero credit for a cell — physically grounded); nearest-agent assignment prevents the overlap incentive present in Gaussian formulations where two agents near the same cell each collect positive reward; battery-urgency weighting embeds resource management directly in the coverage metric; result: a single shaped reward naturally produces spatial repulsion, sensor-range awareness, and charging coordination without separate auxiliary terms for each behavior

#### 2.2.4 Coverage with energy constraints
- Most RL coverage papers treat battery as a hard termination condition (episode ends at battery = 0), not as a continuous shaped reward component
- The battery-urgency weighting introduced in this thesis (§5.10.2): `coverage_weight_i = 1 - urgency_i`, with urgency ramping linearly from 0→1 as battery drops below 80%; this reduces an agent's effective coverage capacity continuously as energy depletes — providing a gradient toward charging rather than a cliff at death
- Symmetric decomposition (coverage_weight + charging_weight = 1): no agent can be counted fully for both coverage and charging simultaneously; prevents the double-reward exploit where an agent near both the charger and the ROI collects both benefits at once

**Positioning:** Lloyd's algorithm and geometric coverage control optimize known density functions under perfect sensing with no resource constraints. Patrol scheduling is optimal for deterministic targets but fragile under stochastic events. Existing RL coverage neglects sensor physics and battery management. The sensor-range coverage formulation in §5.10.2 fills all three gaps simultaneously.

---

### 2.3 Multi-UAV Surveillance and Monitoring (≈ 2 pages)

#### 2.3.1 Fixed sensor networks vs. mobile aerial agents
- Fixed deployment advantages: deterministic coverage, no energy management, simpler fault analysis
- UAV advantages: repositionable to respond to events; cost-effective for large or irregular ROIs (oil platform cross shape); ability to cluster at a detected disaster while maintaining partial coverage elsewhere
- UAV limitations relevant to this thesis: finite battery with a single return-to-base constraint; coordination overhead (N agents competing for 1 charger); no GPS in some environments (addressed via onboard localization, outside scope); partial observability of stochastic events

#### 2.3.2 Optimization-based multi-UAV task allocation
- Vehicle Routing Problem (VRP) variants with vehicle capacity: provably optimal at small scale; intractable at N > 10 without heuristics; requires a known task set (no dynamic disasters)
- CBBA (Choi et al., 2009): distributed auction for task assignment; handles dynamic tasks; no battery management; requires explicit discrete task definition — cannot handle the continuous coverage objective
- Key limitation: all optimization-based approaches require centralized or communication-intensive coordination; the CTDE paradigm avoids explicit communication by conditioning actor behavior on team state observed locally

#### 2.3.3 RL-based multi-UAV coordination
- Early work (Shim et al., 2003): fixed-wing UAV formation via RL; single-agent rewards; no cooperative coverage
- Recent: MAPPO for multi-UAV target search [Ye et al., 2020, cite]; discrete actions; no battery model; deterministic target (not a Poisson random event)
- Energy-aware UAV coordination in the literature: most work addresses offline charging slot assignment (optimization, queuing theory) rather than learned real-time policies
- Gap: no prior work combines (a) a learned continuous-control policy with (b) stochastic rare-event detection, (c) single-charger turn-taking, and (d) a systematic reward engineering protocol that identifies and resolves behavioral pathologies

#### 2.3.4 The disaster-switch mechanism
- `DISASTER_REWARD_SWITCH_ON_DETECTION = true` (§5.5): reward switches from coverage-optimizing to disaster-response mode only after at least one agent detects the disaster; before detection, agents receive no incentive to approach an undetected disaster even if it exists
- This is a novel reward mechanism: it prevents ground-truth exploitation (agents cannot converge on a disaster they have not sensed), maintains coverage as the primary objective during disaster-free intervals, and creates a natural two-phase structure (coverage phase → response phase) in the learned policy
- No directly comparable mechanism identified in prior UAV surveillance literature

**Positioning:** Optimization-based multi-UAV coordination cannot handle stochastic events or continuous coverage; existing RL-based multi-UAV work lacks battery management and single-charger coordination. The disaster-switch mechanism and the documented behavioral pathologies are specific to this combination and absent from prior literature.

---

### 2.4 Energy Management in Multi-Robot Systems (≈ 2 pages)

#### 2.4.1 Single-robot battery-aware planning
- **Bingo fuel return planning** (Nigam, 2014): deterministic: reserve enough fuel for return trip before mission battery is exhausted; optimal for single agent but fails for multi-agent settings where charging must be coordinated
- **RL with battery termination:** treating battery = 0 as a terminal state (standard practice) does not teach agents to actively return to charge; the agent only learns to avoid the terminal state, not when and how to leave coverage to recharge efficiently
- **Battery urgency as a continuous shaping signal** (this thesis, §5.11): a linear ramp from 80%→0% battery provides a gradient toward the charger well before death — the agent experiences increasing charging pull proportional to its deficit, not a cliff; critical for learning to leave a disaster or coverage position in time to reach the charger safely

#### 2.4.2 Multi-robot charging coordination
- **Queuing theory** (M/M/1 model): models arrivals at the charger as a Poisson process; derives optimal threshold policy for when to return; requires known arrival rate and service time — inapplicable when the arrival distribution is determined by the learned policy itself (circular dependency)
- **Optimization-based scheduling:** assign time slots to robots to minimize coverage gaps; centralized; not adaptive to real-time battery levels or disaster events
- **RL for charging coordination:** very limited prior work; most assumes multiple chargers (no occupancy conflict) or binary battery (no shaping); single-charger turn-taking with a shaped penalty has not been formulated as a MARL reward in prior work to the author's knowledge
- **Charger occupancy penalty** (this thesis, §5.12.2): -β × (charging_count - 1)/step discourages simultaneous charging while allowing emergency override when urgency is high enough; calibrated so that a second agent charging is net-negative unless battery < 45%; the first principled RL reward term specifically designed for single-charger turn-taking

#### 2.4.3 Behavioral pathologies specific to energy-constrained MARL
- The four pathologies documented in Chapter 5 §5.11–5.15 (charger clustering, group charging, hovering exploit, terminal-state battery gaming) emerge from the interaction between: (1) a shared physical resource with limited capacity; (2) a multi-objective reward mixing coverage and charging; (3) finite episode horizon and discount factor
- None of these pathologies appear in non-energy-constrained MARL or single-agent settings:
  - Charger clustering: requires a locally attractive point (charger) that coincides with a region of the ROI
  - Group charging: requires multiple agents observing a common low-battery signal and a centralized critic that correlates their actions
  - Hovering exploit: requires a shaped proximity reward that can be collected without committing to the resource
  - Terminal-state battery gaming: requires finite episodes where the expected cost of death approaches zero near the end
- No systematic prior treatment of these pathologies in the MARL literature has been identified; the 10-configuration ablation methodology in Chapter 5 §5.8 is the primary contribution in this regard

**Positioning:** Single-robot battery planning does not generalize to multi-agent settings. Queuing theory assumes a fixed arrival distribution, not a learned policy. Existing RL work does not formulate single-charger turn-taking as a shaped reward. The behavioral pathologies documented here are specific to resource-constrained cooperative MARL and their systematic resolution is a novel contribution.

---

### 2.5 High-Performance RL Frameworks (≈ 1 page)

#### 2.5.1 Python-based frameworks
- **RLlib** (Liang et al., 2018): multi-agent support (PPO, SAC, MADDPG); Python/Ray overhead; memory footprint unsuitable for embedded MCUs; GPU training requires significant infrastructure; good for research prototyping
- **Stable-Baselines3** (Raffin et al., 2021): high-quality single-agent implementations; PyTorch backend; no native multi-agent support; Python overhead at inference
- **CleanRL** (Huang et al., 2022): minimal single-file implementations for reproducibility; single-agent; Python only; not suitable for high-frequency embedded control
- **Isaac Gym / MuJoCo MJX:** GPU-parallelized physics + RL; excellent for massively parallel training; Python interface; not designed for embedded deployment; inference latency dominated by Python interop

#### 2.5.2 High-performance C++ frameworks
- **rl-tools** (Eschmann et al., 2023): header-only C++17/20; all dispatch resolved at compile time via template specialization (zero virtual function overhead); `static constexpr` hyperparameters guarantee reproducibility (same binary = same experiment); CPU (MKL, BLAS) and CUDA backends; actor inference < 10 µs on embedded ARM; published with benchmark results showing 1–2 orders of magnitude faster training than Python-based equivalents on CPU
  - **Multi-agent MAPPO** already supported: `multi_agent_wrapper` applies any inner module N times over per-agent observation slices; used in bottleneck environment; PPO training loop compatible
  - **SAC gap before this work:** `ConfigApproximatorsMLP` provides a single large MLP mapping the full concatenated observation to all actions with no per-agent structure; `SampleAndSquash` input layout `[all_μ | all_log σ]` is incompatible with `multi_agent_wrapper` per-agent block output `[μ₀, log σ₀ | μ₁, log σ₁ | ...]` without an intermediate permutation step
- No other high-performance C++ RL framework with equivalent characteristics (zero-overhead templates, embedded deployment capability, multi-agent primitives) is known to the author

#### 2.5.3 Positioning
- The gap addressed in Chapter 4 (§4.2–4.4): a multi-agent SAC extension to rl-tools comprising (a) the permutation buffer bridging `multi_agent_wrapper` output to `SampleAndSquash` input, (b) `ConfigApproximatorsMLPMultiAgent` as a drop-in replacement for `ConfigApproximatorsMLP`, and (c) `ActorGradient` with correct gradient routing through the inverse permutation back to shared inner MLP weights. The result: the first multi-agent SAC actor in a C++ framework suitable for embedded real-time deployment.

---

### 2.6 Research Gap Synthesis (≈ 0.5 pages)

The table below summarizes the gaps identified across the five areas and maps each to the specific thesis chapter that addresses it:

| Dimension | State of the Art | Limitation | Addressed in |
|---|---|---|---|
| CTDE algorithms | MADDPG, MASAC (Python); QMIX (discrete only); MAPPO in rl-tools | No multi-agent SAC with privileged asymmetric critic in C++ | Ch. 4, §4.2–4.4 |
| Coverage reward | Lloyd's algorithm; Gaussian potential; RL coverage (no sensor cutoff) | No battery-weighted sensor-range coverage | Ch. 5, §5.10.2 |
| Multi-UAV surveillance | Optimization-based allocation; RL without battery or stochastic events | No combined coverage + detection + charging in continuous MARL | Ch. 5, Ch. 6 |
| Charging coordination | Queuing theory (offline); multi-charger RL; no turn-taking reward | No principled single-charger occupancy penalty in MARL | Ch. 5, §5.12.2 |
| RL frameworks | Python-based (RLlib, SB3); rl-tools (no multi-agent SAC) | No C++ multi-agent SAC for embedded deployment | Ch. 4, §4.2–4.4 |
| Reward pathologies | Not systematically documented in resource-constrained MARL | 4 pathologies undocumented; no structured resolution methodology | Ch. 5, §5.11–5.15 |

---

## Chapter 3 — Preparatory Works (≈ 20–25 pages)

**Purpose of this chapter:** Trace the progression from the two survey papers (C0a, C0b) through the tabular MARL work (C1) to identify the open problems that motivated the main thesis contributions. This is not a literature survey — it is a first-person research narrative showing how each published piece shaped the next.

### 3.1 The early PhD survey work: scoping the field (≈ 3 pages)

**Purpose of this section:** Establish C0a and C0b as genuine PhD contributions, not just background reading. Summarize the key findings of each survey in 1–2 pages each, then draw the connecting line to the H-SURF project and the oil platform work.

#### 3.1.1 Underwater swarm robotics: challenges and perspectives [C0a]
- Survey scope: review of 7 active underwater swarm projects (CoCoRo, subCULTron, M-AUE, MONSUN, RoboFish, etc.), taxonomy of communication modalities (acoustic, optical, magnetic, RF), and simulator landscape (Gazebo, MORSE, MARS, Webots)
- Key finding on energy: Section VI (Perspectives) identifies energy autonomy as one of the primary unsolved challenges — *"moving or stationary chargers have to be considered to conduct longer missions; they have to be part of the path planning algorithms"* — a direct precursor to the battery model and charging station in Chapter 5
- Key finding on compute: on-board AI must run on the edge with embedded hardware; this motivates the choice of rl-tools (Chapter 4) over Python-based frameworks
- Key finding on communication: most real underwater systems are limited to range/bearing due to acoustic modem bandwidth constraints; the oil platform design takes the opposite simplifying assumption — unlimited communication range — so each agent's full state (position, velocity, battery, status flags) is directly observable by all other agents; this is explicitly a design choice that relaxes the underwater constraint and would need revisiting for a real deployment

#### 3.1.2 Mission planning for multi-AUV systems: centralized vs. decentralized [C0b]
- Survey scope: taxonomy of path planning approaches: centralized (mathematical optimization / VRP variants, algebraic and differential geometry) vs. decentralized (RL, GNN, evolutionary computation)
- Key finding on centralized limitations: four structural drawbacks identified — requires heavy a priori computation, large datasets stored off-board, stable communication link with a base station, single point of failure; all four apply with full force in underwater environments
- Key finding on CTDE: Section III explicitly identifies *centralized-learning-decentralized-execution* (CTDE) as the state-of-the-art approach for decentralized multi-agent systems; this directly motivates the SAC architecture in Chapter 4 (actor deployed on each drone, critic used only at training time)
- Key finding on RL for dynamic environments: energy-aware path planning (Li et al. [11] cited in the survey) addresses charging station placement in multi-AUV missions; this is the closest prior work to the oil platform charging coordination problem

##### 3.1.3 The connecting thread
One-page synthesis: what the two surveys collectively established as the open problem that the rest of the thesis addresses:
- Swarm survey → *energy management is unsolved*
- Mission planning survey → *CTDE RL is the right paradigm*
- Tabular MARL paper (§3.2) → *cooperation works from individual rewards, but cannot handle continuous state/action or resource constraints*
- Oil platform project (Chapters 4–6) → *the answer*

### 3.2 Tabular MARL for underwater pipe inspection (H-SURF) (≈ 10 pages)

**Purpose of this section:** Present the tabular MARL pipe-inspection study (C1) as a single published contribution within the H-SURF project: the task, its RL formulation, the learning algorithm, and the results. Spine is the ESWA paper; the master thesis is drawn on only for additional depth.

#### 3.2.1 The H-SURF project and the inspection task (≈ 2 pages)
- Description of the H-SURF robotic fish: bio-inspired AUV, frontal camera, acoustic modem, lateral LEDs for communication, three thrusters
- Reference Fig. 1 from the published paper (two robotic fish)
- The inspection task: visual following of a partially buried underwater pipeline; 10⁵ km of global underwater pipelines; corrosion and sand burial create visibility challenges
- Why this is hard for a single robot: limited visibility (ξ = 0.5 means 50% of pipe is buried), sensor noise, actuator noise, no global localization, no GPS
- Why MARL is the right approach: redundancy overcomes individual visibility failures; cooperation propagates pipe-orientation information along the swarm

#### 3.2.2 The RL formulation (≈ 3 pages)
- Keep the theory lean and model-specific: describe only this model's states, actions, and reward; defer generic Dec-POMDP / CTDE / value-based theory to Chapter 2
- Dec-POMDP formulation: agents cannot observe each other's Q-tables or rewards; only local observations are shared indirectly through observable behavior
- State representation: three-component tuple (information state sᵢ, neighbors state sₙ, pipe state sₚ)
  - Information state sᵢ: 5-state graph (very close / close right / close left / just lost / lost); encodes quality of agent's pipe knowledge
  - Neighbors state sₙ: signed angle between agent heading and weighted average heading of neighbors; weight w depends on sᵢ of each neighbor (0.8/0.4/0.2/0.0)
  - Pipe state sₚ: signed angle between agent heading and detected pipe orientation; maintained via running average with forgetting factor β=0.99
- Action space: 7 discrete rotation angles in [-3π/16, 3π/16]
- Reward: Rᵢ = cos(βᵢ - ζ) - 1 if agent sees pipe, -1 otherwise (individual, not cooperative)
- Key design choice: no explicit cooperation reward; cooperation emerges solely from shared observations

#### 3.2.3 The Expected-SARSA algorithm (≈ 2 pages)
- Why tabular: explainability and interpretability; ability to visualize the full Q-matrix as a policy map
- Why Expected-SARSA over Q-learning: reduces variance from random action selection; accounts for the exploration policy in the update target
- Update rule: ΔQᵢ = Rᵢᵗ⁺¹ + Eπ[Qᵢ(sᵢᵗ⁺¹, aᵢᵗ⁺¹) | sᵢᵗ⁺¹] - Qᵢ(sᵢᵗ, aᵢᵗ)
- Visit-based learning rate: α decreases as state-action pair is visited more often; avoids convergence problems for rarely-visited states
- Optimistic initialization: all Q-values start at 0 (maximum possible); encourages early exploration
- ε-greedy exploration: decays from ε₀ = 0.3 over training; formula in Appendix A of published paper

#### 3.2.4 Results (≈ 3 pages)
- Reproduce key results from the paper with additional commentary:
  - Metric 1 (collective pipe coverage fraction): cooperative > independent at all visibility levels; improvement grows with lower visibility
  - Metric 2 (individual agent coverage): huge gap between cooperative and independent — independent agents often lose the pipe for the whole episode
  - Scaling: 8+ agents needed for convergence at ξ=0.5; 2 agents sufficient at ξ=1.0
- Figures reused: information-state transition graph; per-visibility metric curves (collective + per-agent gap); policy Q-matrix maps. Chain-formation frames are not reused as a figure; chain behavior is carried in prose only
- Learning curves by visibility level: explain the shape (fast early improvement, oscillation, convergence)
- **Collective learning accelerates convergence:** cooperative agents reach a good policy in fewer episodes than independent agents; the mechanism is implicit knowledge transfer — an agent observing an informed neighbor's behavior updates toward a correct policy faster than one that must rediscover pipe orientation from scratch; this is a distinct benefit from the steady-state coverage improvement and should be plotted separately (convergence episode vs. ξ)
- Policy visualization (Q-matrix heatmaps): describe the learned strategies per information state:
  - sᵢ=0 (very close): align with pipe, small neighbor influence
  - sᵢ=1,2 (close right/left): return immediately to sᵢ=0, ignore pipe orientation
  - sᵢ=3,4 (lost recently/long): oscillate when no neighbors; follow informed neighbors when available
- Chain behavior: emergent formation where agents follow each other in a chain along the pipe; enables navigation through long hidden sections
- Null hypothesis comparison: pure numerosity advantage vs. genuine cooperation; second metric definitively shows cooperation is the driver

### 3.3 What this work achieves and what it cannot do (≈ 2 pages)
- Achievements: RQ1 answered positively — cooperative monitoring behavior emerges from individual rewards; explainable policies; lightweight enough for edge deployment; collective learning accelerates convergence (fewer training episodes needed), not just improves steady-state performance
- Limitations that motivated the oil platform project:
  1. **Discrete state/action space:** cannot capture continuous velocity control or precise positioning
  2. **No resource constraints:** energy consumption explicitly flagged in paper's Future Work section as the main missing piece
  3. **Simple geometry:** infinite 1D pipe; no 2D spatial coverage, no multi-region monitoring
  4. **Tabular scaling wall:** Q-matrix grows exponentially with state dimensionality; cannot add new state variables without redesign
  5. **No disaster events:** static, persistent pipe; no rare, stochastic events requiring rapid response

### 3.4 The bridge: from AUV pipe inspection to UAV oil platform surveillance (≈ 3 pages)
Explicit conceptual bridge — this section is unique to the thesis and is the narrative glue. It merges the conceptual mapping (preserved / changed / new) with the concrete lessons that shaped specific design decisions, so each lesson lands on the design element it motivated.
- **Preserved:** individual reward + shared observation architecture; cooperative behavior without explicit incentives; partial observability
- **Changed:** tabular → deep neural network (handles continuous state/action); 1D pipe → 2D cross-shaped ROI; static task → dynamic disaster events; no energy → battery-constrained with charging station
- **New challenges introduced by the changes:** reward shaping for multi-objective tasks; non-stationarity of disaster events; terminal-state exploitation under finite episodes; behavioral pathologies unique to deep RL (charger clustering, hovering exploit)
- **Why rl-tools:** the transition to continuous deep RL required a framework capable of high-performance C++ training; rl-tools was chosen and extended (Chapter 4)
- **Lessons that shaped design decisions** (state the lesson, then the exact oil-platform design element it motivated):
  - Individual rewards suffice → preserved in oil platform (collective reward to all agents at every step, no per-agent decomposition)
  - Weight-based communication → generalized to direct observation of other agents' states (no discrete LEDs, but per-agent observation blocks include battery, status flags)
  - Explainability sacrificed for scale → compensated by the browser-based trajectory visualization (Chapter 4)
  - Energy flagged as future work → addressed through battery model, charging station, and battery-urgency weighting in the reward

---

## Chapter 4 — The rl-tools Framework and Multi-Agent SAC Extension (≈ 18–22 pages)

**Purpose:** Document the engineering contribution. This chapter answers RQ2 and provides the technical foundation for Chapter 5.

### 4.1 Overview of rl-tools (≈ 3 pages)

#### 4.1.1 Design philosophy
- Header-only C++17/20: all code in `.h` files; no separate compilation needed for the library itself
- Policy-typed via template structs: `DEVICE`, `SPEC`, `PARAMETERS` are compile-time types; all dispatch is resolved at compile time (zero virtual function overhead)
- Static constexpr parameters: every hyperparameter is a `static constexpr` in a `DefaultParameters` struct; changing a hyperparameter requires recompilation, guaranteeing that compiled binaries are always self-consistent and reproducible
- Device abstraction: `CPU_MKL`, `CPU_BLAS`, `CUDA` devices; same code compiles for all
- Memory model: `malloc`/`free` functions allocate on-device memory; `DYNAMIC_ALLOCATION` template flag allows compile-time selection between stack and heap

#### 4.1.2 Key components relevant to this work
- Matrix/Tensor containers: `Matrix<SPEC>` and `Tensor<SPEC>` with compile-time shapes
- MLP: `nn_models::mlp::Configuration` → `BindConfiguration` → `Build<CAPABILITY, CHAIN, INPUT_SHAPE>`; capability encodes Forward/Gradient/Adam parameter storage
- Sequential model: `nn_models::sequential::Module` chains layers; used to build actor and critic networks
- SampleAndSquash (SAS): samples actions from Gaussian, applies tanh squash, computes log-probability for entropy; adaptive temperature α learning
- SAC loop: `sac::loop::core::Config` parameterizes the full training loop (buffer, environments, training intervals, warmup steps, network architecture via `APPROXIMATOR_CONFIG`)
- Zoo: `src/rl/zoo/` contains environment-specific configurations; `oil_platform-v1/` contains the configurations developed in this thesis

#### 4.1.3 What was missing before this work
- `ConfigApproximatorsMLP`: the existing actor is a single MLP that maps the full concatenated observation (all agents) to all actions; no per-agent structure
- Consequence: all agents share not just weights (good for scalability) but also see all observations mixed together (prevents learning agent-specific policies)
- rl-tools already had a `multi_agent_wrapper` module and MAPPO support (used in the bottleneck environment); however, SAC had no multi-agent support — the existing SAC actor (`ConfigApproximatorsMLP`) is a single MLP with no per-agent structure, and `SampleAndSquash` expects a specific `[all_μ | all_σ]` layout incompatible with the per-agent block output of `multi_agent_wrapper`

### 4.2 The multi_agent_wrapper module (≈ 4 pages)

#### 4.2.1 Architecture
- `multi_agent_wrapper::Configuration<TYPE_POLICY, TI, N_AGENTS, INNER_MODULE_CHAIN>`: wraps any sequential module chain (reused unchanged from the PPO support; in our actor the inner chain is a single MLP)
- The wrapper splits the swarm observation into N equal per-agent blocks (requires total obs dim divisible by N); each block = one agent's full observation, which under unlimited communication already contains that agent's own state, the other agents' states, and shared scene info (Ch. 5)
- Inner MLP input: one per-agent block (= `Observation::DIM / N_AGENTS`); output: `2 × PER_AGENT_ACTION_DIM` (mean and log-std for one agent's actions)
- Wrapper output: `N × 2 × PER_AGENT_ACTION_DIM` (per-agent blocks concatenated, in agent order)

#### 4.2.2 Weight sharing mechanics
- The inner MLP has one set of weights; the wrapper applies it to every agent
- Mechanism (NOT N sequential passes): the per-agent observation blocks are reshaped into the batch dimension, so a batch of B transitions over N agents becomes a batch of B·N per-agent observations; `INTERNAL_BATCH_SIZE = BATCH * N_AGENTS` in `multi_agent_wrapper/model.h`
- `forward`/`backward`: a single call to the inner MLP over the enlarged batch; the reshape is a view (`reshape_row_major`), no extra copy beyond making the blocks contiguous
- Weight sharing is automatic: one set of weights processes all agents as batch elements; the backward pass accumulates the gradient over the enlarged batch, summing the contributions of all agents and transitions into the shared weights, exactly as standard mini-batch training sums over independent samples

#### 4.2.3 Why weight sharing is appropriate here
- Homogeneous agents: all drones have identical capabilities, sensors, and battery constraints, so the policy optimal for one is optimal for any other once expressed in that agent's own observation
- Parameter count independent of N (one inner network reused, not replicated) → cheap training and on-device inference; same inner network deployable on a different swarm size without retraining
- Permutation equivariance: relabeling agents permutes the output blocks without changing them; no agent is tied to a fixed index
- Differentiation between agents, when needed, must enter through the observation (e.g., a per-agent identifier appended to each block), not through the weights; the shared network cannot otherwise tell agents apart

### 4.3 Bridging the wrapper and sample-and-squash layouts (≈ 2 pages)

**Framing (do not oversell):** the sample-and-squash layer is used UNMODIFIED. The only thing added here is an index map (a permutation) from the wrapper's per-agent block layout to the grouped `[all μ | all log σ]` layout SAS expects, plus its exact inverse for the gradient. Describe it as a permutation/reshape, not a "trick". The contribution worth emphasizing is not the cleverness of the reshape but (i) the resulting drop-in `ActorGradient` actor that makes the unmodified wrapper and SAS compose into a SAC-compatible module, and (ii) the deployability argument (the trained per-agent network is exactly the one each drone runs). Right-size accordingly: one short subsection on the layout mismatch, one on the forward/inverse permutation, the rest is the module plumbing in §4.4.

#### 4.3.1 The layout mismatch problem
- multi_agent_wrapper output for N=3, K=2 (action dims per agent):
  `[μ₀₀, μ₀₁, log σ₀₀, log σ₀₁ | μ₁₀, μ₁₁, log σ₁₀, log σ₁₁ | μ₂₀, μ₂₁, log σ₂₀, log σ₂₁]`
  (per-agent blocks: mean then log-std for each agent)
- SampleAndSquash expected input:
  `[μ₀₀, μ₀₁, μ₁₀, μ₁₁, μ₂₀, μ₂₁ | log σ₀₀, log σ₀₁, log σ₁₀, log σ₁₁, log σ₂₀, log σ₂₁]`
  (all means first, then all log-stds)
- Reason: SAS samples `ε ~ N(0,1)` of shape `[BATCH, ACTION_DIM]` and computes `a = tanh(μ + σ·ε)` vectorially across all action dimensions simultaneously; it needs all μ and all σ contiguous

#### 4.3.2 Forward permutation
- For agent a ∈ [0, N), action dim k ∈ [0, K):
  - Mean: `perm_buf[row, a·K + k] = wrapper_out[row, a·2K + k]`
  - Log-std: `perm_buf[row, N·K + a·K + k] = wrapper_out[row, a·2K + K + k]`
- Implemented in `apply_permutation<DEVICE, SPEC>(device, src, dst)` in `per_agent_actor.h`
- Complexity: O(N × K × BATCH) — trivially parallelizable

#### 4.3.3 Backward (inverse) permutation
- Gradient from SAS (`d_perm_buf`) must be mapped back to gradient w.r.t. wrapper output (`d_wrapper_output`)
- Inverse:
  - `d_wrapper_out[row, a·2K + k] = d_perm_buf[row, a·K + k]` (mean gradient)
  - `d_wrapper_out[row, a·2K + K + k] = d_perm_buf[row, N·K + a·K + k]` (log-std gradient)
- Implemented in `apply_inverse_permutation<DEVICE, SPEC>(device, src, dst)` in `per_agent_actor.h`
- This is the exact transpose of the forward permutation matrix — mathematically exact, no approximation

#### 4.3.4 Full forward pass (ActorGradient::forward)
1. `wrapper.forward(input)` → stores output in `wrapper.content`
2. `matrix_view(wrapper_output_tensor)` → get Matrix view
3. `apply_permutation(wrapper_out_m, buffer.perm_buf)`
4. `sas.forward(buffer.perm_buf, buffer.sas_buffer, rng, mode)` → stores squashed actions in sas.output
5. Copy `sas.output` → output tensor

#### 4.3.5 Full backward pass (ActorGradient::backward)
1. `backward_full(sas, buffer.perm_buf, d_out_m, buffer.d_perm_buf, buffer.sas_buffer)` → fills `d_perm_buf`
2. `apply_inverse_permutation(buffer.d_perm_buf, d_wrapper_output_tensor)`
3. `wrapper.backward(input, d_wrapper_output_tensor, buffer.wrapper_buffer)` → accumulates into shared inner MLP weights

### 4.4 ConfigApproximatorsMLPMultiAgent (≈ 2 pages)

#### 4.4.1 Template interface
- Same signature as `ConfigApproximatorsMLP`:
  ```cpp
  template<typename TYPE_POLICY, typename TI, typename ENVIRONMENT,
           typename PARAMETERS, bool DYNAMIC_ALLOCATION>
  struct ConfigApproximatorsMLPMultiAgent { ... };
  ```
- Contains nested `Actor<CAPABILITY>` and `Critic<CAPABILITY>` structs with `MODEL` typedef
- The SAC loop uses `APPROXIMATOR_CONFIG::Actor<CAPABILITY_ACTOR>::MODEL` and `APPROXIMATOR_CONFIG::Critic<CAPABILITY_CRITIC>::MODEL` — identical in both templates

#### 4.4.2 Actor configuration
- INNER_MLP_CONFIG: PER_AGENT_OBS_DIM → [ACTOR_HIDDEN_DIM] × ACTOR_NUM_LAYERS → 2 × PER_AGENT_ACTION_DIM, ReLU activations
- WRAPPER_CONFIG: multi_agent_wrapper::Configuration<N_AGENTS, INNER_MODULE_CHAIN>
- SAS_PARAMETERS: inherits LOG_STD_LOWER/UPPER_BOUND, ADAPTIVE_ALPHA, TARGET_ENTROPY from SAC_PARAMETERS
- Final MODEL type: ActorGradient<ActorSpec<...>>

#### 4.4.3 Critic configuration
- INPUT_DIM = ObservationPrivileged::DIM + ACTION_DIM (all agents' actions)
- MLP_CONFIG: INPUT_DIM → [CRITIC_HIDDEN_DIM] × CRITIC_NUM_LAYERS → 1, ReLU
- Same as standard `ConfigApproximatorsMLP` critic — unchanged

### 4.5 The browser-based visualization (≈ 1–1.5 pages) — DRAFTED in ch04.tex

> NOTE: written grounded in `operations_cpu.h` (json + get_ui). The earlier "what is rendered" list and the "pathologies described in Chapter 5" framing were OUTDATED and have been corrected: no Gaussian potential overlay (that was the discarded multi-Gaussian reward), and Ch5 no longer has dedicated pathology sections. See the cross-cutting pathology-reference cleanup list in the chat / TODO before relying on this section's neighbors.

#### 4.5.1 Architecture
- The environment supplies two functions in `operations_cpu.h`: `json()` serializes the state per step; `get_ui()` returns the JavaScript render module (raw string literal). rl-tools' built-in UI server delivers both to the browser.
- Renderer is plain JS on an HTML5 canvas, no external dependencies, no build step.

#### 4.5.2 What is rendered (grounded in the render function)
- Cross-shaped ROI (central platform + two pipe arms) over a light reference grid
- Charging station as a marked circle with its docking radius drawn as a ring
- Active disaster (when present) as a shaded circle with its detection (sensor) range ring
- Per-drone: colored disc carrying its index (color = normal / charging / dead), velocity arrow, battery gauge (color changes as battery drains), sensor-range circle
- Text panel: per-step reward and the value of each reward term (coverage, charging, occupancy, abandonment, death, …), colored by sign
- (Do NOT list: Gaussian potential overlay — outdated; per-drone ⚡/👁/✕ icons — charging/dead are shown by disc color, ⚡ is on the station only)

#### 4.5.3 Role in development
- Primary instrument during reward shaping: behaviors invisible in the scalar return were spotted here first, then confirmed in logged metrics
- Concrete examples to keep (tie to reward terms that exist in Ch5 §5.4): agents lingering just outside the docking ring without charging (motivated the reduced proximity credit), multiple agents charging at once (motivated the occupancy penalty); turn-taking at the charger as a behavior to preserve
- The per-term reward panel next to the trajectories links each behavior to the responsible reward term — what a scalar learning curve cannot do
- AVOID claiming the four named pathologies are "described in Chapter 5" — they are not currently sections there

---

## Chapter 5 — Environment and Reward Design (≈ 34–44 pages)

**Structure:** This chapter has two parts. §5.1–5.8 describe the environment — kinematics, battery model, disaster dynamics, observation spaces — grounded entirely in `oil_platform.h`. §5.8 tell the story of the reward design as a scientific investigation: each section corresponds to an identified behavioral pathology, its root cause, the fix, and what was learned.

### 5.1 Design objectives (≈ 1 page)
- Physically plausible: kinematics and battery model should be close to real UAV behavior
- Compile-time parameterization: every design decision exposed as a `static constexpr` flag, enabling ablation without runtime overhead
- Testable at multiple difficulty levels: battery can be disabled, disaster can be disabled, charging station can be randomized — enabling curriculum-style evaluation
- Efficient: zero heap allocation in the environment step; all computation in O(N × GRID_RES²) per step

### 5.2 Platform geometry (≈ 2 pages)
- 20×20 m continuous 2D world; cross-shaped ROI centered at (10,10): 4×4 m central platform (PLATFORM_HALF_SIZE=2) + two orthogonal pipe arms (PIPE_WIDTH=2 m, extending to world boundary)
- ROI catalogue: compile-time array of ROI cell centers; built in `build_roi_catalogue()` and stored as `static constexpr ROI_CATALOGUE` + `ROI_SIZE`; ROI cell count determined by GRID_RES, PLATFORM_HALF_SIZE, PIPE_WIDTH
- ROI cells serve a dual purpose: define the surveillance objective and act as candidate disaster *spawn* locations; once active, disasters can drift outside the ROI — agents must learn autonomously when to leave coverage to track them (forward ref → §5.5)
- Charging station at (5,5) m — fixed in an open quadrant, outside the ROI; creates a spatial tension between coverage and charging
- All geometry is parameterized via compile-time constants (PLATFORM_HALF_SIZE, PIPE_WIDTH, GRID_SIZE_X, GRID_SIZE_Y, GRID_RES); changing and recompiling produces a different ROI without touching any other code
- **Gaussian catalogue:** NOT covered here — belongs to §5.10.1 (multi-Gaussian coverage reward, first attempt), where it is introduced as part of the reward engineering story

### 5.3 UAV kinematics (≈ 2 pages)
- Action: 2D vector in [-1,1]² representing target velocity direction and magnitude
- Clamping: action is clamped to [-1,1]² before application (necessary for PPO which uses identity output layer)
- First-order inertia filter: `v_next = v + α · (a · MAX_SPEED - v)` with α = 0.6; gives realistic acceleration/deceleration dynamics
- Speed clamping: velocity magnitude clamped to MAX_SPEED after filter
- Position integration: `p_next = p + v_next · DT` with DT = 0.05 s
- Boundary handling: position clamped to [0, GRID_SIZE]; velocity zeroed in the clamped direction
- Charging freeze: when is_charging = true, position and velocity are overridden to remain at charging station; the drone physically cannot move during charging
- DT = 0.05 s gives 20 Hz control; EPISODE_STEP_LIMIT_MAX = 1300 steps → 65 s maximum episode

### 5.4 Battery model (≈ 3 pages)
- Battery range: [0, 100] (percentage)
- Initial condition: sampled uniformly from [50, 100] per drone per episode (forces agents to handle mid-charge situations from the start of training)
- Discharge: constant rate DISCHARGE_RATE_BASE = 0.15%/step (during flight)
- Charging conditions (all must hold simultaneously):
  - Distance to charger < CHARGING_STATION_RANGE = 2 m
  - Speed < CHARGING_VELOCITY_THRESHOLD = 0.75 m/s (agent must be nearly stationary)
  - battery < MINIMUM_BATTERY_FOR_CHARGING = 80% (prevents "top-up" visits when battery is high)
  - was_charging = false (charging must be initiated fresh)
- Charging rate: +1%/step while is_charging = true
- Charging termination: session ends when hold counter reaches 0 OR battery reaches 100%, whichever occurs first; the hold (70 steps) prevents arbitrarily short visits for critically depleted agents
- Death: battery = 0 → agent.dead = true; velocity = 0; battery frozen at 0; no further actions
- POINT_OF_NO_RETURN_DEATH_ENABLED = false in final config (tested but ablated out)

**Battery design rationale:**
- MINIMUM_BATTERY_FOR_CHARGING = 80%: agents should not charge when they have 90% battery; this forces them to stay deployed during coverage
- MIN_CHARGE_STEPS = 70: prevents rapid cycling (go to charger, charge 1%, leave); 70 steps × 0.15%/step discharge → an agent that charges at 80% and stays for 70 steps arrives at ~70% battery if no charging occurred (approximation)
- Constant discharge (not velocity-dependent): velocity-dependent discharge is physically more accurate and identified as an important next step; it was tested but added complexity during reward shaping without measurable benefit at that stage

### 5.5 Disaster dynamics (≈ 3 pages)
- Spawn: Bernoulli(p = 0.01) per step when step_count ≥ DISASTER_MINIMUM_SPAWN_STEP = 0; position sampled uniformly from ROI catalogue
- Initial velocity: random angle, speed sampled uniformly from [0, DISASTER_MAX_SPEED = 1.0 m/s]
- Wandering update (each step):
  1. Angular jitter: rotate current velocity by Δθ ~ U(-0.10, +0.10) rad (≤5.7° per step)
  2. Speed jitter: add Δv_frac ~ U(-0.10, +0.10) to fractional speed; clamp to [0, DISASTER_MAX_SPEED]
  3. Renormalize to target speed
  4. Integrate: `pos_next = pos + v_next · DT`
- Exit: if position leaves world boundary → disaster.active = false; if never detected before exit → disasters_missed += 1
- Multiple disasters: only one disaster can be active at a time (simplification vs. multiple simultaneous events)
- Detection bookkeeping: disaster_detected_global, last_detected_disaster_position, disaster_undetected_steps, disaster_spawn_step, cumulative_detection_latency

**Disaster design rationale:**
- Stochastic walk (not fixed velocity): makes disaster location unpredictable; agents cannot learn a fixed patrol pattern that happens to always intercept it
- p = 0.01/step: expected time to first disaster ≈ 100 steps; ensures some disaster-free coverage at episode start before any disaster-response training
- Exit on boundary: simplifies bookkeeping and means disaster episodes are finite; missed disasters are tracked

### 5.6 Detection model (≈ 2 pages)
- Detection probability: logistic function of distance
  `p_det(d) = σ(β · (SENSOR_RANGE - d))` where `β = log(P0/(1-P0)) / ε_inside`
  - SENSOR_RANGE = 5.0 m, P0_DETECT = 0.95, ε_inside = 0.5 m (10% of sensor range inside rim)
  - Result: p_det = 95% at 4.5 m, drops steeply to 50% at 5.0 m, near-zero beyond 6 m
- Per-step Bernoulli draw: `is_detecting = Bernoulli(p_det)`
- Dead or charging agents: is_detecting = false (excluded from detection)
- Global detection: `disaster_detected_global = true` as soon as any agent detects; remains true until disaster exits
- last_detected_disaster_position: updated every step at least one agent detects; used as observation fallback when disaster is not currently detected

**Detection design rationale:**
- Logistic (not hard threshold): provides a smooth gradient for reward shaping near sensor range; a hard threshold would create discontinuous reward that is harder to learn from
- Separate from disaster_active: a disaster can exist but not be detected; global detection flag is the policy signal (DISASTER_REWARD_SWITCH_ON_DETECTION = true); this prevents agents from being rewarded for approaching an undetected disaster based on ground-truth position (which they cannot know)

### 5.7 Observation spaces (≈ 4 pages)

#### 5.7.1 Actor observation (local, deployed)
Layout for the SAC actor (RL_TOOLS_USE_MULTI_AGENT_PPO not defined):

Per agent (PER_AGENT_DIM components):
1. **Own state (BASE_PER_AGENT_DIM = 8):**
   - pos[0], pos[1]: absolute position normalized to [-1,1] via `(pos / GRID_SIZE) * 2 - 1`
   - vel[0], vel[1]: velocity normalized by MAX_SPEED
   - is_detecting: {0,1}
   - battery: normalized to [0,1]
   - dead: {0,1}
   - is_charging: {0,1}
2. **Relative extras (RELATIVE_EXTRA_DIM = 5, when OBSERVE_RELATIVE_POSITIONS=true, ACTOR_OBSERVE_CHARGING_STATION_POSITION=true):**
   - (disaster_pos - own_pos)[0], [1]: relative displacement to last known disaster position (or 0 if never detected); normalized by MAX_GRID_DIM
   - (charger_pos - own_pos)[0], [1]: relative displacement to charger
   - disaster_detected_global: {0,1}
3. **Other agents ((N-1) × PER_OTHER_AGENT_DIM = 8 each, when OTHER_AGENTS_OBSERVE_RELATIVE_VELOCITY=true):**
   - (other_pos - own_pos)[0], [1]: relative position
   - (other_vel - own_vel)[0], [1]: relative velocity, normalized by 2×MAX_SPEED
   - other_battery, other_dead, other_is_charging, other_is_detecting
4. **Agent ID (AGENT_ID_DIM = N):** one-hot encoding

Total: 8 + 5 + (N-1)×8 + N = 13 + 8N - 8 = 5 + 8N dims per agent; full obs = N × (5 + 8N) dims

**Key design choices:**
- Relative positions: translation-invariant; consistent with random charger position (randomization tested, fixed in final config)
- Relative velocity: fully relative inter-agent frame; see §5.14 for the rationale
- One-hot ID: necessary for asymmetric role learning (e.g., "I'm agent 0 and I should charge now")
- Absolute own position kept: provides world anchor without which agents cannot navigate to the charger or platform

#### 5.7.2 Critic observation (privileged, training-only)
ObservationPrivileged (absolute world coordinates throughout):

Per agent (PER_AGENT_DIM = 8):
- Absolute position (2): `2 * (pos / GRID_SIZE) - 1`
- Absolute velocity (2): `vel / MAX_SPEED`
- is_detecting (1), battery (1), dead (1), is_charging (1)

Shared (SHARED_DIM = 9):
- disaster_active (1)
- disaster_detected_global (1)
- disaster_pos (2): absolute, ground truth (even if undetected)
- disaster_vel (2): absolute, ground truth
- charger_pos (2): absolute
- remaining_normalized (1): `max(episode_step_limit - step_count, 0) / EPISODE_STEP_LIMIT_MAX`

Total: N×8 + 9 dims

**Why privileged:** The critic estimates Q(s,a) during training; ground-truth disaster position gives it a much cleaner signal for evaluating how well agents are positioned relative to an undetected threat. At deployment, the actor only sees what was detected.

**Why remaining_normalized is critic-only (and why episode lengths are randomized):**
- In a finite-horizon episode the true Q-value depends on remaining steps: the cost of dying scales with the number of future steps over which the per-step death penalty accumulates
- Without remaining_normalized, the critic receives inconsistent Bellman targets for the same (s,a) pair depending on episode position → loss spikes → unstable training
- Episode lengths are randomized at each reset (U[700, 1300]) to remove any fixed-length structure from the training distribution
- remaining_normalized = max(L - t, 0) / L_max ∈ [0,1] (where L is the sampled step limit, t is the current step, L_max = 1300) is provided to the critic so it produces consistent, time-conditional Q-values across episodes of different lengths
- The actor does NOT receive remaining_normalized — the deployed policy must be independent of episode duration, as required for real missions of unknown length
- (config. A6 introduced randomization; A7 added remaining_normalized to the critic)

### 5.8 Reward function (≈ 6–8 pages)

All reward terms are non-positive; optimal behavior yields a total reward of zero.
This penalty formulation penalizes unused capacity rather than rewarding achieved value: a perfect agent that covers all assigned cells receives zero coverage penalty, not a bonus.
The combined per-step reward is the sum of all active terms, each scaled by a weight determined at compile time.

The full combined expression:
`R = coverage_penalty + charging_penalty + charger_occupancy_penalty + disaster_penalty + abandonment_penalty + death_penalty + ongoing_death_penalty`

#### 5.8.1 Sensor-range coverage reward
- Sweep ROI catalogue; for each cell, find nearest alive agent within SENSOR_RANGE
- Hard sensor cutoff: no credit for agents outside SENSOR_RANGE (unlike Gaussian which gives reward at any distance)
- Nearest-agent assignment: each cell is assigned to its nearest agent within SENSOR_RANGE → no overlap incentive
- Battery-weighted: coverage_weight_i = 1 - battery_urgency_i; full-battery agents cover, low-battery agents are "released" to charge
- coverage_penalty = -GAUSS_BETA_COVER × (fleet_coverage_capacity - fleet_coverage_value)
- An earlier multi-Gaussian formulation was tested but introduced an overlap incentive (two agents near the same Gaussian peak both collect positive reward, causing clustering); details are in Appendix D

**Key parameters:** GAUSS_BETA_COVER = 1.0; SENSOR_RANGE = 5.0 m

#### 5.8.2 Battery urgency and charging shaping
- Battery urgency: zero when battery ≥ CHARGING_SHAPING_BATTERY_THRESHOLD (= 80%); linear ramp from 0→1 as battery drops from 80%→0% (CHARGING_URGENCY_RAMP_POWER = 1.0)
- Gate rationale: without the gate, agents feel a small charging pull at all battery levels, making the charger a locally attractive point regardless of need; the gate suppresses this pull entirely until battery requires it (A2)
- Spatial charging Gaussian: charging_value_i = GAUSS_SIGMA_CHARGING Gaussian evaluated at distance to charger, scaled by battery_urgency_i
- CHARGING_SHAPING_SCALE = 0.3: scales the Gaussian down so that hovering just outside the docking zone collects substantially less reward than actually docking; the 0.7× gap creates a docking incentive against the 70-step lock-in cost (A3)
- Charger occupancy penalty: -CHARGER_OCCUPANCY_BETA × (charging_count - 1) per step when charging_count > 1; calibrated so a second agent charging is net-negative unless battery < 45% (A3)
- GAUSS_SIGMA_CHARGING = 5.0 m (asymmetric vs. GAUSS_SIGMA_EVENT = 10.0 m): provides a meaningful spatial gradient toward the charger at typical approach distances; equal σ=10 in a 20×20 world is nearly flat and produced deadlocks when multiple agents simultaneously reached critical battery (A9)

**Key parameters:** CHARGING_SHAPING_GATE_ENABLED = true; CHARGING_SHAPING_BATTERY_THRESHOLD = 0.80; CHARGING_URGENCY_RAMP_POWER = 1.0; CHARGING_SHAPING_SCALE = 0.30; CHARGER_OCCUPANCY_BETA = 0.30; GAUSS_SIGMA_CHARGING = 5.0 m

#### 5.8.3 Disaster response reward
- DISASTER_REWARD_SWITCH_ON_DETECTION = true: disaster-response reward is active only after at least one agent has detected the disaster; before detection, agents receive no incentive to approach an undetected disaster based on ground-truth position (which they cannot observe)
- disaster_penalty = -GAUSS_BETA_EVENT × Gaussian(dist_to_disaster, GAUSS_SIGMA_EVENT) per alive non-charging agent; wide Gaussian (σ=10.0 m) provides attraction from across the world
- Abandonment penalty: -ABANDONMENT_PENALTY per step when the disaster has been detected but no agent is currently observing it; prevents agents from returning to coverage after detection and abandoning the disaster
- Symmetric decomposition: coverage_weight_i + charging_weight_i = 1; an agent cannot be simultaneously credited for both full coverage and full charging pull

**Key parameters:** DISASTER_REWARD_SWITCH_ON_DETECTION = true; GAUSS_BETA_EVENT = 1.0; GAUSS_SIGMA_EVENT = 10.0 m; ABANDONMENT_PENALTY = -0.50

#### 5.8.4 Death penalty
- One-time penalty of DEATH_PENALTY = -10.0 at the step an agent's battery reaches zero
- Ongoing penalty of -1.0 per step per dead agent for the remainder of the episode (dead agents cannot contribute to coverage or detection)
- Calibration: the one-time penalty must be large enough to motivate charging; the ongoing penalty must persist long enough that dying early is genuinely worse than dying late

**Key parameters:** DEATH_PENALTY = -10.0; ONGOING_DEATH_PENALTY = -1.0

---

### 5.9 Metrics tracking (≈ 1 page)
The State struct includes a Metrics substruct logged per episode:
- total_coverage_ratio, coverage_measurement_count: running sum and count for average priority-area coverage (computed only during non-disaster steps)
- disaster_active_steps: steps with disaster active
- total_disasters_spawned, disasters_missed: disaster lifecycle
- total_charging_sessions, appropriate_charging_count, inappropriate_charging_count: charging behavior (appropriate = battery < 50% at session start)
- death_count
- cumulative_detection_latency, detection_count: latency statistics
- coverage_penalty, charging_penalty, repulsion_penalty, charger_occupancy_penalty, abandonment_penalty, death_penalty, ongoing_death_penalty, movement_penalty, per_step_reward: per-step reward component logging

---

## Chapter 6 — Results (≈ 18–22 pages)

### 6.1 Experimental protocol (≈ 2 pages)
- Training: 15M loop steps, 4 parallel environments, 4 seeds per configuration
- Evaluation: every 50,000 loop steps, 100 deterministic episodes (initial_state with fixed seed), 1300 steps each
- Hardware: [fill in GPU/CPU specs, wall-clock time per run]
- All results: mean ± std over seeds; learning curves show mean with shaded std band

#### 6.1.1 Training setup and SAC hyperparameters
Reproducibility table; this is the deep-model counterpart to the tabular schedules stated in §3.2.3. Values are `static constexpr` in the config: read them from `src/rl/zoo/oil_platform-v1/sac.h` and the SAC loop config before filling.
| Group | Hyperparameters |
|---|---|
| Optimization | discount γ; actor / critic / temperature learning rates; optimizer (Adam) |
| SAC | Polyak τ; target entropy; log-std bounds; initial temperature and whether it is learned |
| Replay | buffer capacity; batch size; warmup steps; training interval |
| Networks | actor width/depth; critic width/depth; activation |
| Rollout | number of parallel environments; control frequency (DT); episode-length range |

### 6.2 Baseline and ablation configurations
List all 10+ ablation configs (corresponding to the table in Chapter 5):
- A0: No battery, no disaster (coverage only — clean baseline)
- A1: + Battery + disaster, no charging gate — exposes charger clustering pathology
- A2: + Battery-urgency gate (CHARGING_SHAPING_GATE_ENABLED = true)
- A3: + Charger occupancy penalty and reduced shaping scale (CHARGER_OCCUPANCY_PENALTY_ACTIVE = true, CHARGING_SHAPING_SCALE = 0.3)
- A4: + Equal-sigma expansion (GAUSS_SIGMA_CHARGING = GAUSS_SIGMA_EVENT = 10 m)
- A5: + Relative inter-agent velocity observation (OTHER_AGENTS_OBSERVE_RELATIVE_VELOCITY = true)
- A6: + Randomized episode length (RANDOMIZE_EPISODE_LENGTH = true)
- A7: + remaining_normalized in privileged critic (PRIVILEGED_OBSERVE_REMAINING_NORMALIZED = true)
- A8: + Full privileged critic (absolute positions + ground-truth disaster state)
- A9: + Asymmetric sigma (GAUSS_SIGMA_CHARGING = 5 m < GAUSS_SIGMA_EVENT = 10 m) — **best config**
- B1: PPO with same observation/reward as A9 (algorithm comparison)
- B2: Multi-Gaussian coverage instead of sensor-range coverage, else = A9 (reward formulation comparison)

### 6.3 Main results (≈ 4 pages)
- Learning curves: return/mean, coverage/priority_area, share_terminated vs. training steps for all configs
- Final performance table: all metrics for all configs at 15M steps
- Key observations to highlight:
  - A0→A1: adding battery drops coverage (charger clustering)
  - A1→A2: charging gate restores coverage to near-A0 level
  - A2→A3: occupancy penalty further improves coverage (fewer group charging interruptions)
  - A4→A5: relative velocity improves detection rate (better coordination during disaster phase)
  - A6→A7: critic stability; learning curves smoother once remaining_normalized is added to the critic
  - A8→A9: best config; highest coverage + detection + survival

### 6.4 Behavioral analysis (≈ 4 pages)
For best config (A9), show:
- **Coverage phase (no disaster):** trajectory plot over 200 steps; agents spread across ROI; one agent returns to charger every ~500 steps; turn-taking pattern clearly visible
- **Disaster detection:** trajectory plot from disaster spawn to detection; agent(s) near disaster detect within X steps; other agents continue coverage
- **Disaster response:** trajectory plot showing swarm converging toward disaster after detection; agents maintain ≥1 observer at all times (abandonment penalty effect)
- **Charging during disaster:** trajectory plot showing low-battery agent departing disaster, visiting charger, returning; other agents maintain coverage/detection during absence

For key pathologies (A1, A2 without fix), show corresponding trajectory plots demonstrating the failure mode.

### 6.5 PPO vs. SAC comparison (≈ 2 pages)
- Convergence speed: SAC reaches 90% of final performance in X steps vs. PPO in Y steps
- Final performance: SAC vs. PPO on all metrics
- Policy variance: SAC lower variance in coverage metric (off-policy, more sample-efficient)
- Failure modes: PPO more prone to charger clustering; SAC more prone to policy collapse when reward shaping is incorrect

### 6.6 Sensor-range coverage vs. multi-Gaussian comparison (≈ 2 pages)
- Coverage metric: sensor-range coverage higher by X% at convergence
- Qualitative difference: multi-Gaussian agents cluster at Gaussian peaks; sensor-range coverage agents spread continuously
- Learning curves: sensor-range coverage converges faster (cleaner gradient signal)

### 6.7 Scaling: number of agents (≈ 2 pages)
- N = 2, 3, 4 agents; ROI fixed at same size
- Coverage vs. N: near-linear improvement in coverage fraction
- Detection latency vs. N: decreasing; 4 agents achieve X steps average latency
- Survival vs. N: similar across N (charging coordination becomes harder with more agents at same charger)
- Discussion: the single-charger bottleneck becomes the limiting factor at N≥4

### 6.8 Sensitivity analysis (≈ 1 page)
- Vary CHARGING_SHAPING_BATTERY_THRESHOLD: 0.6, 0.7, 0.8, 0.9 → U-shaped curve; too low = agents don't return in time; too high = constant pull
- Vary CHARGER_OCCUPANCY_BETA: 0.1, 0.3, 0.5 → trade-off between group charging suppression and emergency charging ability
- Vary ABANDONMENT_PENALTY: 0, -0.25, -0.5, -1.0 → too high suppresses coverage; -0.5 is the balance point

### 6.9 Reward engineering as a research methodology (≈ 1 page)
- One-change-at-a-time ablation (A0→A9) is essential for disentangling multi-objective reward interactions; changing multiple terms simultaneously makes it impossible to attribute improvements or regressions to specific design choices
- The behavioral pathologies addressed in §5.8 (charger clustering, group charging, hovering exploit, terminal-state battery gaming) emerge from the combination of shared physical resources, multi-objective rewards, and finite episodes with a discount factor; they are likely to recur in any resource-constrained cooperative MARL problem
- The browser-based visualization (Chapter 4, §4.5) made the methodology tractable: every pathology was first identified visually before being confirmed with logged metrics; without visual debugging, identifying charger clustering from numeric metrics alone would have required many more training runs
- Generalizable pattern: gate-then-shape (first gate the resource behavior with a binary condition, then shape its spatial intensity) is more robust than shaping from scratch; the battery-urgency gate (A2) before the spatial shaping scale (A3) is the clearest instance

### 6.10 The CTDE asymmetry finding (≈ 1 page)
- Config. A8 (privileged critic with ground-truth disaster state, without remaining_normalized) outperforms A6 without any privileged information, confirming that centralized training genuinely helps beyond purely decentralized training in this environment
- remaining_normalized is the critical component: A8 alone is less stable than the A7+A8 combination present in the final best config A9; the critic needs horizon information to produce consistent Bellman targets under variable episode length (see §5.7.2)
- Implication for MARL system design: whenever episodes have variable or unknown duration (most real deployments), the privileged critic should receive remaining episode time; without it, the same (state, action) pair yields different Q-value targets depending on episode position, producing contradictory gradient signals
- The strictly privileged information in this thesis (ground-truth disaster position and velocity, physically unavailable to deployed actors) goes beyond standard CTDE; the empirical result confirms that CTDE benefits scale with the degree of privileged information asymmetry

---

## Chapter 7 — Conclusion (≈ 5–6 pages)

### 7.1 Summary of contributions
- Restate C0a, C0b, C1–C5 with brief evidence from the results, structured as a progression:
  - C0a, C0b: field surveys establishing energy management as the key open problem and CTDE as the right paradigm (Chapter 3)
  - C1: tabular MARL for AUV inspection confirming cooperation from individual rewards (Chapter 3)
  - C2: multi-agent SAC extension to rl-tools — permutation buffer, `ConfigApproximatorsMLPMultiAgent`, browser visualization (Chapter 4)
  - C3: oil platform environment — kinematics, battery model, stochastic disaster, sensor-range coverage reward (Chapter 5, §5.1–5.8)
  - C4: systematic reward engineering — 10-configuration ablation resolving four behavioral pathologies (Chapter 5, §5.8)
  - C5: experimental validation — ablation learning curves, behavioral analysis, algorithm and reward comparisons (Chapter 6)
- **The energy management thread:** quote the Future Work section of the published underwater paper ("A key aspect not considered in this study is energy consumption...") and state directly: "This thesis is the answer to that open problem." The oil platform project is the generalization of the AUV work to continuous state/action spaces with a single shared charging station, explicit battery constraints, and stochastic event response

### 7.2 Answers to the research questions
For each RQ1–RQ5, one paragraph stating the answer supported by evidence from Chapters 3–7:
- **RQ1** (tabular MARL cooperation from individual rewards, no explicit incentives): answered positively in Chapter 3; collective learning also accelerates convergence speed, not just steady-state performance; chain formation is an emergent cooperative strategy
- **RQ2** (extending rl-tools for multi-agent continuous control): answered in Chapter 4; the permutation buffer + `ConfigApproximatorsMLPMultiAgent` provides a drop-in multi-agent SAC extension with zero changes to the existing training loop
- **RQ3** (reward formulation for joint coverage, detection, and battery management): answered in Chapter 5 §5.8; the sensor-range coverage reward with battery-urgency weighting, disaster switch on detection, and charging gate is the key combination
- **RQ4** (behavioral pathologies in multi-objective resource-constrained MARL): answered in Chapter 5 §5.8; four pathologies identified and resolved through one-change-at-a-time ablation; the gate-then-shape pattern is the reusable design principle
- **RQ5** (asymmetric CTDE with variable episode length): answered in Chapter 6 §6.10; privileged remaining_normalized is the critical component; the benefit of CTDE scales with the degree of privileged information asymmetry

### 7.3 Limitations
- Single charging station: with multiple chargers the occupancy penalty and turn-taking analysis change qualitatively; the current formulation penalizes simultaneous charging regardless of which charger is occupied
- Homogeneous agents: all drones have identical capabilities, sensors, and battery capacities; real platforms may have heterogeneous fleets where parameter sharing is no longer appropriate
- 2D world: real UAV surveillance is 3D; altitude management, wind disturbance, and 3D collision avoidance are not modelled
- Unlimited communication range: all agents' full states are directly observable by all other agents at all times; real deployments require explicit bandwidth-limited communication protocols
- Bootstrapping at episode truncation: truncated episodes should use a bootstrapped value estimate but rl-tools currently treats them as terminated — a small bias accepted as an approximation

### 7.4 Future work and broader implications
- **Recurrent actor (LSTM):** agents currently lose disaster position information the moment no agent is detecting; a recurrent actor could maintain a belief state about last-known disaster position
- **Multi-charger coordination:** dynamic assignment of agents to multiple chargers; auction-based or learned turn-taking; the occupancy penalty must generalize to per-charger counts
- **Sim-to-real transfer:** the continuous action space and first-order inertia model (α = 0.6) are physically grounded; direct transfer experiments on real quadrotor platforms are the natural next step
- **Heterogeneous roles:** separate actor networks per role (scout / first responder / charger duty); requires departure from pure parameter sharing; could resolve the simultaneous-critical-battery deadlock
- **Explicit communication:** message-passing between agents to signal charging intent or disaster position; could be implemented as a learned communication channel alongside the existing observation structure
- **rl-tools framework extensions:** add truncated flag to replay buffer for correct handling of step-limit transitions; multi-GPU training for scaling to larger swarms (N > 8)
- **For MARL practitioners:** the gate-then-shape pattern and the one-change-at-a-time ablation methodology are reusable templates for any resource-constrained cooperative MARL problem; the behavioral pathologies documented here are a diagnostic checklist
- **For autonomous infrastructure monitoring:** the trained policy demonstrates that a small fleet of battery-constrained UAVs can maintain persistent coverage and rapid disaster response without centralized coordination — a step toward real deployment on oil platforms and similar critical infrastructure

---

## Appendix A — rl-tools Multi-Agent Actor: API Reference (≈ 4 pages)

Complete documentation of:
- `ActorSpec` struct and all template parameters
- `ActorGradient` public interface: `forward`, `backward`, `evaluate`, `evaluate_step`
- `ActorBuffer` members and their purpose
- `ConfigApproximatorsMLPMultiAgent` template usage example
- `apply_permutation` / `apply_inverse_permutation` signatures and complexity
- How to add a new environment to rl-tools zoo (step-by-step, using oil_platform-v1 as example)

---

## Appendix B — Environment Parameter Reference (≈ 4 pages)

Complete table of all `DefaultParameters` fields with:
- Name, type, default value
- Description
- Which ablation configuration (A0–A9) introduced or modified it
- Whether it is active in the final best configuration

---

## Appendix C — Visualization System (≈ 2 pages)

- Screenshot of the render function output with all elements labeled
- Description of the WebSocket JSON format
- Code excerpt showing the JavaScript canvas update loop
- Instructions for running the visualization during training

---

## Appendix D — Discarded Reward Formulations (≈ 2 pages)

Documents alternative reward designs that were tested and rejected, with the reason for rejection. Included here rather than in the main text because the final design is what matters; these are provided for reproducibility and as a reference for practitioners facing similar choices.

### D.1 Multi-Gaussian coverage
- 5 Gaussian bumps: 1 platform (round) + 4 pipe arms (elliptical); see `build_gauss_catalogue()`
- Softmax aggregation with temperature τ = 0.5: smooth max over bumps
- Per-agent reward: `coverage_potential_i = exp(-0.5·((x-cx)/σx)² - 0.5·((y-cy)/σy)²) × A`
- Rejection reason: two agents near the same Gaussian both get positive reward → overlap incentive; agents cluster at bump peaks rather than spreading along arms; no sensor-range enforcement
- Comparison table vs. sensor-range coverage: overlap incentive, sensor enforcement, spatial spreading, gradient smoothness, computational cost

### D.2 Equal-sigma charging/disaster Gaussians
- Initial fix for disaster-phase coordination used GAUSS_SIGMA_CHARGING = GAUSS_SIGMA_EVENT = 10.0 m
- Rejection reason: σ=10 in a 20×20 world produces a nearly flat gradient (≈0.02/m); provides no spatial direction signal; all agents reaching critical battery simultaneously produced a symmetric deadlock with no gradient toward the charger
- Replaced by asymmetric σ: GAUSS_SIGMA_CHARGING = 5.0 m, GAUSS_SIGMA_EVENT = 10.0 m (A9)

### D.3 Point-of-no-return death (POINT_OF_NO_RETURN_DEATH_ENABLED)
- Variant: agent dies as soon as battery is estimated to be insufficient to reach the charger
- Rejection reason: requires accurate distance-to-charger estimate and discharge model; adds implementation complexity; the gradual death penalty + battery urgency shaping achieves the same behavioral goal without the additional approximation

---

- Screenshot of the render function output with all elements labeled
- Description of the WebSocket JSON format
- Code excerpt showing the JavaScript canvas update loop
- Instructions for running the visualization during training

---

## Figures Checklist (full thesis)

- [ ] Fig. 1.1: Overview diagram — thesis structure and contribution mapping
- [ ] Fig. 2.1: CTDE schematic — centralized training (all agents + global state → centralized critic) vs. decentralized execution (local obs → actor); annotate privileged info gap (disaster GT, remaining time)
- [ ] Fig. 2.2: Algorithm comparison table (visual) — rows: MADDPG, QMIX, MAPPO, MASAC, this work; columns: action space, on/off-policy, parameter sharing, privileged critic, C++ deployment
- [ ] Fig. 2.3: Coverage control comparison (visual) — Lloyd's centroid update vs. Gaussian potential vs. sensor-range coverage (side-by-side diagram showing cell assignment and overlap behavior for 2 agents)
- [ ] Fig. 3.1–3.5: Reproduced from published paper (with permission): H-SURF robot, pipeline scenario, state graph, Q-matrix policy visualization, chain formation behavior
- [ ] Fig. 3.6: Taxonomy diagram from mission planning survey (Fig. 2 of [C0b]) — centralized vs. decentralized methods tree
- [ ] Fig. 3.7: Underwater swarms overview diagram (Fig. 1 of [C0a]) — collective behaviors, communication, use cases
- [ ] Fig. 4.1: rl-tools architecture overview (component diagram)
- [ ] Fig. 4.2: Multi-agent actor architecture (multi_agent_wrapper + permutation buffer + SAS)
- [ ] Fig. 4.3: Permutation forward/backward (matrix diagram for N=3, K=2)
- [ ] Fig. 4.4: Screenshot of browser-based render function with annotations
- [ ] Fig. 5.1: Oil platform environment overview (ROI, grid, charger, disaster example)
- [ ] Fig. 5.2: Battery model state machine (idle / charging / dead transitions)
- [ ] Fig. 5.3: Disaster detection probability curve (logistic, p_det vs. distance)
- [ ] Fig. 5.4: Actor vs. privileged critic observation structure diagram
- [ ] Fig. 5.5: Reward decomposition overview diagram
- [ ] Fig. 5.6: Sensor-range coverage vs. Gaussian coverage comparison (visual, side-by-side)
- [ ] Fig. 5.7: Battery urgency and symmetric weight decomposition (plot)
- [ ] Fig. 5.8–5.12: Trajectory screenshots for each major pathology and its fix
- [ ] Fig. 6.1: Learning curves for all ablation configs
- [ ] Fig. 6.2: Final performance table (heatmap)
- [ ] Fig. 6.3: Trajectory plots for best config (4 panels: coverage / detection / response / charging)
- [ ] Fig. 6.4: PPO vs. SAC learning curves
- [ ] Fig. 6.5: Sensor-range coverage vs. Gaussian ablation
- [ ] Fig. 6.6: Scaling analysis (N=2,3,4)
- [ ] Fig. 6.7: Sensitivity analysis plots

---

## Estimated Page Count by Chapter

| Chapter | Topic | Target pages | Notes |
|---|---|---|---|
| Front matter | — | 5 | |
| 1 | Introduction | 10–12 | |
| 2 | Background and State of the Art | 12–15 | |
| 3 | Preparatory Works | 20–25 | |
| 4 | rl-tools extension | 18–22 | |
| 5 | Environment and Reward Design | 14–18 | §5.1–5.8 written (11 pp); §5.9 remaining (~1–2 pp); balance from figures |
| 6 | Results | 20–24 | |
| 7 | Conclusion | 5–6 | |
| Appendices A–C | Reference / supplementary | 10 | |
| Appendix D | Discarded reward formulations | 2–3 | Multi-Gaussian, equal-sigma, point-of-no-return |
| References | — | 8–10 | |
| **Total** | | **~124–150** | |
