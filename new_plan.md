# Oil Platform SAC — Fix Plan

## Issues to fix

**Issue 1:** During coverage phase, agents cluster near the charger instead of spreading over the ROI.

**Issue 2a:** During disaster following, agents with low battery fail to leave in time and die.

**Issue 2b:** After charging, agents stay at the charger instead of returning to the disaster.

---

## Root causes

### Issue 1
`CHARGING_SHAPING_GATE_ENABLED = false` means every non-charging agent always has a
charging proximity pull proportional to `1 - battery_normalized`, even at 90% battery.
This creates a constant low-level pull toward the charger that biases agents to stay near it
during coverage. The charger at (5,5) also happens to cover a subset of the ROI (left
horizontal pipe arm, lower vertical pipe, nearest platform corner), making it a stable
local optimum.

### Issue 2a
Two compounding problems:
1. `GAUSS_SIGMA_CHARGING = 2.0` (= CHARGING_STATION_RANGE) is too narrow. The charging
   proximity Gaussian is essentially zero beyond ~4 units from the charger. An agent at the
   disaster 10–15 units away feels no gradient toward the charger, even with critically low
   battery. The escalating charging penalty (charging_weight grows as battery drains) IS
   present in the reward but there is no shaped gradient to translate it into movement.
2. The previous attempt at enabling the gate used threshold=0.75 with quadratic ramp (power=2),
   making urgency negligibly small across most of the battery range and too close to the
   charging eligibility boundary.

### Issue 2b
`GAUSS_SIGMA_EVENT = SENSOR_RANGE/3 ≈ 1.67` is extremely narrow. After charging, an agent
at the charger (5,5) with disaster at (15,10) gets coverage_potential ≈ 0. There is no
gradient toward the disaster from far away. The coverage penalty is flat regardless of whether
the agent moves toward or away from the disaster — it only improves in the last few units.

---

## Parameter changes

All changes are in `oil_platform.h` (`DefaultParameters`).

### Step 1 — Enable and re-tune the charging gate

This is the primary fix for Issue 1. The gate removes the charging pull entirely when battery
is high, letting coverage dominate.

| Parameter | Current | New |
|---|---|---|
| `CHARGING_SHAPING_GATE_ENABLED` | `false` | `true` |
| `CHARGING_SHAPING_BATTERY_THRESHOLD` | `0.75` | `0.80` |
| `CHARGING_URGENCY_RAMP_POWER` | `2.0` | `1.0` |

**Rationale for threshold = 0.80:**
- Aligns exactly with `MINIMUM_BATTERY_FOR_CHARGING = 80` (charging becomes eligible below 80%)
- No pull at ≥ 80% battery → agents do not feel pulled toward the charger before they need to charge
- Pull grows naturally as battery drains below 80%: urgency = (0.80 − battery) / 0.80
  - At 60%: urgency = 0.25 (gentle)
  - At 40%: urgency = 0.50 (moderate)
  - At 20%: urgency = 0.75 (strong)

**Rationale for linear ramp (power = 1.0):**
- Quadratic ramp at power=2 gave negligible urgency across most of the battery range.
  Example with threshold=0.75: urgency at 60% battery = ((0.75−0.60)/0.75)² = 0.04. Too small.
- Linear ramp gives urgency = 0.25 at 60% battery — a usable gradient across the whole range.

### Step 1.5 — Fix hovering exploit and emergency charging blockage

Must be applied before Step 2. Two issues observed after Step 1:

1. **Hovering near charger without charging.** With `CHARGING_SHAPING_SCALE = 1.0` an agent
   hovering at the charger (proximity ≈ 1.0) gets the same `fleet_charging_value` contribution
   as an agent that is actually charging. The reward is identical whether or not `is_charging`
   is true, so the policy has no incentive to stop and initiate a charging session. This becomes
   worse after Step 2 because sigma=10 makes proximity ≈ 0.98 even at distance 2 (just outside
   `CHARGING_STATION_RANGE`), further flattening the gradient near the charger.

2. **Occupancy penalty blocking emergency charging.** With `CHARGER_OCCUPANCY_BETA = 1.0` the
   penalty for a second simultaneous charger (−1.0/step) can outweigh the urgency signal of a
   critically low agent and prevent it from starting a session.

| Parameter | Current | New |
|---|---|---|
| `CHARGING_SHAPING_SCALE` | `1.0` | `0.3` |
| `CHARGER_OCCUPANCY_BETA` | `1.0` | `0.3` |

**Rationale for CHARGING_SHAPING_SCALE = 0.3:**

The charging penalty for an agent at the charger (proximity ≈ 1.0, `charging_weight = cw`):
- Hovering (not charging): penalty = `−cw × (1 − 0.3) = −0.7 × cw`
- Actually charging: penalty = `0`

Gap of `0.7 × cw` per step creates a clear incentive to initiate the charging session.
At 40% battery (`cw = 0.5`): gap = 0.35/step — meaningful relative to other terms.

The approach gradient from far away (Step 2, sigma=10, D=14, cw=0.5) reduces from
0.010/unit to 0.008/unit — still fully learnable, since directional guidance only
requires a non-zero gradient, not a large one.

**Rationale for CHARGER_OCCUPANCY_BETA = 0.3:**

Net fleet benefit of a second agent charging = `0.7 × cw₂ − 0.3`.
This is positive when `cw₂ > 0.43`, i.e., battery below ≈ 45%.

| Battery | Should second agent charge? |
|---|---|
| 10% (cw=0.875) | ✓ net +0.31 — emergency overrides penalty |
| 30% (cw=0.625) | ✓ net +0.14 |
| 45% (cw≈0.44) | break-even |
| 60% (cw=0.25) | ✗ net −0.12 — wait turn |
| 79% (cw≈0.01) | ✗ net −0.29 — strongly defer |

Emergency charging (battery < 45%) remains beneficial despite the occupancy penalty;
moderate/high battery agents respect turn-taking.

### Step 2 — Increase both Gaussians together

Fix Issues 2a and 2b. The two sigmas must be changed together and kept equal to preserve
the designed property that battery urgency alone governs the charger-vs-disaster tradeoff.

| Parameter | Current | New |
|---|---|---|
| `GAUSS_SIGMA_CHARGING` | `2.0` | `10.0` |
| `GAUSS_SIGMA_EVENT` | `SENSOR_RANGE/3 ≈ 1.67` | `10.0` |

**Rationale for equal sigmas:**
- During disaster phase, low-battery agents feel two opposing pulls: sigma_event toward the
  disaster, sigma_charging toward the charger.
- If sigma_charging >> sigma_event: charger pull dominates → agents leave disaster too early.
- If sigma_event >> sigma_charging: disaster pull dominates → agents stay too long and die.
- Equal sigmas mean the crossover (charger pull = disaster pull) is determined solely by
  `charging_weight = coverage_weight`, i.e., at urgency = 0.5. With threshold=0.80 and
  linear ramp: urgency = 0.5 → battery = 40%.
- At the midpoint between charger and disaster, agents with battery < 40% prefer charger;
  agents above 40% prefer to stay at disaster. This is the intended behavior.

**Rationale for sigma = 10.0 (= SENSOR_RANGE × 2):**
- Worst-case disaster position: ~15 units from charger (pipe arm tips).
- With sigma=10 at distance 15: proximity = exp(−225/200) ≈ 0.32. Non-trivial signal.
- At the disaster position (gradient = 0 of coverage Gaussian at r=0), the first steps away
  from disaster cost essentially zero coverage. The charger gradient (charging_weight × 14/100
  × 0.14 ≈ 0.010/unit at 40% battery) is the dominant gradient.
- Once departing the disaster at ~35% battery, travel cost for 15 units ≈ 22.5% battery;
  agent arrives with ~12.5%, below the 80% cap → charging eligible. Safe.
- Large sigma provides gradient from far away for directional guidance. Precise docking at
  charger (within CHARGING_STATION_RANGE=2, velocity < 0.75) and precise disaster approach
  are handled by policy learning from the observed relative positions, not by the Gaussian
  gradient alone.
- The abandonment penalty (−0.5/step when disaster known but no agent detects) ensures agents
  approach within actual sensor range even if the broad coverage Gaussian would otherwise
  reward stopping slightly farther away.

---

## Testing order

**Test 1:** Apply Step 1 only. Train briefly (~1M steps), verify:
- Agents spread across ROI during coverage (Issue 1 resolved)
- No regression in charging behavior (agents still charge)

**Test 2:** Apply Step 2 on top of Step 1. Train, verify:
- Agents leave disaster to charge proactively (Issue 2a)
- Agents return to disaster after charging (Issue 2b)
- No over-frequent charging during coverage (gate protects high-battery agents)

**If Issue 2a persists near distant disasters:** Increase `GAUSS_SIGMA_CHARGING` (and
`GAUSS_SIGMA_EVENT` equally) toward 12.0. Do not change them independently.

---

## Step 2.5 — Fully relative inter-agent observations

### Issue

Other agents' **positions** are encoded as `(other_pos - self_pos) / GRID_SIZE` (relative),
but their **velocities** are encoded as `other_vel / MAX_SPEED` (absolute world-frame). This
mixed frame means the network cannot trivially predict where agent j will be next step — it
must implicitly learn to subtract its own velocity from the other agent's absolute velocity to
compute approach/separation rate.

### Fix

Add flag `OTHER_AGENTS_OBSERVE_RELATIVE_VELOCITY` to `DefaultParameters`. When `true`,
other agents' velocities are encoded as:

```
(other_vel - self_vel) / (2 * MAX_SPEED)  ∈ [-1, 1]
```

This gives a fully relative inter-agent frame consistent with the relative position encoding.
Division by `2 * MAX_SPEED` (not `MAX_SPEED`) keeps the range in [-1, 1] since relative
velocity spans [-2*MAX_SPEED, 2*MAX_SPEED].

| Parameter | Value |
|---|---|
| `OTHER_AGENTS_OBSERVE_RELATIVE_VELOCITY` | `true` |

**Why fully relative is more principled:**
- Coverage and coordination are translationally invariant: "agent j is closing on me from the
  right" is the same policy trigger regardless of absolute world position.
- Keeping positions relative but velocities absolute forces the network to learn the frame
  conversion implicitly, adding representational burden.
- Switching positions to absolute (the alternative) would be inconsistent with disaster and
  charger positions, which remain relative.

**Note:** `GAUSS_SIGMA_COVER` is dead code in this configuration (`USE_VORONOI_COVERAGE = true`)
and is unaffected by this change.

**Requires retraining from scratch** — the observation layout is unchanged (same dimensions),
but the values in dims `base+2` and `base+3` of each other-agent slot now carry different
semantics.

---

## Step 3 — Randomize episode length to fix end-of-episode behavior

### Issue

Agents show inconsistent survival behavior during the last ~50 steps of the episode. They stop
returning to the charger even when battery is critically low.

### Root cause

This is a **finite-horizon discounting effect**, not a policy failure. Two mechanisms interact:

1. **The value function implicitly encodes episode position.** Without an explicit step counter
   in the observation, the agent still infers approximate episode age through correlated features:
   - Battery level is a partial clock (drains at `DISCHARGE_RATE_BASE = 0.15/step`)
   - Disaster state constrains episode age (disaster spawns at 0.01/step, so late-episode
     disaster + already-charged-once implies the episode is old)
   - The critic learns these correlations during training

2. **Late-episode death is genuinely cheaper in the discounted objective.** With `GAMMA = 0.99`:
   - Die at step 100: pay `DEATH_PENALTY` + ~900 steps of `ongoing_death_penalty` (large sum)
   - Die at step 990: pay `DEATH_PENALTY` + only ~10 steps of `ongoing_death_penalty`
   - The discounted cost of dying at step 990, viewed from step 950, is 0.99^40 × cost ≈ 0.67 × cost
   - The agent is solving the correct discounted objective; the behavior is mathematically rational

### Fix

Randomize `EPISODE_STEP_LIMIT` at each episode reset by sampling from a uniform distribution.
This breaks the implicit time signal: no observable feature can reliably predict when the episode
will end, so the agent cannot rationally discount survival.

Note: `GAUSS_SIGMA_COVER` does **not** need to be changed — it is dead code when
`USE_VORONOI_COVERAGE = true` (coverage_potential is set to 0 in the per-agent loop and computed
collectively via Voronoi after the loop; sigma is never read during coverage phase).

| Parameter | Current | New |
|---|---|---|
| `EPISODE_STEP_LIMIT_MIN` | — | `700` |
| `EPISODE_STEP_LIMIT_MAX` | — | `1300` |

Add `TI episode_step_limit` to the `State` struct and sample it during `reset()`. The SAC
config's `EPISODE_STEP_LIMIT` (used for replay buffer sizing) stays at the max value.

**Why these bounds:**
- Minimum 700: with `DISASTER_PROBABILITY_SPAWN = 0.01`, expected first spawn is step 100.
  Episodes shorter than ~500 steps risk having too few disaster-phase steps for effective
  learning. 700 gives ample disaster exposure in all episodes.
- Maximum 1300: keeps episode length within 30% of the nominal 1000, preserving training
  efficiency and replay buffer statistics.
- Range [700, 1300]: wide enough that no battery level or disaster-state combination reliably
  predicts episode end. An agent at 20% battery could face 300 more steps or 5 — it cannot
  safely assume the episode is ending.

**What this does NOT fix on its own:** The discounting effect itself persists — if the episode
happens to end in 5 steps, the Q-value for "go charge" is still suppressed. But since the agent
cannot predict when that will happen, the policy must learn to charge whenever battery is
critical, averaged across all possible remaining durations.

**Observed failure mode without the complement below:** Randomization alone causes critic loss
explosion (spikes to 30+, gradient norms to 200+) because the same (obs, action) pair appears
at different points in episodes of different lengths, giving the critic inconsistent Bellman
targets for identical inputs. The policy regresses: agents die more frequently and struggle to
follow the disaster (share_terminated stays ~0.7 vs ~0.05 without randomization).

### Step 3.5 — Add remaining time to the critic only (CTDE)

**Fix for the critic instability.** Add a single scalar to `ObservationPrivileged` (the critic's
input) encoding how much of the episode remains:

```
remaining_normalized = max(episode_step_limit − step_count, 0) / EPISODE_STEP_LIMIT_MAX  ∈ [0, 1]
```

Normalized by the constant `EPISODE_STEP_LIMIT_MAX = 1300` so the value is comparable across
episodes of different sampled lengths. Reaches 0 at the terminal step.

**Why critic-only (CTDE pattern):**
- The critic's role is to estimate Q(s, a) = expected future return. Without remaining time,
  identical (obs, action) pairs at different episode positions have different Q-values, causing
  inconsistent Bellman targets and loss explosions.
- Adding remaining time to `ObservationPrivileged` gives the critic the information to learn
  time-conditional Q-values — eliminating the inconsistency.
- The actor does NOT receive this scalar. It is trained against the improved critic gradients
  and learns a time-averaged policy that works across variable episode lengths without
  observing the episode counter at deployment time. This follows the standard
  Centralized Training with Decentralized Execution (CTDE) paradigm.
- Adding remaining time to the actor would also be valid (missions have known durations in
  practice), but is not necessary to fix the critic instability and preserves deployment
  simplicity.

**Implementation changes:**
1. In `ObservationPrivileged` in `oil_platform.h`: bump `SHARED_DIM` from 8 → 9.
2. In the privileged `observe()` in `operations_generic.h`: set index `shared_offset + 8`
   to `remaining_normalized`.
3. `sac.h` critic input dim updates automatically from `ObservationPrivileged::DIM`.

### Implementation changes (Step 3)

1. Add to `DefaultParameters` in `oil_platform.h`:
   ```cpp
   static constexpr bool RANDOMIZE_EPISODE_LENGTH = false;  // set true to enable
   static constexpr TI EPISODE_STEP_LIMIT_MIN = 700;
   static constexpr TI EPISODE_STEP_LIMIT_MAX = 1300;
   ```
2. Add `TI episode_step_limit` field to `State` struct in `oil_platform.h`.
3. In stochastic `initial_state()` in `operations_generic.h`: sample `state.episode_step_limit`
   uniformly from `[PARAMS::EPISODE_STEP_LIMIT_MIN, PARAMS::EPISODE_STEP_LIMIT_MAX]` when flag
   is true; else use `PARAMS::EPISODE_STEP_LIMIT`.
4. In deterministic `initial_state()` (no RNG, used for eval): always use fixed
   `PARAMS::EPISODE_STEP_LIMIT`.
5. In `step()`: carry `state.episode_step_limit` forward to `next_state` — **critical**, without
   this `next_state.episode_step_limit` is zero-initialized and episodes terminate after 1 step.
6. In `terminated()`: check `state.step_count >= state.episode_step_limit` and set
   `terminate = true`. Log `episode/step_limit` alongside `episode/total_steps`.
7. In `sac.h`: keep `EPISODE_STEP_LIMIT = 1300` (the max) for replay buffer sizing.
8. In `operations_cpu.h`: display `Step: X/episode_step_limit` on the map canvas.

---

## Step 3.6 — Fix privileged critic observation

### Issue

The run implementing steps 1+1.5+2+2.5 was trained with a bug: `USE_PRIVILEGED_CRITIC_OBSERVATION`
defaulted to `false` in `ENVIRONMENT_FACTORY`, so the critic received the exact same observation
as the actor (`Observation`, 96 dims) instead of `ObservationPrivileged`. The critic had zero
centralization advantage.

### What the critic was actually seeing (the bug)

The actor's `Observation` is structured as N_AGENTS=3 concatenated per-agent blocks of 32 dims,
where every position is expressed relative to THAT agent's own frame:

- Block 0 (32 dims): agent 0's view — disaster/charger relative to agent 0, others relative to agent 0, one-hot ID [1,0,0]
- Block 1 (32 dims): agent 1's view — disaster/charger relative to agent 1, others relative to agent 1, one-hot ID [0,1,0]
- Block 2 (32 dims): agent 2's view — disaster/charger relative to agent 2, others relative to agent 2, one-hot ID [0,0,1]

The critic saw the disaster position three times in three different reference frames — redundant
and mixed. The one-hot agent IDs are meaningless for a global critic. There was no privileged
global state accessible beyond what the actor already processes.

### What ObservationPrivileged provides (the fix)

The privileged observation uses **absolute world coordinates** throughout:

- Per-agent: absolute position `2*(pos/GRID_SIZE)-1`, absolute velocity `vel/MAX_SPEED`,
  is_detecting, battery, dead, is_charging — same for all agents, no frame ambiguity
- Shared: ground-truth absolute disaster position + velocity (visible even when no agent
  detects it), absolute charger position, disaster_active flag, disaster_detected_global flag

| Parameter | Was (bug) | Now (fix) |
|---|---|---|
| `USE_PRIVILEGED_CRITIC_OBSERVATION` | `false` | `true` |
| `PRIVILEGED_OBSERVE_REMAINING_NORMALIZED` | — | `false` |
| `RANDOMIZE_EPISODE_LENGTH` | `false` | `false` |
| `GAMMA` | `0.99` | `0.99` |

Same reward and policy parameters as steps 1+1.5+2+2.5, but with the critic genuinely using
centralized information for the first time. The goal is to understand whether the privileged
critic alone changes convergence speed or final performance before making any other modifications.

**Requires retraining from scratch.**

---

## Step 3.7 — Add remaining_normalized to critic without randomization

Add a single scalar to `ObservationPrivileged` encoding how much of the episode remains:

```
remaining_normalized = max(episode_step_limit − step_count, 0) / EPISODE_STEP_LIMIT_MAX  ∈ [0, 1]
```

| Parameter | Value |
|---|---|
| `PRIVILEGED_OBSERVE_REMAINING_NORMALIZED` | `true` |
| `RANDOMIZE_EPISODE_LENGTH` | `false` |

**Why test before adding randomization:** With a fixed episode length, remaining_normalized is
deterministic and the actor can already infer approximate episode position from battery level.
However, giving it explicitly to the critic may improve Q-value accuracy and convergence speed
without adding the training difficulty of randomized episode lengths.

**Why critic-only (CTDE):** The actor should not observe remaining time — this keeps the policy
deployment-ready and prevents it from learning to discount end-of-episode survival.

If this run is stable and comparable to Step 3.8, it is the correct foundation before
introducing episode length randomization (Step 3 path).

**Requires retraining from scratch.**

---

## Step 3.8 — Re-run of Step 3.6 with metric bug fixed

Steps 3.6 and 3.7 had a bug in `terminated()`: the step-limit check fired regardless of
`RANDOMIZE_EPISODE_LENGTH`, making `terminated()` return `true` at the end of every episode
(either by death or by step limit). Since `share_terminated` counts every episode where
`terminated()` fires, this caused `share_terminated = 1` in all evaluation runs — making it
impossible to distinguish episodes ending in death from those completed successfully.

**Root cause:** The step-limit check inside `terminated()` is only necessary when
`RANDOMIZE_EPISODE_LENGTH = true` (because the SAC loop does not know the per-episode sampled
limit). With a fixed episode length the loop's own `EPISODE_STEP_LIMIT` handles truncation
externally. Including the check unconditionally caused it to fire on every episode end.

**Fix:** Gate the check with `if constexpr (PARAMS::RANDOMIZE_EPISODE_LENGTH)` in
`terminated()`. When false, `terminated()` only returns `true` for agent death — restoring the
original semantics and making `share_terminated` a meaningful death-rate metric.

| Parameter | Value |
|---|---|
| `USE_PRIVILEGED_CRITIC_OBSERVATION` | `true` |
| `PRIVILEGED_OBSERVE_REMAINING_NORMALIZED` | `false` |
| `RANDOMIZE_EPISODE_LENGTH` | `false` |
| `GAMMA` | `0.99` |

Same parameters as Step 3.6, but with the metric bug fixed. **Requires retraining from scratch.**

---

## Step 3.9 — Re-run of Step 3.7 with metric bug fixed

Same parameters as Step 3.7, but with the metric bug fixed.

| Parameter | Value |
|---|---|
| `USE_PRIVILEGED_CRITIC_OBSERVATION` | `true` |
| `PRIVILEGED_OBSERVE_REMAINING_NORMALIZED` | `true` |
| `RANDOMIZE_EPISODE_LENGTH` | `false` |
| `GAMMA` | `0.99` |

**Requires retraining from scratch.**

---

## Step 3.10 — Re-run of Step 3.9 with extended EPISODE_STEP_LIMIT (wrong parameter)

Intended to allow episodes up to 1300 steps to use the full [700, 1300] randomization range.
The change was made to `EPISODE_STEP_LIMIT` in `oil_platform.h` (DefaultParameters), but this
is the **wrong** parameter for this purpose.

**What was changed vs Step 3.9:**

| Parameter | Location | Value |
|---|---|---|
| `EPISODE_STEP_LIMIT` | `oil_platform.h` | `1300` (no effect on training) |
| `RANDOMIZE_EPISODE_LENGTH` | `oil_platform.h` | `true` |
| `PRIVILEGED_OBSERVE_REMAINING_NORMALIZED` | `oil_platform.h` | `true` |
| SAC loop `EPISODE_STEP_LIMIT` | `sac.h:36` | `1000` (unchanged — the real cap) |

**Why the change had no effect:** There are two separate `EPISODE_STEP_LIMIT` constants.
`PARAMS::EPISODE_STEP_LIMIT` in `oil_platform.h` is used for fixed-length episodes and the
`log_episode_end` metric check. The SAC loop's `EPISODE_STEP_LIMIT` in `sac.h:36` is the
external hard cap that calls `reset()` on the environment. With `sac.h` still at 1000,
episodes with sampled limit > 1000 (roughly 50%) continue to be truncated by the loop before
`terminated()` fires, so the effective episode length distribution is
`min(U[700,1300], 1000)` with mean ≈ 925 — identical to Step 3.9.

**Observed result:** Evaluation curves (episode_length/mean, share_terminated, return/mean)
are indistinguishable from Steps 3.8 and 3.9. Confirmed that the environment parameter alone
does not extend episode length.

**Additional code changes active in this run (vs Steps 3.6–3.9):**
- `episode/terminated_by_death` metric added — logs 1 when all agents dead, 0 when episode
  ends by step limit. Separates true death rate from the `share_terminated` metric which
  conflates deaths with randomized step-limit hits.
- `log_episode_end` fix — for fixed-length runs (`RANDOMIZE=false`), episode metrics now
  log at both death AND step-limit truncation, not only at death.
- Dead disaster OOB check removed from `terminated()` — this check was dead code since
  `step()` already deactivates the disaster before `terminated()` is called.

---

## Step 3.11 — Fix both EPISODE_STEP_LIMIT parameters and evaluation episode length

### Issue diagnosed from Step 3.10

There are two separate `EPISODE_STEP_LIMIT` constants that must be set independently, plus
a bug in how the deterministic `initial_state()` sets evaluation episode length.

The three problems:
1. **`sac.h:36` EPISODE_STEP_LIMIT = 1000** caps training rollouts at 1000 steps, truncating
   ~50% of episodes (those with sampled limit > 1000) before `terminated()` fires.
   Fix: raise to 1300.
2. **Deterministic `initial_state()` always uses `PARAMS::EPISODE_STEP_LIMIT`** for evaluation,
   regardless of `RANDOMIZE_EPISODE_LENGTH`. This makes evaluation non-representative: with
   EPISODE_STEP_LIMIT=1000, evaluation episodes differ from most training episodes (which sample
   limits up to 1300). With EPISODE_STEP_LIMIT=1300 (Step 3.10), evaluation changed but training
   didn't — explaining why those curves differed from Step 3.9 despite identical training.
   Fix: modify deterministic `initial_state()` to use `EPISODE_STEP_LIMIT_MAX` when
   `RANDOMIZE_EPISODE_LENGTH=true`, matching the maximum training episode length.
3. **`PARAMS::EPISODE_STEP_LIMIT` in oil_platform.h** should be reverted to 1000 (its use
   is now restricted to fixed-length runs only; it no longer affects the randomized case).

### Code change (applied)

In `operations_generic.h`, deterministic `initial_state()`:
```cpp
if constexpr (PARAMS::RANDOMIZE_EPISODE_LENGTH) {
    state.episode_step_limit = PARAMS::EPISODE_STEP_LIMIT_MAX;  // 1300: test hardest case
} else {
    state.episode_step_limit = PARAMS::EPISODE_STEP_LIMIT;
}
```

### Parameter changes

| Parameter | Location | Step 3.10 | Step 3.11 |
|---|---|---|---|
| `EPISODE_STEP_LIMIT` | `oil_platform.h` | `1300` | `1000` (reverted) |
| `EPISODE_STEP_LIMIT` | `sac.h:36` | `1000` | `1300` |
| `RANDOMIZE_EPISODE_LENGTH` | `oil_platform.h` | `true` | `true` |
| `PRIVILEGED_OBSERVE_REMAINING_NORMALIZED` | `oil_platform.h` | `true` | `true` |

**Why EPISODE_STEP_LIMIT_MAX for evaluation:** Evaluation at the maximum episode length (1300)
tests the most demanding case in the training distribution. It is consistent with the SAC loop
cap (also 1300), so `terminated()` fires at step 1300 for every non-death evaluation episode —
meaning every evaluation episode is properly terminated. `episode/terminated_by_death` correctly
distinguishes deaths from step-limit completions.

**Why evaluation is NOT the mean (1000) or randomized:** Using the mean would miss testing
the long-episode behavior the policy must handle. Randomizing evaluation lengths would require
code infrastructure (evaluation RNG) and reduces comparability across runs.

**Expected effects:**
- Training: full `U[700, 1300]` distribution — `terminated()` fires for every training episode.
- Evaluation: fixed 1300-step episodes — `share_terminated ≈ 1.0` (all episodes terminate at
  1300 or earlier by death), but `episode/terminated_by_death` shows the true death rate.
- `episode/terminated_by_death` logs for every training episode, filling the gaps from Step 3.10.

**Requires retraining from scratch.**

---

## Step 3.12 — Reduce GAUSS_SIGMA_CHARGING to create meaningful spatial gradient

### Issue

With `GAUSS_SIGMA_CHARGING = 10.0` in a 20×20 environment, the charging proximity Gaussian
is essentially flat. The gradient between the charger position and the disaster position is
only ~12% of the max proximity value, giving the policy no useful directional signal for
deciding *where* to move when battery is critical.

This causes a coordination deadlock in rare episodes where all 3 agents simultaneously
reach critical battery (~13–20%) while clustered near the disaster:
- All agents have nearly identical observations (same battery, same detection status)
- The shared-weight policy outputs nearly identical actions for all three
- The charging shaping signal provides no spatial asymmetry to break the deadlock
- None of the agents breaks away to charge; all die

Most episodes avoid this deadlock because agents typically have asymmetric battery levels
(e.g., one at 25%, others at 75%), allowing the shared-weight policy to map "I'm low,
others are high" → "I go charge". The deadlock occurs roughly 1 in 20–30 episodes when
all three agents happen to deplete symmetrically.

### Fix

Reduce `GAUSS_SIGMA_CHARGING` from `10.0` to `5.0`. `GAUSS_SIGMA_EVENT` stays at `10.0`.

| Parameter | Was | Now |
|---|---|---|
| `GAUSS_SIGMA_CHARGING` | `10.0` | `5.0` |
| `GAUSS_SIGMA_EVENT` | `10.0` | `10.0` (unchanged) |

Proximity comparison at key distances:

| Distance from charger | sigma=10 (current) | sigma=5 (new) |
|---|---|---|
| 0 (at charger) | 1.000 | 1.000 |
| 2 (edge of charge zone) | 0.980 | 0.923 |
| 5.4 (near disaster) | 0.863 | 0.558 |
| 10 (far area) | 0.607 | 0.135 |

For a 20%-battery agent at the disaster (5.4 units from charger), the per-step reward gain
from moving one unit toward the charger is:
- sigma=10: +0.023 (nearly flat — no usable direction signal)
- sigma=5: +0.059 (2.5× stronger gradient)

**Tradeoff with equal-sigma principle from Step 2:** Step 2 set both sigmas equal so the
charger vs. disaster crossover is determined purely by battery urgency. With
`sigma_charging < sigma_event`, agents must reach lower battery (higher urgency) before the
charging pull wins. Since the deadlock scenario already has agents at 13–20% battery
(urgency ≈ 0.75–0.87), the asymmetric sigmas do not prevent the charging pull from
dominating at critical battery.

**All other parameters same as Step 3.11. Requires retraining from scratch.**

---

## Step 4 (on hold) — Increase discount factor to fix long-range credit assignment

### Status: on hold — complete Steps 3.6 and 3.7 first

**Outcome of the attempted run:** γ = 0.997 caused complete training collapse. share_terminated
stayed at ~0.93 throughout the full 5M step run. Critic gradient norms exploded to 350+,
Q-values oscillated between −100 and −500 with no convergence. The jump from 0.99 to 0.997
was too large for the current network size and learning rates.

**The mathematical case remains valid** — see analysis below. If credit assignment to
proactive charging is still insufficient after Steps 3.6/3.7/5/6, revisit this with the more
conservative γ = 0.995 instead of 0.997.

### Issue

### Issue

After Steps 1–3, agents still show suboptimal proactive charging behavior, especially when
the disaster is far from the charger. The root cause is that the discount factor γ = 0.99
makes the agent effectively myopic over the timescales this environment requires.

### Mathematical analysis

The effective planning horizon (where rewards are discounted to 1/e ≈ 37%) is:

```
H = 1 / (1 − γ)  →  at γ = 0.99: H = 100 steps
```

The critical timescales in this environment:

| Decision | Steps required | 0.99^t | Signal remaining |
|---|---|---|---|
| Travel to charger (D=15) | ~150 steps | 0.22 | 22% |
| Charging lock-in (MIN_CHARGE_STEPS=70) | +70 steps | 0.10 | 10% |
| Return to disaster | +150 steps | 0.022 | 2.2% |
| **Full round-trip** | **~370 steps** | **0.025** | **2.5%** |

At the moment an agent at the disaster decides "I should go charge now", the future reward
from completing the full round-trip (returning to disaster with full battery, avoiding death)
is discounted to **2.5% of face value**. The agent cannot rationally act on a signal that
weak — it is almost entirely reliant on the shaped approach gradient (`CHARGING_SHAPING_SCALE`)
to make the charging trip decision, rather than on the actual future return.

This also explains why reducing `CHARGING_SHAPING_SCALE` is risky at γ = 0.99: the shaped
gradient is acting as a substitute for the long-range value signal that the discount factor
suppresses. Removing it without first fixing the underlying horizon would leave agents with
no usable credit assignment signal for proactive charging.

### Why higher γ also partially fixes hovering

With higher γ, the value function correctly attributes high Q-value to the act of actually
completing a docking session (full battery → return to disaster → avoid death → accumulate
future rewards). Hovering at D=3 outside the zone gives proximity reward now but none of
that future value — the Q-value of docking becomes clearly superior without any reward
shaping change. The gamma increase attacks the hovering exploit at the value function level,
not just the reward level.

### Fix

| Parameter | Current | Attempted | Candidate | Notes |
|---|---|---|---|---|
| `GAMMA` | `0.99` | `0.997` (failed) | `0.995` | γ=0.997 collapsed; try 0.995 first |

At γ = 0.997:
- One-way charging trip (150 steps): discounted to **61%** face value (vs 22%)
- Full round-trip (370 steps): discounted to **33%** face value (vs 2.5%)
- Effective horizon matches the one-way trip length — the agent can now plan a full approach

γ = 0.999 would give even better signal (effective horizon = 1000 steps, matching episode
length) but risks critic instability due to very long bootstrapping chains. γ = 0.997 is the
mathematically justified middle ground: covers the charging round-trip without over-extending
the bootstrapping horizon.

**No other parameter changes.** In particular, do **not** reduce `CHARGING_SHAPING_SCALE`
at the same time — first verify whether the gamma change alone reduces or eliminates hovering
before applying Step 5.

**Requires retraining from scratch** — Q-value scales change.

---

## Step 5 (on hold) — Reduce CHARGING_SHAPING_SCALE if hovering persists

### Status: on hold — complete Step 3.6 first; apply if hovering is still observed

Apply after confirming hovering is still present in the Step 3.6 run.

### Issue

After Steps 1–4, agents correctly dock and charge when they enter the charging zone
(`CHARGING_STATION_RANGE = 2`). However they may still exhibit a hovering exploit: they
decelerate heavily at distance D ≈ 2–5 and loiter just outside the charging range rather
than entering and locking in.

**The charging zone gate (Step 1) does not prevent this.** The gate suppresses the approach
gradient only when battery ≥ `CHARGING_SHAPING_BATTERY_THRESHOLD = 0.80`. Hovering occurs
at low battery, exactly when the gate is open and the proximity reward is strongest.

### Root cause: reward parity between hovering and docking

When an agent enters the charging zone at velocity < 0.75, it is **locked in position** for
`MIN_CHARGE_STEPS = 70` steps minimum, or until battery = 100%. This is a hardware constraint.

With `GAUSS_SIGMA_CHARGING = 10` and `CHARGING_SHAPING_SCALE = 0.3`, at D = 3 (just outside
`CHARGING_STATION_RANGE = 2`):

```
proximity ≈ exp(−9 / 200) ≈ 0.956
r_hover_per_step ≈ 0.3 × 0.956 × cw ≈ 0.287 × cw
r_hover(70 steps) ≈ 0.287 × 0.5 × 70 ≈ 10.0   (at cw = 0.5, 40% battery)
```

This is comparable to the undiscounted charging reward over the same 70-step window. Even
with γ = 0.997, if the shaped proximity reward is large enough to compete with the discounted
future value of completing the charge, hovering can persist.

### Why s = 0.1 eliminates hovering

At D = 3 with s = 0.1:

```
r_hover_per_step ≈ 0.1 × 0.956 × cw ≈ 0.096 × cw
r_hover(70 steps) ≈ 0.096 × 0.5 × 70 ≈ 3.4   (at cw = 0.5)
```

With γ = 0.997, the discounted future value of completing the charge (full battery → return
to disaster → avoid death) clearly exceeds 3.4. The hovering policy becomes irrational.

### Why s = 0.1 is safe at γ = 0.997 (but was risky at γ = 0.99)

The approach gradient at s = 0.1 from D = 15:

```
G_approach ≈ 0.1 × 0.5 × 10 × 1.253 × 0.77 × [1 - 0.997^150]
           ≈ 0.1 × 3.75 × 0.36 ≈ 0.135
```

At γ = 0.997, the discounted terminal charging reward viewed from departure is ~61% of face
value — a usable learning signal. The approach gradient at s = 0.1 supplements this rather
than substituting for it. At γ = 0.99 (22% face value), s = 0.1 was insufficient because
the gradient was too thin relative to a very weak terminal signal.

### Fix

| Parameter | Current | New |
|---|---|---|
| `CHARGING_SHAPING_SCALE` | `0.3` | `0.1` |

If hovering still persists at s = 0.1, reduce to s = 0.05. Do **not** go below 0.05 — the
approach gradient becomes negligible and the terminal charging reward must carry all credit
assignment weight alone, which is unreliable even at γ = 0.997.

**Requires retraining from scratch** — reward magnitude changes.

---

## Parameters NOT changed

- `GAUSS_BETA_COVER`, `GAUSS_BETA_EVENT`, `GAUSS_BETA_CHARGING`: reward term weights kept at 1.0
- `DEATH_PENALTY`, `ongoing_death_penalty`: death signals already present; escalating
  charging_weight as battery drains is the primary mechanism incentivizing proactive charging
- `ABANDONMENT_PENALTY`: kept at −0.5; only fires when ALL agents lose detection
- `MIN_CHARGE_STEPS = 70`: hardware requirement; irrelevant for normal charging trips starting
  at < 80% battery (those complete in ≤ 80 steps driven by battery reaching 100%)
- `MINIMUM_BATTERY_FOR_CHARGING = 80`: unchanged; this is the cap preventing charging above 80%
- `VORONOI_VARIANCE_PENALTY_WEIGHT`: parameter is defined but unused (dead code); not addressed
  in this plan to minimize scope

---

## Appendix — Known approximations and limitations

### A2 — Training duration

5M loop steps (= 20M env steps with `N_ENVIRONMENTS=4`) is insufficient for convergence
from Step 3.8 onwards. At 5M steps, `share_terminated` was still clearly declining (~0.25
for Steps 3.8/3.9) compared to the converged reference run (steps 1+1.5+2+2.5, ~0.07).

The privileged critic runs learn a larger, more complex Q-function (absolute positions for
all agents + global disaster/charger state), which requires more gradient steps to converge.

**Recommended training budget:**

| Budget | Loop steps | Env steps | Approx. wall time (at 955 SPS) |
|---|---|---|---|
| Minimum | 10,000,000 | 40M | ~12 hours |
| Preferred | 15,000,000 | 60M | ~17.5 hours |

Set in `sac.h`: `static constexpr TI STEP_LIMIT = 15000000;`

If share_terminated is still declining at 15M, extend by another 5M. Do not compare runs
trained for different step counts — always train to the same STEP_LIMIT before drawing
conclusions.

### A1 — Bootstrapping error for step-limit transitions (RANDOMIZE_EPISODE_LENGTH=true)

**Context:** Steps 3 and 3.5 use randomized episode lengths sampled from [700, 1300]. To end
episodes at the sampled limit, `terminated()` returns `true` when
`step_count >= episode_step_limit`. The rl_tools SAC implementation uses the `terminated` flag
to zero out the bootstrap term in the Bellman target:

```cpp
T future_value = IGNORE_TERMINATION || !terminated
    ? GAMMA * min_next_state_action_value : 0;
```

**The problem:** Episodes ending at the step limit should be **truncated** (future value
bootstrapped), not **terminated** (future value zeroed). A true terminal state is an absorbing
state — the agent cannot act further. A step-limit ending is not absorbing: the agent would
continue accumulating reward if the episode were longer. Zeroing the bootstrap term
underestimates Q(s, a) for all transitions at the final step of each episode.

**Why rl_tools cannot fix this without framework changes:** There is no separate `truncated`
flag in the replay buffer or Bellman target computation. The only existing workaround
(`IGNORE_TERMINATION=true`) is not viable because it also bootstraps through death transitions,
where the next state in the replay buffer is an unrelated episode-reset state.

**Magnitude of the error:**
- Frequency: one wrong transition per episode (~0.1% of the 1M-entry replay buffer)
- Bias per wrong transition: `γ * |V(s_next)| ≈ 0.99 × 150 ≈ 150` units underestimation
- Distribution: spread across all episode-boundary states uniformly (randomized limit means
  no single state is consistently at the boundary)
- Net effect: small, distributed downward bias on Q-values near episode boundaries

**Accepted as approximation.** A proper fix would add a `truncated` field to the replay
buffer alongside `terminated`, and modify the Bellman target to bootstrap when `truncated=true`
even if `terminated=true`. This is a framework-level change deferred to future work.
