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

**What this does NOT fix:** The discounting effect itself persists — if the episode happens to
end in 5 steps, the Q-value for "go charge" is still suppressed. But since the agent cannot
predict when that will happen, the policy must learn to charge whenever battery is critical,
averaged across all possible remaining durations.

**Optional complement:** Add `remaining_fraction = episode_step_limit - step_count) / episode_step_limit`
to the observation. This lets the critic produce more accurate value estimates conditioned on
remaining time, but is not strictly necessary if the randomization range is wide enough.

### Implementation changes

1. Add to `DefaultParameters` in `oil_platform.h`:
   ```cpp
   static constexpr TI EPISODE_STEP_LIMIT_MIN = 700;
   static constexpr TI EPISODE_STEP_LIMIT_MAX = 1300;
   ```
2. Add `TI episode_step_limit` field to `State` struct in `oil_platform.h`.
3. In `reset()` in `operations_generic.h`: sample `state.episode_step_limit` uniformly from
   `[PARAMS::EPISODE_STEP_LIMIT_MIN, PARAMS::EPISODE_STEP_LIMIT_MAX]`.
4. In the termination condition in `operations_generic.h`: replace compile-time
   `PARAMS::EPISODE_STEP_LIMIT` with `state.episode_step_limit`.
5. In `sac.h`: keep `EPISODE_STEP_LIMIT = 1300` (the max) for replay buffer sizing.

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
