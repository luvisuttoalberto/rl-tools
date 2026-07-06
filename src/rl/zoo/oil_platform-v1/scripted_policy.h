#pragma once
// Scripted (non-learning) baseline policy for the oil-platform environment.
//
// Behavior:
//   - Patrol: all agents follow a closed diamond tour around the platform
//     (4 waypoints at the arm directions) and keep equal arc-length spacing
//     by modulating their cruise speed based on the gap to the next
//     patrolling agent ahead on the tour.
//   - Charging: an agent returns to the charger when its battery drops below
//     CHARGE_ENTER_BATTERY. Single occupancy is enforced by a decentralized
//     tie-break: an agent yields if another alive agent is already charging
//     or has a strictly lower battery while also below the threshold
//     (ties broken by agent index). Below CHARGE_CRITICAL_BATTERY it goes
//     regardless. While the environment reports is_charging the agent
//     commands zero velocity; the charging session itself (minimum hold,
//     stop at full battery) is governed by the environment.
//   - Disaster: when the globally shared detection flag is set, all agents
//     that are not charging and not returning to charge converge on the
//     last-known disaster position, each targeting a point on a small ring
//     around it (offset by agent index) to avoid stacking.
//
// The policy is decentralized and information-equivalent to the learned SAC
// actor: each agent's action is computed exclusively from its own per-agent
// block of the actor observation vector (the same vector the neural actor
// receives), decoded exactly by inverting the affine encodings in observe().
//
// The struct implements the rl-tools policy interface (INPUT_SHAPE /
// OUTPUT_SHAPE, State, Buffer, CHANGE_BATCH_SIZE, reset, evaluate_step) so it
// can be plugged into rl_tools::evaluate() and the surrounding tooling
// (return.json, trajectory dumps, TensorBoard metrics) exactly like a
// trained actor.

#include <rl_tools/version.h>
#include <rl_tools/rl/environments/multi_agent/oil_platform/oil_platform.h>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools::rl::zoo::oil_platform_v1::scripted {
    namespace rlt = rl_tools;

    template <typename T_POLICY>
    struct ScriptedPolicyState {
        using POLICY = T_POLICY;
        typename POLICY::AgentMemory memory[POLICY::BATCH_SIZE][POLICY::N_AGENTS];
    };
    template <typename T_POLICY>
    struct ScriptedPolicyBuffer {};

    template <typename T_TYPE_POLICY, typename T_TI, typename T_ENVIRONMENT, T_TI T_BATCH_SIZE>
    struct ScriptedPolicy {
        using TYPE_POLICY = T_TYPE_POLICY;
        using T = typename TYPE_POLICY::DEFAULT;
        using TI = T_TI;
        using ENVIRONMENT = T_ENVIRONMENT;
        using PARAMS = typename ENVIRONMENT::PARAMETERS;
        using OBS = typename ENVIRONMENT::Observation;
        static constexpr TI BATCH_SIZE = T_BATCH_SIZE;
        static constexpr TI N_AGENTS = PARAMS::N_AGENTS;
        static constexpr TI OBS_DIM = OBS::DIM;
        static constexpr TI ACTION_DIM = ENVIRONMENT::ACTION_DIM;

        static_assert(PARAMS::OBSERVE_RELATIVE_POSITIONS, "ScriptedPolicy decodes the relative-position observation layout");
        static_assert(PARAMS::ACTOR_OBSERVE_CHARGING_STATION_POSITION, "ScriptedPolicy requires the charger position in the observation");

        using INPUT_SHAPE = tensor::Shape<TI, 1, BATCH_SIZE, OBS_DIM>;
        using OUTPUT_SHAPE = tensor::Shape<TI, 1, BATCH_SIZE, ACTION_DIM>;
        template <typename NEW_TI, NEW_TI NEW_BATCH_SIZE>
        using CHANGE_BATCH_SIZE = ScriptedPolicy<TYPE_POLICY, TI, ENVIRONMENT, NEW_BATCH_SIZE>;

        // --- tuning constants (world units: meters, m/s, battery in %) ---
        // Sized for DISCHARGE_RATE_BASE=0.30 (hover/cruise drain 0.15/step): the worst-case
        // ~130-step transit from the tour to the charger costs ~20 battery, so the critical
        // threshold must leave that as reserve.
        static constexpr T CHARGE_ENTER_BATTERY = 40;     // request charging below this
        static constexpr T CHARGE_CRITICAL_BATTERY = 25;  // ignore the occupancy tie-break below this
        static constexpr T CHARGE_TIE_EPSILON = 0.5;      // batteries closer than this are tied (tie-break by index)
        static constexpr T TOUR_RADIUS = 75;              // L1 radius of the diamond tour around the platform
        static constexpr T WAYPOINT_CAPTURE_RADIUS = 15;  // advance to the next waypoint within this distance
        static constexpr T TOUR_REJOIN_DISTANCE = 30;     // farther than this from the tour: head to the nearest edge
        static constexpr T CRUISE_SPEED_FRACTION = T(0.8);
        static constexpr T PATROL_SPEED_MIN_FRACTION = T(0.4);
        static constexpr T PATROL_SPEED_MAX_FRACTION = T(0.95);
        static constexpr T SPACING_GAIN = T(1.0);         // speed response to the spacing error on the tour
        static constexpr T TRANSIT_SPEED_FRACTION = T(0.9);
        static constexpr T DOCK_APPROACH_GAIN = T(0.25);  // desired speed = gain * distance when approaching the charger
        static constexpr T TRACK_GAIN = T(0.7);           // P-gain on the distance to the disaster ring point
        static constexpr T TRACK_SPEED_MIN = 6;
        static constexpr T TRACK_RING_RADIUS = 12;        // per-agent standoff ring around the last-known disaster position

        struct AgentMemory {
            TI target_vertex;
            bool entered_tour;
        };
        template <bool T_DYNAMIC_ALLOCATION = true>
        using State = ScriptedPolicyState<ScriptedPolicy>;
        template <bool T_DYNAMIC_ALLOCATION = true>
        using Buffer = ScriptedPolicyBuffer<ScriptedPolicy>;

        // --- decoded per-agent view of the observation ------------------
        struct OtherAgentView {
            T position[2]; // absolute, reconstructed from the relative encoding
            T battery;     // percent
            bool dead;
            bool is_charging;
        };
        struct AgentView {
            T position[2];
            T velocity[2];
            T battery; // percent
            bool dead;
            bool is_charging;
            bool disaster_detected_global;
            T disaster_position[2]; // absolute last-known, valid iff disaster_detected_global
            T charger_position[2];  // absolute
            OtherAgentView others[N_AGENTS - 1];
            TI other_index[N_AGENTS - 1]; // absolute agent index of each slot
        };

        // --- diamond tour geometry --------------------------------------
        static constexpr T CX = T(PARAMS::GRID_SIZE_X) * T(0.5);
        static constexpr T CY = T(PARAMS::GRID_SIZE_Y) * T(0.5);
        static constexpr TI N_VERTICES = 4;
        static constexpr T VERTEX_X[N_VERTICES] = {CX, CX + TOUR_RADIUS, CX, CX - TOUR_RADIUS};
        static constexpr T VERTEX_Y[N_VERTICES] = {CY + TOUR_RADIUS, CY, CY - TOUR_RADIUS, CY};
    };
}
RL_TOOLS_NAMESPACE_WRAPPER_END

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template <typename DEVICE, typename POLICY>
    void malloc(DEVICE& device, rl::zoo::oil_platform_v1::scripted::ScriptedPolicyState<POLICY>& state){ }
    template <typename DEVICE, typename POLICY>
    void free(DEVICE& device, rl::zoo::oil_platform_v1::scripted::ScriptedPolicyState<POLICY>& state){ }
    template <typename DEVICE, typename POLICY>
    void malloc(DEVICE& device, rl::zoo::oil_platform_v1::scripted::ScriptedPolicyBuffer<POLICY>& buffer){ }
    template <typename DEVICE, typename POLICY>
    void free(DEVICE& device, rl::zoo::oil_platform_v1::scripted::ScriptedPolicyBuffer<POLICY>& buffer){ }
    template <typename DEVICE, typename TP, typename TI, typename ENV, TI BS, typename RNG>
    void malloc(DEVICE& device, rl::zoo::oil_platform_v1::scripted::ScriptedPolicy<TP, TI, ENV, BS>& policy, RNG& rng){ }
    template <typename DEVICE, typename TP, typename TI, typename ENV, TI BS>
    void malloc(DEVICE& device, rl::zoo::oil_platform_v1::scripted::ScriptedPolicy<TP, TI, ENV, BS>& policy){ }
    template <typename DEVICE, typename TP, typename TI, typename ENV, TI BS>
    void free(DEVICE& device, rl::zoo::oil_platform_v1::scripted::ScriptedPolicy<TP, TI, ENV, BS>& policy){ }

    template <typename DEVICE, typename TP, typename TI, typename ENV, TI BS, typename POLICY_STATE, typename RNG>
    void reset(DEVICE& device, const rl::zoo::oil_platform_v1::scripted::ScriptedPolicy<TP, TI, ENV, BS>& policy, POLICY_STATE& state, RNG& rng){
        using POLICY = rl::zoo::oil_platform_v1::scripted::ScriptedPolicy<TP, TI, ENV, BS>;
        for(TI batch_i = 0; batch_i < POLICY::BATCH_SIZE; batch_i++){
            for(TI agent_i = 0; agent_i < POLICY::N_AGENTS; agent_i++){
                state.memory[batch_i][agent_i].target_vertex = agent_i % POLICY::N_VERTICES;
                state.memory[batch_i][agent_i].entered_tour = false;
            }
        }
    }

    namespace rl::zoo::oil_platform_v1::scripted {
        // Decode the per-agent block of the observation vector (inverse of observe()).
        template <typename DEVICE, typename POLICY, typename INPUT>
        static typename POLICY::AgentView decode_agent(DEVICE& device, const INPUT& input, typename POLICY::TI batch_i, typename POLICY::TI agent_i){
            using T = typename POLICY::T;
            using TI = typename POLICY::TI;
            using PARAMS = typename POLICY::PARAMS;
            using OBS = typename POLICY::OBS;
            typename POLICY::AgentView view;
            const TI offset = agent_i * OBS::PER_AGENT_DIM;
            auto obs = [&](TI col) -> T {
                if constexpr (INPUT::SPEC::SHAPE::LENGTH == 3){
                    return get(device, input, 0, batch_i, col);
                } else {
                    return get(device, input, batch_i, col);
                }
            };
            view.position[0] = (obs(offset + 0) + T(1)) * T(0.5) * T(PARAMS::GRID_SIZE_X);
            view.position[1] = (obs(offset + 1) + T(1)) * T(0.5) * T(PARAMS::GRID_SIZE_Y);
            view.velocity[0] = obs(offset + 2) * PARAMS::MAX_SPEED;
            view.velocity[1] = obs(offset + 3) * PARAMS::MAX_SPEED;
            view.battery = (obs(offset + 5) + T(1)) * T(0.5) * T(100);
            view.dead = obs(offset + 6) > 0;
            view.is_charging = obs(offset + 7) > 0;
            view.disaster_detected_global = obs(offset + OBS::BASE_PER_AGENT_DIM + OBS::RELATIVE_EXTRA_DIM - 1) > 0;
            view.disaster_position[0] = view.position[0] + obs(offset + 8) * T(PARAMS::GRID_SIZE_X);
            view.disaster_position[1] = view.position[1] + obs(offset + 9) * T(PARAMS::GRID_SIZE_Y);
            view.charger_position[0] = view.position[0] + obs(offset + 10) * T(PARAMS::GRID_SIZE_X);
            view.charger_position[1] = view.position[1] + obs(offset + 11) * T(PARAMS::GRID_SIZE_Y);
            const TI others_offset = offset + OBS::BASE_PER_AGENT_DIM + OBS::RELATIVE_EXTRA_DIM;
            TI slot = 0;
            for(TI agent_j = 0; agent_j < POLICY::N_AGENTS; agent_j++){
                if(agent_j == agent_i){ continue; }
                const TI base = others_offset + slot * OBS::PER_OTHER_AGENT_DIM;
                auto& other = view.others[slot];
                other.position[0] = view.position[0] + obs(base + 0) * T(PARAMS::GRID_SIZE_X);
                other.position[1] = view.position[1] + obs(base + 1) * T(PARAMS::GRID_SIZE_Y);
                other.battery = (obs(base + 4) + T(1)) * T(0.5) * T(100);
                other.dead = obs(base + 5) > 0;
                other.is_charging = obs(base + 6) > 0;
                view.other_index[slot] = agent_j;
                slot++;
            }
            return view;
        }

        template <typename DEVICE, typename T>
        static T norm2(DEVICE& device, T dx, T dy){
            return math::sqrt(device.math, dx * dx + dy * dy);
        }

        // Arc-length position of a point projected onto the closed diamond tour.
        template <typename DEVICE, typename POLICY>
        static typename POLICY::T tour_arc_position(DEVICE& device, typename POLICY::T x, typename POLICY::T y){
            using T = typename POLICY::T;
            using TI = typename POLICY::TI;
            constexpr TI NV = POLICY::N_VERTICES;
            T best_distance = 0;
            T best_arc = 0;
            T arc_offset = 0;
            for(TI edge_i = 0; edge_i < NV; edge_i++){
                const T ax = POLICY::VERTEX_X[edge_i], ay = POLICY::VERTEX_Y[edge_i];
                const T bx = POLICY::VERTEX_X[(edge_i + 1) % NV], by = POLICY::VERTEX_Y[(edge_i + 1) % NV];
                const T ex = bx - ax, ey = by - ay;
                const T edge_length = norm2(device, ex, ey);
                T t = ((x - ax) * ex + (y - ay) * ey) / (edge_length * edge_length);
                t = math::clamp(device.math, t, T(0), T(1));
                const T px = ax + t * ex, py = ay + t * ey;
                const T d = norm2(device, x - px, y - py);
                if(edge_i == 0 || d < best_distance){
                    best_distance = d;
                    best_arc = arc_offset + t * edge_length;
                }
                arc_offset += edge_length;
            }
            return best_arc;
        }
        template <typename DEVICE, typename POLICY>
        static typename POLICY::T tour_distance(DEVICE& device, typename POLICY::T x, typename POLICY::T y){
            using T = typename POLICY::T;
            using TI = typename POLICY::TI;
            constexpr TI NV = POLICY::N_VERTICES;
            T best_distance = 0;
            for(TI edge_i = 0; edge_i < NV; edge_i++){
                const T ax = POLICY::VERTEX_X[edge_i], ay = POLICY::VERTEX_Y[edge_i];
                const T bx = POLICY::VERTEX_X[(edge_i + 1) % NV], by = POLICY::VERTEX_Y[(edge_i + 1) % NV];
                const T ex = bx - ax, ey = by - ay;
                const T edge_length = norm2(device, ex, ey);
                T t = ((x - ax) * ex + (y - ay) * ey) / (edge_length * edge_length);
                t = math::clamp(device.math, t, T(0), T(1));
                const T d = norm2(device, x - (ax + t * ex), y - (ay + t * ey));
                if(edge_i == 0 || d < best_distance){
                    best_distance = d;
                }
            }
            return best_distance;
        }
        // Index of the end vertex of the tour edge nearest to a point (used to rejoin the tour).
        template <typename DEVICE, typename POLICY>
        static typename POLICY::TI nearest_edge_end_vertex(DEVICE& device, typename POLICY::T x, typename POLICY::T y){
            using T = typename POLICY::T;
            using TI = typename POLICY::TI;
            constexpr TI NV = POLICY::N_VERTICES;
            T best_distance = 0;
            TI best_vertex = 0;
            for(TI edge_i = 0; edge_i < NV; edge_i++){
                const T ax = POLICY::VERTEX_X[edge_i], ay = POLICY::VERTEX_Y[edge_i];
                const T bx = POLICY::VERTEX_X[(edge_i + 1) % NV], by = POLICY::VERTEX_Y[(edge_i + 1) % NV];
                const T ex = bx - ax, ey = by - ay;
                const T edge_length = norm2(device, ex, ey);
                T t = ((x - ax) * ex + (y - ay) * ey) / (edge_length * edge_length);
                t = math::clamp(device.math, t, T(0), T(1));
                const T d = norm2(device, x - (ax + t * ex), y - (ay + t * ey));
                if(edge_i == 0 || d < best_distance){
                    best_distance = d;
                    best_vertex = (edge_i + 1) % NV;
                }
            }
            return best_vertex;
        }

        // Compute the action (desired velocity in [-1,1]) for one agent from its own observation block.
        template <typename DEVICE, typename POLICY>
        static void agent_action(DEVICE& device, const typename POLICY::AgentView& view, typename POLICY::AgentMemory& memory, typename POLICY::TI agent_i, typename POLICY::T& action_x, typename POLICY::T& action_y){
            using T = typename POLICY::T;
            using TI = typename POLICY::TI;
            using PARAMS = typename POLICY::PARAMS;
            action_x = 0;
            action_y = 0;
            if(view.dead || view.is_charging){
                return; // charging drones are frozen by the environment; command zero to stay docked
            }

            // --- charging decision (decentralized single occupancy) ---
            const bool below_enter = view.battery < POLICY::CHARGE_ENTER_BATTERY;
            const bool below_critical = view.battery < POLICY::CHARGE_CRITICAL_BATTERY;
            bool charger_busy = false;
            bool other_has_priority = false;
            for(TI slot = 0; slot < POLICY::N_AGENTS - 1; slot++){
                const auto& other = view.others[slot];
                if(other.dead){ continue; }
                if(other.is_charging){ charger_busy = true; }
                if(other.battery < POLICY::CHARGE_ENTER_BATTERY && !other.is_charging){
                    const T diff = view.battery - other.battery;
                    if(diff > POLICY::CHARGE_TIE_EPSILON || (diff > -POLICY::CHARGE_TIE_EPSILON && view.other_index[slot] < agent_i)){
                        other_has_priority = true;
                    }
                }
            }
            const bool go_charge = below_enter && (below_critical || (!charger_busy && !other_has_priority));

            const T max_speed = PARAMS::MAX_SPEED;
            auto command_towards = [&](T tx, T ty, T desired_speed){
                const T dx = tx - view.position[0];
                const T dy = ty - view.position[1];
                const T d = norm2(device, dx, dy);
                if(d > T(1e-4)){
                    action_x = (dx / d) * (desired_speed / max_speed);
                    action_y = (dy / d) * (desired_speed / max_speed);
                    action_x = math::clamp(device.math, action_x, T(-1), T(1));
                    action_y = math::clamp(device.math, action_y, T(-1), T(1));
                }
            };

            if(go_charge){
                const T d = norm2(device, view.charger_position[0] - view.position[0], view.charger_position[1] - view.position[1]);
                // Proportional approach: drops below the docking speed threshold inside the charging ring
                const T desired_speed = math::min(device.math, POLICY::TRANSIT_SPEED_FRACTION * max_speed, POLICY::DOCK_APPROACH_GAIN * d);
                command_towards(view.charger_position[0], view.charger_position[1], desired_speed);
                return;
            }

            if(view.disaster_detected_global){
                // All-converge: target a point on a small ring around the last-known position,
                // with a per-agent angular offset to avoid stacking.
                const T angle = T(2) * math::PI<T> * T(agent_i) / T(POLICY::N_AGENTS);
                const T tx = view.disaster_position[0] + POLICY::TRACK_RING_RADIUS * math::cos(device.math, angle);
                const T ty = view.disaster_position[1] + POLICY::TRACK_RING_RADIUS * math::sin(device.math, angle);
                const T d = norm2(device, tx - view.position[0], ty - view.position[1]);
                T desired_speed = POLICY::TRACK_GAIN * d;
                desired_speed = math::clamp(device.math, desired_speed, POLICY::TRACK_SPEED_MIN, POLICY::PATROL_SPEED_MAX_FRACTION * max_speed);
                command_towards(tx, ty, desired_speed);
                return;
            }

            // --- patrol on the diamond tour ---
            const T distance_to_tour = tour_distance<DEVICE, POLICY>(device, view.position[0], view.position[1]);
            if(memory.entered_tour && distance_to_tour > POLICY::TOUR_REJOIN_DISTANCE){
                // e.g. returning from the charger or from a disaster: rejoin at the nearest edge
                memory.target_vertex = nearest_edge_end_vertex<DEVICE, POLICY>(device, view.position[0], view.position[1]);
            }
            if(!memory.entered_tour && distance_to_tour < POLICY::WAYPOINT_CAPTURE_RADIUS){
                memory.entered_tour = true;
            }
            const T wx = POLICY::VERTEX_X[memory.target_vertex];
            const T wy = POLICY::VERTEX_Y[memory.target_vertex];
            if(norm2(device, wx - view.position[0], wy - view.position[1]) < POLICY::WAYPOINT_CAPTURE_RADIUS){
                memory.target_vertex = (memory.target_vertex + 1) % POLICY::N_VERTICES;
            }

            T speed_fraction = POLICY::CRUISE_SPEED_FRACTION;
            if(memory.entered_tour){
                // Equal-spacing controller: adjust speed based on the arc-length gap to the
                // next patrolling agent ahead on the tour.
                constexpr TI NV = POLICY::N_VERTICES;
                T tour_length = 0;
                for(TI edge_i = 0; edge_i < NV; edge_i++){
                    const T ex = POLICY::VERTEX_X[(edge_i + 1) % NV] - POLICY::VERTEX_X[edge_i];
                    const T ey = POLICY::VERTEX_Y[(edge_i + 1) % NV] - POLICY::VERTEX_Y[edge_i];
                    tour_length += norm2(device, ex, ey);
                }
                const T own_arc = tour_arc_position<DEVICE, POLICY>(device, view.position[0], view.position[1]);
                TI n_patrolling = 1;
                T gap_ahead = tour_length;
                bool any_ahead = false;
                for(TI slot = 0; slot < POLICY::N_AGENTS - 1; slot++){
                    const auto& other = view.others[slot];
                    const bool patrolling = !other.dead && !other.is_charging
                        && other.battery >= POLICY::CHARGE_ENTER_BATTERY
                        && tour_distance<DEVICE, POLICY>(device, other.position[0], other.position[1]) < POLICY::TOUR_REJOIN_DISTANCE;
                    if(patrolling){
                        n_patrolling++;
                        const T other_arc = tour_arc_position<DEVICE, POLICY>(device, other.position[0], other.position[1]);
                        T gap = other_arc - own_arc;
                        while(gap < 0){ gap += tour_length; }
                        if(!any_ahead || gap < gap_ahead){
                            gap_ahead = gap;
                            any_ahead = true;
                        }
                    }
                }
                if(any_ahead){
                    const T gap_target = tour_length / T(n_patrolling);
                    speed_fraction = POLICY::CRUISE_SPEED_FRACTION * (T(1) + POLICY::SPACING_GAIN * (gap_ahead - gap_target) / gap_target);
                    speed_fraction = math::clamp(device.math, speed_fraction, POLICY::PATROL_SPEED_MIN_FRACTION, POLICY::PATROL_SPEED_MAX_FRACTION);
                }
            }
            command_towards(wx, wy, speed_fraction * PARAMS::MAX_SPEED);
        }
    }

    template <typename DEVICE, typename TP, typename TI, typename ENV, TI BS, typename INPUT_SPEC, typename POLICY_STATE, typename OUTPUT_SPEC, typename BUFFER, typename RNG, typename MODE>
    void evaluate_step(DEVICE& device, const rl::zoo::oil_platform_v1::scripted::ScriptedPolicy<TP, TI, ENV, BS>& policy, const Tensor<INPUT_SPEC>& input, POLICY_STATE& state, Tensor<OUTPUT_SPEC>& output, BUFFER& buffer, RNG& rng, const Mode<MODE>& mode){
        using POLICY = rl::zoo::oil_platform_v1::scripted::ScriptedPolicy<TP, TI, ENV, BS>;
        using T = typename POLICY::T;
        for(TI batch_i = 0; batch_i < POLICY::BATCH_SIZE; batch_i++){
            for(TI agent_i = 0; agent_i < POLICY::N_AGENTS; agent_i++){
                auto view = rl::zoo::oil_platform_v1::scripted::decode_agent<DEVICE, POLICY>(device, input, batch_i, agent_i);
                T action_x, action_y;
                rl::zoo::oil_platform_v1::scripted::agent_action<DEVICE, POLICY>(device, view, state.memory[batch_i][agent_i], agent_i, action_x, action_y);
                if constexpr (OUTPUT_SPEC::SHAPE::LENGTH == 3){
                    set(device, output, action_x, 0, batch_i, agent_i * 2 + 0);
                    set(device, output, action_y, 0, batch_i, agent_i * 2 + 1);
                } else {
                    set(device, output, action_x, batch_i, agent_i * 2 + 0);
                    set(device, output, action_y, batch_i, agent_i * 2 + 1);
                }
            }
        }
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END
