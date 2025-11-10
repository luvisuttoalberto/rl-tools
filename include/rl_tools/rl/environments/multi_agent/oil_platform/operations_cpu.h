#include "../../../../version.h"
#if (defined(RL_TOOLS_DISABLE_INCLUDE_GUARDS) || !defined(RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_CPU_H)) && (RL_TOOLS_USE_THIS_VERSION == 1)
#pragma once
#define RL_TOOLS_RL_ENVIRONMENTS_MULTI_AGENT_OIL_PLATFORM_OPERATIONS_CPU_H

#include "oil_platform.h"
#include "operations_generic.h"
#include <string>
#include <sstream>
#include <cstddef>

RL_TOOLS_NAMESPACE_WRAPPER_START
namespace rl_tools {
    template<typename DEVICE, typename SPEC>
    struct OilPlatformCPU : public rl::environments::multi_agent::OilPlatform<SPEC> {
        using ENVIRONMENT = rl::environments::multi_agent::OilPlatform<SPEC>;
        using State = typename ENVIRONMENT::State;
        using Parameters = typename ENVIRONMENT::Parameters;
        using TI = typename DEVICE::index_t;
    };

    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE&, rl::environments::multi_agent::OilPlatform<SPEC>& env,
                     const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters) {
        std::string result = "{";
        result += "\"N_AGENTS\":" + std::to_string(SPEC::PARAMETERS::N_AGENTS) + ",";
        result += "\"GRID_SIZE_X\":" + std::to_string(SPEC::PARAMETERS::GRID_SIZE_X) + ",";
        result += "\"GRID_SIZE_Y\":" + std::to_string(SPEC::PARAMETERS::GRID_SIZE_Y) + ",";
        result += "\"PLATFORM_HALF_SIZE\":" + std::to_string(SPEC::PARAMETERS::PLATFORM_HALF_SIZE) + ",";
        result += "\"PIPE_WIDTH\":" + std::to_string(SPEC::PARAMETERS::PIPE_WIDTH) + ",";
        result += "\"SENSOR_RANGE\":" + std::to_string(SPEC::PARAMETERS::SENSOR_RANGE) + ",";
        result += "\"CHARGING_STATION_POSITION_X\":" + std::to_string(SPEC::PARAMETERS::CHARGING_STATION_POSITION_X) + ",";
        result += "\"CHARGING_STATION_POSITION_Y\":" + std::to_string(SPEC::PARAMETERS::CHARGING_STATION_POSITION_Y) + ",";
        result += "\"CHARGING_STATION_RANGE\":" + std::to_string(SPEC::PARAMETERS::CHARGING_STATION_RANGE);
        result += "}";
        return result;
    }

    template <typename DEVICE, typename SPEC>
    std::string json(DEVICE& device, rl::environments::multi_agent::OilPlatform<SPEC>& env,
                     const typename rl::environments::multi_agent::OilPlatform<SPEC>::Parameters& parameters,
                     const typename rl::environments::multi_agent::oil_platform::State<SPEC>& state) {
        using TI = typename DEVICE::index_t;
        using T = typename SPEC::T;

        // Create drone states JSON
        std::string drone_states = "[";
        for (TI agent_i = 0; agent_i < SPEC::PARAMETERS::N_AGENTS; agent_i++) {
            if (agent_i > 0) {
                drone_states += ",";
            }
            const auto& agent_state = state.drone_states[agent_i];
            std::string drone_state = "{";
            drone_state += "\"position\": [" + std::to_string(agent_state.position[0]) + "," +
                           std::to_string(agent_state.position[1]) + "],";
            drone_state += "\"velocity\": [" + std::to_string(agent_state.velocity[0]) + "," +
                           std::to_string(agent_state.velocity[1]) + "],";
            drone_state += "\"battery\": " + std::to_string(agent_state.battery) + ",";
            drone_state += "\"dead\": " + std::string(agent_state.dead ? "true" : "false") + ",";
            drone_state += "\"is_charging\": " + std::string(agent_state.is_charging ? "true" : "false") + ",";
            drone_state += "\"disaster_detected\": " + std::string(agent_state.is_detecting ? "true" : "false");
            drone_state += "}";
            drone_states += drone_state;
        }
        drone_states += "]";

// Create disaster JSON
        std::string disaster = "{";
        disaster += "\"active\": " + std::string(state.disaster.active ? "true" : "false") + ",";
        disaster += "\"position\": [" +
                    std::to_string(state.disaster.position[0]) + "," +
                    std::to_string(state.disaster.position[1]) + "],";
        // ADDED: Include disaster velocity in JSON
        disaster += "\"velocity\": [" +
                    std::to_string(state.disaster.velocity[0]) + "," +
                    std::to_string(state.disaster.velocity[1]) + "]";
        disaster += "}";
        // Assemble final JSON
        std::string result = "{";
        result += "\"drone_states\": " + drone_states + ",";
        result += "\"disaster\": " + disaster + ",";
        result += "\"last_detected_disaster_position\": [" + std::to_string(state.last_detected_disaster_position[0]) + "," + std::to_string(state.last_detected_disaster_position[1]) + "],";
        result += "\"step_count\": " + std::to_string(state.step_count) + ",";
        result += "\"disaster_undetected_steps\": " + std::to_string(state.disaster_undetected_steps) + ",";
        result += "\"per_step_reward\": " + std::to_string(state.metrics.per_step_reward);
        result += "}";
        return result;
    }

    template<typename DEVICE, typename SPEC>
    std::string get_ui(DEVICE & /*device*/, rl::environments::multi_agent::OilPlatform<SPEC> & /*env*/) {
        std::string ui = R"RL_TOOLS_LITERAL(
export async function init(canvas, parameters, options) {
    // Performance tip: Set willReadFrequently to true for canvases that will be read often
    const ctx = canvas.getContext('2d', { willReadFrequently: false });

    // Calculate time for FPS
    const now = performance.now();

    return {
        ctx: ctx,
        lastUpdateTime: now,
        frameCount: 0,
        lastFpsUpdate: now,
        fps: 0
    };
}

export async function render(ui_state, parameters, state, action) {
    const ctx = ui_state.ctx;
    const width = ctx.canvas.width;
    const height = ctx.canvas.height;

    // Calculate FPS
    const now = performance.now();
    ui_state.frameCount++;

    if (now - ui_state.lastFpsUpdate > 1000) {
        ui_state.fps = Math.round((ui_state.frameCount * 1000) / (now - ui_state.lastFpsUpdate));
        ui_state.frameCount = 0;
        ui_state.lastFpsUpdate = now;
    }

    // Clear the canvas
    ctx.clearRect(0, 0, width, height);

    // Calculate scaling
    const scaleX = width / parameters.GRID_SIZE_X;
    const scaleY = height / parameters.GRID_SIZE_Y;
    const centerX = width / 2;
    const centerY = height / 2;

    const chargeX = parameters.CHARGING_STATION_POSITION_X;
    const chargeY = parameters.CHARGING_STATION_POSITION_Y;

    // Draw light grid
    ctx.strokeStyle = '#f0f0f0';
    ctx.lineWidth = 0.5;

    // Draw grid lines every 5 units for better performance
    for (let x = 0; x <= parameters.GRID_SIZE_X; x += 5) {
        ctx.beginPath();
        ctx.moveTo(x * scaleX, 0);
        ctx.lineTo(x * scaleX, height);
        ctx.stroke();
    }

    for (let y = 0; y <= parameters.GRID_SIZE_Y; y += 5) {
        ctx.beginPath();
        ctx.moveTo(0, y * scaleY);
        ctx.lineTo(width, y * scaleY);
        ctx.stroke();
    }

    // Draw platform (center square)
    const platformHalfSize = parameters.PLATFORM_HALF_SIZE * scaleX;
    ctx.fillStyle = 'rgba(100, 100, 220, 0.7)';
    ctx.fillRect(
        centerX - platformHalfSize,
        centerY - platformHalfSize,
        platformHalfSize * 2,
        platformHalfSize * 2
    );

    // Add platform border
    ctx.strokeStyle = 'rgba(50, 50, 150, 0.9)';
    ctx.lineWidth = 2;
    ctx.strokeRect(
        centerX - platformHalfSize,
        centerY - platformHalfSize,
        platformHalfSize * 2,
        platformHalfSize * 2
    );

    // Draw pipes
    const pipeWidth = parameters.PIPE_WIDTH * scaleX;
    ctx.fillStyle = 'rgba(100, 100, 220, 0.6)';

    // Horizontal pipe
    ctx.fillRect(
        0,
        centerY - pipeWidth/2,
        width,
        pipeWidth
    );

    // Vertical pipe
    ctx.fillRect(
        centerX - pipeWidth/2,
        0,
        pipeWidth,
        height
    );

    // Pipe borders
    ctx.strokeStyle = 'rgba(50, 50, 150, 0.7)';
    ctx.lineWidth = 1;
    ctx.strokeRect(0, centerY - pipeWidth/2, width, pipeWidth);
    ctx.strokeRect(centerX - pipeWidth/2, 0, pipeWidth, height);

    // Draw charging base (green circle at origin)
    ctx.fillStyle = 'rgba(100, 200, 100, 0.7)';
    ctx.beginPath();
    ctx.arc(chargeX * scaleX, chargeY * scaleY, scaleX * 0.7, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#008800';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Add charging symbol
    ctx.fillStyle = 'white';
    ctx.font = '16px Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('⚡', chargeX * scaleX, chargeY * scaleY);

    // Draw charging station range indicator
    ctx.beginPath();
    ctx.arc(chargeX * scaleX, chargeY * scaleY, parameters.CHARGING_STATION_RANGE * scaleX, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(0, 255, 0, 0.3)';
    ctx.setLineDash([2, 2]);
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.setLineDash([]);

    // Display per-step reward
    if (state.per_step_reward !== undefined) {
        ctx.fillStyle = 'black';
        ctx.font = '12px Arial';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';

        const rewardColor = state.per_step_reward > 0 ? 'green' : state.per_step_reward < 0 ? 'red' : 'gray';
        ctx.fillStyle = rewardColor;
        ctx.fillText(`Per-step Reward: ${state.per_step_reward.toFixed(3)}`, 10, 30);
    }


    // Draw disaster if active
    if (state.disaster && state.disaster.active) {
        const disasterX = state.disaster.position[0] * scaleX;
        const disasterY = state.disaster.position[1] * scaleY;

        // Draw disaster area with simple gradient
        const radius = scaleX * 0.6;
        const gradient = ctx.createRadialGradient(
            disasterX, disasterY, radius * 0.2,
            disasterX, disasterY, radius
        );
        gradient.addColorStop(0, 'rgba(255, 0, 0, 0.8)');
        gradient.addColorStop(1, 'rgba(255, 0, 0, 0.1)');

        ctx.beginPath();
        ctx.arc(disasterX, disasterY, radius, 0, Math.PI * 2);
        ctx.fillStyle = gradient;
        ctx.fill();

        // Draw sensor range indicator
        ctx.beginPath();
        ctx.arc(disasterX, disasterY, parameters.SENSOR_RANGE * scaleX, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(255, 165, 0, 0.5)';
        ctx.setLineDash([5, 5]);
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // Draw drones
    if (state.drone_states) {
        const droneRadius = 0.35 * scaleX;

        for (let i = 0; i < state.drone_states.length; i++) {
            const drone = state.drone_states[i];
            const posX = drone.position[0] * scaleX;
            const posY = drone.position[1] * scaleY;

            // Drone color based on status
            let fillColor;
            if (drone.battery <= 0) {
                fillColor = '#444444'; // Dead drone
            } else if (drone.is_charging) {
                fillColor = '#90EE90'; // Charging drone
            } else {
                fillColor = '#7DB9B6'; // Normal drone
            }

            // Draw drone body
            ctx.beginPath();
            ctx.arc(posX, posY, droneRadius, 0, Math.PI * 2);
            ctx.fillStyle = fillColor;
            ctx.fill();
            ctx.strokeStyle = 'black';
            ctx.lineWidth = 1;
            ctx.stroke();

            // Draw drone ID
            ctx.fillStyle = 'white';
            ctx.font = `${Math.round(droneRadius * 0.8)}px Arial`;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(i.toString(), posX, posY);

            // Draw velocity vector if drone is moving
            if (!drone.dead && (Math.abs(drone.velocity[0]) > 0.01 || Math.abs(drone.velocity[1]) > 0.01)) {
                const velocityScale = 3; // Scale factor for velocity visualization
                const vx = drone.velocity[0] * velocityScale * scaleX;
                const vy = drone.velocity[1] * velocityScale * scaleY;

                // Draw velocity arrow
                ctx.strokeStyle = drone.is_charging ? '#90EE90' : '#FF6B6B';
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.moveTo(posX, posY);
                ctx.lineTo(posX + vx, posY + vy);
                ctx.stroke();

                // Draw arrowhead
                const angle = Math.atan2(vy, vx);
                const arrowLength = 8;
                ctx.beginPath();
                ctx.moveTo(posX + vx, posY + vy);
                ctx.lineTo(
                    posX + vx - arrowLength * Math.cos(angle - Math.PI/6),
                    posY + vy - arrowLength * Math.sin(angle - Math.PI/6)
                );
                ctx.moveTo(posX + vx, posY + vy);
                ctx.lineTo(
                    posX + vx - arrowLength * Math.cos(angle + Math.PI/6),
                    posY + vy - arrowLength * Math.sin(angle + Math.PI/6)
                );
                ctx.stroke();
            }

            // Battery indicator
            const batteryHeight = droneRadius * 0.8;
            const batteryWidth = droneRadius * 0.2;
            const batteryX = posX + droneRadius * 1.2;
            const batteryY = posY - batteryHeight / 2;

            // Battery outline
            ctx.strokeStyle = '#444';
            ctx.lineWidth = 1;
            ctx.strokeRect(batteryX, batteryY, batteryWidth, batteryHeight);

            // Battery fill
            const fillHeight = (drone.battery / 100) * batteryHeight;
            const batteryColor = drone.battery > 70 ? 'green' :
                                drone.battery > 30 ? 'orange' : 'red';

            ctx.fillStyle = batteryColor;
            ctx.fillRect(
                batteryX,
                batteryY + batteryHeight - fillHeight,
                batteryWidth,
                fillHeight
            );

            // Draw sensor range circle if in normal mode
            ctx.beginPath();
            ctx.arc(posX, posY, parameters.SENSOR_RANGE * scaleX, 0, Math.PI * 2);
            ctx.strokeStyle = 'rgba(100, 100, 200, 0.15)';
            ctx.setLineDash([2, 3]);
            ctx.lineWidth = 1;
            ctx.stroke();
            ctx.setLineDash([]);
        }
    }

    // Draw step count and other info
    ctx.fillStyle = 'black';
    ctx.font = '14px Arial';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText(`Step: ${state.step_count} | FPS: ${ui_state.fps || 0}`, 10, 10);

    // Disaster status if active
    if (state.disaster && state.disaster.active) {
        ctx.fillStyle = 'red';
        ctx.fillText(`DISASTER ACTIVE`, 10, 50);


        // Check if detected
        let detected = false;
        for (const drone of state.drone_states) {
            if (drone.disaster_detected) {
                detected = true;
                break;
            }
        }

        if (detected) {
            ctx.fillStyle = 'green';
            ctx.fillText(`DETECTED`, 150, 50);
        } else {
            ctx.fillStyle = 'orange';
            ctx.fillText(`UNDETECTED`, 150, 50);
        }
    }

    // show exact disaster coords
    ctx.fillStyle = 'black';
    ctx.font = '12px Arial';
    ctx.fillText(
      `Location: (${state.last_detected_disaster_position[0].toFixed(1)}, ` +
      `${state.last_detected_disaster_position[1].toFixed(1)})`,
      10, 90
    );
    // Add display of undetected steps
    ctx.fillText(
      `Undetected steps: ${state.disaster_undetected_steps}`,
      10, 110
    );


    // Display critical battery status
    let criticalCount = 0;
    let lowCount = 0;
    let deadCount = 0;

    for (const drone of state.drone_states) {
        if (drone.battery <= 0) deadCount++;
        if (drone.battery < 20) criticalCount++;
        else if (drone.battery < 50) lowCount++;
    }

    if (deadCount > 0) {
        ctx.fillStyle = 'darkred';
        ctx.fillText(`${deadCount} drones dead`, 10, height - 60);
    }

    if (criticalCount > 0) {
        ctx.fillStyle = 'red';
        ctx.fillText(`${criticalCount} drones critical`, 10, height - 20);
    }

    if (lowCount > 0) {
        ctx.fillStyle = 'orange';
        ctx.fillText(`${lowCount} drones low battery`, 10, height - 40);
    }

    // Add drone status list with charging and battery info
    if (state.drone_states && state.drone_states.length > 0) {
        ctx.font = '12px Arial';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';

        // Create a background for better readability
        const padding = 5;
        const lineHeight = 16;
        const textWidth = 340; // Increased width for more text
        const boxHeight = state.drone_states.length * lineHeight + padding * 2;

        ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
        ctx.fillRect(10, height - 80 - boxHeight, textWidth, boxHeight);
        ctx.strokeStyle = '#ddd';
        ctx.lineWidth = 1;
        ctx.strokeRect(10, height - 80 - boxHeight, textWidth, boxHeight);

        // Draw the drone status
        state.drone_states.forEach((drone, i) => {
            const batteryValue = Math.round(drone.battery);
            let batteryColor;
            let statusColor;

            // Battery color
            if (batteryValue <= 0) batteryColor = 'black';
            else if (batteryValue > 70) batteryColor = 'green';
            else if (batteryValue > 30) batteryColor = 'orange';
            else batteryColor = 'red';

            // Charging status color
            if (drone.battery <= 0) {
                statusColor = 'black';
            } else if (drone.is_charging) {
                statusColor = 'green';
            } else {
                statusColor = 'blue';
            }

            const y = height - 80 - boxHeight + padding + i * lineHeight;

            // Draw drone number
            ctx.fillStyle = 'black';
            ctx.fillText(`Drone ${i}:`, 15, y);

            // Draw battery percentage
            ctx.fillStyle = batteryColor;
            ctx.fillText(`${batteryValue}%`, 70, y);

            // Draw charging status
            ctx.fillStyle = statusColor;
            const chargingStatus = drone.battery <= 0 ? 'DEAD' :
                                  drone.is_charging ? 'CHARGING' : 'ACTIVE';
            ctx.fillText(chargingStatus, 110, y);

            // Show current position
            ctx.fillStyle = '#333';
            const posStr = `(${drone.position[0].toFixed(1)}, ${drone.position[1].toFixed(1)})`;
            ctx.fillText(posStr, 200, y);

            // Show velocity magnitude
            const velocityMag = Math.sqrt(drone.velocity[0] * drone.velocity[0] +
                                          drone.velocity[1] * drone.velocity[1]);
            ctx.fillStyle = '#666';
            const velStr = `v:${velocityMag.toFixed(2)}`;
            ctx.fillText(velStr, 270, y);
        });
    }

    // Draw disaster detection legend
    if (state.drone_states && state.drone_states.length > 0) {
        // Create a legend showing disaster detection status for each drone
        const legendPadding = 5;
        const legendLineHeight = 16;
        const legendWidth = 140;
        const legendTitle = "Disaster Detection:";
        const legendHeight = state.drone_states.length * legendLineHeight + legendLineHeight + legendPadding * 2;

        // Position the legend on the right side
        const legendX = width - legendWidth - 10;
        const legendY = height - 60 - legendHeight;

        // Draw legend background
        ctx.fillStyle = 'rgba(255, 255, 255, 0.7)';
        ctx.fillRect(legendX, legendY, legendWidth, legendHeight);
        ctx.strokeStyle = '#ddd';
        ctx.lineWidth = 1;
        ctx.strokeRect(legendX, legendY, legendWidth, legendHeight);

        // Draw legend title
        ctx.fillStyle = 'black';
        ctx.font = 'bold 12px Arial';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'top';
        ctx.fillText(legendTitle, legendX + legendPadding, legendY + legendPadding);

        // Draw detection status for each drone
        ctx.font = '12px Arial';
        state.drone_states.forEach((drone, i) => {
            const y = legendY + legendPadding + legendLineHeight + (i * legendLineHeight);

            // Draw drone number
            ctx.fillStyle = 'black';
            ctx.fillText(`Drone ${i}:`, legendX + legendPadding, y);

            // Draw detection status
            if (drone.disaster_detected) {
                ctx.fillStyle = 'green';
                ctx.fillText('TRUE', legendX + legendPadding + 60, y);
            } else {
                ctx.fillStyle = 'red';
                ctx.fillText('FALSE', legendX + legendPadding + 60, y);
            }
        });
    }
}
    )RL_TOOLS_LITERAL";
        return ui;
    }
}
RL_TOOLS_NAMESPACE_WRAPPER_END

#endif