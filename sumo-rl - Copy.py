import numpy as np
import os
import sys
import time
import random
from collections import defaultdict
import csv

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import traci.constants

sumoCmd = ["sumo-gui", "-c", "inter.sumo.cfg", "--start"]
traci.start(sumoCmd)

print("Starting SUMO")
traci.gui.setSchema("View #0", "real world")

step = 0
emergencyVehicles = []  # '0','18','28','12','25','500','40','54','69'
Junc_edges = {"B1": ["B0B1", "C1B1", "B2B1", "A1B1"]}
Junc_detectors = {"B1": ['e3_1', 'e3_2', 'e3_3', 'e3_4']}
ev_priority = [[]]

# define different signal phases for an intersection
p1 = "rrrrrrrrrrrrGGGGGGrrrrrrrrrr"
p2 = "rrrrrrGGGGGGrrrrrrrrrrrrrrrr"
p3 = "GGGGGGrrrrrrrrrrrrrrrrrrrrrr"
p4 = "rrrrrrrrrrrrrrrrrrGGGGGGrrrr"

p1_g = "rrrrrrrrrrrGGGGggrrrrr"  # rrrrrrrrrrrGGGGggrrrrr
p2_g = "rrrrrrGGGggrrrrrrrrrrr"  # rrrrrrGGGggrrrrrrrrrrr
p3_g = "GGGGggrrrrrrrrrrrrrrrr"  # GGGGggrrrrrrrrrrrrrrrr
p4_g = "rrrrrrrrrrrrrrrrGGGGgg"  # rrrrrrrrrrrrrrrrGGGGgg

p1_y = "rrrrrrrrrrryyyyyyrrrrr"  # rrrrrrrrrrrGGGGggrrrrr
p2_y = "rrrrrryyyyyrrrrrrrrrrr"  # rrrrrrGGGggrrrrrrrrrrr
p3_y = "yyyyyyrrrrrrrrrrrrrrrr"  # GGGGggrrrrrrrrrrrrrrrr
p4_y = "rrrrrrrrrrrrrrrryyyyyy"  # rrrrrrrrrrrrrrrrGGGGgg

# Initialize Q-table for RL
state_space = [(x, y, z, w) for x in range(5) for y in range(5) for z in range(5) for w in range(5)]  # Vehicle densities
action_space = [(a, b, c, d) for a in range(56, 75, 1) for b in range(30, 60, 1) 
                for c in range(37, 75, 1) for d in range(57, 60, 1)]  # Phase durations
q_table = np.zeros((len(state_space), len(action_space)))

# RL Parameters
alpha = 0.1  # Learning rate
gamma = 0.9  # Discount factor
epsilon = 0.1  # Exploration rate

def get_state(edges):
    """Get state based on vehicle counts on each edge."""
    return tuple([min(traci.edge.getLastStepVehicleNumber(edge) // 10, 4) for edge in edges])

def choose_action(state):
    """Epsilon-greedy action selection."""
    if np.random.rand() < epsilon:
        return np.random.choice(len(action_space))  # Explore
    return np.argmax(q_table[state_space.index(state)])  # Exploit

def update_q_table(state, action, reward, next_state):
    """Update Q-table using the Q-learning formula."""
    current_q = q_table[state_space.index(state)][action]
    max_future_q = np.max(q_table[state_space.index(next_state)])
    q_table[state_space.index(state)][action] = current_q + alpha * (reward + gamma * max_future_q - current_q)

# Define the reward function
def calculate_reward(edges):
    """Reward based on waiting time, throughput, and penalties for idle states."""
    vehicles = traci.vehicle.getIDList()
    total_waiting_time = sum([traci.edge.getWaitingTime(edge) for edge in edges])
    total_throughput = sum([traci.edge.getLastStepVehicleNumber(edge) for edge in edges])
    
    # Add penalties for excessive waiting
    penalty = 0
    if total_waiting_time > 100:
        penalty = -50

    # Reward structure
    reward = -total_waiting_time + (5 * total_throughput) + penalty
    return reward


# Initialize tracking for overall best action and reward across epochs
overall_best_action = None
overall_best_reward = float('-inf')

# Number of epochs
epochs = 100
# traci.close()

for epoch in range(epochs):
    print(f"Starting Epoch {epoch + 1}/{epochs}")

    # Reset SUMO for each epoch
    # traci.close()
    # traci.start(sumoCmd)
    # Reload the simulation for the next episode
    traci.load(["-c", "inter.sumo.cfg", "--start"])


    # Reset simulation parameters
    step = 0
    best_action = None
    best_reward = float('-inf')
    # print(best_reward, "b rew\n")

    # Initialize cycle tracking variables
    current_cycle_step = 0  # Tracks time within the current cycle
    durations = action_space[0]  # Initial durations
    cycle_time = 0  # Initialize total cycle time

    while step < 10000:
        # At the start of a new cycle, choose a new action
        if current_cycle_step == 0:
            # Get the current state
            state = get_state(Junc_edges["B1"])

            # Choose an action
            action_index = choose_action(state)
            durations = action_space[action_index]

            # Extract durations for each direction
            n_green, e_green, s_green, w_green = durations
            yellow = 4

            # Define the total cycle time
            cycle_time = n_green + e_green + s_green + w_green + 4 * yellow

            # Define the time plan with a fixed sequence: north, east, south, west
            time_plan = [
                (0, n_green),  # North green phase
                (n_green, n_green + yellow),  # North yellow phase
                (n_green + yellow, n_green + yellow + e_green),  # East green phase
                (n_green + yellow + e_green, n_green + yellow + e_green + yellow),  # East yellow phase
                (n_green + yellow + e_green + yellow, n_green + yellow + e_green + yellow + s_green),  # South green phase
                (n_green + yellow + e_green + yellow + s_green, n_green + yellow + e_green + yellow + s_green + yellow),  # South yellow phase
                (n_green + yellow + e_green + yellow + s_green + yellow, n_green + yellow + e_green + yellow + s_green + yellow + w_green),  # West green phase
                (n_green + yellow + e_green + yellow + s_green + yellow + w_green, n_green + yellow + e_green + yellow + s_green + yellow + w_green + yellow)  # West yellow phase
            ]

        # Determine the current phase based on the time within the cycle
        for idx, (start, end) in enumerate(time_plan):
            if start <= current_cycle_step < end:
                # Use the appropriate signal phase
                signal_phases = [p1_g, p1_y, p2_g, p2_y, p3_g, p3_y, p4_g, p4_y]
                traci.trafficlight.setRedYellowGreenState("B1", signal_phases[idx])
                break

        # Step the simulation
        traci.simulationStep()

        # Increment the step and cycle step
        step += 1
        current_cycle_step += 1

        # Reset cycle step when the cycle ends
        if current_cycle_step >= cycle_time:
            current_cycle_step = 0

        # Get the next state and reward
        next_state = get_state(Junc_edges["B1"])
        reward = calculate_reward(Junc_edges["B1"])

        # Update Q-table
        update_q_table(state, action_index, reward, next_state)

        # Update best action for the current epoch
        if reward > best_reward:
            # print(reward,"\n")
            best_reward = reward
            best_action = durations

    # End of epoch
    # traci.close()

    print(f"Best reward for Epoch {epoch + 1}: {best_reward}")
    if best_action:
        print(f"Best signal durations for Epoch {epoch + 1}: {best_action}")

    # Update overall best action across all epochs
    if best_reward > overall_best_reward:
        overall_best_reward = best_reward
        overall_best_action = best_action

# Final output
if overall_best_action:
    best_n_green, best_e_green, best_w_green, best_s_green = overall_best_action
    print("\nOptimal Signal Durations Across All Epochs:")
    print(f"best_n_green = {best_n_green}")
    print(f"best_e_green = {best_e_green}")
    print(f"best_w_green = {best_w_green}")
    print(f"best_s_green = {best_s_green}")
    print(f"Highest reward achieved: {overall_best_reward}")
