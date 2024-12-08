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


def def_value():
    return 0


veh_waitTime = defaultdict(def_value)
veh_co2_consumption = defaultdict(def_value)
veh_fuel_consumption = defaultdict(def_value)
veh_travelTime = defaultdict(def_value)
veh_distance = defaultdict(def_value)
veh_speed = defaultdict(def_value)
step_waitTime = defaultdict(def_value)


def returnSum(myDict):
    sum = 0
    for i in myDict:
        sum = sum + myDict[i]

    return sum


def calcSpeed(distance=0, time=0):
    try:
        return distance/time
    except:
        return 0


def set_traditionalFlow(junc, edges, step):
    global p1, p2, p3, p4
    d1 = traci.edge.getLastStepVehicleNumber(edges[0])
    d2 = traci.edge.getLastStepVehicleNumber(edges[1])
    d3 = traci.edge.getLastStepVehicleNumber(edges[2])
    d4 = traci.edge.getLastStepVehicleNumber(edges[3])
    detector_max = max(d1, d2, d3, d4)
    # print("traditional traffic",detector_max," d1: ",d1," d2: ",d2," d3: ",d3," d4: ",d4)

    # if step > 10:
    # 	step = str(step)
    # 	step = step[-2: ]
    # 	step = int(step)# print(step)
    step = step % 190

    if d1 == detector_max != 0 and False:
        return traci.trafficlight.setRedYellowGreenState(junc, p1)
    elif d2 == detector_max != 0:
        return traci.trafficlight.setRedYellowGreenState(junc, p2)
    elif d3 == detector_max != 0:
        return traci.trafficlight.setRedYellowGreenState(junc, p3)
    elif d4 == detector_max != 0:
        return traci.trafficlight.setRedYellowGreenState(junc, p4)
    else:
        if 0 <= step < 51:
            return traci.trafficlight.setRedYellowGreenState(junc, p1_g)
        elif 51 <= step < 55:
            return traci.trafficlight.setRedYellowGreenState(junc, p1_y)
        elif 55 <= step < 91:
            return traci.trafficlight.setRedYellowGreenState(junc, p2_g)
        elif 91 <= step < 95:
            return traci.trafficlight.setRedYellowGreenState(junc, p2_y)
        elif 95 <= step < 145:
            return traci.trafficlight.setRedYellowGreenState(junc, p3_g)
        elif 145 <= step < 149:
            return traci.trafficlight.setRedYellowGreenState(junc, p3_y)
        elif 149 <= step < 185:
            return traci.trafficlight.setRedYellowGreenState(junc, p4_g)
        elif 185 <= step < 190:
            return traci.trafficlight.setRedYellowGreenState(junc, p4_y)


def set_trafficFlow(junc, edges, step):
    global p1_g, p2_g, p3_g, p4_g
    s_time = 0
    #56 30 37 57
    n_green = 40
    e_green = 30
    w_green = 40
    s_green = 30
    yellow = 4
    time_plan = []

    time_plan.append(s_time)
    s_time += n_green
    time_plan.append(s_time)
    s_time += yellow
    time_plan.append(s_time)
    s_time += e_green
    time_plan.append(s_time)
    s_time += yellow
    time_plan.append(s_time)
    s_time += s_green
    time_plan.append(s_time)
    s_time += yellow
    time_plan.append(s_time)
    s_time += w_green
    time_plan.append(s_time)
    s_time += yellow
    time_plan.append(s_time)

    time_cycle = s_time
    step = step % time_cycle

    if time_plan[0] <= step < time_plan[1]:
        return traci.trafficlight.setRedYellowGreenState(junc, p1_g)
    elif time_plan[1] <= step < time_plan[2]:
        return traci.trafficlight.setRedYellowGreenState(junc, p1_y)
    elif time_plan[2] <= step < time_plan[3]:
        return traci.trafficlight.setRedYellowGreenState(junc, p2_g)
    elif time_plan[3] <= step < time_plan[4]:
        return traci.trafficlight.setRedYellowGreenState(junc, p2_y)
    elif time_plan[4] <= step < time_plan[5]:
        return traci.trafficlight.setRedYellowGreenState(junc, p3_g)
    elif time_plan[5] <= step < time_plan[6]:
        return traci.trafficlight.setRedYellowGreenState(junc, p3_y)
    elif time_plan[6] <= step < time_plan[7]:
        return traci.trafficlight.setRedYellowGreenState(junc, p4_g)
    elif time_plan[7] <= step < time_plan[8]:
        return traci.trafficlight.setRedYellowGreenState(junc, p4_y)


def set_EVpriority(junc, lane1, lane2,lane3,lane4,i):
    global ev_priority
    global p1, p2, p3,p4

    if any(ambulance in lane1 for ambulance in emergencyVehicles) and p1 not in ev_priority[i]:
        ev_priority[i].append(p1)
    elif any(ambulance in lane2 for ambulance in emergencyVehicles) and p2 not in ev_priority[i]:
        ev_priority[i].append(p2)
    elif any(ambulance in lane3 for ambulance in emergencyVehicles) and p3 not in ev_priority[i]:
        ev_priority[i].append(p3)
    elif any(ambulance in lane4 for ambulance in emergencyVehicles) and p4 not in ev_priority[i]:
        ev_priority[i].append(p4)

    if  any(ambulance in lane1 for ambulance in emergencyVehicles) == False and p1 in ev_priority[i]:
        ev_priority[i].remove(p1)
    elif  any(ambulance in lane2 for ambulance in emergencyVehicles) == False and p2 in ev_priority[i]:
        ev_priority[i].remove(p2)
    elif  any(ambulance in lane3 for ambulance in emergencyVehicles) == False and p3 in ev_priority[i]:
        ev_priority[i].remove(p3)
    elif  any(ambulance in lane4 for ambulance in emergencyVehicles) == False and p4 in ev_priority[i]:
        ev_priority[i].remove(p4)

    # print(ev_priority)
    if len(ev_priority[i]) >= 1:
        return traci.trafficlight.setRedYellowGreenState(junc, ev_priority[i][0])
    else:
        return -1


while step < 2000:
    # time.sleep(0.001)
    traci.simulationStep()

    ev_count = 0

    wait_vehicle = True
    while wait_vehicle:
        vehicles = traci.vehicle.getIDList()
        for i in vehicles:
            veh_waitTime["vehID{0}".format(i)] = traci.vehicle.getAccumulatedWaitingTime(
                i)  # waiting time
            veh_co2_consumption["vehID{0}".format(i)] = veh_co2_consumption["vehID{0}".format(
                i)] + traci.vehicle.getCO2Emission(i)  # CO2 consumption
            veh_fuel_consumption["vehID{0}".format(i)] = veh_fuel_consumption["vehID{0}".format(
                i)] + traci.vehicle.getFuelConsumption(i)  # fuel consumption
            veh_travelTime["vehID{0}".format(
                i)] = veh_travelTime["vehID{0}".format(i)] + 1  # travel time
            veh_distance["vehID{0}".format(i)] = traci.vehicle.getDistance(
                i)  # distance travelled
            step_waitTime["step{0}".format(step)] = step_waitTime["step{0}".format(
                step)] + traci.vehicle.getWaitingTime(i)  # cumulative step waiting time
        wait_vehicle = False

    for key, value in Junc_edges.items():
        if step%5 == 0:
            set_trafficFlow(key, value, step)

    for key, value in Junc_detectors.items():
        d1_ev = []
        d2_ev = []
        d3_ev = []
        d4_ev = []

        d1_ev.extend(traci.multientryexit.getLastStepVehicleIDs(value[0]))
        d2_ev.extend(traci.multientryexit.getLastStepVehicleIDs(value[1]))
        d3_ev.extend(traci.multientryexit.getLastStepVehicleIDs(value[2]))
        d4_ev.extend(traci.multientryexit.getLastStepVehicleIDs(value[3]))

        set_EVpriority(key, d1_ev, d2_ev,d3_ev,d4_ev,ev_count)
        ev_count += 1

    step += 1


# CALCULATE VEHICLE SPEED using veh_travelTime and veh_distance
for i in range(len(veh_travelTime)):
    veh_speed["vehID{0}".format(i)] = calcSpeed(
        veh_distance["vehID{0}".format(i)], veh_travelTime["vehID{0}".format(i)])


# print vehicle waiting time
# print(veh_waitTime)
totalWait = returnSum(veh_waitTime)
print("Total waiting time for all vehicles:", returnSum(veh_waitTime))
print("Average waiting time:", int(totalWait)/len(veh_travelTime))

# print(veh_co2_consumption)
# print(veh_fuel_consumption)
# print(veh_travelTime)
# print(veh_distance)
# print(veh_speed)
# print(step_waitTime)
print("Total waiting time for all vehicles: calc using step",
      returnSum(step_waitTime))

# get network parameters
# IDsOfEdges=traci.edge.getIDList();
# print("IDs of the edges:", IDsOfEdges)
# IDsOfJunctions=traci.junction.getIDList();
# print("IDs of junctions:", IDsOfJunctions)


with open("output/smart_waitingTime.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_waitTime.items():
        writer.writerow([key, value])

with open("output/smart_travelTime.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_travelTime.items():
        writer.writerow([key, value])

with open("output/smart_avgSpeed.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_speed.items():
        writer.writerow([key, value])

with open("output/smart_Fuelconsumption.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_fuel_consumption.items():
        writer.writerow([key, value])

with open("output/smart_CO2consumption.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_co2_consumption.items():
        writer.writerow([key, value])

with open("output/smart_Distance.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_distance.items():
        writer.writerow([key, value])

traci.close()
