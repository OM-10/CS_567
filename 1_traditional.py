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
myjunction = ["B1"]


p1_g = "rrrrrrrrrrrGGGGggrrrrr" #rrrrrrrrrrrGGGGggrrrrr
p2_g = "rrrrrrGGGggrrrrrrrrrrr" #rrrrrrGGGggrrrrrrrrrrr
p3_g = "GGGGggrrrrrrrrrrrrrrrr" #GGGGggrrrrrrrrrrrrrrrr
p4_g = "rrrrrrrrrrrrrrrrGGGGgg" #rrrrrrrrrrrrrrrrGGGGgg

p1_y = "rrrrrrrrrrryyyyyyrrrrr" #rrrrrrrrrrrGGGGggrrrrr
p2_y = "rrrrrryyyyyrrrrrrrrrrr" #rrrrrrGGGggrrrrrrrrrrr
p3_y = "yyyyyyrrrrrrrrrrrrrrrr" #GGGGggrrrrrrrrrrrrrrrr
p4_y = "rrrrrrrrrrrrrrrryyyyyy" #rrrrrrrrrrrrrrrrGGGGgg


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


def set_traditionalFlow(junc, step):
    global p1, p2, p3, p4
    step = step%190
    # step = int(step/2)

    # if step > 10:
    #     step = str(step)
    #     step = step[-2:]
    #     step = int(step)
    #     # print(step)

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


while step < 1000:
    time.sleep(0.01)
    traci.simulationStep()

    wait_vehicle = True
    while wait_vehicle:
        vehicles = traci.vehicle.getIDList()
        # print('Number of vehicles',len(vehicles))
        for i in vehicles:
            # print(i)
            veh_waitTime["vehID{0}".format(
                i)] = traci.vehicle.getAccumulatedWaitingTime(i)
            veh_co2_consumption["vehID{0}".format(i)] = veh_co2_consumption["vehID{0}".format(
                i)] + traci.vehicle.getCO2Emission(i)
            veh_fuel_consumption["vehID{0}".format(i)] = veh_fuel_consumption["vehID{0}".format(
                i)] + traci.vehicle.getFuelConsumption(i)
            veh_travelTime["vehID{0}".format(
                i)] = veh_travelTime["vehID{0}".format(i)] + 1
            veh_distance["vehID{0}".format(i)] = traci.vehicle.getDistance(i)
            step_waitTime["step{0}".format(step)] = step_waitTime["step{0}".format(
                step)] + traci.vehicle.getWaitingTime(i)
        wait_vehicle = False

    for j_id in myjunction:
        set_traditionalFlow(j_id, step)

    step += 1


# calculate veh speed using veh_travelTime and veh_distance
for i in range(len(veh_travelTime)):
    veh_speed["vehID{0}".format(i)] = calcSpeed(
        veh_distance["vehID{0}".format(i)], veh_travelTime["vehID{0}".format(i)])


# print vehicle waiting time
# print(veh_waitTime)
total = returnSum(veh_waitTime)
print("Total waiting time for all vehicles:", returnSum(veh_waitTime))
print("Average waiting time:", int(total)/len(veh_travelTime))

# print(veh_co2_consumption)
# print(veh_fuel_consumption)
# print(veh_travelTime)
# print(veh_distance)
# print(veh_speed)
# print(step_waitTime)
# print("Total waiting time for all vehicles: calc using step", returnSum(step_waitTime))

# get network parameters
# IDsOfEdges=traci.edge.getIDList();
# print("IDs of the edges:", IDsOfEdges)
# IDsOfJunctions=traci.junction.getIDList();
# print("IDs of junctions:", IDsOfJunctions)


with open("output/traditional_waitingTime.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_waitTime.items():
        writer.writerow([key, value])

with open("output/traditional_travelTime.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_travelTime.items():
        writer.writerow([key, value])

with open("output/traditional_avgSpeed.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_speed.items():
        writer.writerow([key, value])

with open("output/traditional_Fuelconsumption.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_fuel_consumption.items():
        writer.writerow([key, value])

with open("output/traditional_CO2consumption.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_co2_consumption.items():
        writer.writerow([key, value])

with open("output/traditional_Distance.csv", "w", newline='') as csv_file:
    writer = csv.writer(csv_file)
    for key, value in veh_distance.items():
        writer.writerow([key, value])

traci.close()
