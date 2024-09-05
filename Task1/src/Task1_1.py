#!/usr/bin/env python3.10

import json
import time
from pyrep import PyRep
from pyrep.objects.joint import Joint
from pyrep.backend import sim

py = PyRep()

py.launch('../frankaSim1_1.ttt', headless=False)

py.start()

# define input and output files
input = open("../files/self_collision_specific_joint_poses.data", "r")
output = open("../files/task1_1_self_collision.data", "w")

joint = []
link = []

# append all joints and links to a list
for i in range(7):
    joint.append(Joint("Franka_joint" + str(i+1)))
    link.append(sim.simGetObjectHandle("Franka_link" + str(i+1)))

link.append(sim.simGetObjectHandle("Franka_link8"))
link.append(sim.simGetObjectHandle("FrankaGripper"))

# test each configuration
for line in input:
    # load configuration
    q = json.loads(line)["q"]
    py.step()

    # set joint positions
    for j in range(7):
        joint[j].set_joint_position(q[j], disable_dynamics=True)

    # create output dictionary
    output_dict = {"collision": "false", "collision groups": {}}
    collision_amount = 0

    # test for collisions
    for entity_1 in range(7):
        for entity_2 in range(9):
            # don't test neighboring links for collision
            if entity_1 < entity_2 - 1:
                if sim.simCheckCollision(link[entity_1], link[entity_2]):
                    output_dict["collision"] = "true"
                    collision_amount = collision_amount + 1
                    output_dict["collision groups"][str(collision_amount)] = str(sim.simGetObjectName(link[entity_1])) + " " + str(sim.simGetObjectName(link[entity_2]))

    # save result for configuration in output file
    json.dump(output_dict, output)
    output.write("\n")
    output.flush()
    py.step()

py.stop()
py.shutdown()
input.close()
output.close()
