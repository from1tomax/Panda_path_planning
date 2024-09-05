#!/usr/bin/env python3
import math
import json
import time
import numpy as np
from pyrep import PyRep
from pyrep.errors import ConfigurationPathError
from pyrep.objects.object import Object
from PandaClasses import PandaLeftGripper, PandaLeft, PandaRight, PandaRightGripper

# set joint position for both robots parallel
def set_joint_position(py, panda1, state1, panda2, state2):

    panda1.set_joint_target_positions(state1)
    before1 = np.array(panda1.get_joint_positions())
    before2 = np.array(panda2.get_joint_positions())
    panda2.set_joint_target_positions(state2)

    for i in range(5):
        py.step()

    while np.any(np.abs( before1 - np.array(panda1.get_joint_positions())) > 0.0001) or np.any(np.abs( before2 - np.array(panda2.get_joint_positions())) > 0.0001):
        before1 = np.array(panda1.get_joint_positions())
        before2 = np.array(panda2.get_joint_positions())
        py.step()

    for i in range(5):
        py.step()


# TODO parallelize path planning for faster computation
# calculate and execute path for both robots sequently
def get_path_sequently(py, panda1, position1, euler1, panda2, position2, euler2):
    try:
        path1, list1 = panda1.get_path(position=position1, euler=euler1, distance_threshold=0.2, trials=300)
    except ConfigurationPathError:
        print("no path for panda1")
        return False

    time_start1 = time.time()
    while not path1.step():
        py.step()
    time_stop1 = time.time()
    try:
        path2, list2 = panda2.get_path(position=position2, euler=euler2, distance_threshold=0.2, trials=300)
    except ConfigurationPathError:
        print("no path for panda2")
        return False

    time_start2 = time.time()
    while not path2.step():
        py.step()

    return [time_start2 - (time_stop1 - time_start1), list1, list2]

# calculate and execute paths semi-parallelized
def get_path_before(py, panda1, position1, euler1, panda2, position2, euler2):

    try:
        path1, list1 = panda1.get_path(position=position1, euler=euler1, distance_threshold=0.2, trials=300)
        path2, list2 = panda2.get_path(position=position2, euler=euler2, distance_threshold=0.2, trials=300)
    except ConfigurationPathError:
        return get_path_sequently(py, panda1, position1, euler1, panda2, position2, euler2)

    time_start = time.time()

    bool1 = not path1.step()
    bool2 = not path2.step()

    while bool1 or bool2:
        if bool1:
            bool1 = not path1.step()
        if bool2:
            bool2 = not path2.step()
        py.step()
    return [time_start, list1, list2]

if __name__ == '__main__':

    py = PyRep()

    py.launch('../Pandas.ttt',
              headless=False)
    py.start()

    # initialize objects
    panda_right = PandaRight()
    gripper_right = PandaRightGripper()
    panda_left = PandaLeft()
    gripper_left = PandaLeftGripper()
    cubid = Object.get_object("Cuboid")

    # open all output files
    time_output = open("../files/task3_1_times.data", "a")
    path_output = open("../files/task3_1_paths.data", "a")

    # create output dictionaries
    time_output_dict = {"PandaRight graps object": 0., "PandaLeft graps object": 0., "turnover time": 0.}
    path_output_dict = {"PandaRight to grasping position": [], "PandaLeft to (0.7, -0.1, 0.2)": [], "PandaRight to grasping position of PandaLeft": [], "PandaLeft to grasping position": []}

    # set up start position
    set_joint_position(py, panda_right, [0.6981316804885864, -0.7853981852531433, 0.0, -2.356194496154785, 0.0, 1.5707963705062866, 0.7853981852531433],
                       panda_left, [0.6981316804885864, -0.7853981852531433, 0.0, -2.356194496154785, 0.0, 1.5707963705062866, 0.7853981852531433])

    # start measuring time
    # move right robot to cuboid and make left robot ready to grasp
    time_start, path1_1, path2_1 = get_path_before(py, panda_right, [0.55, 0., 0.01], [math.pi, 0, 0], panda_left, [0.7, -0.1, 0.3], [-math.pi / 2, 0, -math.pi / 2])

    # right robot grasps
    while not gripper_right.actuate(0, 0.7):
        py.step()

    gripper_right.grasp(cubid)
    py.step()

    # start timer for first grasp
    time_grasp1 = time.time()

    # move right and left robot to a common position
    time_start2, path1_2, path2_2 = get_path_sequently(py, panda_right, [0.7, -0.01, 0.3], [math.pi/2, 0, 0], panda_left, [0.7, 0.01, 0.3], [-math.pi / 2, 0, -math.pi / 2])

    # left robot grasps
    while not gripper_left.actuate(0, 0.7):
        py.step()

    gripper_left.grasp(cubid)
    py.step()

    # time is stopped
    time_grasp2 = time.time()

    # document all times
    time_output_dict["turnover time"] = time_grasp2 - time_start2
    time_output_dict["PandaLeft graps object"] = (time_grasp2 - time_start2) + (time_grasp1 - time_start)
    time_output_dict["PandaRight graps object"] = time_grasp1 - time_start

    # document all paths corresponding to the times
    path_output_dict["PandaRight to grasping position"] = np.array(path1_1).reshape(-1, 7).tolist()
    path_output_dict["PandaLeft to (0.7, -0.1, 0.2)"] = np.array(path2_1).reshape(-1, 7).tolist()
    path_output_dict["PandaRight to grasping position of PandaLeft"] = np.array(path1_2).reshape(-1, 7).tolist()
    path_output_dict["PandaLeft to grasping position"] = np.array(path2_2).reshape(-1, 7).tolist()

    # write results to files
    json.dump(time_output_dict, time_output)
    json.dump(path_output_dict, path_output)
    time_output.write("\n")
    path_output.write("\n")
    time_output.flush()
    path_output.flush()

    # stop shortly to show result
    for i in range(100):
        py.step()

    #
    py.stop()
    py.shutdown()
    time_output.close()