#!/usr/bin/env python3

import json
import numpy as np
import pinocchio as pin
from pyrep import PyRep
from pyrep.backend import sim
from Franka import Franka
from pyrep.errors import ConfigurationPathError, ConfigurationError
from pyrep.robots.configuration_paths.arm_configuration_path import (
    ArmConfigurationPath)
from pyrep.objects.object import Object

# reachability limitations defined in https://frankaemika.github.io/docs/control_parameters.html
joint_minimum=[-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]
joint_maximum=[2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973]

# saves handles to all link and start configuration
link = []
q_start = []

# tests if configuration is reachable
def reachable(joint_config):
    config = np.array(joint_config)
    joint_min = np.array(joint_minimum)
    joint_max = np.array(joint_maximum)
    return np.all(joint_min < config) & np.all(config < joint_max)


# checks for collisions
def self_collision():
    for entity_1 in range(8):
        for entity_2 in range(11):
            if entity_1 < entity_2 - 1:
                if sim.simCheckCollision(link[entity_1], link[entity_2]):
                    return True
    return False

# generates normalized path
def normalized_path(panda, py, q_goal):
    goal = np.array(q_goal)
    start = np.array(q_start)

    difference = goal - start
    trajectory = []

    # calculate joint positions for normalized path
    for k in range(20):
        trajectory.append((start + (k/20) * difference).tolist())

    # execute calculated path and test for collisions
    for state in trajectory:
        if not check_trajectory(panda, py, state):
            return False
    set_joint_position(panda, py, q_start)

    # generate a path out of normalized path and execute it
    path = ArmConfigurationPath(panda, np.array(trajectory).flatten())
    execute_path(py, path)

    return True


# calculate and execute linear path
def linear_path(panda, py, position, quaternion):

    try:
        path = panda.get_linear_path(position=position, euler=quaternion)
    except ConfigurationPathError:
        return False

    execute_path(py, path)
    return True


# calculate nonlinear path if normalized path could not be calculated
def replan_path_with_joint_configuration(panda, py, q_goal):
    try:
        path = panda.get_nonlinear_path_with_config(q_goal)
    except ConfigurationPathError:
        return False

    execute_path(py, path)
    return True


# calculate nonlinear path if linear path could not be calculated
def replan_path(panda, py, position, quaternion):
    try:
        path = panda.get_nonlinear_path(position=position, quaternion=quaternion)
    except ConfigurationPathError:
        return False

    execute_path(py, path)
    return True


# checks if the joint configuration given is reachable and does not cause self collision
def check_trajectory(panda, py, state):
    if reachable(state):
        set_joint_position(panda, py, state)
        return not self_collision()
    else:
        return False


# moves the robot to the given configuration
def set_joint_position(panda, py, state):
    panda.set_joint_target_positions(state)

    before = np.array(panda.get_joint_positions())
    for i in range(5):
        py.step()

    count = 0
    # wait until the robot does not move anymore and therefore has reached its configuration
    while np.any(np.abs( before - np.array(panda.get_joint_positions())) > 0.001):
        if count >= 1000:
            break
        before = np.array(panda.get_joint_positions())
        py.step()
        count += 1


    for i in range(5):
        py.step()

# executes given path step by step and visualizes it while executing
def execute_path(py, trajectory):
    trajectory.visualize()
    while not trajectory.step():
        py.step()
    trajectory.clear_visualization()


# transforms the given transformation matrix into position and quaternion coordinates
def cartesian_to_position_and_quaternion(cartesian):
    se3 = np.transpose(np.array(cartesian).reshape((4, 4)))
    position = se3[:3, 3]
    q = pin.Quaternion(se3[:3, :3])
    quaternion = [q.x, q.y, q.z, q.w]
    return [position, quaternion]


# validates configuration by moving to the position and checking if the config is reachable
# and does not cause self collision
def valid_conf(panda, py, q):
    set_joint_position(panda, py, q)
    valid = not self_collision()
    # after testing, reset to position before
    set_joint_position(panda, py, q_start)
    return valid

if __name__ == '__main__':

    py = PyRep()

    py.launch('../Panda_1_2.ttt',
              headless=False)
    py.start()
    panda = Franka()

    # define files for input and output
    joint_input = open("../files/self_collision_specific_joint_poses.data", "r")
    cartesian_input = open("../files/self_collision_target_cartesian_poses.data", "r")
    joint_output = open("../files/task1_2_joint_trajectory.data", "w")
    cartesian_output = open("../files/task1_2_cartesian_trajectory.data", "w")

    # get handles of all links
    for i in range(8):
        link.append(sim.simGetObjectHandle("Panda_link" + str(i) + "_visual"))

    link.append(sim.simGetObjectHandle("Panda_gripper_visual"))
    link.append(sim.simGetObjectHandle("Panda_leftfinger_visible"))
    link.append(sim.simGetObjectHandle("Panda_rightfinger_visual"))

    # save starting configuration
    q_start = panda.get_joint_positions()

    # test all transformation matrixes given
    for line in cartesian_input:
        # reset robot to starting stage
        set_joint_position(panda, py, q_start)

        # load cartesian transformation
        cartesian = json.loads(line)["O_T_EE_d"]

        # transform transformation into
        q = cartesian_to_position_and_quaternion(cartesian)

        # generate dictionary for output file
        output_dict = {"reachable pose": False, "valid linear trajectory": False, "valid trajectory": False}

        try:
            # try to find a valid configuration for the given transformation
            configs = panda.solve_ik_via_sampling(position=q[0], quaternion=q[1], trials=600)

            if reachable(configs):
                # document that the transformation was reachable
                output_dict["reachable pose"] = True

                # check if it is possible to calculate a linear path
                valid = linear_path(panda, py, q[0], q[1])
                if valid:
                    # document that a linear path was found
                    output_dict["valid linear trajectory"] = True
                    output_dict["valid trajectory"] = True
                else:
                    # document if a nonlinear path could be calculated
                    output_dict["valid trajectory"] = replan_path(panda, py, q[0], q[1])
        except ConfigurationError:
            pass

        # write result into output file
        json.dump(output_dict, cartesian_output)
        cartesian_output.write("\n")
        cartesian_output.flush()
        py.step()

    # check each configurations
    for line in joint_input:
        # reset robot to starting configuration
        set_joint_position(panda, py, q_start)
        py.step()

        # load configuration
        q = json.loads(line)["q"]

        # define output dictionary
        output_dict = {"reachable": False, "valid_configuration": False, "valid normalized trajectory": False, "valid trajectory": False}

        if reachable(q):
            # document that the configuration was reachable
            output_dict["reachable"] = True

            # test if the configuration is reachable and does not cause self collision
            if valid_conf(panda, py, q):
                # document result
                output_dict["valid_configuration"] = True

                if normalized_path(panda, py, q):
                    # document that a normalized path could be found
                    output_dict["valid normalized trajectory"] = True
                    output_dict["valid trajectory"] = True
                else:
                    # document if a nonlinear path could be calculated
                    output_dict["valid trajectory"] = replan_path_with_joint_configuration(panda, py, q)

        # write results to output file
        json.dump(output_dict, joint_output)
        joint_output.write("\n")
        joint_output.flush()
        py.step()

    # close all files and simulation
    py.stop()
    py.shutdown()
    joint_input.close()
    cartesian_input.close()
    joint_output.close()
    cartesian_output.close()
