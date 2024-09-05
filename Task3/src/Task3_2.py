#!/usr/bin/env python3
import numpy
import math
import json
import time
import numpy as np
from pyrep import PyRep
from PandaClasses import PandaRight, PandaLeft, PandaLeftGripper, PandaRightGripper
from pyrep.errors import ConfigurationPathError, ConfigurationError, IKError
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.objects.object import Object

# TODO impediance control for improvement
def impediance_control(py, panda, desired_conf, position, euler):
    # makes sure end effector does not leave given pose{position, euler} while changing config to desired_conf
    # additionally checks for collisions

    # py is pyrep simulation for py.step() for example
    # all arrays are given as numpy arrays

    # returns False if no solution is found
    return False

# set the target postions for both robots and move to the positions simultanously
def set_joint_position_both(py, panda1, state1, panda2, state2):

    panda1.set_joint_target_positions(state1)
    before1 = np.array(panda1.get_joint_positions())
    before2 = np.array(panda2.get_joint_positions())
    panda2.set_joint_target_positions(state2)

    for i in range(5):
        py.step()
    count = 0

    while np.any(np.abs( before1 - np.array(panda1.get_joint_positions())) > 0.0001) or np.any(np.abs( before2 - np.array(panda2.get_joint_positions())) > 0.0001):
        count = count + 1
        before1 = np.array(panda1.get_joint_positions())
        before2 = np.array(panda2.get_joint_positions())
        py.step()

    for i in range(5):
        py.step()


# set the target postions for one robots and move to the position
def set_joint_position(py, panda, state):
    if np.any(np.abs(np.array(panda.get_joint_positions()) - np.array(state)) > 0.001):

        panda.set_joint_target_positions(state)
        before1 = np.array(panda.get_joint_positions())

        for i in range(5):
            py.step()

        while np.any(np.abs(before1 - np.array(panda.get_joint_positions())) > 0.0001):
            before1 = np.array(panda.get_joint_positions())
            py.step()
        for i in range(5):
            py.step()

# uses the set_joint_target_positions to test if configurations are automatically impediance controlled
def set_joint_check_impediance(py, state, panda, pos_before, rot_before):
    panda.set_joint_target_positions(state)
    before = np.array(panda.get_joint_positions())

    for i in range(5):
        py.step()
    while np.any(np.abs(before - np.array(panda.get_joint_positions())) > 0.0001):
        before = np.array(panda.get_joint_positions())
        py.step()
        if not check_collision_and_movement_of_gripper(panda, pos_before, rot_before):
            return False
    for i in range(5):
        py.step()
    return True

# trys to set new joint configurations
# if the process to reach these configurations lead to collision or moving away from initial pose, the method returns false
def set_joint_position_without_moving_gripper(py, panda, state):
    if np.any(np.abs(np.array(panda.get_joint_positions()) - np.array(state)) > 0.001):
        conf_before = panda.get_joint_positions()
        pos_before = np.array(panda.get_tip().get_position())
        rot_before = np.array(panda.get_tip().get_orientation())

        if not impediance_control(py, panda, np.array(state), pos_before, rot_before):
            for i in range(10):
                try:
                    path = panda.get_nonlinear_path_with_config(state)

                    while not path.step():
                        py.step()
                        if not check_collision_and_movement_of_gripper(panda, pos_before, rot_before):
                            set_joint_position(py, panda, conf_before)
                            return set_joint_check_impediance(py, state, panda, pos_before, rot_before)

                    return True
                except ConfigurationPathError:
                    continue
            return set_joint_check_impediance(py, state, panda, pos_before, rot_before)
    return True

# changes the joint configuration of both robots while being in dual mode
# dual mode is interrupted if impediance control failed
def change_joint_conf(py, panda_r, state_r, gripper_l, panda_l, state_l, calculate_mode):
    bool1 = not impediance_control(py, panda_l, np.array(state_l), np.array(panda_l.get_tip().get_position()),
                           np.array(panda_l.get_tip().get_orientation()))
    bool2 = not impediance_control(py, panda_r, np.array(state_r), np.array(panda_r.get_tip().get_position()), np.array(panda_r.get_tip().get_orientation()))

    if bool1 or bool2:

        while not gripper_l.actuate(1, 0.7):
            py.step()

        get_path_semi_parallelized(py, panda_l, [0.7, -0.1, 0.8], [-math.pi / 2, 0, -math.pi / 2], panda_r, [0.7, 0.1, 0.8], [math.pi/2, 0, 0])

        get_path_with_conf_sequently(py, panda_l, state_l, panda_r, state_r)

        if not calculate_mode:
            while not gripper_l.actuate(0, 0.7):
                py.step()

# check if current pose is in collision or away from starting pose
def check_collision_and_movement_of_gripper(panda, pos_before, rot_before):
    if panda.check_arm_collision():
        return False
    pos_current = np.array(panda.get_tip().get_position())
    rot_current = np.array(panda.get_tip().get_orientation())
    if np.any(np.abs(pos_before - pos_current) > 0.05) or np.any(np.abs(rot_before - rot_current) > 5 * math.pi / 180):
        return False
    return True


# TODO parallelize path planning for faster computation
# execute calculated paths sequently
def get_path_sequently(py, panda1, position1, euler1, panda2, position2, euler2):
    try:
        path1, list1 = panda1.get_path(position=position1, euler=euler1, distance_threshold=0.2, trials=300)
    except ConfigurationPathError:
        return False

    time_start1 = time.time()
    while not path1.step():
        py.step()
    time_stop1 = time.time()

    try:
        path2, list2 = panda2.get_path(position=position2, euler=euler2, distance_threshold=0.2, trials=300)
    except ConfigurationPathError:
        return False

    time_start2 = time.time()
    while not path2.step():
        py.step()

    return [time_start2 - (time_stop1 - time_start1), list1, list2]


# execute both paths parallel even though they are created sequently
def get_path_semi_parallelized(py, panda1, position1, euler1, panda2, position2, euler2):

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


# calculate waypoints on circle
def calculate_circular_path_positions(degree_offset):
    radius = 0.2
    degrees = np.arange(0, 360, degree_offset)

    x = np.repeat([0.7], degrees.shape[0])
    y = np.zeros_like(degrees) - radius * np.sin(degrees * np.pi / 180.)
    z = 0.6 - radius * np.cos(degrees * np.pi / 180)

    return [numpy.column_stack((x, y, z)), degrees]


# calculate path for circle
def get_circle(py, panda_right, panda_left, gripper_left, file, calculate_mode):

    # get waypoints
    degree_offset = 30
    position_right, degrees = calculate_circular_path_positions(degree_offset)

    waypoints = position_right.shape[0]
    euler_right = [math.pi/2, 0, 0]
    euler_left = [-math.pi / 2, 0, -math.pi / 2]
    position_left = position_right

    # calculate path for each waypoint
    for w_p in range(waypoints):

        # reset velocity
        zeros = np.zeros(7)
        panda_right.set_joint_target_velocities(zeros)
        panda_left.set_joint_target_velocities(zeros)

        # determine starting and ending degree for path inbetween waypoints
        degree1 = degrees[w_p]
        degree2 = 360
        if w_p < waypoints - 1:
            degree2 = degrees[w_p+1]

        if degree2 <= 90 or degree1 >= 270:
            # calculate circular path depending if it is used for finding out initialization configs or not
            if calculate_mode:
                if not calculate_path_between_waypoints(py, True, file, degree1, degree2, panda_left, panda_right,
                                                        position_left[w_p], position_right[w_p], euler_left,
                                                        euler_right):
                    return False
            else:
                if not execute_path(py, gripper_left, True, file, degree1, degree2, panda_left, panda_right, euler_left, euler_right):
                    return False
        else:
            # change position if critical pose reached
            if degree1 == 180:
               change_joint_conf(py, panda_right,
                                  [-0.77614498 , 1.70053887, -2.70511484, -2.4469986,  -2.03350592,  0.85343289, -2.038836],
                                  gripper_left, panda_left, [-0.06590486, -1.4442215, 0.44399443, -2.51862645, 0.01053264, 2.59154797, 2.78772593], calculate_mode)
            # calculate circular path depending if it is used for finding out initialization configs or not
            if calculate_mode:
                if not calculate_path_between_waypoints(py, False, file, degree1, degree2, panda_right, panda_left, position_right[w_p], position_left[w_p], euler_right, euler_left):
                    return False
            else:
                if not execute_path(py, gripper_left, False, file, degree1, degree2, panda_right, panda_left, euler_right, euler_left):
                    return False
    return True


# recalculate path with initialization configs given by file
def execute_path(py, gripper_left, left_right, file, degree1, degree2, panda_1, panda_2, euler_1, euler_2):

    line = file.readline()

    if line:
        # get configs for robot 1 and 2
        conf_dict = json.loads(line)
        conf_1 = []
        conf_2 = []
        if left_right:
            conf_1 = conf_dict["Left"]
            conf_2 = conf_dict["Right"]
        else:
            conf_1 = conf_dict["Right"]
            conf_2 = conf_dict["Left"]
    else:
        print("no initializaiton configs found!! Calculate them first!")
        return False

    # set initialization configs for both robots
    for i in range(10):
        if set_joint_position_without_moving_gripper(py, panda_1, conf_1):
            break
        if i == 4:
            return False

    for i in range(10):
        if set_joint_position_without_moving_gripper(py, panda_2, conf_2):
            break
        if i == 4:
            return False

    # calculate path again for initialization configs
    path_1, points_1 = get_cartesian_path(degree1, degree2, euler_1, panda_1)
    path_2, points_2 = get_cartesian_path(degree1, degree2, euler_2, panda_2)

    # move both robots simultanously along paths
    if points_2 and points_1:
        bool1 = not path_1.step()
        bool2 = not path_2.step()

        while bool1 or bool2:
            if bool1:
                bool1 = not path_1.step()
            if bool2:
                bool2 = not path_2.step()
            py.step()
    else:
        return False
    return True

# calculate joint position for pose via jacobian
def get_jaco(panda, position, euler):
    try:
        return panda.solve_ik_via_jacobian(position, euler)
    except IKError:
        return []

# calculates the initialization configurations needed for calculating a cartesian path to the next waypoint at degree2
def calculate_path_between_waypoints(py, left_right, file, degree1, degree2, panda_1, panda_2, position_1, position_2, euler_1, euler_2):
    # sample and set configurations for pose to correct miscalculations
    found_right = False
    try:
        # sample configurations and calculate a possible configuration through jacobian
        conf_1 = order_sampling(
            panda_1.solve_ik_via_sampling(position_1, euler_1, distance_threshold=0.1, max_configs=10),
            panda_1.get_joint_positions())
        jac_1 = get_jaco(panda_1, position_1, euler_1)

        # add current joint position and existing jacobian solution as possible initialization configuration
        before_1 = panda_1.get_joint_positions()
        if jac_1:
            conf_1 = np.row_stack((before_1, jac_1, conf_1))
        else:
            conf_1 = np.row_stack((before_1, conf_1))

        for c_1 in range(len(conf_1)):

            # set reset pose for panda1
            if not set_joint_position_without_moving_gripper(py, panda_1, conf_1[c_1]):
                # if configuration led to moving away from gripper position or collides, discard config option
                set_joint_position(py, panda_1, before_1)
                continue

            # sample configurations and calculate a possible configuration through jacobian
            try:
                conf_2 = order_sampling(
                    panda_2.solve_ik_via_sampling(position_2, euler_2, distance_threshold=0.1, max_configs=10),
                    panda_2.get_joint_positions())
            except ConfigurationError:
                # if no initialization configuration of robot 2 is found, change initialization configuration of robot 1
                break
            jac_2 = get_jaco(panda_2, position_2, euler_2)

            before_2 = panda_2.get_joint_positions()
            if jac_2:
                conf_2 = np.row_stack((before_2, jac_2, conf_2))
            else:
                conf_2 = np.row_stack((before_2, conf_2))

            for c_2 in range(len(conf_2)):
                # set reset pose for panda2
                if not set_joint_position_without_moving_gripper(py, panda_2, conf_2[c_2]):
                    # if configuration led to moving away from gripper position or collides, discard config option
                    set_joint_position(py, panda_2, before_2)
                    continue

                # calculate path depending on corrected pose for left panda
                path_1, points_1 = get_cartesian_path(degree1, degree2, euler_1, panda_1)
                if not points_1:
                    # if no path is found, try another initialization configuration
                    if c_1 == len(conf_1) - 1:
                        # if no fitting initialization found, stop program with failure
                        return False
                    continue
                # calculate and execute paths depending on corrected pose for right panda
                path_2, points_2 = get_cartesian_path(degree1, degree2, euler_2, panda_2)
                if points_2:

                    # save path initializations in document
                    conf_dict = {"Left": [], "Right": []}
                    if left_right:
                        conf_dict["Left"] = conf_1[c_1].tolist()
                        conf_dict["Right"] = conf_2[c_2].tolist()
                    else:
                        conf_dict["Left"] = conf_2[c_2].tolist()
                        conf_dict["Right"] = conf_1[c_1].tolist()
                    json.dump(conf_dict, file)
                    file.write("\n")
                    file.flush()

                    # set variable to true to break out of both loops
                    found_right = True

                    # execute paths simultanously
                    bool1 = not path_1.step()
                    bool2 = not path_2.step()

                    while bool1 or bool2:
                        if bool1:
                            bool1 = not path_1.step()
                        if bool2:
                            bool2 = not path_2.step()
                        py.step()

                    break
                else:
                    if (c_1 == len(conf_1) - 1) and (c_2 == len(conf_2) - 1):
                        # if no fitting initialization found, stop program with failure
                        return False
            # if path for right and left panda found, go to next waypoint
            if found_right:
                return True

    except ConfigurationError:
        # if no fitting initialization found, stop program with failure
        return False

    return False

# calculates additional waypoints for pyrep, returns ArmConfigurationPath and configurations in a list
def get_cartesian_path(degree1, degree2, euler, panda):
    # get additional waypoints for pyrep
    degrees = np.linspace(degree1, degree2, 4)
    radius = 0.2

    x = np.repeat([0.7], degrees.shape[0])
    y = np.zeros_like(degrees) - radius * np.sin(degrees * np.pi / 180.)
    z = 0.6 - radius * np.cos(degrees * np.pi / 180)

    _euler = np.repeat([euler], [degrees.shape[0]], axis=0)

    # create cartesian path
    circle = CartesianPath.create(automatic_orientation=False)
    circle.insert_control_points(np.column_stack((x, y, z, _euler)).tolist())

    for i in range(20):
        try:
            # calculate cartesian path
            return panda.get_path_from_cartesian_path(circle)
        except ConfigurationPathError:
            continue

    # TODO Improvement: delete path if it didn't work out
    return [None, []]


# orders all sampling configuration after their difference to the current one
def order_sampling(samples, ref):
    distances = np.linalg.norm(samples - ref, axis=1)
    sorted_indices = np.argsort(distances)
    return samples[sorted_indices]


# move both robots sequently to joint positions
def get_path_with_conf_sequently(py, panda1, state1, panda2, state2):
    try:
        path_1 = panda1.get_nonlinear_path_with_config(state1)
        while not path_1.step():
            py.step()
        path_2 = panda2.get_nonlinear_path_with_config(state2)
        while not path_2.step():
            py.step()
    except ConfigurationPathError:
        set_joint_position(py, panda1, state1)
        set_joint_position(py, panda2, state2)


# move robots as if they are picking up the cuboid and then position then into the initialization pose for the circle
def set_initialisation_for_calculation_circular_path(panda_right, panda_left):
    # start position
    set_joint_position_both(py, panda_right,
                            [0.6981316804885864, -0.7853981852531433, 0.0, -2.356194496154785, 0.0, 1.5707963705062866,
                             0.7853981852531433], panda_left,
                            [0.6981316804885864, -0.7853981852531433, 0.0, -2.356194496154785, 0.0, 1.5707963705062866,
                             0.7853981852531433])

    # simulate picking up cuboid
    get_path_sequently(py, panda_right, [0.55, 0., 0.1], [math.pi, 0, 0], panda_left,
                       [0.7, -0.1, 0.4], [-math.pi / 2, 0, -math.pi / 2])

    # move both robots into initialization pose for circular path
    get_path_with_conf_sequently(py, panda_right, [0.0650801807641983, -1.3919715881347656, 0.40745463967323303, -2.5227458477020264, -0.2463798224925995,
         2.5780768394470215, -1.7419333457946777], panda_left, [-0.78128886, -1.16594267, -0.22946176, -2.44404912, 1.10950899, 2.15002394, 1.20134878])


if __name__ == '__main__':
    py = PyRep()

    py.launch('../Pandas.ttt',
              headless=False)
    py.start()

    # initialization of variables
    panda_right = PandaRight()
    gripper_right = PandaRightGripper()
    panda_left = PandaLeft()
    gripper_left = PandaLeftGripper()
    cubid = Object.get_object("Cuboid")
    found = False

    # asking if old solution should be played?
    while True:
        answer = input("Do you want to reuse the initialization configurations calculated from last solution?[y,n]\n "
                       "If not new initialization configurations will be calculated before starting the scenario\n\n")
        if answer.lower() == "yes" or answer.lower() == "y":
            set_joint_position_both(py, panda_right,
                                    [0.6981316804885864, -0.7853981852531433, 0.0, -2.356194496154785, 0.0,
                                     1.5707963705062866,
                                     0.7853981852531433], panda_left,
                                    [0.6981316804885864, -0.7853981852531433, 0.0, -2.356194496154785, 0.0,
                                     1.5707963705062866,
                                     0.7853981852531433])
            found = True
            break
        elif answer.lower() == "no" or answer.lower() == "n":
            break
        else:
            print("Please give an answer in the form of \"yes\", \"y\", \"no\", \"n\"!")
            continue

    # find new solution for initialization configurations at waypoints
    if not found:
        init_confs_write = open("../files/Initialisation_Conf.data", "w")
        set_initialisation_for_calculation_circular_path(panda_right, panda_left)
        found = get_circle(py, panda_right, panda_left, gripper_left, init_confs_write, True)
        init_confs_write.close()
        # going to start position and recalculate path with initialization configurations - may take some trials
        get_path_sequently(py, panda_right, [0.7, 0.1, 0.2], [math.pi / 2, 0, 0], panda_left,
                           [0.7, -0.2, 0.4], [-math.pi / 2, 0, -math.pi / 2])

        for i in range(10):
            py.step()

    if not found:
        # erase content from file if no path is found
        print("no initialization configurations found. Try again by starting program again!")
        open("../files/Initialisation_Conf.data", "w").close()
    else:
        # open solution file for initialization configurations
        init_confs_read = open("../files/Initialisation_Conf.data", "r")

        # set some time to be able to move the robots into right position with cursor
        for i in range(300):
            py.step()

        # move toward the cuboid to pick it up
        get_path_sequently(py, panda_right, [0.55, 0., 0.02], [math.pi, 0, 0], panda_left,
                           [0.7, -0.1, 0.4], [-math.pi / 2, 0, -math.pi / 2])

        # pick cuboid up
        while not gripper_right.actuate(0, 0.7):
            py.step()

        gripper_right.grasp(cubid)
        py.step()

        # move cuboid to left robot and initialization position of circular path
        get_path_with_conf_sequently(py, panda_right,
                                     [0.0650801807641983, -1.3919715881347656, 0.40745463967323303, -2.5227458477020264,
                                      -0.2463798224925995,
                                      2.5780768394470215, -1.7419333457946777], panda_left,
                                     [-0.78128886, -1.16594267, -0.22946176, -2.44404912, 1.10950899, 2.15002394,
                                      1.20134878])

        while not gripper_left.actuate(0, 0.7):
            py.step()

        # measure time of recalculation of path with initialization configurations
        time_before = time.time()
        done = get_circle(py, panda_right, panda_left, gripper_left, init_confs_read, False)
        time_after = time.time()
        time_inbetween = time_after - time_before

        # write result in file and in terminal
        f = open("../files/Initialisation_Conf.data", "a")
        if done:
            print("moved along circular path in dual-mode with time in seconds: " + str(time_inbetween))
            f.write("\n\nAchieved in " + str(time_inbetween) + " seconds!")

        else:
            print("path could not successfully be recalculated... try to run the program again!")
            f.write("\n\nFailed to reproduce path! Try again!")

        f.flush()
        f.close()
    py.stop()
    py.shutdown()
