from pyrep.robots.arms.panda import Panda
from pyrep.const import PYREP_SCRIPT_TYPE
from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.errors import ConfigurationPathError
from pyrep.backend import utils
from pyrep.robots.configuration_paths.arm_configuration_path import (
    ArmConfigurationPath)
from typing import List, Union
import numpy as np
from pyrep.robots.arms.arm import Arm


class Franka(Arm):

    def __init__(
            self,
            count: int = 0,
            max_velocity: float = 1.0,
            max_acceleration: float = 4.0
    ):
        super().__init__(
            count,
            'Panda',
            7,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
        )
    # method created by copy and paste from pyrep code
    def get_nonlinear_path_with_config(self, config: Union[List[float], np.ndarray],
                                       ignore_collisions=False,
                                       trials_per_goal=1,
                                       algorithm=Algos.SBL,
                                       ) -> ArmConfigurationPath:
        handles = [j.get_handle() for j in self.joints]

        _, ret_floats, _, _ = utils.script_call(
            'getNonlinearPath@PyRep', PYREP_SCRIPT_TYPE,
            ints=[self._collision_collection, int(ignore_collisions),
                  trials_per_goal] + handles,
            floats=config,
            strings=[algorithm.value])

        if len(ret_floats) == 0:
            raise ConfigurationPathError('Could not create path.')
        return ArmConfigurationPath(self, ret_floats)
