from flow.envs.base_env import Env
from flow.core import rewards
from flow.core import multi_agent_rewards

from gym.spaces.box import Box
from gym.spaces.tuple_space import Tuple

import numpy as np


class AccelEnv(Env):
    """Environment used to train autonomous vehicles to improve traffic flows
    when acceleration actions are permitted by the rl agent.

    States
    ------
    The state consists of the velocities and absolute position of all vehicles
    in the network. This assumes a constant number of vehicles.

    Actions
    -------
    Actions are a list of acceleration for each rl vehicles, bounded by the
    maximum accelerations and decelerations specified in EnvParams.

    Rewards
    -------
    The reward function is the two-norm of the distance of the speed of the
    vehicles in the network from a desired speed.

    Termination
    -----------
    A rollout is terminated if the time horizon is reached or if two vehicles
    collide into one another.
    """

    @property
    def action_space(self):
        return Box(low=-np.abs(self.env_params.max_decel),
                   high=self.env_params.max_accel,
                   shape=(self.vehicles.num_rl_vehicles,),
                   dtype=np.float32)

    @property
    def observation_space(self):
        self.obs_var_labels = ["Velocity", "Absolute_pos"]
        speed = Box(low=0, high=np.inf, shape=(self.vehicles.num_vehicles,),
                    dtype=np.float32)
        pos = Box(low=0., high=np.inf, shape=(self.vehicles.num_vehicles,),
                  dtype=np.float32)
        return Tuple((speed, pos))

    def _apply_rl_actions(self, rl_actions):
        sorted_rl_ids = [veh_id for veh_id in self.sorted_ids
                         if veh_id in self.vehicles.get_rl_ids()]
        self.apply_acceleration(sorted_rl_ids, rl_actions)

    def compute_reward(self, state, rl_actions, **kwargs):
        return rewards.desired_velocity(self, fail=kwargs["fail"])

    def get_state(self, **kwargs):
        scaled_pos = [self.vehicles.get_absolute_position(veh_id) /
                      self.scenario.length for veh_id in self.sorted_ids]
        scaled_vel = [self.vehicles.get_speed(veh_id) /
                      self.env_params.get_additional_param("target_velocity")
                      for veh_id in self.sorted_ids]
        state = [[vel, pos] for vel, pos in zip(scaled_vel, scaled_pos)]

        # specify observed vehicles
        if self.vehicles.num_rl_vehicles > 0:
            for veh_id in self.vehicles.get_human_ids():
                self.vehicles.set_observed(veh_id)

        return np.array(state)


class AccelMAEnv(AccelEnv):
    """Multiagent version of AccelEnv

    States
    ------

    Actions
    -------

    Rewards
    -------

    Termination
    -----------
    """

    @property
    def action_space(self):
        """
        See parent class

        Actions are a set of accelerations from max-deacc to max-acc for each
        rl vehicle.
        """
        action_space = []
        for _ in self.vehicles.get_rl_ids():
            action_space.append(Box(low=self.env_params.max_deacc,
                                    high=self.env_params.max_acc,
                                    shape=(1,), dtype=np.float32))
        return action_space

    @property
    def observation_space(self):
        """
        See parent class
        """
        num_vehicles = self.vehicles.num_vehicles
        observation_space = []
        speed = Box(low=0, high=np.inf, shape=(num_vehicles,),
                    dtype=np.float32)
        absolute_pos = Box(low=0., high=np.inf, shape=(num_vehicles,),
                           dtype=np.float32)
        obs_tuple = Tuple((speed, absolute_pos))
        for _ in self.vehicles.get_rl_ids():
            observation_space.append(obs_tuple)
        return observation_space

    def compute_reward(self, state, rl_actions, **kwargs):
        """
        See parent class
        """
        return multi_agent_rewards.desired_velocity(
            state, rl_actions,
            fail=kwargs["fail"],
            target_velocity=self.env_params.additional_params["target_velocity"]
        )

    def get_state(self, **kwargs):
        """
        See parent class
        The state is an array the velocities and absolute positions for
        each vehicle.
        """
        obs_arr = []
        for rl_id in self.rl_ids:
            # Re-sort based on the rl agent being in front
            # Probably should try and do this less often
            sorted_indx = np.argsort(
                [(self.vehicles.get_absolute_position(veh_id) -
                  self.vehicles.get_absolute_position(rl_id))
                 % self.scenario.length for veh_id in self.ids])
            sorted_ids = np.array(self.ids)[sorted_indx]

            speed = [self.vehicles.get_speed(veh_id) for veh_id in sorted_ids]
            abs_pos = [(self.vehicles.get_absolute_position(veh_id) -
                        self.vehicles.get_absolute_position(rl_id))
                       % self.scenario.length for veh_id in sorted_ids]

            tup = (speed, abs_pos)
            obs_arr.append(tup)

        # specify observed vehicles
        if self.vehicles.num_rl_vehicles > 0:
            for veh_id in self.vehicles.get_human_ids():
                self.vehicles.set_observed(veh_id)

        return obs_arr
