"""
Environment used to train a stop-and-go dissipating controller.

This is the environment that was used in:

C. Wu, A. Kreidieh, K. Parvate, E. Vinitsky, A. Bayen, "Flow: Architecture and
Benchmarking for Reinforcement Learning in Traffic Control," CoRR, vol.
abs/1710.05465, 2017. [Online]. Available: https://arxiv.org/abs/1710.05465
"""

import numpy as np
from gym.spaces.box import Box
from flow.envs.multiagent.base import MultiEnv

ADDITIONAL_ENV_PARAMS = {
    # maximum acceleration of autonomous vehicles
    'max_accel': 1,
    # maximum deceleration of autonomous vehicles
    'max_decel': 1,
    # bounds on the ranges of ring road lengths the autonomous vehicle is
    # trained on
    'ring_length': [220, 270],
}


class MultiWaveAttenuationPOEnv(MultiEnv):
    """Multiagent shared model version of WaveAttenuationPOEnv.

    Intended to work with Lord Of The Rings Network.
    Note that this environment current
    only works when there is one autonomous vehicle
    on each ring.

    Required from env_params: See parent class

    States
        See parent class
    Actions
        See parent class

    Rewards
        See parent class

    Termination
        See parent class

    """

    @property
    def observation_space(self):
        """See class definition."""
        return Box(low=-float('inf'), high=float('inf'), shape=(3,), dtype=np.float32)

    @property
    def action_space(self):
        """See class definition."""
        add_params = self.net_params.additional_params
        if 'num_rings' in add_params:
            num_rings = add_params['num_rings']
        else:
            num_rings = 1

        return Box(
            low=-np.abs(self.env_params.additional_params['max_decel']),
            high=self.env_params.additional_params['max_accel'],
            shape=(1,),
            dtype=np.float32)

    def get_state(self):
        """See class definition."""
        obs = {}
        for rl_id in self.k.vehicle.get_rl_ids():
            lead_id = self.k.vehicle.get_leader(rl_id) or rl_id

            # normalizers
            max_speed = 15.
            if self.env_params.additional_params['ring_length'] is not None:
                max_length = self.env_params.additional_params['ring_length'][1]
            else:
                max_length = self.k.network.length()

            observation = np.array([
                self.k.vehicle.get_speed(rl_id) / max_speed,
                (self.k.vehicle.get_speed(lead_id) -
                 self.k.vehicle.get_speed(rl_id))
                / max_speed,
                self.k.vehicle.get_headway(rl_id) / max_length
            ])
            obs.update({rl_id: observation})

        return obs

    def _apply_rl_actions(self, rl_actions):
        """Split the accelerations by ring."""
        if rl_actions:
            rl_ids = list(rl_actions.keys())
            accel = list(rl_actions.values())  
            self.k.vehicle.apply_acceleration(rl_ids, accel)

    def compute_reward(self, rl_actions, **kwargs):
        """See class definition."""
        
        # in the warmup steps
        if rl_actions is None:
            return {}
    
        rew = {}
        for rl_id in rl_actions.keys():
            vel = np.array([
                self.k.vehicle.get_speed(veh_id)
                for veh_id in self.k.vehicle.get_ids()
            ])    ##array of speeds of all the vehicles on the ring.
            if any(vel < -100) or kwargs['fail']:
                return 0.

            # reward average velocity
            eta_2 = 4.
            reward = eta_2 * np.mean(vel) / 20

            # punish accelerations (should lead to reduced stop-and-go waves)
            eta = 4  # 0.25
            mean_actions = np.mean(np.abs(np.array(rl_actions[rl_id])))   
            accel_threshold = 0

            if mean_actions > accel_threshold:
                reward += eta * (accel_threshold - mean_actions)

            rew[rl_id] = reward
   
        return rew

    def additional_command(self):
        """Define which vehicles are observed for visualization purposes."""
        # specify observed vehicles
        for rl_id in self.k.vehicle.get_rl_ids():
            lead_id = self.k.vehicle.get_leader(rl_id) or rl_id
            self.k.vehicle.set_observed(lead_id)

    def gen_edges(self, i):
        """Return the edges corresponding to the rl id."""
        return ['top_{}'.format(i), 'left_{}'.format(i),
                'right_{}'.format(i), 'bottom_{}'.format(i)]
