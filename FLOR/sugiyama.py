"""Used as an example of sugiyama experiment.

This example consists of 22 IDM cars on a ring creating shockwaves.
"""

from flow.controllers import IDMController, ContinuousRouter, LACController, FollowerStopper, PISaturation
from car_following_models import FuzzyController_New, FuzzyController_Old, ModifiedLyapunovTypeControllerU1, ModifiedLyapunovTypeControllerU2, Augmented_OV_FTL, OV_FTL
from car_following_models import AVRider
from flow.core.experiment import Experiment
from flow.core.params import SumoParams, EnvParams, \
    InitialConfig, NetParams, SumoCarFollowingParams
from flow.core.params import VehicleParams

from flow.envs.ring.accel import AccelEnv, ADDITIONAL_ENV_PARAMS   # from flow.envs.loop.loop_accel import AccelEnv, ADDITIONAL_ENV_PARAMS
from flow.scenarios.loop import ADDITIONAL_NET_PARAMS

from flow.networks.ring import RingNetwork

def sugiyama_example(render=None):
    """
    Perform a simulation of vehicles on a ring road.

    Parameters
    ----------
    render : bool, optional
        specifies whether to use the gui during execution

    Returns
    -------
    exp: flow.core.experiment.Experiment
        A non-rl experiment demonstrating the performance of human-driven
        vehicles on a ring road.
    """
    sim_params = SumoParams(sim_step=0.1, render=True)

    import ipdb; ipdb.set_trace()

    if render is not None:
        sim_params.render = render

    vehicles = VehicleParams()
    vehicles.add(
        veh_id="AV",
        acceleration_controller=(FollowerStopper, {}),    #acceleration_controller=(AVRider, {"AVController":FollowerStopper}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=1)
    vehicles.add(
        veh_id="idm",
        acceleration_controller=(IDMController, {"noise":0.1}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=21)

    env_params = EnvParams(additional_params=ADDITIONAL_ENV_PARAMS)

    # additional_net_params = ADDITIONAL_NET_PARAMS.copy()
    # net_params = NetParams(additional_params=additional_net_params)

    net_params = NetParams(
        additional_params={
            'length': 260,
            'lanes': 1,
            'speed_limit': 30,
            'resolution': 40
        }
    )

    #initial_config = InitialConfig(bunching=20)
    initial_config = InitialConfig(perturbation=1, spacing='uniform')

    scenario = RingNetwork(
        name="sugiyama",
        vehicles=vehicles,
        net_params=net_params,
        initial_config=initial_config)

    env = AccelEnv(env_params, sim_params, scenario)

    return Experiment(env)


if __name__ == "__main__":
    # import the experiment variable
    exp = sugiyama_example(render= True)

    # run for a set number of rollouts / time steps
    exp.run(1, 6000)
