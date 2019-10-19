"""Used as an example of sugiyama experiment.
This example consists of 22 OVM cars on a ring creating shockwaves.
"""

from flow.controllers import IDMController,BCMController,OVMController, ContinuousRouter, LACController, FollowerStopper, PISaturation
from car_following_models import AVRider

from car_following_models import LinOpt_Controller_IDM
from car_following_models import FuzzyController_New, FuzzyController_Old, ModifiedLyapunovTypeControllerU1, ModifiedLyapunovTypeControllerU2, Augmented_OV_FTL, OV_FTL

from flow.core.experiment import Experiment
from flow.core.params import SumoParams, EnvParams,InitialConfig, NetParams, SumoCarFollowingParams
from flow.core.params import VehicleParams
from flow.envs.ring.accel import AccelEnv, ADDITIONAL_ENV_PARAMS   # from flow.envs.loop.loop_accel import AccelEnv, ADDITIONAL_ENV_PARAMS

from flow.scenarios.loop import ADDITIONAL_NET_PARAMS
from flow.networks.ring import RingNetwork
from flow.core.util import emission_to_csv

import numpy as np
import os


def sugiyama_example1(render=True, x=0, cont='OV_FTL'):
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
    
    sim_params = SumoParams(sim_step=0.1, render=False, emission_path='/home/lorr/flow/FLOR/IDM_AVRider_{}/{}IDM_{}{}'.format(cont,22-x,x,cont))
    
    if render is not None:
        sim_params.render = render

    #Switch the controller to be deployed
    if cont == "AUG":
        controller = Augmented_OV_FTL    
    if cont == "MLYAU1":
        controller = ModifiedLyapunovTypeControllerU1
    if cont == "MLYAU2":
        controller = ModifiedLyapunovTypeControllerU2
    if cont == "FUZO":
        controller = FuzzyController_Old
    if cont == "FUZN":
        controller = FuzzyController_New
    if cont == "FS":
        controller = FollowerStopper
    if cont == "LACC":
        controller = LACController    
    if cont == "PI":
        controller = PISaturation
    if cont == "BCM":
        controller = BCMController
    if cont == "LinOpt":
        controller = LinOpt_Controller_IDM

    vehicles = VehicleParams()
    vehicles.add(
        veh_id="IDM",
        acceleration_controller=(IDMController, {"noise":0.1}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0
        ),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=22-x)
    vehicles.add(
        veh_id="{}".format(cont),
        acceleration_controller=(AVRider, {"AVController":controller}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0
        ),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=x)


    # additional_net_params = ADDITIONAL_NET_PARAMS.copy()
    # net_params = NetParams(additional_params=additional_net_params)


    env_params = EnvParams(additional_params=ADDITIONAL_ENV_PARAMS)

    net_params = NetParams(
        additional_params={
            'length': 260,
            'lanes': 1,
            'speed_limit': 30,
            'resolution': 40
        }
    )

    initial_config = InitialConfig(perturbation=1, spacing='uniform')

    scenario = RingNetwork(
        name="sugiyama",
        vehicles=vehicles,
        net_params=net_params,
        initial_config=initial_config)

    env = AccelEnv(env_params, sim_params, scenario)

    return Experiment(env)

"""-------------------------------------------------------------"""

import random

if __name__ == "__main__":
    #==================Run One Controller =================
    # AV_case = ['AUG','MLYAU1','MLYAU2','FUZN','FUZO','LACC','PI','FS','BCM','LinOpt']
    # import the experiment variable
    # NumAV = 1
    # TypeAV = 'LACC'
    # exp = sugiyama_example1(x = NumAV, cont=TypeAV, render=False)
    # # run for a set number of rollouts / time steps
    # exp.run(1, 6000)

    #==================Run all controllers in a bacth=================
    #import the experiment variable
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='OVFTL', render=False)
    #     print(exp1.env.sim_params.emission_path)
    #     exp1.run(1, 6000, convert_to_csv=True)
    #     print('{} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    AV_case = ['AUG','MLYAU1','MLYAU2','FUZN','FUZO','LACC','PI','FS','BCM','LinOpt']
    # AV_case = ['FS']

    for av in AV_case:
        print('Start {}'.format(av)) 
        for x in range(23):
            exp1 = sugiyama_example1(x = x, cont=av, render=False)
            exp1.run(1, 6000, convert_to_csv=True)
            print('{} {} left'.format(av,23-x)) 
            print('done') 
            del exp1


    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='AUG', render=False)
    #     exp1.run(1, 6000, convert_to_csv=True)
    #     print('AUG {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='MLYAU1', render=False)
    #     exp1.run(1, 6000, convert_to_csv=True)
    #     print('MLYAU1 {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='MLYAU2', render=False)
    #     exp1.run(1, 6000, convert_to_csv=True)
    #     print('MLYAU2 {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='FUZO', render=False)
    #     exp1.run(1, 6000, convert_to_csv=True)
    #     print('FUZO {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='FUZN', render=False)
    #     exp1.run(1, 6000, convert_to_csv=True)
    #     print('FUZN {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='LACC', render=False)
    #     exp1.run(1, 500, convert_to_csv=True)
    #     print('LACC {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='PI', render=False)
    #     exp1.run(1, 500, convert_to_csv=True)
    #     print('PI {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='FS', render=False)
    #     exp1.run(1, 500, convert_to_csv=True)
    #     print('FS {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='BCM', render=False)
    #     exp1.run(1, 500, convert_to_csv=True)
    #     print('BCM {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    # for x in range(23):
    #     exp1 = sugiyama_example1(x = x,cont='LinOpt', render=False)
    #     exp1.run(1, 500, convert_to_csv=True)
    #     print('LinOpt {} left'.format(23-x)) 
    #     print('done') 
    #     del exp1
    