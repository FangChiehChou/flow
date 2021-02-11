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


def sugiyama_even_interval(render=True, x=0, cont='OV_FTL'):
    """
    Perform a simulation of vehicles on a ring road.
    AVs are placed with 1 HV intervals.
    """
    
    if x == 2:
        hv_distribution = [10,10]
        av_distribution = [1,1]
    if x == 3:
        hv_distribution = [6,7,6]
        av_distribution = [1,1,1]
    if x == 4:
        hv_distribution = [4,5,4,5]
        av_distribution = [1,1,1,1]
    if x == 5:
        hv_distribution = [3,4,3,4,3]
        av_distribution = [1,1,1,1,1]
    if x == 6:
        hv_distribution = [3,2,3,3,2,3]
        av_distribution = [1,1,1,1,1,1]
    if x == 7:
        hv_distribution = [2,2,2,3,2,2,2]
        av_distribution = [1,1,1,1,1,1,1]
    if x == 8:
        hv_distribution = [2,1,2,2,2,1,2,2]
        av_distribution = [1,1,1,1,1,1,1,1]
    if x == 9:
        hv_distribution = [1,2,1,2,1,2,1,2,1]
        av_distribution = [1,1,1,1,1,1,1,1,1]
    if x == 10:
        hv_distribution = [2,1,1,1,1,2,1,1,1,1]
        av_distribution = [1,1,1,1,1,1,1,1,1,1]
    if x == 11:
        hv_distribution = [1,1,1,1,1,1,1,1,1,1,1]
        av_distribution = [1,1,1,1,1,1,1,1,1,1,1]
    # if x == 12:
    #     hv_distribution = [1,1,1,1,1,1,1,1,1,1]
    #     av_distribution = [2,1,1,1,1,2,1,1,1,1]
    # if x == 13:

    # if x == 14:

    # if x == 15:

    # if x == 16:

    # if x == 17:

    # if x == 18:

    # if x == 19:

    # if x == 20:           


    if x>22:
        return False

    sim_params = SumoParams(sim_step=0.1, render=False, emission_path='/media/lorr/TOSHIBA EXT/LORR/new_test_0209/Evenly_Distributed/IDM_AVRider_{}_EvenInterval/IDM{}_{}{}'.format(cont,22-x,cont,x))
    
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

    i = 0
    for hv_num in hv_distribution:
        i = i + 1
        vehicles.add(
            veh_id="IDM_{}".format(i),
            acceleration_controller=(IDMController, {"noise":0.1}),
            car_following_params=SumoCarFollowingParams(
                min_gap=0.1
            ),
            routing_controller=(ContinuousRouter, {}),
            num_vehicles=hv_num)
        vehicles.add(
            veh_id="{}_{}".format(cont,i),
            acceleration_controller=(AVRider, {"AVController":controller}),
            car_following_params=SumoCarFollowingParams(
                min_gap=0.1
            ),
            routing_controller=(ContinuousRouter, {}),
            num_vehicles=1)

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




def sugiyama_10HV_interval(render=True, cont='OV_FTL'):
    """
    Perform a simulation of vehicles on a ring road.
    2 AVs are placed with 10 HV intervals.
    """
    sim_params = SumoParams(sim_step=0.1, render=False, emission_path='/home/lorr/flow/FLOR/IDM_AVRider_{}_10Interval/IDM_{}'.format(cont,cont))
    
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
        veh_id="IDM1",
        acceleration_controller=(IDMController, {"noise":0.1}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0
        ),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=10)
    vehicles.add(
        veh_id="{}1".format(cont),
        acceleration_controller=(AVRider, {"AVController":controller}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0
        ),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=1)
    vehicles.add(
        veh_id="IDM2",
        acceleration_controller=(IDMController, {"noise":0.1}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0
        ),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=10)
    vehicles.add(
        veh_id="{}2".format(cont),
        acceleration_controller=(AVRider, {"AVController":controller}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0
        ),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=1)

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


def sugiyama_1HV_interval(render=True, x=0, cont='OV_FTL'):
    """
    Perform a simulation of vehicles on a ring road.
    AVs are placed with 1 HV intervals.
    """
    
    if x>11:
        return False

    sim_params = SumoParams(sim_step=0.1, render=False, emission_path='/home/lorr/flow/FLOR/IDM_AVRider_{}_2Interval/IDM{}_{}{}'.format(cont,22-x,cont,x))
    
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

    for i in range(x):
        vehicles.add(
            veh_id="IDM_{}".format(i),
            acceleration_controller=(IDMController, {"noise":0.1}),
            car_following_params=SumoCarFollowingParams(
                min_gap=0
            ),
            routing_controller=(ContinuousRouter, {}),
            num_vehicles=1)
        vehicles.add(
            veh_id="{}_{}".format(cont,i),
            acceleration_controller=(AVRider, {"AVController":controller}),
            car_following_params=SumoCarFollowingParams(
                min_gap=0
            ),
            routing_controller=(ContinuousRouter, {}),
            num_vehicles=1)

    vehicles.add(
        veh_id="IDM",
        acceleration_controller=(IDMController, {"noise":0.1}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0
        ),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=22-2*x)    

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
    
    sim_params = SumoParams(sim_step=0.1, render=False, emission_path='/home/lorr/flow/FLOR/Sim1025_IDM_AVRider_{}/{}IDM_{}{}'.format(cont,22-x,x,cont))
    
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
    # TypeAV = 'PI'
    # exp = sugiyama_example1(x = NumAV, cont=TypeAV, render=False)
    # exp = sugiyama_10HV_interval(render=True, cont=TypeAV)
    # exp = sugiyama_1HV_interval(render=False, x=NumAV, cont=TypeAV)
    # run for a set number of rollouts / time steps
    # exp.run(1, 6000)

    #==================Run all controllers (10 HVs interval) in a bacth=================
    ## 
    # AV_case = ['AUG','MLYAU1','MLYAU2','FUZN','FUZO','LACC','PI','FS','BCM','LinOpt']
    # AV_case = ['LinOpt']
    
    # for av in AV_case:
    #     print('Start {}'.format(av)) 
    #     exp1 = sugiyama_10HV_interval(render=False, cont=av)
    #     exp1.run(1, 6000, convert_to_csv=True)
    #     del exp1
            
    #==================Run all controllers (1 HVs interval) in a bacth=================
    # for av in AV_case:
    #     print('Start {}'.format(av)) 
    #     for x in range(2,12):
    #         AV_num = x
    #         exp1 = sugiyama_1HV_interval(x = AV_num, cont=av, render=False)
    #         exp1.run(1, 6000, convert_to_csv=True)
    #         print('{} {} placed on the ring, {} iterations left'.format(AV_num,av,11-AV_num)) 
    #         print('done') 
    #         del exp1 

    #==================Run all controllers in a bacth=================
    # AV_case = ['AUG','MLYAU1','MLYAU2','FUZN','FUZO','LACC','PI','FS','BCM','LinOpt']
    # AV_case = ['LACC']
    # for av in AV_case:
    #     print('Start {}'.format(av)) 
    #     for x in range(23):
    #         exp1 = sugiyama_example1(x = x, cont=av, render=False)
    #         exp1.run(1, 6000, convert_to_csv=True)
    #         print('{} {} left'.format(av,23-x)) 
    #         print('done') 
    #         del exp1


##    ==================Run all controllers evenly distributed in a bacth=================
    # AV_case = ['AUG','MLYAU1','MLYAU2','FUZN','FUZO','LACC','PI','FS','BCM','LinOpt']
    AV_case = ['FS']
    # for av in AV_case:
    #     print('Start {}'.format(av)) 
    #     AV_num = 1
    #     for x in range(2):
    #         AV_num = x
    #         for sim_runs in range(10):
    #             exp1 = sugiyama_even_interval(x = x, cont=av, render=True)
    #             exp1.run(1, 2000, convert_to_csv=False)
    #             print('Run {} : {} {} placed on the ring, {} iterations left'.format(sim_runs,AV_num,av,11-AV_num)) 
    #             print('done') 
    #             del exp1


    av = AV_case[0]
    exp1 = sugiyama_even_interval(x = 2, cont=av, render=True)
    exp1.run(1, 200000, convert_to_csv=False)
    print('Run {} : {} {} placed on the ring, {} iterations left'.format(sim_runs,AV_num,av,11-AV_num)) 
    print('done') 
    del exp1