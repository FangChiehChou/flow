"""Visualizer for rllib experiments.

Attributes
----------
EXAMPLE_USAGE : str
    Example call to the function, which is
    ::

        python ./visualizer_rllib.py /tmp/ray/result_dir 1

parser : ArgumentParser
    Command-line argument parser
"""

import argparse
from datetime import datetime
import gym
import numpy as np
import os
import sys
import time

import ray
try:
    from ray.rllib.agents.agent import get_agent_class
except ImportError:
    from ray.rllib.agents.registry import get_agent_class
from ray.tune.registry import register_env

from flow.core.util import emission_to_csv
from flow.utils.registry import make_create_env
from flow.utils.rllib import get_flow_params
from flow.utils.rllib import get_rllib_config
from flow.utils.rllib import get_rllib_pkl

from flow.core.params import VehicleParams
from flow.controllers import ContinuousRouter
from flow.controllers import IDMController
from flow.controllers import RLController
from flow.core.params import SumoCarFollowingParams
from flow.core.params import EnvParams
EXAMPLE_USAGE = """
example usage:
    python ./visualizer_rllib.py /ray_results/experiment_dir/result_dir 1

Here the arguments are:
1 - the path to the simulation results
2 - the number of the checkpoint
"""


def vehicles_1HV_interval(NumAV=1):
    vehicles = VehicleParams()
    for i in range(NumAV):
        vehicles.add(
            veh_id="human_{}".format(i),
            acceleration_controller=(IDMController, {'noise': 0.1}),
            car_following_params=SumoCarFollowingParams(
                min_gap=0
            ),
            routing_controller=(ContinuousRouter, {}),
            num_vehicles=1)
        vehicles.add(
            veh_id="rl_{}".format(i),
            acceleration_controller=(RLController,{}),
            routing_controller=(ContinuousRouter, {}),
            num_vehicles=1)

    vehicles.add(
        veh_id="human",
        acceleration_controller=(IDMController, {"noise":0.1}),
        car_following_params=SumoCarFollowingParams(
            min_gap=0
        ),
        routing_controller=(ContinuousRouter, {}),
        num_vehicles=22-2*NumAV)  

    return vehicles


def vehicles_even_interval(NumAV=1):
    if NumAV == 2:
        hv_distribution = [10,10]
        av_distribution = [1,1]
    if NumAV == 3:
        hv_distribution = [6,7,6]
        av_distribution = [1,1,1]
    if NumAV == 4:
        hv_distribution = [4,5,4,5]
        av_distribution = [1,1,1,1]
    if NumAV == 5:
        hv_distribution = [3,4,3,4,3]
        av_distribution = [1,1,1,1,1]
    if NumAV == 6:
        hv_distribution = [3,2,3,3,2,3]
        av_distribution = [1,1,1,1,1,1]
    if NumAV == 7:
        hv_distribution = [2,2,2,3,2,2,2]
        av_distribution = [1,1,1,1,1,1,1]
    if NumAV == 8:
        hv_distribution = [2,1,2,2,2,1,2,2]
        av_distribution = [1,1,1,1,1,1,1,1]
    if NumAV == 9:
        hv_distribution = [1,2,1,2,1,2,1,2,1]
        av_distribution = [1,1,1,1,1,1,1,1,1]
    if NumAV == 10:
        hv_distribution = [2,1,1,1,1,2,1,1,1,1]
        av_distribution = [1,1,1,1,1,1,1,1,1,1]
    if NumAV == 11:
        hv_distribution = [1,1,1,1,1,1,1,1,1,1,1]
        av_distribution = [1,1,1,1,1,1,1,1,1,1,1]

    if NumAV>22:
        return False

    vehicles = VehicleParams()
    i=0
    for hv_num in hv_distribution:
        i = i + 1
        vehicles.add(
            veh_id="human_{}".format(i),
            acceleration_controller=(IDMController, {"noise":0.1}),
            car_following_params=SumoCarFollowingParams(
                min_gap=0
            ),
            routing_controller=(ContinuousRouter, {}),
            num_vehicles=hv_num)
        vehicles.add(
            veh_id="rl_{}".format(i),
            acceleration_controller=(RLController,{}),
            car_following_params=SumoCarFollowingParams(
                min_gap=0
            ),
            routing_controller=(ContinuousRouter, {}),
            num_vehicles=1)  

    return vehicles  


def visualizer_rllib(args,sim_steps=6000,NumAV=1,emission_path=None,AV_distribution='Platoon'):
    """Visualizer for RLlib experiments.

    This function takes args (see function create_parser below for
    more detailed information on what information can be fed to this
    visualizer), and renders the experiment associated with it.
    """
    result_dir = args.result_dir if args.result_dir[-1] != '/' \
        else args.result_dir[:-1]

    config = get_rllib_config(result_dir)

    # check if we have a multiagent environment but in a
    # backwards compatible way
    if config.get('multiagent', {}).get('policies', None):
        multiagent = True
        pkl = get_rllib_pkl(result_dir)
        config['multiagent'] = pkl['multiagent']
    else:
        multiagent = False

    # Run on only one cpu for rendering purposes
    config['num_workers'] = 0

    flow_params = get_flow_params(config)


    # Create the vehicle instance I need and replace the one in the training environment
    if AV_distribution == 'Platoon':   #clustered AV distribution
        vehicles = VehicleParams()
        vehicles.add(
            veh_id='human',
            acceleration_controller=(IDMController, {
                'noise': 0.1}),
            car_following_params=SumoCarFollowingParams(
                    min_gap=0),
            routing_controller=(ContinuousRouter, {}),
            num_vehicles=22-NumAV)
        vehicles.add(
            veh_id='rl',
            acceleration_controller=(RLController, {}),
             routing_controller=(ContinuousRouter, {}),
            num_vehicles=NumAV)
    
    if AV_distribution == '1HV':   #1HV interval distribution
        vehicles = vehicles_1HV_interval(NumAV)

    if AV_distribution == 'Even':    #evenly distribution
        vehicles = vehicles_even_interval(NumAV)

    flow_params['veh'] = vehicles
    
    sim_env=EnvParams(
        horizon = 7500,
        warmup_steps=3000,   #300 seconds simulation before the learning starts
        clip_actions=False,
        additional_params={
            'max_accel': 1,
            'max_decel': 1,
            "ring_length": [260, 260],
            'target_velocity': 4
        },
    )

    flow_params['env'] = sim_env


    # hack for old pkl files
    # TODO(ev) remove eventually
    sim_params = flow_params['sim']
    setattr(sim_params, 'num_clients', 1)

    # Determine agent and checkpoint
    config_run = config['env_config']['run'] if 'run' in config['env_config'] \
        else None

    if args.run and config_run:
        if args.run != config_run:
            print('visualizer_rllib.py: error: run argument '
                  + '\'{}\' passed in '.format(args.run)
                  + 'differs from the one stored in params.json '
                  + '\'{}\''.format(config_run))
            sys.exit(1)
    if args.run:
        agent_cls = get_agent_class(args.run)
    elif config_run:
        agent_cls = get_agent_class(config_run)
    else:
        print('visualizer_rllib.py: error: could not find flow parameter '
              '\'run\' in params.json, '
              'add argument --run to provide the algorithm or model used '
              'to train the results\n e.g. '
              'python ./visualizer_rllib.py /tmp/ray/result_dir 1 --run PPO')
        sys.exit(1)

    sim_params.restart_instance = True
    
    #set the path to save the emission_simulation results
    # dir_path = os.path.dirname(os.path.realpath(__file__))
    # emission_path = '{0}/test_time_rollout/IDM{1}_RL{2}/'.format(dir_path,22-NumAV,NumAV)
    sim_params.emission_path = emission_path if args.gen_emission else None
    
    # pick your rendering mode
    if args.render_mode == 'sumo_web3d':
        sim_params.num_clients = 2
        sim_params.render = False
    elif args.render_mode == 'drgb':
        sim_params.render = 'drgb'
        sim_params.pxpm = 4
    elif args.render_mode == 'sumo_gui':
        sim_params.render = True
        print('NOTE: With render mode {}, an extra instance of the SUMO GUI '
              'will display before the GUI for visualizing the result. Click '
              'the green Play arrow to continue.'.format(args.render_mode))
    elif args.render_mode == 'no_render':
        sim_params.render = False
    if args.save_render:
        sim_params.render = 'drgb'
        sim_params.pxpm = 4
        sim_params.save_render = True

    # Create and register a gym+rllib env
    create_env, env_name = make_create_env(params=flow_params, version=0)
    register_env(env_name, create_env)
    

    # check if the environment is a single or multiagent environment, and
    # get the right address accordingly
    # single_agent_envs = [env for env in dir(flow.envs)
    #                      if not env.startswith('__')]

    # if flow_params['env_name'] in single_agent_envs:
    #     env_loc = 'flow.envs'
    # else:
    #     env_loc = 'flow.envs.multiagent'

    # Start the environment with the gui turned on and a path for the
    # emission file
    env_params = flow_params['env']
    env_params.horizon = sim_steps
    env_params.restart_instance = False
    if args.evaluate:
        env_params.evaluate = True

    # lower the horizon if testing
    if args.horizon:
        config['horizon'] = args.horizon
        env_params.horizon = args.horizon
    # create the agent that will be used to compute the actions
    agent = agent_cls(env=env_name, config=config)
    checkpoint = result_dir + '/checkpoint_' + args.checkpoint_num
    checkpoint = checkpoint + '/checkpoint-' + args.checkpoint_num
    agent.restore(checkpoint)
    
    if hasattr(agent, "local_evaluator") and \
            os.environ.get("TEST_FLAG") != 'True':
        env = agent.local_evaluator.env
    else:
        env = gym.make(env_name)
    
    if multiagent:
        rets = {}
        # map the agent id to its policy
        policy_map_fn = config['multiagent']['policy_mapping_fn'].func
        for key in config['multiagent']['policies'].keys():
            rets[key] = []
    else:
        rets = []

    if config['model']['use_lstm']:
        use_lstm = True
        if multiagent:
            state_init = {}
            # map the agent id to its policy
            policy_map_fn = config['multiagent']['policy_mapping_fn'].func
            size = config['model']['lstm_cell_size']
            for key in config['multiagent']['policies'].keys():
                state_init[key] = [np.zeros(size, np.float32),
                                   np.zeros(size, np.float32)]
        else:
            state_init = [
                np.zeros(config['model']['lstm_cell_size'], np.float32),
                np.zeros(config['model']['lstm_cell_size'], np.float32)
            ]
    else:
        use_lstm = False

    env.restart_simulation(sim_params=sim_params, render=sim_params.render)

    # Simulate and collect metrics
    final_outflows = []
    final_inflows = []
    mean_speed = []
    std_speed = []
    for i in range(args.num_rollouts):
        vel = []        
        state = env.reset()
        
        if multiagent:
            ret = {key: [0] for key in rets.keys()}
        else:
            ret = 0
        for j in range(env_params.horizon):
            vehicles = env.unwrapped.k.vehicle
            vel.append(np.mean(vehicles.get_speed(vehicles.get_ids())))
            if multiagent:
                action = {}
                for agent_id in state.keys():
                    if use_lstm:
                        action[agent_id], state_init[agent_id], logits = \
                            agent.compute_action(
                            state[agent_id], state=state_init[agent_id],
                            policy_id=policy_map_fn(agent_id))
                    else:
                        action[agent_id] = agent.compute_action(
                            state[agent_id], policy_id=policy_map_fn(agent_id))
            else:
                action = agent.compute_action(state)
                  
            state, reward, done, _ = env.step(action)
            if multiagent:
                for actor, rew in reward.items():
                    ret[policy_map_fn(actor)][0] += rew
            else:
                ret += reward
            if multiagent and done['__all__']:
                break
            if not multiagent and done:
                break

        if multiagent:
            for key in rets.keys():
                rets[key].append(ret[key])
        else:
            rets.append(ret)
        outflow = vehicles.get_outflow_rate(500)
        final_outflows.append(outflow)
        inflow = vehicles.get_inflow_rate(500)
        final_inflows.append(inflow)
        if np.all(np.array(final_inflows) > 1e-5):
            throughput_efficiency = [x / y for x, y in
                                     zip(final_outflows, final_inflows)]
        else:
            throughput_efficiency = [0] * len(final_inflows)
        mean_speed.append(np.mean(vel))
        std_speed.append(np.std(vel))
        if multiagent:
            for agent_id, rew in rets.items():
                print('Round {}, Return: {} for agent {}'.format(
                    i, ret, agent_id))
        else:
            print('Round {}, Return: {}'.format(i, ret))

    print('==== Summary of results ====')
    print("Return:")
    print(mean_speed)
    if multiagent:
        for agent_id, rew in rets.items():
            print('For agent', agent_id)
            print(rew)
            print('Average, std return: {}, {} for agent {}'.format(
                np.mean(rew), np.std(rew), agent_id))
    else:
        print(rets)
        print('Average, std: {}, {}'.format(
            np.mean(rets), np.std(rets)))

    print("\nSpeed, mean (m/s):")
    print(mean_speed)
    print('Average, std: {}, {}'.format(np.mean(mean_speed), np.std(
        mean_speed)))
    print("\nSpeed, std (m/s):")
    print(std_speed)
    print('Average, std: {}, {}'.format(np.mean(std_speed), np.std(
        std_speed)))

    # Compute arrival rate of vehicles in the last 500 sec of the run
    print("\nOutflows (veh/hr):")
    print(final_outflows)
    print('Average, std: {}, {}'.format(np.mean(final_outflows),
                                        np.std(final_outflows)))
    # Compute departure rate of vehicles in the last 500 sec of the run
    print("Inflows (veh/hr):")
    print(final_inflows)
    print('Average, std: {}, {}'.format(np.mean(final_inflows),
                                        np.std(final_inflows)))
    # Compute throughput efficiency in the last 500 sec of the
    print("Throughput efficiency (veh/hr):")
    print(throughput_efficiency)
    print('Average, std: {}, {}'.format(np.mean(throughput_efficiency),
                                        np.std(throughput_efficiency)))

    # terminate the environment
    env.unwrapped.terminate()

    # if prompted, convert the emission file into a csv file
    if args.gen_emission:
        time.sleep(0.1)
        dir_path = os.path.dirname(os.path.realpath(__file__))
        emission_filename = '{0}-emission.xml'.format(env.network.name)

        # emission_path = '{0}/{1}'.format(sim_params.emission_path, emission_filename) 
        emission_path = os.path.join(sim_params.emission_path, emission_filename)

        # convert the emission file into a csv file
        emission_to_csv(emission_path)

        # print the location of the emission csv file
        emission_path_csv = emission_path[:-4] + ".csv"
        print("\nGenerated emission file at " + emission_path_csv)
        
        # delete the .xml version of the emission file
        os.remove(emission_path)

    # if we wanted to save the render, here we create the movie
    if args.save_render:
        dirs = os.listdir(os.path.expanduser('~')+'/flow_rendering')
        # Ignore hidden files
        dirs = [d for d in dirs if d[0] != '.']
        dirs.sort(key=lambda date: datetime.strptime(date, "%Y-%m-%d-%H%M%S"))
        recent_dir = dirs[-1]
        # create the movie
        movie_dir = os.path.expanduser('~') + '/flow_rendering/' + recent_dir
        save_dir = os.path.expanduser('~') + '/flow_movies'
        if not os.path.exists(save_dir):
            os.mkdir(save_dir)
        os_cmd = "cd " + movie_dir + " && ffmpeg -i frame_%06d.png"
        os_cmd += " -pix_fmt yuv420p " + dirs[-1] + ".mp4"
        os_cmd += "&& cp " + dirs[-1] + ".mp4 " + save_dir + "/"
        os.system(os_cmd)


def create_parser():
    """Create the parser to capture CLI arguments."""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='[Flow] Evaluates a reinforcement learning agent '
                    'given a checkpoint.',
        epilog=EXAMPLE_USAGE)

    # required input parameters
    parser.add_argument(
        'result_dir', type=str, help='Directory containing results')
    parser.add_argument('checkpoint_num', type=str, help='Checkpoint number.')

    # optional input parameters
    parser.add_argument(
        '--run',
        type=str,
        help='The algorithm or model to train. This may refer to '
             'the name of a built-on algorithm (e.g. RLLib\'s DQN '
             'or PPO), or a user-defined trainable function or '
             'class registered in the tune registry. '
             'Required for results trained with flow-0.2.0 and before.')
    parser.add_argument(
        '--num_rollouts',
        type=int,
        default=1,
        help='The number of rollouts to visualize.')
    parser.add_argument(
        '--gen_emission',
        action='store_true',
        help='Specifies whether to generate an emission file from the '
             'simulation')
    parser.add_argument(
        '--evaluate',
        action='store_true',
        help='Specifies whether to use the \'evaluate\' reward '
             'for the environment.')
    parser.add_argument(
        '--render_mode',
        type=str,
        default='sumo_gui',
        help='Pick the render mode. Options include sumo_web3d, '
             'rgbd and sumo_gui')
    parser.add_argument(
        '--save_render',
        action='store_true',
        help='Saves a rendered video to a file. NOTE: Overrides render_mode '
             'with pyglet rendering.')
    parser.add_argument(
        '--horizon',
        type=int,
        help='Specifies the horizon.')
    return parser


if __name__ == '__main__':
    #initialize simulation set up
    parser = create_parser()
    args = parser.parse_args()
    ray.init(num_cpus=1)
    dir_policy = '/home/lorr/ray_results/lord_of_numrings1/PPO_MultiWaveAttenuationPOEnv-v0_0_lr=1e-05_2019-12-20_10-10-36230jsi8f'
    num_check_point = 200

    # args.render_mode= 'no_render'
    # av_num = 6
    # args.gen_emission=False
    # dir_sim_out = '/media/lorr/TOSHIBA EXT/lorr_sim_out/IDM{0}_RL{1}/'.format(22-av_num,av_num)
    # visualizer_rllib(args,sim_steps=200,NumAV=av_num,emission_path=dir_sim_out)

    #===============================================
    # No render but Yes emission csv 
    args.render_mode='no_render'   #render /no_render   
    args.gen_emission=True

    # Platooned RL vehicles
    # for av_num in range(1,6):
    #     #assign different number of AVs on the ring
    #     dir_sim_out = '/media/lorr/TOSHIBA EXT/lorr_sim_out/Platoon/IDM{0}_RL{1}/'.format(22-av_num,av_num)
    #     for _ in range(10):
    #         # ten runs for each scenario
    #         visualizer_rllib(args,sim_steps=20000,NumAV=av_num,emission_path=dir_sim_out,AV_distribution='Platoon')

    #Evenly distributed RL vehicles
    for av_num in range(5,8):
        #assign different number of AVs on the ring
        dir_sim_out = '/media/lorr/TOSHIBA EXT/lorr_sim_out/Even/IDM{0}_RL{1}/'.format(22-av_num,av_num)
        for _ in range(10):
            # ten runs for each scenario
            visualizer_rllib(args,sim_steps=20000,NumAV=av_num,emission_path=dir_sim_out,AV_distribution='Even')

    #1HV interval distributed RL vehicles
    # for av_num in range(2,12):
    #     #assign different number of AVs on the ring
    #     dir_sim_out = '/media/lorr/TOSHIBA EXT/lorr_sim_out/1HV/IDM{0}_RL{1}/'.format(22-av_num,av_num)
    #     for _ in range(10):
    #         # ten runs for each scenario
    #         visualizer_rllib(args,sim_steps=20000,NumAV=av_num,emission_path=dir_sim_out,AV_distribution='1HV')