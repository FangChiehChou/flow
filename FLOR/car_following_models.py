"""
Contains several custom car-following control models.

These controllers can be used to modify the acceleration behavior of vehicles
in Flow to match various prominent car-following models that can be calibrated.

Each controller includes the function ``get_accel(self, env) -> acc`` which,
using the current state of the world and existing parameters, uses the control
model to return a vehicle acceleration.
"""
import math
import numpy as np

from flow.controllers.base_controller import BaseController
from flow.controllers import IDMController


class AVRider(BaseController):
    """AVRider controller.

    This class switch the Human drive vehicle model into AV model at certain time stamp.

    Attributes
    ----------

    """
    def __init__(self,
                 veh_id,
                 AVController=IDMController,
                 v0=30,
                 T=1,
                 a=1,
                 b=1.5,
                 delta=4,
                 s0=2,
                 time_delay=0.0,
                 dt=0.1,
                 noise=0,
                 hv_noise = 0.1,
                 fail_safe=None,
                 car_following_params=None):
        """Instantiate an AVRider controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)
        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0
        self.dt = dt
        self.AV = AVController(veh_id = veh_id, car_following_params= car_following_params)
        self.HV = IDMController(veh_id = veh_id, car_following_params= car_following_params)
        self.hv_noise = hv_noise
        self.steps = 0

    def get_accel(self, env):
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        this_id = self.veh_id
        HVAccelCmd = self.HV.get_accel(env)
        if self.hv_noise > 0:
            HVAccelCmd += np.random.normal(0, self.hv_noise)

        AccelCmd = HVAccelCmd    #Acceleratoin controller of the IDM controller
        AVAccelCmd = self.AV.get_accel(env)   #Acceleratoin controller of the specified controller
        
        self.steps = self.steps + 1
        if (self.steps*env.sim_step >= 350):
            AccelCmd = AVAccelCmd

        return max(min(AccelCmd, self.max_accel), -self.max_deaccel)

class CFMController(BaseController):
    """CFM controller.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : SumoCarFollowingParams
        see parent class
    k_d : float
        headway gain (default: 1)
    k_v : float
        gain on difference between lead velocity and current (default: 1)
    k_c : float
        gain on difference from desired velocity to current (default: 1)
    d_des : float
        desired headway (default: 1)
    v_des : float
        desired velocity (default: 8)
    time_delay : float, optional
        time delay (default: 0.0)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 k_d=1,
                 k_v=1,
                 k_c=1,
                 d_des=1,
                 v_des=8,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None):
        """Instantiate a CFM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)

        self.veh_id = veh_id
        self.k_d = k_d
        self.k_v = k_v
        self.k_c = k_c
        self.d_des = d_des
        self.v_des = v_des

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            return self.max_accel

        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)

        d_l = env.k.vehicle.get_headway(self.veh_id)

        return self.k_d*(d_l - self.d_des) + self.k_v*(lead_vel - this_vel) + \
            self.k_c*(self.v_des - this_vel)


class BCMController(BaseController):
    """Bilateral car-following model controller.

    This model looks ahead and behind when computing its acceleration.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    k_d : float
        gain on distances to lead/following cars (default: 1)
    k_v : float
        gain on vehicle velocity differences (default: 1)
    k_c : float
        gain on difference from desired velocity to current (default: 1)
    d_des : float
        desired headway (default: 1)
    v_des : float
        desired velocity (default: 8)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 k_d=1,
                 k_v=1,
                 k_c=1,
                 d_des=1,
                 v_des=8,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None):
        """Instantiate a Bilateral car-following model controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)

        self.veh_id = veh_id
        self.k_d = k_d
        self.k_v = k_v
        self.k_c = k_c
        self.d_des = d_des
        self.v_des = v_des

    def get_accel(self, env):
        """See parent class.

        From the paper:
        There would also be additional control rules that take
        into account minimum safe separation, relative speeds,
        speed limits, weather and lighting conditions, traffic density
        and traffic advisories
        """
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            return self.max_accel

        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)

        trail_id = env.k.vehicle.get_follower(self.veh_id)
        trail_vel = env.k.vehicle.get_speed(trail_id)

        headway = env.k.vehicle.get_headway(self.veh_id)
        footway = env.k.vehicle.get_headway(trail_id)

        return self.k_d * (headway - footway) + \
            self.k_v * ((lead_vel - this_vel) - (this_vel - trail_vel)) + \
            self.k_c * (self.v_des - this_vel)


class OVMController(BaseController):
    """Optimal Vehicle Model controller.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    alpha : float
        gain on desired velocity to current velocity difference
        (default: 0.6)
    beta : float
        gain on lead car velocity and self velocity difference
        (default: 0.9)
    h_st : float
        headway for stopping (default: 5)
    h_go : float
        headway for full speed (default: 35)
    v_max : float
        max velocity (default: 30)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 alpha=1,
                 beta=1,
                 h_st=2,
                 h_go=15,
                 v_max=30,
                 time_delay=0,
                 noise=0,
                 fail_safe=None):
        """Instantiate an Optimal Vehicle Model controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)
        self.veh_id = veh_id
        self.v_max = v_max
        self.alpha = alpha
        self.beta = beta
        self.h_st = h_st
        self.h_go = h_go

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            return self.max_accel

        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        h_dot = lead_vel - this_vel

        # V function here - input: h, output : Vh
        if h <= self.h_st:
            v_h = 0
        elif self.h_st < h < self.h_go:
            v_h = self.v_max / 2 * (1 - math.cos(math.pi * (h - self.h_st) /
                                                 (self.h_go - self.h_st)))
        else:
            v_h = self.v_max

        return self.alpha * (v_h - this_vel) + self.beta * h_dot


class LinearOVM(BaseController):
    """Linear OVM controller.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    v_max : float
        max velocity (default: 30)
    adaptation : float
        adaptation constant (default: 0.65)
    h_st : float
        headway for stopping (default: 5)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 v_max=30,
                 adaptation=0.65,
                 h_st=5,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None):
        """Instantiate a Linear OVM controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)
        self.veh_id = veh_id
        # 4.8*1.85 for case I, 3.8*1.85 for case II, per Nakayama
        self.v_max = v_max
        # TAU in Traffic Flow Dynamics textbook
        self.adaptation = adaptation
        self.h_st = h_st

    def get_accel(self, env):
        """See parent class."""
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)

        # V function here - input: h, output : Vh
        alpha = 1.689  # the average value from Nakayama paper
        if h < self.h_st:
            v_h = 0
        elif self.h_st <= h <= self.h_st + self.v_max / alpha:
            v_h = alpha * (h - self.h_st)
        else:
            v_h = self.v_max

        return (v_h - this_vel) / self.adaptation



class SimCarFollowingController(BaseController):
    """Controller whose actions are purely defined by the simulator.

    Note that methods for implementing noise and failsafes through
    BaseController, are not available here. However, similar methods are
    available through sumo when initializing the parameters of the vehicle.
    """

    def get_accel(self, env):
        """See parent class."""
        return None

class LinOpt_Controller_IDM(BaseController): 
    def __init__(self,
                 veh_id,
                 car_following_params,
                 a=0,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None):
        """Instantiate a Linear Adaptive Cruise controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)

        self.veh_id = veh_id
        self.a = a

    def get_accel(self, env):
        """See parent class."""
        #This is the set of the linear opt parameters when other vheilces are IDM
        v_star = 4.18157
        s_star = 6.8180
        sc_star = 6.8180
        N = 22
        K=[-0.95464,2.2032,0.26147,1.2331,0.63633,0.92024,0.86654,0.52388,0.94024,0.12825,0.87704,-0.19717,0.71731,-0.412,0.50987,-0.50829,0.29889,-0.50586,0.11532,-0.44032,-0.026243,-0.34996,-0.12584,-0.26547,-0.19315,-0.20448,-0.24149,-0.17126,-0.28275,-0.16005,-0.32452,-0.15974,-0.36905,-0.15803,-0.41373,-0.14533,-0.45294,-0.12019,-0.48196,-0.099521,-0.50502,-0.13163,-0.5481,-0.28571]

        #This is the set of the linear opt parameters when other vheilces are OVM
        # v_star = 9.0694
        # s_star = 6.8180
        # sc_star = 6.8180
        # N = 22
        # K=[-0.79041,4.3082,5.7045,2.2852,8.163,1.0668,7.8095,-0.83604,4.5724,-2.3559,0.083534,-2.7035,-3.4705,-1.8792,-4.7642,-0.55469,-3.9456,0.48891,-2.1702,0.86311,-0.63773,0.67655,0.082931,0.287,0.096708,-0.010375,-0.19433,-0.12364,-0.4668,-0.11216,-0.61086,-0.067385,-0.66183,-0.039267,-0.68395,-0.031113,-0.70706,-0.026942,-0.72516,-0.017337,-0.7276,-0.010351,-0.72994,-0.020352] 
        
        #lead_id = env.k.vehicle.get_leader(self.veh_id)
        #lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        headway = env.k.vehicle.get_headway(self.veh_id)
        
        u = K[0]*(headway-sc_star)+K[1]*(this_vel-v_star)  #State feedback of the subject vehicle

        temp_id = env.k.vehicle.get_leader(self.veh_id)   
        for i in range (0,21):  #iterate through the 
            temp_headway = env.k.vehicle.get_headway(temp_id)
            temp_vel = env.k.vehicle.get_speed(temp_id)
            u = u + K[N*2-2-i*2]*(temp_headway-s_star)+K[N*2-1-i*2]*(temp_vel-v_star)
            temp_id = env.k.vehicle.get_leader(temp_id)
        return -u


class LinOpt_Controller_OVM(BaseController): 
    def __init__(self,
                 veh_id,
                 car_following_params,
                 a=0,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None):
        """Instantiate a Linear Adaptive Cruise controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)

        self.veh_id = veh_id
        self.a = a

    def get_accel(self, env):
        """See parent class."""
        #This is the set of the linear opt parameters when other vheilces are IDM
        # v_star = 4.18157
        # s_star = 6.8180
        # sc_star = 6.8180
        # N = 22
        # K=[-0.95464,2.2032,0.26147,1.2331,0.63633,0.92024,0.86654,0.52388,0.94024,0.12825,0.87704,-0.19717,0.71731,-0.412,0.50987,-0.50829,0.29889,-0.50586,0.11532,-0.44032,-0.026243,-0.34996,-0.12584,-0.26547,-0.19315,-0.20448,-0.24149,-0.17126,-0.28275,-0.16005,-0.32452,-0.15974,-0.36905,-0.15803,-0.41373,-0.14533,-0.45294,-0.12019,-0.48196,-0.099521,-0.50502,-0.13163,-0.5481,-0.28571]

        #This is the set of the linear opt parameters when other vheilces are OVM
        v_star = 9.0694
        s_star = 6.8180
        sc_star = 6.8180
        N = 22
        K=[-0.79041,4.3082,5.7045,2.2852,8.163,1.0668,7.8095,-0.83604,4.5724,-2.3559,0.083534,-2.7035,-3.4705,-1.8792,-4.7642,-0.55469,-3.9456,0.48891,-2.1702,0.86311,-0.63773,0.67655,0.082931,0.287,0.096708,-0.010375,-0.19433,-0.12364,-0.4668,-0.11216,-0.61086,-0.067385,-0.66183,-0.039267,-0.68395,-0.031113,-0.70706,-0.026942,-0.72516,-0.017337,-0.7276,-0.010351,-0.72994,-0.020352] 
        
        #lead_id = env.k.vehicle.get_leader(self.veh_id)
        #lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        headway = env.k.vehicle.get_headway(self.veh_id)
        
        u = K[0]*(headway-sc_star)+K[1]*(this_vel-v_star)  #State feedback of the subject vehicle

        temp_id = env.k.vehicle.get_leader(self.veh_id)   
        for i in range (0,21):  #iterate through the 
            temp_headway = env.k.vehicle.get_headway(temp_id)
            temp_vel = env.k.vehicle.get_speed(temp_id)
            u = u + K[N*2-2-i*2]*(temp_headway-s_star)+K[N*2-1-i*2]*(temp_vel-v_star)
            temp_id = env.k.vehicle.get_leader(temp_id)
        return -u

class LACController(BaseController):
    """Linear Adaptive Cruise Control.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    k_1 : float
        design parameter (default: 0.8)
    k_2 : float
        design parameter (default: 0.9)
    h : float
        desired time gap  (default: 1.0)
    tau : float
        lag time between control input u and real acceleration a (default:0.1)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 k_1=0.3,
                 k_2=0.4,
                 h=1,
                 tau=0.1,
                 a=0,
                 time_delay=0.0,
                 noise=0,
                 fail_safe=None):
        """Instantiate a Linear Adaptive Cruise controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)

        self.veh_id = veh_id
        self.k_1 = k_1
        self.k_2 = k_2
        self.h = h
        self.tau = tau
        self.a = a

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        headway = env.k.vehicle.get_headway(self.veh_id)
        #L = env.k.vehicle.get_length(self.veh_id)
        ex = headway - self.h * this_vel
        ev = lead_vel - this_vel
        u = self.k_1*ex + self.k_2*ev
        a_dot = -(self.a/self.tau) + (u/self.tau)
        self.a = a_dot*env.sim_step + self.a

        return self.a


class OV_FTL(BaseController):
    """pptimal-velocity-follow-the-leader-model [Cui et al. 2017].

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    alpha : float
        gain on desired velocity to current velocity difference
        (default: 0.6)
    beta : float
        gain on lead car velocity and self velocity difference
        (default: 0.9)
    h_st : float
        headway for stopping (default: 5)
    h_go : float
        headway for full speed (default: 35)
    v_max : float
        max velocity (default: 30)
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 alpha=1,
                 beta=1,
                 h_st=2,
                 h_go=15,
                 v_max=30,                 
                 time_delay=0,
                 noise=0,
                 fail_safe=None):
        """Instantiate an Optimal Vehicle Model controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)
        self.veh_id = veh_id
        self.v_max = v_max
        self.alpha = alpha
        self.beta = beta
        self.h_st = h_st
        self.h_go = h_go

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            return self.max_accel

        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        h_dot = lead_vel - this_vel

        # V function here - input: h, output : Vh
        if h <= self.h_st:
            v_h = 0
        elif self.h_st < h < self.h_go:
            v_h = self.v_max / 2 * (1 - math.cos(math.pi * (h - self.h_st) /
                                                 (self.h_go - self.h_st)))
        else:
            v_h = self.v_max

        return self.alpha * (v_h - this_vel) + self.beta * ((h_dot)/h**2)



class Augmented_OV_FTL(BaseController):
    """Augmented OV FTL model [Cui et al. 2017].

    Usage
    -----
    See BaseController for usage example.

    Attributes
    ----------
    veh_id : str
        Vehicle ID for SUMO identification
    car_following_params : flow.core.params.SumoCarFollowingParams
        see parent class
    alpha : float
        gain on desired velocity to current velocity difference
        (default: 0.6)
    beta : float
        gain on lead car velocity and self velocity difference
        (default: 0.9)
    h_st : float
        headway for stopping (default: 5)
    h_go : float
        headway for full speed (default: 35)
    v_max : float
        max velocity (default: 30)
    v_eq : float (default : ??)
        ring equilibrium speed. 
    k_c : float (default : ??)
        gain for the speed difference between desired speed and the real speed
    time_delay : float
        time delay (default: 0.5)
    noise : float
        std dev of normal perturbation to the acceleration (default: 0)
    fail_safe : str
        type of flow-imposed failsafe the vehicle should posses, defaults
        to no failsafe (None)
    """

    def __init__(self,
                 veh_id,
                 car_following_params,
                 alpha=1,
                 beta=1,
                 h_st=2,
                 h_go=15,
                 v_max=30,
                 v_eq = 15.0,
                 k_c = 11.0,
                 time_delay=0,
                 noise=0,
                 fail_safe=None):
        """Instantiate an Optimal Vehicle Model controller."""
        BaseController.__init__(
            self,
            veh_id,
            car_following_params,
            delay=time_delay,
            fail_safe=fail_safe,
            noise=noise)
        self.veh_id = veh_id
        self.v_max = v_max
        self.alpha = alpha
        self.beta = beta
        self.h_st = h_st
        self.h_go = h_go
        self.v_eq = v_eq
        self.k_c = k_c

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        if not lead_id:  # no car ahead
            return self.max_accel

        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        h = env.k.vehicle.get_headway(self.veh_id)
        h_dot = lead_vel - this_vel

        # V function here - input: h, output : Vh
        if h <= self.h_st:
            v_h = 0
        elif self.h_st < h < self.h_go:
            v_h = self.v_max / 2 * (1 - math.cos(math.pi * (h - self.h_st) /
                                                 (self.h_go - self.h_st)))
        else:
            v_h = self.v_max

        return self.alpha * (v_h - this_vel) + self.beta * (h_dot/h**2) + self.k_c*(self.v_eq-this_vel)


class ModifiedLyapunovTypeControllerU1(BaseController):
    """[Mondache et al. 2019: Feedback control algorithms for dissipation of traffic waves with autonomous vehicles].

    Usage
    -----
    See base class for example.

    Parameters
    ----------
    veh_id : str
        unique vehicle identifier
    car_following_params : flow.core.params.SumoCarFollowingParams
        object defining sumo-specific car-following parameters
    """

    def __init__(self, veh_id, car_following_params):
        """Instantiate PISaturation."""
        BaseController.__init__(self, veh_id, car_following_params, delay=1.0)

        # maximum achievable acceleration by the vehicle
        self.max_accel = car_following_params.controller_params['accel']

        # other parameters
        self.gamma = 2

        # values that are updated by using their old information
        self.alpha = 0
        self.beta = 1 - 0.5 * self.alpha
        self.v_target = 0
        self.v_cmd = 0
        self.avg_lead_vel = 0
        self.avg_v_cmd = 0
        self.ctrl_steps = 0

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        #temp = 0
        #if lead_vel == 0:
        #    temp = temp+1

        dx = env.k.vehicle.get_headway(self.veh_id)
        dv = lead_vel - this_vel                
        dx_s = max(2 * dv, 4)

        dt = env.sim_step #discretization step size

        # update the average speed of the preceding vehicle and the speed command of the subject vehicle
        if self.ctrl_steps==0:
            self.avg_lead_vel = lead_vel
            self.avg_v_cmd = this_vel

        self.avg_lead_vel = (self.avg_lead_vel*self.ctrl_steps + lead_vel)/(self.ctrl_steps+1)
        self.avg_v_cmd = (self.avg_v_cmd*self.ctrl_steps + self.v_cmd)/(self.ctrl_steps+1)
        self.ctrl_steps = self.ctrl_steps+1

        v_bar = min(self.avg_lead_vel,self.avg_v_cmd)

        # update desired velocity values
        v_target = (self.v_cmd-v_bar)*np.exp(-dt) + v_bar

        # update the alpha and beta values
        alpha = min(max((dx - dx_s) / self.gamma, 0), 1)
        beta = 1 - 0.5 * alpha

        # compute desired velocity
        self.v_cmd = beta * (alpha * v_target + (1 - alpha) * lead_vel) \
            + (1 - beta) * self.v_cmd

        # compute the acceleration
        accel = (self.v_cmd - this_vel) / env.sim_step

        return min(accel, self.max_accel)


class ModifiedLyapunovTypeControllerU2(BaseController):
    """[Mondache et al. 2019: Feedback control algorithms for dissipation of traffic waves with autonomous vehicles].

    Usage
    -----
    See base class for example.

    Parameters
    ----------
    veh_id : str
        unique vehicle identifier
    car_following_params : flow.core.params.SumoCarFollowingParams
        object defining sumo-specific car-following parameters
    """

    def __init__(self, veh_id, car_following_params):
        """Instantiate PISaturation."""
        BaseController.__init__(self, veh_id, car_following_params, delay=1.0)

        # maximum achievable acceleration by the vehicle
        self.max_accel = car_following_params.controller_params['accel']

        # other parameters
        self.gamma = 2

        # values that are updated by using their old information
        self.alpha = 0
        self.beta = 1 - 0.5 * self.alpha
        self.U = 0
        self.v_target = 0
        self.v_cmd = 0
        self.avg_lead_vel = 0
        self.avg_v_cmd = 0
        self.ctrl_steps = 0

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)

        dx = env.k.vehicle.get_headway(self.veh_id)
        dv = lead_vel - this_vel
        dx_s = max(2 * dv, 4)
        dt = env.sim_step #discretization step size

        # update the average speed of the preceding vehicle and the speed command of the subject vehicle
        if self.ctrl_steps==0:
            self.avg_lead_vel = lead_vel
            self.avg_v_cmd = this_vel

        self.avg_lead_vel = (self.avg_lead_vel*self.ctrl_steps + lead_vel)/(self.ctrl_steps+1)
        self.avg_v_cmd = (self.avg_v_cmd*self.ctrl_steps + self.v_cmd)/(self.ctrl_steps+1)
        self.ctrl_steps = self.ctrl_steps+1

        v_bar = min(self.avg_lead_vel,self.avg_v_cmd)

        # update desired velocity values
        v_target = (self.v_cmd-(lead_vel+v_bar)/2)*np.exp(-dt) + (lead_vel+v_bar)/2

        # update the alpha and beta values
        alpha = min(max((dx - dx_s) / self.gamma, 0), 1)
        beta = 1 - 0.5 * alpha

        # compute desired velocity
        self.v_cmd = beta * (alpha * v_target + (1 - alpha) * lead_vel) \
            + (1 - beta) * self.v_cmd

        # compute the acceleration
        accel = (self.v_cmd - this_vel) / env.sim_step

        return min(accel, self.max_accel)


class FuzzyController_New(BaseController):
    """[To be appered in 2020 ACC. Hauley et al. : A Fuzzy-Based Approach to Dampen Emergent Traffic Waves].
    This is the "New model they shared with us in Simulink"
    Usage
    -----
    See base class for example.

    Parameters
    ----------
    veh_id : str
        unique vehicle identifier
    car_following_params : flow.core.params.SumoCarFollowingParams
        object defining sumo-specific car-following parameters
    """

    def __init__(self, veh_id, car_following_params):
        """FuzzyConytoller_New."""
        BaseController.__init__(self, veh_id, car_following_params, delay=1.0)

        # maximum achievable acceleration by the vehicle
        self.max_accel = car_following_params.controller_params['accel']
        self.vmax = 25 #speed limit
    
    def FuzzyRule(self, dist_value, speed_value):
        if dist_value == -4:  
            correction_value = -4
        elif dist_value == -3: 
            if speed_value == 2:
                correction_value = -4
            elif speed_value == 1:
                correction_value = -4
            elif speed_value == 0:
                correction_value = -3
            elif speed_value == -1:
                correction_value = -2
            else :
                correction_value = -1
        elif dist_value == -2:
            if speed_value == 2:
                correction_value = -4
            if speed_value == 1:
                correction_value = -3
            if speed_value == 0:
                correction_value = -2
            if speed_value == -1:
                correction_value = -1
            else:
                correction_value = 0
        elif dist_value == -1:
            if speed_value == 2:
                correction_value = -3
            if speed_value == 1:
                correction_value = -2
            if speed_value == 0:
                correction_value = 10
            if speed_value == -1:
                correction_value = 0
            else:
                correction_value = 1
        elif dist_value == 1:
            if speed_value == 2:
                correction_value  = -2
            elif speed_value == 1:
                correction_value = 0
            elif speed_value == 0:
                correction_value = 10
            elif speed_value == -1:
                correction_value = 1
            else :
                correction_value = 2
        elif dist_value == 2:
            if speed_value == 2:
                correction_value = -1
            elif speed_value == 1:
                correction_value = 0
            elif speed_value == 0:
                correction_value = 1
            elif speed_value == -1:
                correction_value = 2
            else :
                correction_value = 3
        elif dist_value == 3:
            if speed_value ==2:
                correction_value = 0
            elif speed_value == 1:
                correction_value = 1
            elif speed_value == 0:
                correction_value = 2
            elif speed_value == -1:
                correction_value = 3
            else :
                correction_value = 4
        elif dist_value == 4:
            correction_value = 4
        else:
            if speed_value == 2:
                correction_value = -1
            elif speed_value ==1:
                correction_value = 0
            elif speed_value == -1:
                correction_value = 1
            elif speed_value == -2:
                correction_value = 2
            else :
                correction_value = 0

        return correction_value

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        
        dx = env.k.vehicle.get_headway(self.veh_id)
        dv = lead_vel - this_vel
        
        rel_vel = -1*dv

        if this_vel>1:
            dist_realative = dx/this_vel
            speed_percent_dif = (rel_vel/this_vel)*100
            speed_value = speed_percent_dif/10
        else:
            dist_realative = dx
            speed_percent_dif = rel_vel
            speed_value = speed_percent_dif
        
        dist_realative = dist_realative - 4
        dist_value = round(dist_realative)
        
        if dist_value>4:
            dist_value = 4
        
        if speed_value>2:
            speed_value = 2

        if speed_value <-2:
            speed_value = -2 

        dist_value = int(dist_value)
        speed_value = int(speed_value)

        correction_value = self.FuzzyRule(dist_value,speed_value)

        if this_vel > 1:
            velocity_setting = this_vel + (correction_value/20)*this_vel
        else:
            velocity_setting = this_vel + (correction_value/3)

        if correction_value == 10:
            velocity_setting = dv + this_vel

        if (correction_value == -4 or velocity_setting < 0.001):
	        velocity_setting = 0

        if correction_value == 4:
            velocity_setting = this_vel + 1.6        

        if velocity_setting > self.vmax:
            velocity_setting = self.vmax

        # compute the acceleration
        accel = (velocity_setting - this_vel) / env.sim_step
        return min(accel, self.max_accel)


class FuzzyController_Old(BaseController):
    """[To be appered in 2020 ACC. Hauley et al. : A Fuzzy-Based Approach to Dampen Emergent Traffic Waves].
    This is the "Old model they shared with us in Simulink"
    Usage
    -----
    See base class for example.

    Parameters
    ----------
    veh_id : str
        unique vehicle identifier
    car_following_params : flow.core.params.SumoCarFollowingParams
        object defining sumo-specific car-following parameters
    """

    def __init__(self, veh_id, car_following_params):
        """Instantiate PISaturation."""
        BaseController.__init__(self, veh_id, car_following_params, delay=1.0)

        # maximum achievable acceleration by the vehicle
        self.max_accel = car_following_params.controller_params['accel']
        self.vmax = 25 #speed limit
        self.steps = 0
    
    def FuzzyRule(self, dist_value, speed_value):
        if dist_value == -4:  
            correction_value = -4
        elif dist_value == -3: 
            if speed_value == 2:
                correction_value = -3
            elif speed_value == 1:
                correction_value = -2
            elif speed_value == 0:
                correction_value = -1
            elif speed_value == -1:
                correction_value = 0
            else :
                correction_value = 1
        elif dist_value == -2:
            if speed_value == 2:
                correction_value = -3
            if speed_value == 1:
                correction_value = -2
            if speed_value == 0:
                correction_value = -1
            if speed_value == -1:
                correction_value = 0
            else:
                correction_value = 1
        elif dist_value == -1:
            if speed_value == 2:
                correction_value = -2
            if speed_value == 1:
                correction_value = -1
            if speed_value == 0:
                correction_value = 10
            if speed_value == -1:
                correction_value = 0
            else:
                correction_value = 1
        elif dist_value == 1:
            if speed_value == 2:
                correction_value  = -1
            elif speed_value == 1:
                correction_value = 0
            elif speed_value == 0:
                correction_value = 10
            elif speed_value == -1:
                correction_value = 1
            else :
                correction_value = 2
        elif dist_value == 2:
            if speed_value == 2:
                correction_value = -1
            elif speed_value == 1:
                correction_value = 0
            elif speed_value == 0:
                correction_value = 1
            elif speed_value == -1:
                correction_value = 2
            else :
                correction_value = 3
        elif dist_value == 3:
            if speed_value ==2:
                correction_value = -1
            elif speed_value == 1:
                correction_value = 0
            elif speed_value == 0:
                correction_value = 1
            elif speed_value == -1:
                correction_value = 2
            else :
                correction_value = 3
        elif dist_value == 4:
            correction_value = 4
        else:
            if speed_value == 2:
                correction_value = -1
            elif speed_value ==1:
                correction_value = 0
            elif speed_value == -1:
                correction_value = 0
            elif speed_value == -2:
                correction_value = 1
            else :
                correction_value = 0

        return correction_value

    def get_accel(self, env):
        """See parent class."""
        lead_id = env.k.vehicle.get_leader(self.veh_id)
        lead_vel = env.k.vehicle.get_speed(lead_id)
        this_vel = env.k.vehicle.get_speed(self.veh_id)
        
        dx = env.k.vehicle.get_headway(self.veh_id)
        dv = lead_vel - this_vel
        
        rel_vel = -1*dv

        if this_vel>1:
            dist_realative = dx/this_vel
            speed_percent_dif = (rel_vel/this_vel)*100
            speed_value = speed_percent_dif/10
        else:
            dist_realative = dx
            speed_percent_dif = rel_vel
            speed_value = speed_percent_dif
        
        dist_realative = dist_realative - 4
        dist_value = round(dist_realative)
        
        if dist_value>4:
            dist_value = 4
        
        if speed_value>2:
            speed_value = 2

        if speed_value <-2:
            speed_value = -2 

        dist_value = int(dist_value)
        speed_value = int(speed_value)

        correction_value = self.FuzzyRule(dist_value,speed_value)

        if this_vel > 1:
            velocity_setting = this_vel + (correction_value/15)*this_vel
        else:
            velocity_setting = this_vel + (correction_value/3)

        if correction_value == 10:
            velocity_setting = dv + this_vel

        if (correction_value == -4 or velocity_setting < 0.001):
	        velocity_setting = 0

        if correction_value == 4:
            velocity_setting = this_vel + 1.6        

        if velocity_setting > self.vmax:
            velocity_setting = self.vmax
        self.steps = self.steps + 1

        if ( self.steps*env.sim_step < 3 and dist_value >= -1):
            velocity_setting = velocity_setting + 2.5

        # compute the acceleration
        accel = (velocity_setting - this_vel) / env.sim_step
        return min(accel, self.max_accel)