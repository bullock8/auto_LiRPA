from enum import Enum, auto
import copy
from typing import List


class LaneObjectMode(Enum):
    Vehicle = auto()
    Ped = auto()  # Pedestrians
    Sign = auto()  # Signs, stop signs, merge, yield etc.
    Signal = auto()  # Traffic lights
    Obstacle = auto()  # Static (to road/lane) obstacles


class AgentMode(Enum):
    COC = auto()
    WL = auto()
    WR = auto()
    SL = auto()
    SR = auto()


class TrackMode(Enum):
    T0 = auto()
    T1 = auto()
    T2 = auto()
    M01 = auto()
    M12 = auto()
    M21 = auto()
    M10 = auto()


class State:
    x: float
    y: float
    z: float
    theta: float
    psi: float
    v: float
    agent_mode: AgentMode
    track_mode: TrackMode

    def __init__(self, x, y, z, theta, psi, v, agent_mode: AgentMode, track_mode: TrackMode):
        pass

'''
def decisionLogic(ego: State, others: List[State], track_map):
    output = copy.deepcopy(ego)
    # assert not vehicle_close(ego, others)
    return output
'''

def decisionLogic(ego: State, others: List[State]):
    next = copy.deepcopy(ego)
    
    # if ego.mode == AgentMode.SL:
    #     next.AgentMode = AgentMode.WR

    #next.AgentMode = AgentMode.COC

    
    #if any( test_fcn(ego, other) < 50 for other in others):
    #    next.random_attr = 1
    #temp_var = ego.x10
    
    #temp_var = [num * (num > 0) for num in nums]
    
    
    #next.AgentMode = AgentMode.WL
    # if ego.mode == AgentMode.COC: 
    #     if ego.x13 > 0:
    #         next.AgentMode = AgentMode.WL
    # if cmd==0:
    #     next.AgentMode = AgentMode.COC
    # elif cmd==1:
    #     next.AgentMode = AgentMode.WL
    # elif cmd==2:
    #     next.AgentMode = AgentMode.WR
    # elif cmd==3:
    #     next.AgentMode = AgentMode.SL
    # elif cmd==4:
    #     next.AgentMode = AgentMode.SR
    ### generate the mode switching using states of the two agents

    #####
    # ADD ACAS
    #####

    #################
    # State class ref
    #################
    # e_long = state_arr[0:3]
    # e_lat = state_arr[3:6]
    # aircraft_state = np.array(state_arr[6:19])
            # [   0:3  ,   3:6    ,   6:9  ,  9:13 ] + 6
            # [ vel_bEb, Omega_BIb, pos_bii, Q_i2b ]
    # surf_state = state_arr[19:24]
    # random_attr = state_arr[-1]
    
    ## Compute
    
    last_command = 0
    
    rho = ego.rho
    theta = ego.theta
    psi = ego.psi
    vOwn = ego.v_own
    vInt = ego.v_int

    # theta = np.arctan2(dy, dx)

    # psi = np.atan2(2 * (qw*qz + qx*qy), qw**2 + qx**2 - qy**2 - qz**2)

    # #state = [rho, theta, psi, v_own, v_int]
    
    '''
    if rho > 60760 and ego.agent_mode == AgentMode.COC:
        next.AgentMode = AgentMode.WL
        # 
    if rho <= 60760 and ego.agent_mode == AgentMode.COC:
        if rho < 150 and ego.agent_mode == AgentMode.COC:
            next.AgentMode = AgentMode.WR
        # if rho >= 150 and ego.agent_mode == AgentMode.COC:
        #     next.AgentMode = AgentMode.COC
    
    
    if ego.timer > 5:
        if rho > 60760 and ego.timer > 5:
            next.AgentMode = AgentMode.COC
            # 
            
        if rho <= 60760 and ego.timer > 5:
            if rho < 150 and ego.timer > 5:
                next.AgentMode = AgentMode.WR
                
            # if rho >= 150 and ego.agent_mode == AgentMode.COC:
            #     next.AgentMode = AgentMode.COC
    
        '''
    acas_update_time = 0.5 # ACAS will send new advisory every 0.5 seconds
    
    
    assert not (rho <= 50), "Unsafe Separation"
    
    
    if rho < 4000 and ego.initial_mode[0] == AgentMode.COC:
        next.agent_mode = AgentMode.SR
    else:
        next.agent_mode = AgentMode.COC
        next.a
                        

    return next