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
    '''    
    if ego.timer >= acas_update_time:
        if theta <= 0.0:
            if rho <= 28144.68:
                if theta <= -1.4:
                    if rho <= 4978.66:
                        if rho <= 1117.66:
                            if rho <= 508.03:
                                next.AgentMode = AgentMode.SR
                                
                            if rho > 508.03:
                                next.AgentMode = AgentMode.SL
                                
                        if rho > 1117.66:
                            next.AgentMode = AgentMode.WL
                            
                    if rho > 4978.66:
                        next.AgentMode = AgentMode.COC
                        
                if theta > -1.4:
                    if rho <= 8026.82:
                        if theta <= -0.51:
                            if rho <= 2133.71:
                                next.AgentMode = AgentMode.SL
                                
                            if rho > 2133.71:
                                next.AgentMode = AgentMode.WL
                                
                        if theta > -0.51:
                            next.AgentMode = AgentMode.SL
                            
                    if rho > 8026.82:
                        if theta <= -0.7:
                            next.AgentMode = AgentMode.COC
                            
                        if theta > -0.7:
                            if theta <= -0.13:
                                next.AgentMode = AgentMode.WL
                                
                            if theta > -0.13:
                                next.AgentMode = AgentMode.SL
                                
            if rho > 28144.68:
                if theta <= -0.25:
                    next.AgentMode = AgentMode.COC
                    
                if theta > -0.25:
                    next.AgentMode = AgentMode.SL
                    
        if theta > 0.0:
            if rho <= 16358.46:
                if rho <= 1320.87:
                    next.AgentMode = AgentMode.SR
                    
                if rho > 1320.87:
                    if theta <= 1.4:
                        if rho <= 6604.35:
                            if theta <= 0.57:
                                next.AgentMode = AgentMode.SR
                                
                            if theta > 0.57:
                                if rho <= 3759.4:
                                    next.AgentMode = AgentMode.SR
                                    
                                if rho > 3759.4:
                                    next.AgentMode = AgentMode.WR
                                    
                        if rho > 6604.35:
                            if theta <= 0.13:
                                next.AgentMode = AgentMode.SR
                                
                            if theta > 0.13:
                                if theta <= 0.89:
                                    if rho <= 9855.72:
                                        if theta <= 0.38:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if theta > 0.38:
                                            next.AgentMode = AgentMode.WR
                                            
                                    if rho > 9855.72:
                                        next.AgentMode = AgentMode.WR
                                        
                                if theta > 0.89:
                                    next.AgentMode = AgentMode.COC
                                    
                    if theta > 1.4:
                        if rho <= 4775.45:
                            next.AgentMode = AgentMode.WR
                            
                        if rho > 4775.45:
                            next.AgentMode = AgentMode.COC
                            
            if rho > 16358.46:
                if theta <= 0.32:
                    if rho <= 44604.75:
                        next.AgentMode = AgentMode.WR
                        
                    if rho > 44604.75:
                        next.AgentMode = AgentMode.COC
                        
                if theta > 0.32:
                    next.AgentMode = AgentMode.COC
                    
        '''
        
    # Version of DL that takes 8-10 minutes per run
    '''
    if ego.timer >= acas_update_time:
        if ego.agent_mode == AgentMode.COC:  # advisory 0
            if rho <= 15545.62:
                if theta <= -0.13:
                    if rho <= 1727.29:
                        if psi <= 0.13:
                            if rho <= 1320.87:
                                next.AgentMode = AgentMode.SL
                                
                            if rho > 1320.87:
                                if theta <= -1.78:
                                    if psi <= -0.95:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if psi > -0.95:
                                        next.AgentMode = AgentMode.SL
                                        
                                if theta > -1.78:
                                    next.AgentMode = AgentMode.SL
                                    
                        if psi > 0.13:
                            if rho <= 711.24:
                                if psi <= 2.22:
                                    next.AgentMode = AgentMode.SR
                                    
                                if psi > 2.22:
                                    if theta <= -1.97:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if theta > -1.97:
                                        next.AgentMode = AgentMode.SL
                                        
                            if rho > 711.24:
                                if theta <= -2.73:
                                    next.AgentMode = AgentMode.SR
                                    
                                if theta > -2.73:
                                    if theta <= -0.63:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if theta > -0.63:
                                        next.AgentMode = AgentMode.SR
                                        
                    if rho > 1727.29:
                        if theta <= -0.83:
                            if psi <= -0.25:
                                if rho <= 5385.08:
                                    next.AgentMode = AgentMode.WL
                                    
                                if rho > 5385.08:
                                    next.AgentMode = AgentMode.COC
                                    
                            if psi > -0.25:
                                if theta <= -2.73:
                                    if psi <= 0.38:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if psi > 0.38:
                                        if rho <= 5385.08:
                                            next.AgentMode = AgentMode.WR
                                            
                                        if rho > 5385.08:
                                            next.AgentMode = AgentMode.COC
                                            
                                if theta > -2.73:
                                    if rho <= 6197.93:
                                        if psi <= 1.9:
                                            if theta <= -2.03:
                                                next.AgentMode = AgentMode.WL
                                                
                                            if theta > -2.03:
                                                if rho <= 4165.82:
                                                    next.AgentMode = AgentMode.SL
                                                    
                                                if rho > 4165.82:
                                                    if psi <= 0.7:
                                                        next.AgentMode = AgentMode.WL
                                                        
                                                    if psi > 0.7:
                                                        next.AgentMode = AgentMode.SL
                                                        
                                        if psi > 1.9:
                                            if theta <= -1.14:
                                                next.AgentMode = AgentMode.WL
                                                
                                            if theta > -1.14:
                                                next.AgentMode = AgentMode.SL
                                                
                                    if rho > 6197.93:
                                        if theta <= -1.97:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if theta > -1.97:
                                            next.AgentMode = AgentMode.WL
                                            
                        if theta > -0.83:
                            if rho <= 4572.24:
                                if psi <= 0.76:
                                    if theta <= -0.57:
                                        if rho <= 3352.98:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if rho > 3352.98:
                                            next.AgentMode = AgentMode.WL
                                            
                                    if theta > -0.57:
                                        next.AgentMode = AgentMode.SL
                                        
                                if psi > 0.76:
                                    if theta <= -0.44:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if theta > -0.44:
                                        if psi <= 2.54:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if psi > 2.54:
                                            next.AgentMode = AgentMode.SL
                                            
                            if rho > 4572.24:
                                if theta <= -0.38:
                                    if rho <= 9449.3:
                                        if psi <= 1.08:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if psi > 1.08:
                                            if rho <= 7417.19:
                                                next.AgentMode = AgentMode.SL
                                                
                                            if rho > 7417.19:
                                                next.AgentMode = AgentMode.WL
                                                
                                    if rho > 9449.3:
                                        if psi <= 1.21:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if psi > 1.21:
                                            next.AgentMode = AgentMode.WL
                                            
                                if theta > -0.38:
                                    if psi <= 0.83:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if psi > 0.83:
                                        if psi <= 2.6:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if psi > 2.6:
                                            if rho <= 10262.14:
                                                next.AgentMode = AgentMode.SL
                                                
                                            if rho > 10262.14:
                                                next.AgentMode = AgentMode.WL
                                                
                if theta > -0.13:
                    if rho <= 1727.29:
                        if psi <= -0.19:
                            if rho <= 508.03:
                                next.AgentMode = AgentMode.SL
                                
                            if rho > 508.03:
                                if theta <= 0.44:
                                    next.AgentMode = AgentMode.SL
                                    
                                if theta > 0.44:
                                    if theta <= 2.73:
                                        if psi <= -1.78:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if psi > -1.78:
                                            next.AgentMode = AgentMode.SR
                                            
                                    if theta > 2.73:
                                        next.AgentMode = AgentMode.SL
                                        
                        if psi > -0.19:
                            if rho <= 1524.08:
                                next.AgentMode = AgentMode.SR
                                
                            if rho > 1524.08:
                                if theta <= 1.71:
                                    next.AgentMode = AgentMode.SR
                                    
                                if theta > 1.71:
                                    if psi <= 1.14:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > 1.14:
                                        next.AgentMode = AgentMode.WR
                                        
                    if rho > 1727.29:
                        if theta <= 1.33:
                            if rho <= 5181.87:
                                if psi <= -0.32:
                                    if theta <= 0.44:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if theta > 0.44:
                                        next.AgentMode = AgentMode.SR
                                        
                                if psi > -0.32:
                                    if theta <= 0.57:
                                        if rho <= 3759.4:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if rho > 3759.4:
                                            if psi <= 1.59:
                                                next.AgentMode = AgentMode.WR
                                                
                                            if psi > 1.59:
                                                next.AgentMode = AgentMode.SR
                                                
                                    if theta > 0.57:
                                        if rho <= 3556.19:
                                            if rho <= 2133.71:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if rho > 2133.71:
                                                next.AgentMode = AgentMode.WR
                                                
                                        if rho > 3556.19:
                                            next.AgentMode = AgentMode.WR
                                            
                            if rho > 5181.87:
                                if psi <= -1.08:
                                    if theta <= 0.38:
                                        if rho <= 9449.3:
                                            if psi <= -2.22:
                                                next.AgentMode = AgentMode.SL
                                                
                                            if psi > -2.22:
                                                next.AgentMode = AgentMode.WL
                                                
                                        if rho > 9449.3:
                                            next.AgentMode = AgentMode.WL
                                            
                                    if theta > 0.38:
                                        if rho <= 8026.82:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if rho > 8026.82:
                                            next.AgentMode = AgentMode.WR
                                            
                                if psi > -1.08:
                                    if rho <= 9652.51:
                                        if theta <= 0.13:
                                            if psi <= 2.22:
                                                if psi <= 0.13:
                                                    next.AgentMode = AgentMode.WL
                                                    
                                                if psi > 0.13:
                                                    next.AgentMode = AgentMode.WR
                                                    
                                            if psi > 2.22:
                                                next.AgentMode = AgentMode.SR
                                                
                                        if theta > 0.13:
                                            next.AgentMode = AgentMode.WR
                                            
                                    if rho > 9652.51:
                                        if psi <= 2.09:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if psi > 2.09:
                                            next.AgentMode = AgentMode.WR
                                            
                        if theta > 1.33:
                            if rho <= 8636.46:
                                if psi <= 0.38:
                                    if theta <= 2.73:
                                        if theta <= 1.9:
                                            if rho <= 2336.92:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if rho > 2336.92:
                                                if rho <= 5791.51:
                                                    if psi <= -1.59:
                                                        next.AgentMode = AgentMode.WR
                                                        
                                                    if psi > -1.59:
                                                        next.AgentMode = AgentMode.SR
                                                        
                                                if rho > 5791.51:
                                                    next.AgentMode = AgentMode.WR
                                                    
                                        if theta > 1.9:
                                            next.AgentMode = AgentMode.WR
                                            
                                    if theta > 2.73:
                                        if rho <= 3352.98:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if rho > 3352.98:
                                            next.AgentMode = AgentMode.WR
                                            
                                if psi > 0.38:
                                    if rho <= 4369.03:
                                        next.AgentMode = AgentMode.WR
                                        
                                    if rho > 4369.03:
                                        next.AgentMode = AgentMode.COC
                                        
                            if rho > 8636.46:
                                next.AgentMode = AgentMode.COC
                                
            if rho > 15545.62:
                if rho <= 18593.78:
                    if psi <= 2.03:
                        next.AgentMode = AgentMode.COC
                        
                    if psi > 2.03:
                        if theta <= -0.19:
                            next.AgentMode = AgentMode.WL
                            
                        if theta > -0.19:
                            if theta <= 0.25:
                                next.AgentMode = AgentMode.WR
                                
                            if theta > 0.25:
                                next.AgentMode = AgentMode.COC
                                
                if rho > 18593.78:
                    next.AgentMode = AgentMode.COC
                    
        
        if ego.agent_mode == AgentMode.WL: # advisory 1
            if psi <= -2.79:
                if theta <= 0.32:
                    if rho <= 13513.51:
                        if theta <= 0.06:
                            if rho <= 1930.5:
                                next.AgentMode = AgentMode.SL
                                
                            if rho > 1930.5:
                                if theta <= -0.95:
                                    next.AgentMode = AgentMode.WL
                                    
                                if theta > -0.95:
                                    next.AgentMode = AgentMode.SL
                                    
                        if theta > 0.06:
                            next.AgentMode = AgentMode.SR
                            
                    if rho > 13513.51:
                        if theta <= -0.32:
                            next.AgentMode = AgentMode.COC
                            
                        if theta > -0.32:
                            next.AgentMode = AgentMode.WL
                            
                if theta > 0.32:
                    if rho <= 10058.93:
                        if theta <= 1.27:
                            next.AgentMode = AgentMode.SR
                            
                        if theta > 1.27:
                            if rho <= 1727.29:
                                next.AgentMode = AgentMode.SL
                                
                            if rho > 1727.29:
                                next.AgentMode = AgentMode.WL
                                
                    if rho > 10058.93:
                        if rho <= 10871.77:
                            next.AgentMode = AgentMode.WR
                            
                        if rho > 10871.77:
                            next.AgentMode = AgentMode.COC
                            
            if psi > -2.79:
                if rho <= 9042.88:
                    if theta <= -0.51:
                        if rho <= 1930.5:
                            if psi <= 0.13:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > 0.13:
                                if psi <= 1.46:
                                    if theta <= -0.89:
                                        if theta <= -1.52:
                                            if rho <= 1320.87:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if rho > 1320.87:
                                                next.AgentMode = AgentMode.SL
                                                
                                        if theta > -1.52:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if theta > -0.89:
                                        next.AgentMode = AgentMode.SR
                                        
                                if psi > 1.46:
                                    next.AgentMode = AgentMode.SL
                                    
                        if rho > 1930.5:
                            if psi <= -0.57:
                                if rho <= 5588.29:
                                    if theta <= -0.83:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if theta > -0.83:
                                        next.AgentMode = AgentMode.WL
                                        
                                if rho > 5588.29:
                                    next.AgentMode = AgentMode.WL
                                    
                            if psi > -0.57:
                                if theta <= -1.33:
                                    if rho <= 3556.19:
                                        if psi <= 1.78:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if psi > 1.78:
                                            next.AgentMode = AgentMode.WL
                                            
                                    if rho > 3556.19:
                                        next.AgentMode = AgentMode.WL
                                        
                                if theta > -1.33:
                                    if rho <= 6807.56:
                                        if theta <= -0.76:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if theta > -0.76:
                                            if psi <= 2.03:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if psi > 2.03:
                                                next.AgentMode = AgentMode.SL
                                                
                                    if rho > 6807.56:
                                        next.AgentMode = AgentMode.WL
                                        
                    if theta > -0.51:
                        if psi <= -0.25:
                            if theta <= 0.38:
                                if rho <= 4165.82:
                                    next.AgentMode = AgentMode.SL
                                    
                                if rho > 4165.82:
                                    if psi <= -1.33:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if psi > -1.33:
                                        next.AgentMode = AgentMode.WL
                                        
                            if theta > 0.38:
                                if rho <= 1524.08:
                                    if rho <= 914.45:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if rho > 914.45:
                                        next.AgentMode = AgentMode.SL
                                        
                                if rho > 1524.08:
                                    if theta <= 2.03:
                                        if theta <= 0.51:
                                            if psi <= -2.03:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if psi > -2.03:
                                                next.AgentMode = AgentMode.SL
                                                
                                        if theta > 0.51:
                                            next.AgentMode = AgentMode.SR
                                            
                                    if theta > 2.03:
                                        if rho <= 3962.61:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if rho > 3962.61:
                                            next.AgentMode = AgentMode.COC
                                            
                        if psi > -0.25:
                            if theta <= 1.21:
                                if theta <= -0.19:
                                    if psi <= 2.54:
                                        if rho <= 6604.35:
                                            if psi <= 0.25:
                                                next.AgentMode = AgentMode.SL
                                                
                                            if psi > 0.25:
                                                next.AgentMode = AgentMode.SR
                                                
                                        if rho > 6604.35:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if psi > 2.54:
                                        next.AgentMode = AgentMode.SL
                                        
                                if theta > -0.19:
                                    next.AgentMode = AgentMode.SR
                                    
                            if theta > 1.21:
                                if rho <= 1930.5:
                                    if psi <= 0.44:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > 0.44:
                                        next.AgentMode = AgentMode.SL
                                        
                                if rho > 1930.5:
                                    if rho <= 3962.61:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if rho > 3962.61:
                                        next.AgentMode = AgentMode.COC
                                        
                if rho > 9042.88:
                    if theta <= 0.76:
                        if theta <= -1.08:
                            if rho <= 13513.51:
                                if theta <= -1.78:
                                    next.AgentMode = AgentMode.COC
                                    
                                if theta > -1.78:
                                    next.AgentMode = AgentMode.WL
                                    
                            if rho > 13513.51:
                                next.AgentMode = AgentMode.COC
                                
                        if theta > -1.08:
                            if psi <= 0.57:
                                if psi <= -1.33:
                                    if theta <= -0.19:
                                        next.AgentMode = AgentMode.COC
                                        
                                    if theta > -0.19:
                                        if rho <= 13107.09:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if rho > 13107.09:
                                            next.AgentMode = AgentMode.WL
                                            
                                if psi > -1.33:
                                    if psi <= 0.32:
                                        next.AgentMode = AgentMode.COC
                                        
                                    if psi > 0.32:
                                        if theta <= -0.51:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if theta > -0.51:
                                            next.AgentMode = AgentMode.COC
                                            
                            if psi > 0.57:
                                if theta <= 0.06:
                                    if rho <= 11684.62:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if rho > 11684.62:
                                        if theta <= -0.19:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if theta > -0.19:
                                            if psi <= 2.35:
                                                next.AgentMode = AgentMode.COC
                                                
                                            if psi > 2.35:
                                                if rho <= 16358.46:
                                                    next.AgentMode = AgentMode.SL
                                                    
                                                if rho > 16358.46:
                                                    next.AgentMode = AgentMode.WL
                                                    
                                if theta > 0.06:
                                    next.AgentMode = AgentMode.COC
                                    
                    if theta > 0.76:
                        next.AgentMode = AgentMode.COC
                        
        
        if ego.agent_mode == AgentMode.WR: # advisory 2
            if rho <= 7213.98:
                if theta <= 0.44:
                    if theta <= -2.22:
                        if rho <= 1524.08:
                            if psi <= 0.32:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > 0.32:
                                next.AgentMode = AgentMode.SR
                                
                        if rho > 1524.08:
                            if rho <= 4572.24:
                                next.AgentMode = AgentMode.WR
                                
                            if rho > 4572.24:
                                next.AgentMode = AgentMode.COC
                                
                    if theta > -2.22:
                        if theta <= -0.63:
                            if rho <= 1117.66:
                                if psi <= 1.9:
                                    if psi <= -1.33:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > -1.33:
                                        if theta <= -1.52:
                                            if psi <= 0.57:
                                                next.AgentMode = AgentMode.SL
                                                
                                            if psi > 0.57:
                                                next.AgentMode = AgentMode.SR
                                                
                                        if theta > -1.52:
                                            next.AgentMode = AgentMode.SL
                                            
                                if psi > 1.9:
                                    next.AgentMode = AgentMode.SR
                                    
                            if rho > 1117.66:
                                if psi <= -0.13:
                                    next.AgentMode = AgentMode.WR
                                    
                                if psi > -0.13:
                                    next.AgentMode = AgentMode.SL
                                    
                        if theta > -0.63:
                            if psi <= -0.19:
                                if theta <= 0.25:
                                    if rho <= 3149.77:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if rho > 3149.77:
                                        if psi <= -1.33:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if psi > -1.33:
                                            next.AgentMode = AgentMode.WR
                                            
                                if theta > 0.25:
                                    if psi <= -2.6:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > -2.6:
                                        next.AgentMode = AgentMode.SL
                                        
                            if psi > -0.19:
                                if theta <= -0.32:
                                    if psi <= 2.03:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > 2.03:
                                        next.AgentMode = AgentMode.SL
                                        
                                if theta > -0.32:
                                    if rho <= 3759.4:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if rho > 3759.4:
                                        if psi <= 1.27:
                                            next.AgentMode = AgentMode.WR
                                            
                                        if psi > 1.27:
                                            next.AgentMode = AgentMode.SR
                                            
                if theta > 0.44:
                    if rho <= 1727.29:
                        if psi <= -0.19:
                            if psi <= -1.84:
                                next.AgentMode = AgentMode.SR
                                
                            if psi > -1.84:
                                if theta <= 0.83:
                                    next.AgentMode = AgentMode.SL
                                    
                                if theta > 0.83:
                                    if theta <= 2.48:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if theta > 2.48:
                                        next.AgentMode = AgentMode.SL
                                        
                        if psi > -0.19:
                            next.AgentMode = AgentMode.SR
                            
                    if rho > 1727.29:
                        if theta <= 1.52:
                            if psi <= 0.0:
                                if theta <= 0.7:
                                    if psi <= -2.22:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > -2.22:
                                        if psi <= -1.08:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if psi > -1.08:
                                            next.AgentMode = AgentMode.SR
                                            
                                if theta > 0.7:
                                    next.AgentMode = AgentMode.SR
                                    
                            if psi > 0.0:
                                if rho <= 2743.34:
                                    next.AgentMode = AgentMode.SR
                                    
                                if rho > 2743.34:
                                    next.AgentMode = AgentMode.WR
                                    
                        if theta > 1.52:
                            if psi <= 0.25:
                                if psi <= -1.33:
                                    next.AgentMode = AgentMode.WR
                                    
                                if psi > -1.33:
                                    if rho <= 4572.24:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if rho > 4572.24:
                                        next.AgentMode = AgentMode.WR
                                        
                            if psi > 0.25:
                                if rho <= 4978.66:
                                    next.AgentMode = AgentMode.WR
                                    
                                if rho > 4978.66:
                                    next.AgentMode = AgentMode.COC
                                    
            if rho > 7213.98:
                if rho <= 8839.67:
                    if theta <= -0.51:
                        if theta <= -0.83:
                            next.AgentMode = AgentMode.COC
                            
                        if theta > -0.83:
                            if psi <= 2.48:
                                if psi <= 1.71:
                                    next.AgentMode = AgentMode.SR
                                    
                                if psi > 1.71:
                                    next.AgentMode = AgentMode.WL
                                    
                            if psi > 2.48:
                                next.AgentMode = AgentMode.SL
                                
                    if theta > -0.51:
                        if theta <= 1.02:
                            if psi <= -2.16:
                                if theta <= 0.32:
                                    next.AgentMode = AgentMode.SL
                                    
                                if theta > 0.32:
                                    next.AgentMode = AgentMode.SR
                                    
                            if psi > -2.16:
                                next.AgentMode = AgentMode.SR
                                
                        if theta > 1.02:
                            if psi <= 0.13:
                                next.AgentMode = AgentMode.WR
                                
                            if psi > 0.13:
                                next.AgentMode = AgentMode.COC
                                
                if rho > 8839.67:
                    if theta <= -1.02:
                        next.AgentMode = AgentMode.COC
                        
                    if theta > -1.02:
                        if theta <= 0.95:
                            if rho <= 12294.25:
                                next.AgentMode = AgentMode.WR
                                
                            if rho > 12294.25:
                                if psi <= 0.89:
                                    if psi <= -1.59:
                                        if theta <= -0.13:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if theta > -0.13:
                                            next.AgentMode = AgentMode.WR
                                            
                                    if psi > -1.59:
                                        if rho <= 29363.95:
                                            if psi <= 0.25:
                                                if theta <= 0.57:
                                                    next.AgentMode = AgentMode.COC
                                                    
                                                if theta > 0.57:
                                                    next.AgentMode = AgentMode.WR
                                                    
                                            if psi > 0.25:
                                                if theta <= -0.38:
                                                    next.AgentMode = AgentMode.WR
                                                    
                                                if theta > -0.38:
                                                    next.AgentMode = AgentMode.COC
                                                    
                                        if rho > 29363.95:
                                            next.AgentMode = AgentMode.COC
                                            
                                if psi > 0.89:
                                    if theta <= 0.19:
                                        if theta <= -0.76:
                                            if psi <= 1.59:
                                                next.AgentMode = AgentMode.WR
                                                
                                            if psi > 1.59:
                                                next.AgentMode = AgentMode.COC
                                                
                                        if theta > -0.76:
                                            next.AgentMode = AgentMode.WR
                                            
                                    if theta > 0.19:
                                        next.AgentMode = AgentMode.COC
                                        
                        if theta > 0.95:
                            if rho <= 13919.93:
                                if psi <= -0.19:
                                    if theta <= 1.9:
                                        next.AgentMode = AgentMode.WR
                                        
                                    if theta > 1.9:
                                        next.AgentMode = AgentMode.COC
                                        
                                if psi > -0.19:
                                    next.AgentMode = AgentMode.COC
                                    
                            if rho > 13919.93:
                                next.AgentMode = AgentMode.COC
                                
        
        if ego.agent_mode == AgentMode.SL: # advisory 3
            if rho <= 8839.67:
                if rho <= 4775.45:
                    if theta <= -0.44:
                        if psi <= 0.0:
                            next.AgentMode = AgentMode.SL
                            
                        if psi > 0.0:
                            if rho <= 914.45:
                                if psi <= 1.4:
                                    next.AgentMode = AgentMode.SR
                                    
                                if psi > 1.4:
                                    next.AgentMode = AgentMode.SL
                                    
                            if rho > 914.45:
                                if theta <= -0.63:
                                    next.AgentMode = AgentMode.SL
                                    
                                if theta > -0.63:
                                    if psi <= 1.84:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > 1.84:
                                        next.AgentMode = AgentMode.SL
                                        
                    if theta > -0.44:
                        if rho <= 711.24:
                            if psi <= -0.19:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > -0.19:
                                if theta <= 1.02:
                                    if psi <= 2.16:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > 2.16:
                                        next.AgentMode = AgentMode.SL
                                        
                                if theta > 1.02:
                                    next.AgentMode = AgentMode.SL
                                    
                        if rho > 711.24:
                            if theta <= 0.13:
                                if psi <= -0.06:
                                    next.AgentMode = AgentMode.SL
                                    
                                if psi > -0.06:
                                    if psi <= 2.67:
                                        if psi <= 0.63:
                                            if rho <= 2540.13:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if rho > 2540.13:
                                                next.AgentMode = AgentMode.SL
                                                
                                        if psi > 0.63:
                                            next.AgentMode = AgentMode.SR
                                            
                                    if psi > 2.67:
                                        next.AgentMode = AgentMode.SL
                                        
                            if theta > 0.13:
                                if theta <= 2.35:
                                    if theta <= 0.51:
                                        if psi <= -0.44:
                                            if psi <= -2.41:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if psi > -2.41:
                                                next.AgentMode = AgentMode.SL
                                                
                                        if psi > -0.44:
                                            next.AgentMode = AgentMode.SR
                                            
                                    if theta > 0.51:
                                        if psi <= 0.38:
                                            if rho <= 1320.87:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if rho > 1320.87:
                                                if psi <= -1.08:
                                                    if theta <= 1.65:
                                                        next.AgentMode = AgentMode.SR
                                                        
                                                    if theta > 1.65:
                                                        next.AgentMode = AgentMode.WL
                                                        
                                                if psi > -1.08:
                                                    next.AgentMode = AgentMode.SR
                                                    
                                        if psi > 0.38:
                                            if theta <= 1.14:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if theta > 1.14:
                                                next.AgentMode = AgentMode.WL
                                                
                                if theta > 2.35:
                                    if rho <= 1524.08:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if rho > 1524.08:
                                        next.AgentMode = AgentMode.SR
                                        
                if rho > 4775.45:
                    if theta <= 0.95:
                        if theta <= -0.32:
                            if theta <= -1.9:
                                next.AgentMode = AgentMode.COC
                                
                            if theta > -1.9:
                                if psi <= 0.83:
                                    if psi <= -0.38:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if psi > -0.38:
                                        next.AgentMode = AgentMode.WL
                                        
                                if psi > 0.83:
                                    next.AgentMode = AgentMode.SL
                                    
                        if theta > -0.32:
                            if psi <= 1.65:
                                if theta <= 0.38:
                                    if psi <= -2.67:
                                        if theta <= 0.06:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if theta > 0.06:
                                            next.AgentMode = AgentMode.SR
                                            
                                    if psi > -2.67:
                                        if psi <= -1.21:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if psi > -1.21:
                                            next.AgentMode = AgentMode.WL
                                            
                                if theta > 0.38:
                                    if psi <= -1.08:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > -1.08:
                                        next.AgentMode = AgentMode.SR
                                        
                            if psi > 1.65:
                                next.AgentMode = AgentMode.SR
                                
                    if theta > 0.95:
                        if psi <= -0.89:
                            if theta <= 1.78:
                                next.AgentMode = AgentMode.SR
                                
                            if theta > 1.78:
                                next.AgentMode = AgentMode.COC
                                
                        if psi > -0.89:
                            if theta <= 1.4:
                                if psi <= -0.25:
                                    next.AgentMode = AgentMode.WR
                                    
                                if psi > -0.25:
                                    next.AgentMode = AgentMode.COC
                                    
                            if theta > 1.4:
                                if psi <= -0.38:
                                    next.AgentMode = AgentMode.SR
                                    
                                if psi > -0.38:
                                    next.AgentMode = AgentMode.COC
                                    
            if rho > 8839.67:
                if theta <= -1.4:
                    next.AgentMode = AgentMode.COC
                    
                if theta > -1.4:
                    if theta <= 1.14:
                        if rho <= 13919.93:
                            if psi <= -1.59:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > -1.59:
                                if psi <= 2.09:
                                    next.AgentMode = AgentMode.WL
                                    
                                if psi > 2.09:
                                    next.AgentMode = AgentMode.SL
                                    
                        if rho > 13919.93:
                            if rho <= 35257.06:
                                if psi <= 0.51:
                                    if theta <= -0.19:
                                        if psi <= -1.27:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if psi > -1.27:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if theta > -0.19:
                                        if psi <= -0.32:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if psi > -0.32:
                                            next.AgentMode = AgentMode.COC
                                            
                                if psi > 0.51:
                                    if theta <= 0.25:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if theta > 0.25:
                                        next.AgentMode = AgentMode.COC
                                        
                            if rho > 35257.06:
                                if theta <= -1.02:
                                    next.AgentMode = AgentMode.COC
                                    
                                if theta > -1.02:
                                    if psi <= 1.21:
                                        if psi <= -0.7:
                                            if theta <= -0.19:
                                                next.AgentMode = AgentMode.COC
                                                
                                            if theta > -0.19:
                                                next.AgentMode = AgentMode.WL
                                                
                                        if psi > -0.7:
                                            if theta <= -0.76:
                                                next.AgentMode = AgentMode.WL
                                                
                                            if theta > -0.76:
                                                next.AgentMode = AgentMode.COC
                                                
                                    if psi > 1.21:
                                        if theta <= 0.19:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if theta > 0.19:
                                            next.AgentMode = AgentMode.COC
                                            
                    if theta > 1.14:
                        if theta <= 1.9:
                            if rho <= 39727.69:
                                next.AgentMode = AgentMode.COC
                                
                            if rho > 39727.69:
                                if psi <= 0.06:
                                    if psi <= -0.95:
                                        next.AgentMode = AgentMode.COC
                                        
                                    if psi > -0.95:
                                        next.AgentMode = AgentMode.WL
                                        
                                if psi > 0.06:
                                    next.AgentMode = AgentMode.COC
                                    
                        if theta > 1.9:
                            next.AgentMode = AgentMode.COC
                            

        
        if ego.agent_mode == AgentMode.SR: # advisory 4
            if rho <= 11074.98:
                if theta <= -1.08:
                    if rho <= 4978.66:
                        if rho <= 1524.08:
                            if psi <= 0.51:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > 0.51:
                                next.AgentMode = AgentMode.SR
                                
                        if rho > 1524.08:
                            if psi <= -0.19:
                                if theta <= -1.59:
                                    next.AgentMode = AgentMode.COC
                                    
                                if theta > -1.59:
                                    next.AgentMode = AgentMode.WR
                                    
                            if psi > -0.19:
                                if psi <= 1.52:
                                    if theta <= -2.35:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if theta > -2.35:
                                        next.AgentMode = AgentMode.SL
                                        
                                if psi > 1.52:
                                    if theta <= -1.46:
                                        next.AgentMode = AgentMode.WR
                                        
                                    if theta > -1.46:
                                        next.AgentMode = AgentMode.SL
                                        
                    if rho > 4978.66:
                        if theta <= -1.84:
                            next.AgentMode = AgentMode.COC
                            
                        if theta > -1.84:
                            if psi <= 1.14:
                                if psi <= 0.19:
                                    next.AgentMode = AgentMode.COC
                                    
                                if psi > 0.19:
                                    if rho <= 5791.51:
                                        if psi <= 0.32:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if psi > 0.32:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if rho > 5791.51:
                                        if psi <= 0.76:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if psi > 0.76:
                                            if rho <= 7417.19:
                                                next.AgentMode = AgentMode.SL
                                                
                                            if rho > 7417.19:
                                                next.AgentMode = AgentMode.WL
                                                
                            if psi > 1.14:
                                next.AgentMode = AgentMode.SL
                                
                if theta > -1.08:
                    if theta <= 0.51:
                        if psi <= -0.06:
                            if theta <= -0.57:
                                if rho <= 4978.66:
                                    if rho <= 1727.29:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if rho > 1727.29:
                                        if psi <= -2.41:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if psi > -2.41:
                                            next.AgentMode = AgentMode.WR
                                            
                                if rho > 4978.66:
                                    next.AgentMode = AgentMode.COC
                                    
                            if theta > -0.57:
                                if theta <= 0.25:
                                    next.AgentMode = AgentMode.SL
                                    
                                if theta > 0.25:
                                    if rho <= 7213.98:
                                        if psi <= -2.73:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if psi > -2.73:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if rho > 7213.98:
                                        next.AgentMode = AgentMode.SR
                                        
                        if psi > -0.06:
                            if theta <= -0.25:
                                if rho <= 1524.08:
                                    if psi <= 0.32:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if psi > 0.32:
                                        next.AgentMode = AgentMode.SR
                                        
                                if rho > 1524.08:
                                    if theta <= -1.02:
                                        if psi <= 0.57:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if psi > 0.57:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if theta > -1.02:
                                        if theta <= -0.57:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if theta > -0.57:
                                            if psi <= 2.28:
                                                if psi <= 0.57:
                                                    next.AgentMode = AgentMode.SL
                                                    
                                                if psi > 0.57:
                                                    next.AgentMode = AgentMode.SR
                                                    
                                            if psi > 2.28:
                                                next.AgentMode = AgentMode.SL
                                                
                            if theta > -0.25:
                                if theta <= -0.13:
                                    if psi <= 2.79:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > 2.79:
                                        next.AgentMode = AgentMode.SL
                                        
                                if theta > -0.13:
                                    next.AgentMode = AgentMode.SR
                                    
                    if theta > 0.51:
                        if rho <= 2743.34:
                            if psi <= -0.32:
                                if psi <= -1.71:
                                    next.AgentMode = AgentMode.SR
                                    
                                if psi > -1.71:
                                    if theta <= 0.89:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if theta > 0.89:
                                        if rho <= 1117.66:
                                            if psi <= -1.08:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if psi > -1.08:
                                                next.AgentMode = AgentMode.SL
                                                
                                        if rho > 1117.66:
                                            next.AgentMode = AgentMode.SR
                                            
                            if psi > -0.32:
                                next.AgentMode = AgentMode.SR
                                
                        if rho > 2743.34:
                            if psi <= 0.57:
                                if rho <= 7417.19:
                                    if theta <= 1.4:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if theta > 1.4:
                                        next.AgentMode = AgentMode.SR
                                        
                                if rho > 7417.19:
                                    if psi <= -1.59:
                                        if theta <= 1.21:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if theta > 1.21:
                                            next.AgentMode = AgentMode.COC
                                            
                                    if psi > -1.59:
                                        next.AgentMode = AgentMode.WR
                                        
                            if psi > 0.57:
                                if rho <= 4775.45:
                                    next.AgentMode = AgentMode.WR
                                    
                                if rho > 4775.45:
                                    next.AgentMode = AgentMode.COC
                                    
            if rho > 11074.98:
                if theta <= -1.27:
                    next.AgentMode = AgentMode.COC
                    
                if theta > -1.27:
                    if theta <= 1.33:
                        if theta <= 0.51:
                            if psi <= 0.57:
                                if theta <= -0.19:
                                    next.AgentMode = AgentMode.COC
                                    
                                if theta > -0.19:
                                    if psi <= -1.21:
                                        if rho <= 15748.83:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if rho > 15748.83:
                                            next.AgentMode = AgentMode.WR
                                            
                                    if psi > -1.21:
                                        if rho <= 35053.85:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if rho > 35053.85:
                                            if psi <= -0.38:
                                                next.AgentMode = AgentMode.COC
                                                
                                            if psi > -0.38:
                                                next.AgentMode = AgentMode.SR
                                                
                            if psi > 0.57:
                                if theta <= 0.13:
                                    if theta <= 0.0:
                                        if theta <= -0.7:
                                            if psi <= 1.9:
                                                next.AgentMode = AgentMode.WR
                                                
                                            if psi > 1.9:
                                                next.AgentMode = AgentMode.COC
                                                
                                        if theta > -0.7:
                                            if psi <= 1.4:
                                                next.AgentMode = AgentMode.COC
                                                
                                            if psi > 1.4:
                                                next.AgentMode = AgentMode.WR
                                                
                                    if theta > 0.0:
                                        if psi <= 2.54:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if psi > 2.54:
                                            next.AgentMode = AgentMode.WR
                                            
                                if theta > 0.13:
                                    next.AgentMode = AgentMode.COC
                                    
                        if theta > 0.51:
                            if psi <= -0.25:
                                if psi <= -2.22:
                                    next.AgentMode = AgentMode.COC
                                    
                                if psi > -2.22:
                                    if psi <= -0.89:
                                        if theta <= 1.02:
                                            next.AgentMode = AgentMode.WR
                                            
                                        if theta > 1.02:
                                            next.AgentMode = AgentMode.COC
                                            
                                    if psi > -0.89:
                                        if theta <= 0.95:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if theta > 0.95:
                                            next.AgentMode = AgentMode.WR
                                            
                            if psi > -0.25:
                                if psi <= 0.76:
                                    if rho <= 20625.89:
                                        next.AgentMode = AgentMode.COC
                                        
                                    if rho > 20625.89:
                                        next.AgentMode = AgentMode.SR
                                        
                                if psi > 0.76:
                                    next.AgentMode = AgentMode.COC
                                    
                    if theta > 1.33:
                        if rho <= 19203.41:
                            if psi <= 0.25:
                                if psi <= -0.7:
                                    next.AgentMode = AgentMode.COC
                                    
                                if psi > -0.7:
                                    next.AgentMode = AgentMode.WR
                                    
                            if psi > 0.25:
                                next.AgentMode = AgentMode.COC
                                
                        if rho > 19203.41:
                            next.AgentMode = AgentMode.COC
                            
    '''
    # Variation that takes 6 min to run
    if True:
        if ego.agent_mode == AgentMode.COC:  # advisory 0
            if rho <= 15545.62:
                if theta <= -0.13:
                    if rho <= 1727.29:
                        if psi <= 0.13:
                            if rho <= 1320.87:
                                next.AgentMode = AgentMode.SL
                                
                            if rho > 1320.87:
                                if theta <= -1.78:
                                    next.AgentMode = AgentMode.WL
                                    
                                if theta > -1.78:
                                    next.AgentMode = AgentMode.SL
                                    
                        if psi > 0.13:
                            if rho <= 711.24:
                                next.AgentMode = AgentMode.SR
                                
                            if rho > 711.24:
                                next.AgentMode = AgentMode.SL
                                
                    if rho > 1727.29:
                        if theta <= -0.83:
                            if psi <= -0.25:
                                if rho <= 5385.08:
                                    next.AgentMode = AgentMode.WL
                                    
                                if rho > 5385.08:
                                    next.AgentMode = AgentMode.COC
                                    
                            if psi > -0.25:
                                if theta <= -2.73:
                                    if psi <= 0.38:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if psi > 0.38:
                                        next.AgentMode = AgentMode.WR
                                        
                                if theta > -2.73:
                                    if rho <= 6197.93:
                                        if psi <= 1.9:
                                            if theta <= -2.03:
                                                next.AgentMode = AgentMode.WL
                                                
                                            if theta > -2.03:
                                                if rho <= 4165.82:
                                                    next.AgentMode = AgentMode.SL
                                                    
                                                if rho > 4165.82:
                                                    if psi <= 0.7:
                                                        next.AgentMode = AgentMode.WL
                                                        
                                                    if psi > 0.7:
                                                        next.AgentMode = AgentMode.SL
                                                        
                                        if psi > 1.9:
                                            next.AgentMode = AgentMode.WL
                                            
                                    if rho > 6197.93:
                                        next.AgentMode = AgentMode.WL
                                        
                        if theta > -0.83:
                            if rho <= 4572.24:
                                if psi <= 0.76:
                                    next.AgentMode = AgentMode.SL
                                    
                                if psi > 0.76:
                                    if theta <= -0.44:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if theta > -0.44:
                                        next.AgentMode = AgentMode.SR
                                        
                            if rho > 4572.24:
                                if theta <= -0.38:
                                    if rho <= 9449.3:
                                        if psi <= 1.08:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if psi > 1.08:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if rho > 9449.3:
                                        next.AgentMode = AgentMode.WL
                                        
                                if theta > -0.38:
                                    if psi <= 0.83:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if psi > 0.83:
                                        next.AgentMode = AgentMode.SR
                                        
                if theta > -0.13:
                    if rho <= 1727.29:
                        if psi <= -0.19:
                            if rho <= 508.03:
                                next.AgentMode = AgentMode.SL
                                
                            if rho > 508.03:
                                if theta <= 0.44:
                                    next.AgentMode = AgentMode.SL
                                    
                                if theta > 0.44:
                                    if theta <= 2.73:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if theta > 2.73:
                                        next.AgentMode = AgentMode.SL
                                        
                        if psi > -0.19:
                            next.AgentMode = AgentMode.SR
                            
                    if rho > 1727.29:
                        if theta <= 1.33:
                            if rho <= 5181.87:
                                if psi <= -0.32:
                                    if theta <= 0.44:
                                        next.AgentMode = AgentMode.SL
                                        
                                    if theta > 0.44:
                                        next.AgentMode = AgentMode.SR
                                        
                                if psi > -0.32:
                                    if theta <= 0.57:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if theta > 0.57:
                                        if rho <= 3556.19:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if rho > 3556.19:
                                            next.AgentMode = AgentMode.WR
                                            
                            if rho > 5181.87:
                                if psi <= -1.08:
                                    if theta <= 0.38:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if theta > 0.38:
                                        if rho <= 8026.82:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if rho > 8026.82:
                                            next.AgentMode = AgentMode.WR
                                            
                                if psi > -1.08:
                                    next.AgentMode = AgentMode.WR
                                    
                        if theta > 1.33:
                            if rho <= 8636.46:
                                if psi <= 0.38:
                                    next.AgentMode = AgentMode.WR
                                    
                                if psi > 0.38:
                                    if rho <= 4369.03:
                                        next.AgentMode = AgentMode.WR
                                        
                                    if rho > 4369.03:
                                        next.AgentMode = AgentMode.COC
                                        
                            if rho > 8636.46:
                                next.AgentMode = AgentMode.COC
                                
            if rho > 15545.62:
                if rho <= 18593.78:
                    if psi <= 2.03:
                        next.AgentMode = AgentMode.COC
                        
                    if psi > 2.03:
                        next.AgentMode = AgentMode.WL
                        
                if rho > 18593.78:
                    next.AgentMode = AgentMode.COC
                    

        
        if ego.agent_mode == AgentMode.WL: # advisory 1
            if psi <= -2.79:
                if theta <= 0.32:
                    if rho <= 13513.51:
                        if theta <= 0.06:
                            next.AgentMode = AgentMode.SL
                            
                        if theta > 0.06:
                            next.AgentMode = AgentMode.SR
                            
                    if rho > 13513.51:
                        if theta <= -0.32:
                            next.AgentMode = AgentMode.COC
                            
                        if theta > -0.32:
                            next.AgentMode = AgentMode.WL
                            
                if theta > 0.32:
                    if rho <= 10058.93:
                        if theta <= 1.27:
                            next.AgentMode = AgentMode.SR
                            
                        if theta > 1.27:
                            next.AgentMode = AgentMode.SL
                            
                    if rho > 10058.93:
                        if rho <= 10871.77:
                            next.AgentMode = AgentMode.WR
                            
                        if rho > 10871.77:
                            next.AgentMode = AgentMode.COC
                            
            if psi > -2.79:
                if rho <= 9042.88:
                    if theta <= -0.51:
                        if rho <= 1930.5:
                            if psi <= 0.13:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > 0.13:
                                if psi <= 1.46:
                                    next.AgentMode = AgentMode.SR
                                    
                                if psi > 1.46:
                                    next.AgentMode = AgentMode.SL
                                    
                        if rho > 1930.5:
                            if psi <= -0.57:
                                next.AgentMode = AgentMode.WL
                                
                            if psi > -0.57:
                                if theta <= -1.33:
                                    if rho <= 3556.19:
                                        if psi <= 1.78:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if psi > 1.78:
                                            next.AgentMode = AgentMode.WL
                                            
                                    if rho > 3556.19:
                                        next.AgentMode = AgentMode.WL
                                        
                                if theta > -1.33:
                                    next.AgentMode = AgentMode.SL
                                    
                    if theta > -0.51:
                        if psi <= -0.25:
                            if theta <= 0.38:
                                if rho <= 4165.82:
                                    next.AgentMode = AgentMode.SL
                                    
                                if rho > 4165.82:
                                    next.AgentMode = AgentMode.SL
                                    
                            if theta > 0.38:
                                if rho <= 1524.08:
                                    next.AgentMode = AgentMode.SL
                                    
                                if rho > 1524.08:
                                    if theta <= 2.03:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if theta > 2.03:
                                        next.AgentMode = AgentMode.WL
                                        
                        if psi > -0.25:
                            if theta <= 1.21:
                                if theta <= -0.19:
                                    if psi <= 2.54:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > 2.54:
                                        next.AgentMode = AgentMode.SL
                                        
                                if theta > -0.19:
                                    next.AgentMode = AgentMode.SR
                                    
                            if theta > 1.21:
                                if rho <= 1930.5:
                                    if psi <= 0.44:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > 0.44:
                                        next.AgentMode = AgentMode.SL
                                        
                                if rho > 1930.5:
                                    next.AgentMode = AgentMode.COC
                                    
                if rho > 9042.88:
                    if theta <= 0.76:
                        if theta <= -1.08:
                            if rho <= 13513.51:
                                next.AgentMode = AgentMode.COC
                                
                            if rho > 13513.51:
                                next.AgentMode = AgentMode.COC
                                
                        if theta > -1.08:
                            if psi <= 0.57:
                                if psi <= -1.33:
                                    if theta <= -0.19:
                                        next.AgentMode = AgentMode.COC
                                        
                                    if theta > -0.19:
                                        if rho <= 13107.09:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if rho > 13107.09:
                                            next.AgentMode = AgentMode.WL
                                            
                                if psi > -1.33:
                                    next.AgentMode = AgentMode.COC
                                    
                            if psi > 0.57:
                                if theta <= 0.06:
                                    next.AgentMode = AgentMode.WL
                                    
                                if theta > 0.06:
                                    next.AgentMode = AgentMode.COC
                                    
                    if theta > 0.76:
                        next.AgentMode = AgentMode.COC
                        

            
        if ego.agent_mode == AgentMode.WR: # advisory 2
            if rho <= 7213.98:
                if theta <= 0.44:
                    if theta <= -2.22:
                        if rho <= 1524.08:
                            next.AgentMode = AgentMode.SR
                            
                        if rho > 1524.08:
                            next.AgentMode = AgentMode.WR
                            
                    if theta > -2.22:
                        if theta <= -0.63:
                            if rho <= 1117.66:
                                next.AgentMode = AgentMode.SR
                                
                            if rho > 1117.66:
                                if psi <= -0.13:
                                    next.AgentMode = AgentMode.WR
                                    
                                if psi > -0.13:
                                    next.AgentMode = AgentMode.SL
                                    
                        if theta > -0.63:
                            if psi <= -0.19:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > -0.19:
                                if theta <= -0.32:
                                    if psi <= 2.03:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if psi > 2.03:
                                        next.AgentMode = AgentMode.SL
                                        
                                if theta > -0.32:
                                    next.AgentMode = AgentMode.SR
                                    
                if theta > 0.44:
                    if rho <= 1727.29:
                        if psi <= -0.19:
                            if psi <= -1.84:
                                next.AgentMode = AgentMode.SR
                                
                            if psi > -1.84:
                                if theta <= 0.83:
                                    next.AgentMode = AgentMode.SL
                                    
                                if theta > 0.83:
                                    if theta <= 2.48:
                                        next.AgentMode = AgentMode.SR
                                        
                                    if theta > 2.48:
                                        next.AgentMode = AgentMode.SL
                                        
                        if psi > -0.19:
                            next.AgentMode = AgentMode.SR
                            
                    if rho > 1727.29:
                        if theta <= 1.52:
                            if psi <= 0.0:
                                next.AgentMode = AgentMode.SR
                                
                            if psi > 0.0:
                                next.AgentMode = AgentMode.WR
                                
                        if theta > 1.52:
                            next.AgentMode = AgentMode.WR
                            
            if rho > 7213.98:
                if rho <= 8839.67:
                    if theta <= -0.51:
                        if theta <= -0.83:
                            next.AgentMode = AgentMode.COC
                            
                        if theta > -0.83:
                            next.AgentMode = AgentMode.WL
                            
                    if theta > -0.51:
                        if theta <= 1.02:
                            next.AgentMode = AgentMode.SR
                            
                        if theta > 1.02:
                            next.AgentMode = AgentMode.WR
                            
                if rho > 8839.67:
                    if theta <= -1.02:
                        next.AgentMode = AgentMode.COC
                        
                    if theta > -1.02:
                        if theta <= 0.95:
                            if rho <= 12294.25:
                                next.AgentMode = AgentMode.WR
                                
                            if rho > 12294.25:
                                if psi <= 0.89:
                                    if psi <= -1.59:
                                        if theta <= -0.13:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if theta > -0.13:
                                            next.AgentMode = AgentMode.WR
                                            
                                    if psi > -1.59:
                                        next.AgentMode = AgentMode.COC
                                        
                                if psi > 0.89:
                                    if theta <= 0.19:
                                        next.AgentMode = AgentMode.WR
                                        
                                    if theta > 0.19:
                                        next.AgentMode = AgentMode.COC
                                        
                        if theta > 0.95:
                            if rho <= 13919.93:
                                if psi <= -0.19:
                                    next.AgentMode = AgentMode.WR
                                    
                                if psi > -0.19:
                                    next.AgentMode = AgentMode.COC
                                    
                            if rho > 13919.93:
                                next.AgentMode = AgentMode.COC
                                

        if ego.agent_mode == AgentMode.SL: # advisory 3
            if rho <= 8839.67:
                if rho <= 4775.45:
                    if theta <= -0.44:
                        if psi <= 0.0:
                            next.AgentMode = AgentMode.SL
                            
                        if psi > 0.0:
                            if rho <= 914.45:
                                if psi <= 1.4:
                                    next.AgentMode = AgentMode.SR
                                    
                                if psi > 1.4:
                                    next.AgentMode = AgentMode.SL
                                    
                            if rho > 914.45:
                                if theta <= -0.63:
                                    next.AgentMode = AgentMode.SL
                                    
                                if theta > -0.63:
                                    next.AgentMode = AgentMode.SR
                                    
                    if theta > -0.44:
                        if rho <= 711.24:
                            if psi <= -0.19:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > -0.19:
                                if theta <= 1.02:
                                    next.AgentMode = AgentMode.SR
                                    
                                if theta > 1.02:
                                    next.AgentMode = AgentMode.SL
                                    
                        if rho > 711.24:
                            if theta <= 0.13:
                                if psi <= -0.06:
                                    next.AgentMode = AgentMode.SL
                                    
                                if psi > -0.06:
                                    next.AgentMode = AgentMode.SR
                                    
                            if theta > 0.13:
                                if theta <= 2.35:
                                    if theta <= 0.51:
                                        if psi <= -0.44:
                                            if psi <= -2.41:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if psi > -2.41:
                                                next.AgentMode = AgentMode.SL
                                                
                                        if psi > -0.44:
                                            next.AgentMode = AgentMode.SR
                                            
                                    if theta > 0.51:
                                        next.AgentMode = AgentMode.SR
                                        
                                if theta > 2.35:
                                    next.AgentMode = AgentMode.SL
                                    
                if rho > 4775.45:
                    if theta <= 0.95:
                        if theta <= -0.32:
                            if theta <= -1.9:
                                next.AgentMode = AgentMode.COC
                                
                            if theta > -1.9:
                                next.AgentMode = AgentMode.SL
                                
                        if theta > -0.32:
                            if psi <= 1.65:
                                if theta <= 0.38:
                                    next.AgentMode = AgentMode.SL
                                    
                                if theta > 0.38:
                                    next.AgentMode = AgentMode.SR
                                    
                            if psi > 1.65:
                                next.AgentMode = AgentMode.SR
                                
                    if theta > 0.95:
                        if psi <= -0.89:
                            next.AgentMode = AgentMode.SR
                            
                        if psi > -0.89:
                            if theta <= 1.4:
                                next.AgentMode = AgentMode.WR
                                
                            if theta > 1.4:
                                next.AgentMode = AgentMode.COC
                                
            if rho > 8839.67:
                if theta <= -1.4:
                    next.AgentMode = AgentMode.COC
                    
                if theta > -1.4:
                    if theta <= 1.14:
                        if rho <= 13919.93:
                            if psi <= -1.59:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > -1.59:
                                if psi <= 2.09:
                                    next.AgentMode = AgentMode.WL
                                    
                                if psi > 2.09:
                                    next.AgentMode = AgentMode.SL
                                    
                        if rho > 13919.93:
                            if rho <= 35257.06:
                                if psi <= 0.51:
                                    if theta <= -0.19:
                                        if psi <= -1.27:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if psi > -1.27:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if theta > -0.19:
                                        if psi <= -0.32:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if psi > -0.32:
                                            next.AgentMode = AgentMode.COC
                                            
                                if psi > 0.51:
                                    if theta <= 0.25:
                                        next.AgentMode = AgentMode.WL
                                        
                                    if theta > 0.25:
                                        next.AgentMode = AgentMode.COC
                                        
                            if rho > 35257.06:
                                if theta <= -1.02:
                                    next.AgentMode = AgentMode.COC
                                    
                                if theta > -1.02:
                                    if psi <= 1.21:
                                        if psi <= -0.7:
                                            if theta <= -0.19:
                                                next.AgentMode = AgentMode.COC
                                                
                                            if theta > -0.19:
                                                next.AgentMode = AgentMode.WL
                                                
                                        if psi > -0.7:
                                            if theta <= -0.76:
                                                next.AgentMode = AgentMode.WL
                                                
                                            if theta > -0.76:
                                                next.AgentMode = AgentMode.COC
                                                
                                    if psi > 1.21:
                                        if theta <= 0.19:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if theta > 0.19:
                                            next.AgentMode = AgentMode.COC
                                            
                    if theta > 1.14:
                        if theta <= 1.9:
                            if rho <= 39727.69:
                                next.AgentMode = AgentMode.COC
                                
                            if rho > 39727.69:
                                if psi <= 0.06:
                                    if psi <= -0.95:
                                        next.AgentMode = AgentMode.COC
                                        
                                    if psi > -0.95:
                                        next.AgentMode = AgentMode.WL
                                        
                                if psi > 0.06:
                                    next.AgentMode = AgentMode.COC
                                    
                        if theta > 1.9:
                            next.AgentMode = AgentMode.COC
                            

            
        if ego.agent_mode == AgentMode.SR: # advisory 4
            if rho <= 11074.98:
                if theta <= -1.08:
                    if rho <= 4978.66:
                        if rho <= 1524.08:
                            if psi <= 0.51:
                                next.AgentMode = AgentMode.SL
                                
                            if psi > 0.51:
                                next.AgentMode = AgentMode.SR
                                
                        if rho > 1524.08:
                            if psi <= -0.19:
                                next.AgentMode = AgentMode.COC
                                
                            if psi > -0.19:
                                if psi <= 1.52:
                                    next.AgentMode = AgentMode.SL
                                    
                                if psi > 1.52:
                                    next.AgentMode = AgentMode.SL
                                    
                    if rho > 4978.66:
                        if theta <= -1.84:
                            next.AgentMode = AgentMode.COC
                            
                        if theta > -1.84:
                            if psi <= 1.14:
                                if psi <= 0.19:
                                    next.AgentMode = AgentMode.COC
                                    
                                if psi > 0.19:
                                    if rho <= 5791.51:
                                        if psi <= 0.32:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if psi > 0.32:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if rho > 5791.51:
                                        next.AgentMode = AgentMode.WL
                                        
                            if psi > 1.14:
                                next.AgentMode = AgentMode.SL
                                
                if theta > -1.08:
                    if theta <= 0.51:
                        if psi <= -0.06:
                            next.AgentMode = AgentMode.SL
                            
                        if psi > -0.06:
                            if theta <= -0.25:
                                if rho <= 1524.08:
                                    next.AgentMode = AgentMode.SR
                                    
                                if rho > 1524.08:
                                    if theta <= -1.02:
                                        if psi <= 0.57:
                                            next.AgentMode = AgentMode.WL
                                            
                                        if psi > 0.57:
                                            next.AgentMode = AgentMode.SL
                                            
                                    if theta > -1.02:
                                        if theta <= -0.57:
                                            next.AgentMode = AgentMode.SL
                                            
                                        if theta > -0.57:
                                            if psi <= 2.28:
                                                next.AgentMode = AgentMode.SR
                                                
                                            if psi > 2.28:
                                                next.AgentMode = AgentMode.SL
                                                
                            if theta > -0.25:
                                next.AgentMode = AgentMode.SR
                                
                    if theta > 0.51:
                        if rho <= 2743.34:
                            if psi <= -0.32:
                                if psi <= -1.71:
                                    next.AgentMode = AgentMode.SR
                                    
                                if psi > -1.71:
                                    next.AgentMode = AgentMode.SL
                                    
                            if psi > -0.32:
                                next.AgentMode = AgentMode.SR
                                
                        if rho > 2743.34:
                            if psi <= 0.57:
                                next.AgentMode = AgentMode.SR
                                
                            if psi > 0.57:
                                if rho <= 4775.45:
                                    next.AgentMode = AgentMode.WR
                                    
                                if rho > 4775.45:
                                    next.initial_mode[0] = AgentMode.COC
                                    
            if rho > 11074.98:
                if theta <= -1.27:
                    next.AgentMode = AgentMode.COC
                    
                if theta > -1.27:
                    if theta <= 1.33:
                        if theta <= 0.51:
                            if psi <= 0.57:
                                if theta <= -0.19:
                                    next.AgentMode = AgentMode.COC
                                    
                                if theta > -0.19:
                                    if psi <= -1.21:
                                        if rho <= 15748.83:
                                            next.AgentMode = AgentMode.SR
                                            
                                        if rho > 15748.83:
                                            next.AgentMode = AgentMode.WR
                                            
                                    if psi > -1.21:
                                        if rho <= 35053.85:
                                            next.AgentMode = AgentMode.COC
                                            
                                        if rho > 35053.85:
                                            next.AgentMode = AgentMode.SR
                                            
                            if psi > 0.57:
                                if theta <= 0.13:
                                    if theta <= 0.0:
                                        if theta <= -0.7:
                                            if psi <= 1.9:
                                                next.AgentMode = AgentMode.WR
                                                
                                            if psi > 1.9:
                                                next.AgentMode = AgentMode.COC
                                                
                                        if theta > -0.7:
                                            if psi <= 1.4:
                                                next.AgentMode = AgentMode.COC
                                                
                                            if psi > 1.4:
                                                next.AgentMode = AgentMode.WR
                                                
                                    if theta > 0.0:
                                        next.AgentMode = AgentMode.WR
                                        
                                if theta > 0.13:
                                    next.AgentMode = AgentMode.COC
                                    
                        if theta > 0.51:
                            if psi <= -0.25:
                                if psi <= -2.22:
                                    next.AgentMode = AgentMode.COC
                                    
                                if psi > -2.22:
                                    next.AgentMode = AgentMode.WR
                                    
                            if psi > -0.25:
                                if psi <= 0.76:
                                    next.AgentMode = AgentMode.SR
                                    
                                if psi > 0.76:
                                    next.AgentMode = AgentMode.COC
                                    
                    if theta > 1.33:
                        next.AgentMode = AgentMode.COC
                        

    return next