from dubins_agent import CarAgent, NPCAgent
from verse.map.example_map.map_tacas import M1
from verse.scenario.scenario import Benchmark
from enum import Enum, auto
from verse.plotter.plotter2D import *
from verse import Scenario, ScenarioConfig
from verse.analysis.verifier import ReachabilityMethod
import sys
import plotly.graph_objects as go
import torch
from auto_LiRPA import BoundedTensor
from verse.utils.utils import wrap_to_pi
import numpy as np 
import torch

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

def get_acas_state(own_state: np.ndarray, int_state: np.ndarray) -> torch.Tensor:
    dist = np.sqrt((own_state[0]-int_state[0])**2+(own_state[1]-int_state[1])**2)
    theta = wrap_to_pi((2*np.pi-own_state[2])+np.arctan2(int_state[1], int_state[0]))
    psi = wrap_to_pi(int_state[2]-own_state[2])
    return torch.tensor([dist, theta, psi, own_state[3], int_state[3]])

def get_final_states_sim(trace) -> Tuple[List]: 
    n = trace.nodes[0]
    own_state = n.trace['car1'][-1]
    int_state = n.trace['car2'][-1]
    return own_state, int_state

if __name__ == "__main__":
    import os
    script_dir = os.path.realpath(os.path.dirname(__file__))
    input_code_name = os.path.join(script_dir, "controller.py")
    car = CarAgent('car1', file_name=input_code_name)
    car2 = NPCAgent('car2')
    scenario = Scenario(ScenarioConfig(parallel=False))
    car.set_initial(
        initial_state=[[0, -0.5, 0, 1.0], [0.01, 0.5, 0, 1.0]],
        # initial_state=[[0, -0.5, np.pi, 1.0], [0.01, 0.5, np.pi, 1.0]],
        initial_mode=(AgentMode.SR, TrackMode.T1)
    )
    car2.set_initial(
        initial_state=[[15, 15, 0, 0.5], [15, 15, 0, 0.5]],
        # initial_state=[[15, -0.3, 0, 0.5], [15, 0.3, 0, 0.5]],
        initial_mode=(AgentMode.COC, TrackMode.T1)
    )
    scenario.add_agent(car)
    scenario.add_agent(car2)
    trace = scenario.simulate(0.2, 0.1)
    own_state, int_state = get_final_states_sim(trace)
    print(own_state, int_state)
    acas_state = get_acas_state(own_state[1:], int_state[1:])

    net = 0
    model = torch.load(f"../ACASXU_run2a_{net + 1}_1_batch_2000.pth")
    print(model(acas_state))
    exit()
    # trace = scenario.verify(0.2,0.1) # increasing ts to 0.1 to increase learning speed, do the same for dryvr2
    # fig = reachtube_tree(trace) 
    # fig.show() 