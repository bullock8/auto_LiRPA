# Creating Custom Agent
In this tutorial, we are going to describe how to create an agent for Verse. 

## `BaseAgent` class explained
A nice example for all the necessary fields for a Verse agent is the `BaseAgent` class. The `BaseAgent` class holds the following members
- `id` is a `str` identifier for the agent. It is used by interal algorithm of Verse to distinguish between different agents in the scenario. 
- `decision_logic` is the parsed decision logic of the agent generated by Verse {doc}`parser<parser>`. It have to have type `ControllerIR`. The `decision_logic` can be generated using `ControllerIR.parse` given a user defined decision logic, or can be be an trivial decision logic generated by `ControllerIR.empty()`
- `init_cont` is the set of initial continuous states for the agent. It will be used by Verse to initialize the scenario.  
- `init_disc` is the initial mode of the agent. It will be used by Verse to initialize the scenario. 
- `static_parameters` is a set of parameters that will remain constant over the whole execution of the scenario. 
- `uncertain_parameters` is the set of uncertain parameters in the agent dynamics. It is used by Verse's algorithm for handling uncertainty in dynamics. 

Besides the following functions, the `BaseAgent` also contains the following member functions
- `TC_simulate` is the most import function in any agent class. As mentioned in {doc}`Agent<agent>`, it describe how the continuous state evolve for the agent. The only requirement for `TC_simulate` function its input and output should match the requirement described in {doc}`Agent{agent}`. This allows user to plug-in any simulator in to the TC_simulate function beyond Python simulation, including complicated simulators like CARLA. 
- `dynamic` function decribe the continous dynamics of the agent. It is used by the `TC_simulate` function for the `BaseAgent`. 
- `set_initial` sets the `init_cont`, `init_disc`. `static_parameters`, `uncertain_parameters` of the agent.
- `set_initial_state` sets the `init_cont` of the agent.
- `set_initial_mode` sets the `init_disc` of the agent.
- `set_initial_parameter` sets the `static_parameters` of the agent.
- `set_uncertain_parameter` sets the `uncertain_parameters` of the agent.

More detailed documentation for these functions can be found in the API doc below. 

```{eval-rst}
.. currentmodule:: verse
.. autosummary::
    :toctree: _autosummary

    BaseAgent
    parser.ControllerIR
```

## Creating custom agent
Due to the nature of Python, we allow duck typing fo the agent, i.e. an agent can be used in Verse as long as it have the proper members and member functions and the interface of the necessary functions match the requirements. 

The necessary class members are 
- `id`
- `decision_logic`
- `init_cont`
- `init_disc`
- `static_parameters`
- `uncertain_parameters`

Their type (or behavior) should match what's described in the previous section.

The necessary member functions are 
- `TC_simulate`
- `set_initial`
- `set_initial_state`
- `set_initial_mode`
- `set_static_parameter`
- `set_uncertain_parameter`

Similarly, their input/output type should match what's described in the previous section. 

Any other class functions or members can be added freely to the agent class. 