# Verse Library Documentation
This forder contains the documentation template for the Verse library. The documentation is automatically generated using [sphinx](https://www.sphinx-doc.org/en/master/). 

<img src="source/figs/drone-2-8.gif" height="160"/> <img src="source/figs/nondeterm_sensor.png" height="160"/> <img src="source/figs/car-ped-1.png" height="160"/> <img src="source/figs/uam-collision.png" height="160"/>

## Prerequsites 
The following libraries are required for compile verse's document
- sphinx
- myst-parser 
- numpydoc 
- sphinx_rtd_theme

The required packages can be installed using command 
```
python3 -m pip install -r requirements-doc.txt
```

## Compiling documents
The document can be compiled using command 
```
python3 -m sphinx -M html docs/source docs/build
```
in the root folder

## Viewing documents
The compiled result can be found in ```docs/build/html``` folder. The root of the compiled document is stored in ```docs/build/html/index.html```

## Example architectural document
An example highlevel architectural document can be found in file ```docs/source/parser.md```

## Example docstring for class/function
An example docstring for class function can be found in file ```verse/agents/base_agent.py``` for class ```BaseAgent``` and function ```BaseAgent.__init__``` and ```BaseAgent.TC_simulate```
