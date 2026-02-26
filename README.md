<<<<<<< HEAD
# Primitive Generation

## Requirements
- Matlab2023b
- Cora 2024 (preferably [this branch](https://gitlab.lrz.de/cps/cora/-/tree/changed-for-primitive-generation?ref_type=heads))
- Add files in [cora_extensions](./matlab-misc/cora_extensions) to Cora as specified in the [IMPORTANT.txt](../matlab-misc/cora_extensions/IMPORTANT.txt). This is only needed for certain operations - you can try without adding the files, and if errors during reach occur, you can add them.

Also make sure that `matlab` points to your used MATLAB installation
(see https://de.mathworks.com/matlabcentral/answers/93739-how-do-i-launch-matlab-on-linux#answer_103090).

## MATLAB path
Add all directories to Matlab path.
It is recommended to run the matlab script
```matlab
add_this_project
```
for this.

## Style checking and linting
We use *miss_hit* for MATLAB.
Check https://florianschanda.github.io/miss_hit/style_checker.html.

## MATLAB Setup
At best, you transfer the style settings in _miss_hit.cfg_ to
Matlab by going to _Preferences_ > _MATLAB_ > _Editor/Debugger_ > _Language_ and
selecting "Matlab" in the combo box.

## Project Structure

- [configs](configs)
Class definitions and object builders, that populate a template with specific values.

- [controlled_models](controlled_models)
Functions to combine a model and a controller to a system.

- [controllers](controllers)
Several controllers for the models.

- [generation](generation)
Infrastructure for generating primitives.

- [io](io)
Functionality for reading and writing data.

- [models](models)
Everything describing vehicle models.

- [primitive](primitive)
Infrastructure for primitives and their reachability analysis.

- [runner](runner)
Script for starting the generation of primitives.

- [state_space_division](state_space_division)
Infrastructure for dividing the state space into boxes in different ways;
a primitive always starts from such a box.

- [trajectories](trajectories)
Reference trajectories for primitives.

## Disabling CORA arg checking
If you are sure that your usage of CORA does not contain any mistakes,
you should disable CORA argument checking for the sake of a better performance.
This can be done by adding the folder _disable_cora_arg_checking_ to MATLAB
path. Ensure that the variable CHECKS_ENABLED indeed evaluates to false.

## Caching
When generating motion primitives, some intermediate results are cached in case that
the generation is canceled and restarted later.
If you want to restart the generation later from scratch,
delete the corresponding output folder that you have configured in your generation script.
=======
# CommmonRoad Primitive Planning
This project bundles functionality related to trajectory planning
with motion primitives, in particular
- generation of primitives,
- heuristic guided search, and
- reinforcement learning approaches for planning with primitives.

## Development Setup
We use Gitlab LRZ for private dependencies. Please configure SSH.

### Option 1: Dev Container (Recommended)
This project includes a [dev container](https://code.visualstudio.com/docs/devcontainers/containers#_quick-start-open-an-existing-folder-in-a-container) configuration.
Before starting a devcontainer, you need to set the _[AUTOMATON_STORAGE_DIR]_ in [devcontainer.json](.devcontainer/devcontainer.json); for more information, read the sections below.
You need to have [Docker installed](https://docs.docker.com/desktop/setup/install/linux/ubuntu/).
Use VSCode's [Dev Containers: Open Folder in Container](command:remote-containers.reopenInContainer) to reopen the project inside the container.
You might need to [share your SSH key](https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials) with the container.

#### Using Pre-built Image (Recommended)
The project provides a pre-built devcontainer image in the GitLab container registry.
You have to login first by running
```bash
docker login gitlab.lrz.de:5005
```

#### Building Locally (Fallback)
If you need to build the container locally (e.g., for development of the container itself):
Manually edit `devcontainer.json` to comment out the `"image"` line and uncomment the `"build"` section.
The container will then build from the Dockerfile in `.devcontainer/`

#### Troubleshooting
- Container does not start with error message something like "running `docker info` failed": restart Docker (e.g. by restarting Docker Desktop)
- If you somehow messed up the container: Use VSCode's [Dev Containers: Rebuild Without Cache and Reopen Folder in Container](command:remote-containers.rebuildNoCacheAndReopenInContainer)


### Option 2: Manual Setup
This project uses [poetry](https://python-poetry.org/) (>= version 2).
Please follow the instructions to create the virtual development environment.
The dependencies in the private LRZ Gitlab are references via SSH
as using https and setting the access token once did not work in the devcontainer; setting the token was done via [like this](https://python-poetry.org/docs/repositories/):
```bash
poetry config http-basic.tum-gitlab __token__ YOUR_TOKEN
```
However, if you do not use a devcontainer, you can change the dependencies in [pyproject.toml](pyproject.toml) to https and set your access token with the command above.
Also, run `pre-commit install` within the virtual environment for enabling
the pre-commit hook locally.



Check [this](https://stackoverflow.com/a/73218382) for configuring git to clean jupyter notebooks
before committing.

## General Project Structure
This project is structured into two main parts:
### [Primitive Generation](./matlab_primitive_generation)
This part is written in MATLAB and offers functionality to generate
motion primitives for a given vehicle model.
The generated primitives are exported to a common format to be used
in the planning part.
The primitive generation should generally be performed on a dedicated machine with many CPUs and a lot of memory.

If you intend to work on the planning part only, you can use the following
pre-generated primitives [here](https://syncandshare.lrz.de/getlink/fiU1DZFtapTFxdWL8iZnTP/Primitives).
We recommend to take the latest primitives there.
You possibly have to ask the owner of this repository for the password.
The [matlab_primitive_generation](./matlab_primitive_generation) part contains an own [README](./matlab_primitive_generation/README.md).
The rest of this README focuses on the planning part.

### [Primitive Planning](./primitive_planning)
For getting started with the planning with primitives, follow
- the [tutorial for truck only](./src/commonroad_primitive_planning/tutorial_matching_based_search.ipynb) (Jupyter notebook)
    ```bash
    jupyter lab src/commonroad_primitive_planning/tutorial_matching_based_search.ipynb
    ```
- the [tutorial for truck with trailer](./src/commonroad_primitive_planning/tutorial_matching_based_search_for_truck_trailer.py) (Marimo notebook)
    ```bash
    marimo edit src/commonroad_primitive_planning/tutorial_matching_based_search_for_truck_trailer.py
    ```
- the [tutorial for a car](./src/commonroad_primitive_planning/tutorial_matching_based_search_for_truck_trailer.py) (Marimo notebook)
    ```bash
    marimo edit src/commonroad_primitive_planning/tutorial_matching_based_search_for_car.py
    ```
  This notebook also contains a simple search algorithm, which is not only for planning fail-safe trajectories.

Some preliminary hints:
1. Whenever running code here which loads an automaton, you need to set
the environment variable `AUTOMATON_STORAGE_DIR` to point to some directory on your machine.
Downloaded automata and pickle files will be stored there.
You can also manually place generated automata there.
It is not recommended to use a directory in the project folder, as this can cause
severe slowdowns in the IDE (even if the folder is excluded from indexing).
2. Generated automata must be placed in "${AUTOMATON_STORAGE_DIR}/primitive_generation/out".
3. When a certain automaton is loaded for the first time, this will often take some time.
To speed up loading in the future, pickle files are created.

## Scenarios
You can find highD-scenario [here](https://syncandshare.lrz.de/getlink/fiPzVXriy4RQwcimvXS3n7/Scenarios).
The script [add_shoulder_lane_to_highD.py](./src/commonroad_primitive_planning/run/add_shoulder_lane_to_highD.py) can be used to add
a shoulder lane to the highD scenarios.

## Software Architecture
### Global configuration
To avoid passing global configuration parameters through all functions, we use a
[global configuration class](./src/commonroad_primitive_planning/configs/confs.py), which itself references
other configuration classes.
The configuration has to be set up before using any other functionality, which should be done
in a dedicated function in the [configs/setters](./src/commonroad_primitive_planning/configs/setters) module.
Have a look at the existing functions there to see how to set up the configuration and which parameters are available.


### CI
*TODO*: setup CI
- run _miss_hit_
>>>>>>> 67bab2472d8b73b4ec62ad18412b44cd15d2fdf3
