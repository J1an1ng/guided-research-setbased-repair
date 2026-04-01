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
