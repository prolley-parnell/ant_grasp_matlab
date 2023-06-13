# ant_grasp_matlab

**# Changes**

## Position Control
Originally this model used the discrete time steps to plot the approach and grasp of a proposed grasp. However as this was for visual information and wasn't required to generate a synthetic grasp, the movement aspect gradually drifted out of the code and the noise/error was introduced manually (using ray intersection) rather than through the URDF and collision calculation.
The main difficulty arose from trying to generate a reasonable grasp approximation using the URDF, and collision when the ```CollisionObject``` used to estimate antennal collision is essentially an initial approximation before the actual location is generated using triangle-ray intersection.
If you really do want this functionality, you basically need to add back in a point where the position path is generated and don't enable ``` grasp_complete ``` in the Ant class, until you want to exit the trial loop.

### Mandible Control
This makes the following things redundant (and therefore useful if you want to reintroduce the movement back into the model):
```Ant:mandible_state```
```PoseControlClass:moveMandible```

"```actionGen```"```.loadMandibleTrajectory``` has been left in both ```JointActionGen``` and ```SampleActionGen``` so that future work can build on this starting code, however variable naming may have changed. This function is duplicated in both classes and in future versions, it would be nice to not duplicate this code and have it written once in a single location.

## Non-convex shapes
This was always the plan to include this in the model, but the model has been developed in a very iterative sense and generally concave shapes are difficult to generate contacts for. While it is possible to render them in the model, it is by externally separating concave shapes into a set number of convex triangulations and importing that set of convex sub-shapes. The surface normal and point of collision works fairly well for these clusters of shapes but because of how the calculation occurs, the simulation is a lot slower as it checks every sub-shape for collisions individually. There is surely a way that the process could be sped up but it wasn't significant enough for the research at the time to work on it.

## ```obj.RUNTIME_ARGS```
This is basically a copy of the construction instructions for the model, and ideally all significant variables are copied from the class so the variable can be changed in the specific class, but also it's not ideal if ```RUNTIME_ARGS.PLOT``` is enabled in some classes but not all of them, so some variables are hidden within the local ```obj.RUNTIME_ARGS``` copy. If in doubt, check the class initialisation to see what is copied out.

## ```Neck``` Class
This class 