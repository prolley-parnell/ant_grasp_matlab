%% Set the environment by closing any previous figures and variables
close all;
clear;

addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
        'urdf', 'Grasp', 'ExperimentOutput', 'MainScript', 'ToolClasses');

rng('shuffle');

RUNTIME_ARGS = RuntimeArgs();

% Warnings are automatically enabled
%RUNTIME_ARGS.disableWarnings();

%% ----------- Experiment Set up
% Number of Iterations
RUNTIME_ARGS.N_TRIALS = 1;
RUNTIME_ARGS.PLOT.ENABLE = [1 1];

RUNTIME_ARGS.PRINTOUT.ENABLE = 0;
RUNTIME_ARGS.RECORD.ENABLE = 0;

RUNTIME_ARGS.ANTENNA_CONTROL =  ["goals", "joint_traj"];

%RUNTIME_ARGS.BODY_NAV_MODE = "follow";

RUNTIME_ARGS.ANT_POSE = [0 0 0 0 0.3 -0.45 0.85 0 0 0]';


% Weights
weights.Neck = 0.5;
weights.Antenna = 0.1;
weights.Mandible = 0.4;

%% RUN TRIAL FIXED Mand GM %%

RUNTIME_ARGS.TRIAL_NAME = 'FIXED_SPACE';

% ------------- Antenna Motion -------------- %
RUNTIME_ARGS.SEARCH_SPACE.MODE = "fixed";
RUNTIME_ARGS.SEARCH_SPACE.RANGE = [-2, 2; ...
                2, 4; ...
                0, 1];
RUNTIME_ARGS.SEARCH_SPACE.VAR = 0.7;

%RUNTIME_ARGS.BODY_NAV_MODE = "follow";


[exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS);




