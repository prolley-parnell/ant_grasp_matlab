%% Set the environment by closing any previous figures and variables
close all;
clear;

addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
    'urdf', 'Grasp', 'ExperimentOutput', 'MainScript', 'ToolClasses');

rng('shuffle');

RUNTIME_ARGS = RuntimeArgs();

% Warnings are automatically enabled
RUNTIME_ARGS.disableWarnings();

%% ----------- Experiment Set up
% Number of Iterations
RUNTIME_ARGS.N_TRIALS = 1;
RUNTIME_ARGS.RATE = 0.05;
RUNTIME_ARGS.PLOT.ENABLE = [0 0];

RUNTIME_ARGS.PRINTOUT.ENABLE = 1;
RUNTIME_ARGS.RECORD.ENABLE = 0;


RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.1;


RUNTIME_ARGS.BODY_MOTION_ENABLE = 0;


RUNTIME_ARGS.ANT_POSE = [0 0 0 0 0.3 -0.45 0.85 0.3 -0.45 0.85]';


% Weights
weights.Neck = 0.5;
weights.Antenna = 0.1;
weights.Mandible = 0.4;

%% RUN TRIAL FIXED Mand GM %%


% ------------- Antenna Motion -------------- %
RUNTIME_ARGS.SEARCH_SPACE.MODE = "fixed";
RUNTIME_ARGS.SEARCH_SPACE.RANGE = [-1, 1; ...
    2.5, 3.5; ...
    0, 1];
RUNTIME_ARGS.ANTENNA_CONTROL =  ["goals", "joint_traj"];

%No need for a threshold, as it picks the best after 10 contacts, and no
%new goal is generated
RUNTIME_ARGS.SENSE.THRESH = 0;
%NumberOfPoints = {10, 20, 30, 40, 50};
NumberOfPoints = {10, 20, 30, 40, 50, 100, 200};
nExperiment = length(NumberOfPoints);
RUNTIME_ARGS_i = repmat(RUNTIME_ARGS, [1, nExperiment]);
for i = 1: nExperiment
    RUNTIME_ARGS_i(i).TRIAL_NAME = [int2str(NumberOfPoints{i}), '_ContactPts'];
    RUNTIME_ARGS_i(i).ANT_MEMORY = NumberOfPoints{i};
    RUNTIME_ARGS_i(i).SENSE.MINIMUM_N = NumberOfPoints{i};
end


p = gcp;
parfevalOnAll(p,@warning, 0,'off');
opts = parforOptions(p);
parfor (n = 1:nExperiment, opts)
    [exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS_i(n));
end

