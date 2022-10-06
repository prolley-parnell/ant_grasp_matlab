%% Set the environment by closing any previous figures and variables
close all;
clear;
modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\';
scriptFolder = pwd;

cd(modelFolder)

addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
        'urdf', 'Grasp', 'ExperimentOutput', 'MainScript', 'ToolClasses');

rng('shuffle');

RUNTIME_ARGS = RuntimeArgs();

% Warnings are automatically enabled
%RUNTIME_ARGS.disableWarnings();

%% ----------- Experiment Set up
% Number of Iterations
RUNTIME_ARGS.N_TRIALS = 1;
RUNTIME_ARGS.RATE = 0.01;
RUNTIME_ARGS.PLOT.ENABLE = [1 1];

RUNTIME_ARGS.PRINTOUT.ENABLE = 0;
RUNTIME_ARGS.RECORD.ENABLE = 0;


RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.1;
%RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = './Environment/tray.stl';


RUNTIME_ARGS.BODY_MOTION_ENABLE = 0;


RUNTIME_ARGS.ANT_POSE = [0 0 0 0 0.3 -0.45 0.85 0.3 -0.45 0.85]';


% Weights
weights.Neck = 0.5;
weights.Antenna = 0.1;
weights.Mandible = 0.4;

% ------------- Antenna Motion -------------- %
%RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.MODE = "fixed";
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.MODE = "GM";
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.VAR = 0.5;
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.RANGE = [-1, 1; ...
                2.5, 3.5; ...
                0, 1];

RUNTIME_ARGS.SEARCH_SPACE.REFINE.MODE = '';
RUNTIME_ARGS.ANTENNA_CONTROL =  ["goals", "joint_traj"];

%No need for a threshold, as it picks the best after 10 contacts, and no
%new goal is generated
RUNTIME_ARGS.SENSE.THRESH = 0;
RUNTIME_ARGS.SENSE.MINIMUM_N = 4;

[exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS);

