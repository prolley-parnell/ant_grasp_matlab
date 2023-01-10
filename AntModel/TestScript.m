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

RUNTIME_ARGS = RUNTIME_ARGS.setAntennaControl({'p2p', 'mean', 'vardec'});

% Warnings are automatically enabled
RUNTIME_ARGS.disableWarnings();

%% ----------- Experiment Set up
% Number of Iterations
RUNTIME_ARGS.N_TRIALS = 1;
RUNTIME_ARGS.RATE = 0.001;
RUNTIME_ARGS.PLOT.ENABLE = [1 0];

RUNTIME_ARGS.PRINTOUT.ENABLE = 0;
RUNTIME_ARGS.RECORD.ENABLE = 0;

% RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.04;
% RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = './Environment/doughnut_v8_STL';

RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.18;
RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = './Environment/12_sided_tiny_shape.stl';
% RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.02;
% RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = './Environment/plank.stl';
RUNTIME_ARGS.COLLISION_OBJ.POSITION = [0 3 0.5];

RUNTIME_ARGS.BODY_MOTION_ENABLE = 0;


RUNTIME_ARGS.ANT_POSE = [0 0 0 0 0.3 -0.45 0.85 0.3 -0.45 0.85]';




% ------------- Antenna Motion -------------- %


RUNTIME_ARGS.SENSE.MODE = {'dist', 'align'};



%No need for a threshold, as it picks the best after 10 contacts, and no
%new goal is generated
RUNTIME_ARGS.SENSE.THRESH = 0;
RUNTIME_ARGS.SENSE.MINIMUM_N = 15;

[exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS);

