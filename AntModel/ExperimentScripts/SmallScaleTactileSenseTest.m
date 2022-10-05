%% Set the environment by closing any previous figures and variables
close all;
clear;

modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\';
scriptFolder = pwd;

cd(modelFolder)
addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
    'urdf', 'Grasp', 'ExperimentOutput', 'ExperimentScripts', 'MainScript', 'ToolClasses');

rng('shuffle');

%% ----------- Experiment Set up

RUNTIME_ARGS = RuntimeArgs();

% Warnings are automatically enabled
RUNTIME_ARGS.disableWarnings();

% Number of Iterations
RUNTIME_ARGS.N_TRIALS = 50;

NumberOfPoints = [2:1:40];
%NumberOfPoints = [9:1:40];

nExperiment = length(NumberOfPoints);

RUNTIME_ARGS.RATE = 0.05;
RUNTIME_ARGS.PLOT.ENABLE = [0 0];

RUNTIME_ARGS.PRINTOUT.ENABLE = 1;
RUNTIME_ARGS.RECORD.ENABLE = 0;


RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.1;
RUNTIME_ARGS.BODY_MOTION_ENABLE = 0;


RUNTIME_ARGS.ANT_POSE = [0 0 0 0 0.3 -0.45 0.85 0.3 -0.45 0.85]';
% ------------- Antenna Motion -------------- %
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.MODE = "GM";
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.VAR = 0.5;
RUNTIME_ARGS.SEARCH_SPACE.RANGE = [-1, 1; ...
    2.5, 3.5; ...
    0, 1];
RUNTIME_ARGS.ANTENNA_CONTROL =  ["goals", "joint_traj"];

%No need for a threshold, as it picks the best after 10 contacts, and no
%new goal is generated
RUNTIME_ARGS.SENSE.THRESH = 0;



%%
%RUNTIME_ARGS.SEARCH_SPACE.REFINE.MODE = 'IG';
RUNTIME_ARGS.SEARCH_SPACE.REFINE.MODE = '';


RUNTIME_ARGS_i = repmat(RUNTIME_ARGS, [1, nExperiment]);
for i = 1: nExperiment
    RUNTIME_ARGS_i(i).TRIAL_NAME = ['noRefine2-40\', int2str(NumberOfPoints(i)), '_ContactPts_noRefine'];
    RUNTIME_ARGS_i(i).ANT_MEMORY = NumberOfPoints(i);
    RUNTIME_ARGS_i(i).SENSE.MINIMUM_N = NumberOfPoints(i);
end

tic
p = gcp;
parfevalOnAll(p,@warning, 0,'off');
opts = parforOptions(p);
parfor (n = 1:nExperiment, opts)
    [exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS_i(n));
end
toc


% RUNTIME_ARGS.SEARCH_SPACE.REFINE.MODE = '';
% 
% 
% RUNTIME_ARGS_i = repmat(RUNTIME_ARGS, [1, nExperiment]);
% for i = 1: nExperiment
%     RUNTIME_ARGS_i(i).TRIAL_NAME = ['norefine_small2-30\', int2str(NumberOfPoints(i)), '_ContactPts_IGEF'];
%     RUNTIME_ARGS_i(i).ANT_MEMORY = NumberOfPoints(i);
%     RUNTIME_ARGS_i(i).SENSE.MINIMUM_N = NumberOfPoints(i);
% end
% 
% tic
% p = gcp;
% parfevalOnAll(p,@warning, 0,'off');
% opts = parforOptions(p);
% for n = 1:nExperiment
%     [exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS_i(n));
% end
% toc