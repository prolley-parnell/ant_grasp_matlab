%% Set the environment by closing any previous figures and variables
diary lastDiaryFile

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
RUNTIME_ARGS.N_TRIALS = 40;


RUNTIME_ARGS.RATE = 0.05;
RUNTIME_ARGS.PLOT.ENABLE = [0 0];

RUNTIME_ARGS.PRINTOUT.ENABLE = 1;
RUNTIME_ARGS.RECORD.ENABLE = 0;


RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.16;
RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = './Environment/12_sided_tiny_shape.stl';
RUNTIME_ARGS.BODY_MOTION_ENABLE = 0;


RUNTIME_ARGS.ANT_POSE = [0 0 0 0 0.3 -0.45 0.85 0.3 -0.45 0.85]';

RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.MODE = "fixed";
RUNTIME_ARGS.SEARCH_SPACE.SAMPLE.VAR = 0.5;
RUNTIME_ARGS.SEARCH_SPACE.RANGE = [-1, 1; ...
    2.5, 3.5; ...
    0, 1];
RUNTIME_ARGS.ANTENNA_CONTROL =  ["goals", "joint_traj"];

%No need for a threshold, as it picks the best after 10 contacts, and no
%new goal is generated
RUNTIME_ARGS.SENSE.THRESH = 0;
RUNTIME_ARGS.SENSE.MODE = "force_align";
RUNTIME_ARGS.SENSE.MINIMUM_N = 10;
RUNTIME_ARGS.ANT_MEMORY = 10;

RUNTIME_ARGS.SEARCH_SPACE.REFINE.MODE = 'IG';


%% --- Define controlled variables

sig1 = [0.3:0.1:1.2];
sig3 = [0.3:0.1:1.2];
sigalpha = [0.5:0.1:1.5];
mu3 = [0.4:0.1:1.2];

medianVals = [0.75, 0.75, 1, 0.8];

nP1 = length(sig1);
nP2 = length(sig3);
nP3 = length(sigalpha);
nP4 = length(mu3);

nExperiment = nP1+nP2+nP3+nP4;
IGArgArray = zeros([nExperiment, 4]);
experimentCounter = 1;

for a=1:nP1
    IGArgArray(experimentCounter,:) = [sig1(a), medianVals(2), medianVals(3), medianVals(4)];
    experimentCounter = experimentCounter + 1;
end
for b=1:nP2
    IGArgArray(experimentCounter,:) = [medianVals(1), sig3(b), medianVals(3), medianVals(4)];
    experimentCounter = experimentCounter + 1;
end
for c=1:nP3
    IGArgArray(experimentCounter,:) = [medianVals(1), medianVals(2), sigalpha(c), medianVals(4)];
    experimentCounter = experimentCounter + 1;
end
for d=1:nP4
    IGArgArray(experimentCounter,:) = [medianVals(1), medianVals(2), medianVals(3), mu3(d)];
    experimentCounter = experimentCounter + 1;
end
RUNTIME_ARGS_i = repmat(RUNTIME_ARGS, [1, nExperiment]);

for i = 1: nExperiment
    RUNTIME_ARGS_i(i).TRIAL_NAME = ['IGEF10ContactsXe-2_', int2str(IGArgArray(i,1)*100), '_', int2str(IGArgArray(i,2)*100), '_', int2str(IGArgArray(i,3)*100), '_', int2str(IGArgArray(i,4)*100)];
    RUNTIME_ARGS_i(i).SEARCH_SPACE.REFINE.PARAM = IGArgArray(i,:);
end

tic
p = gcp;
parfevalOnAll(p,@warning, 0,'off');
opts = parforOptions(p);
parfor (n = 1:nExperiment, opts)
    [exitflag, fileHandler] = AntModelFunction(RUNTIME_ARGS_i(n));
end
toc

diary off