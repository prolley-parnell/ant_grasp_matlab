
modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel';
scriptFolder = pwd;
cd(modelFolder)

addpath('Environment', 'Grasp', 'ExperimentOutput', 'ToolClasses', 'ExperimentScripts', 'ExperimentScripts\GraphPlot');

cd(scriptFolder)
newForce = 1;



%% Make copy of GPC
GPC_Friction = GraphPlotClass();

%% load the results mat
experimentResultPath = [modelFolder, '\ExperimentScripts\GraphPlot\GPC_full_v2.mat'];
load(experimentResultPath, 'GPC_dice');


    
%% Load a copy of the environment and runtime args
runtimeArgsPath = [GPC_dice.experimentPath{1},'\2_contact_pts_25-02-23_06-23\mat-files\trial_1.mat']
load(runtimeArgsPath, 'setupRuntimeArgs');

env = CollisionObjects(setupRuntimeArgs);




