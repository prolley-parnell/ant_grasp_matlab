%% Given a struct of goal positions, evaluate the quality of these grasps
% Requires knowing the COM of the object and the scale

clear all
close all

modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel';
scriptFolder = pwd;
cd(modelFolder)

addpath('Environment', 'Grasp', 'ExperimentOutput', 'ToolClasses');

folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
file_name = 'allExperimentGraspContacts.mat';

RUNTIME_ARGS = RuntimeArgs();

% Warnings are automatically enabled
RUNTIME_ARGS.disableWarnings();


RUNTIME_ARGS.PLOT.ENABLE = [0 0];
RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.1;
RUNTIME_ARGS.GRASP.FORCE = 1;
RUNTIME_ARGS.GRASP.OBJ_FRICTION = 0.7;

env = CollisionObjects(RUNTIME_ARGS);
gE = graspEvaluator(RUNTIME_ARGS);

load([folder_name, '\', file_name])
nExperiment = length(resultsStruct);
numberOfContacts = nan([nExperiment,1]);

resultsTable = table();
for i = 1:nExperiment
    experimentStructi = resultsStruct(i);
    name_split = split(experimentStructi.experiment_name,'_');
    numberOfContacts = str2num(name_split{1});
    nTrial = length(experimentStructi.experimentGoalStruct);
    qualityObject = repmat(graspQuality, [nTrial,1]);
    for j = 1:nTrial
        trialGoal = experimentStructi.experimentGoalStruct(j);
        qualityObject(j) = gE.evaluateGoal(trialGoal, env);
    end

    experimentQuality = cat(2,[qualityObject(:).volume]', [qualityObject(:).epsilon]', [qualityObject(:).com_offset]');
    resultsTable.nContacts(i) = numberOfContacts;

    resultsTable.meanVolume(i) = mean(experimentQuality(:,1));
    resultsTable.varVolume(i) = var(experimentQuality(:,1));
    resultsTable.minmaxVolume(i,:) = [min(experimentQuality(:,1)) , max(experimentQuality(:,1))];

    resultsTable.meanEpsilon(i) = mean(experimentQuality(:,2));
    resultsTable.varEpsilon(i) = var(experimentQuality(:,2));
    resultsTable.minmaxEpsilon(i,:) = [min(experimentQuality(:,2)) , max(experimentQuality(:,2))];

    resultsTable.meanCOMOffset(i) = mean(experimentQuality(:,3));
    resultsTable.varCOMOffset(i) = var(experimentQuality(:,3));
    resultsTable.minmaxOffset(i,:) = [min(experimentQuality(:,3)) , max(experimentQuality(:,3))];

end

cd(scriptFolder)
save('graspQualityEval.mat', 'resultsTable')