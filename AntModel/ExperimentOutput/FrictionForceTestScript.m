%% Set the environment by closing any previous figures and variables
close all;
clear;

addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
    'urdf', 'Grasp', 'ExperimentOutput', 'MainScript', 'ToolClasses');

rng('shuffle');

RUNTIME_ARGS = RuntimeArgs();

% Warnings are automatically enabled
RUNTIME_ARGS.disableWarnings();


RUNTIME_ARGS.PLOT.ENABLE = [0 0];
RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.1;




testFriction = [5:1:25]/10;
testForce = [7:1:25]/10;
nExperiment = length(testForce) * length(testFriction);
RUNTIME_ARGS_i = repmat(RUNTIME_ARGS, [length(testForce), length(testFriction)]);

for i = 1: length(testFriction)
    for j = 1:length(testForce)
        RUNTIME_ARGS_i(i,j).GRASP.OBJ_FRICTION = testFriction(i);
        RUNTIME_ARGS_i(i,j).GRASP.FORCE = testForce(j);
    end
end


%% Load goal example
folder_name = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\ExperimentOutput\remoteParallelFunction';
fileStruct = load([folder_name, '\goalExample.mat']);
bestGoal = fileStruct.bestGoal;
goalObj = goalStruct();

goalObj.midpoint = [bestGoal.Midpoint_1, bestGoal.Midpoint_2, bestGoal.Midpoint_3];
%goalObj.contact_axis = [bestGoal.ContactAxis_1, bestGoal.ContactAxis_2, bestGoal.ContactAxis_3];
goalObj.contact_point_array(1,:) = [bestGoal.PointA_1, bestGoal.PointA_2, bestGoal.PointA_3];
goalObj.contact_point_array(2,:) = [bestGoal.PointB_1, bestGoal.PointB_2, bestGoal.PointB_3];


%make env
%make grasp eval
%run test on goal struct

tic
p = gcp;
parfevalOnAll(p,@warning, 0,'off');
opts = parforOptions(p);
parfor (n = 1:nExperiment, opts)
    env = CollisionObjects(RUNTIME_ARGS_i(n));
    gE = graspEvaluator(RUNTIME_ARGS_i(n))
    qualityObject(n) = gE.evaluateGoal(goalObj, env);
end
toc
%2.42Hours