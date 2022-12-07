%%Baseline Test
%   Code to generate random poses of the ant head and generate grasps that are
%   random but initiated by the ant moving towards the object with open
%   mandibles

%% Add folders to path
close all;
clear;
modelFolder = 'C:\Users\eroll\Documents\MATLAB\Model\ant_grasp_matlab\AntModel\';
scriptFolder = pwd;

cd(modelFolder)

addpath('MotionControl','ModelAntBody', 'BehaviourControl', 'Environment',...
        'urdf', 'Grasp', 'ExperimentOutput', 'MainScript', 'ToolClasses');

rng('shuffle');

%% Define the object parameters and ant starting pose
RUNTIME_ARGS = RuntimeArgs();

RUNTIME_ARGS.COLLISION_OBJ.SCALE = 0.18;
RUNTIME_ARGS.COLLISION_OBJ.FILE_PATH = './Environment/12_sided_tiny_shape.stl';
RUNTIME_ARGS.COLLISION_OBJ.POSITION = [0 3 0.5];

RUNTIME_ARGS.ANT_POSE = [0 0 0 0 0.3 -0.45 0.85 0.3 -0.45 0.85]';
RUNTIME_ARGS.ANT_POSITION = [0 0 0 0];

%% Load in the model and object
RUNTIME_ARGS.PLOT.ENABLE = [1 0];
RUNTIME_ARGS.initiatePlots();

%Define Ant object
ant = Ant(RUNTIME_ARGS);

%Define the collision objects in the model environment
env = CollisionObjects(RUNTIME_ARGS);


%% Generate a random head and neck pose

randomPose = randomConfiguration(ant.antTree);
maskedPose = ant.neckObj.applyMask(ant.q, randomPose);

ant.q = maskedPose;

ant.plotAnt;

%% Open the mandibles to their maximum position

mandibleLeft = ant.limbs{3};
mandibleRight = ant.limbs{4};

openMandPose_L = mandibleLeft.applyMask(ant.q, mandibleLeft.joint_limits(2));
openMandPose = mandibleRight.applyMask(openMandPose_L, mandibleRight.joint_limits(2));

ant.q = openMandPose;

%% Find the tip of the mandible positions at this pose
mandibleLeft.free_point = tbox.findFKglobalPosition(ant.antTree, ant.q, ant.position, mandibleLeft.end_effector);
mandibleRight.free_point = tbox.findFKglobalPosition(ant.antTree, ant.q, ant.position, mandibleRight.end_effector);

ant.limbs{3} = mandibleLeft;
ant.limbs{4} = mandibleRight;

%% Find the nearest point on the object to grasp

%Find point on ant of reference (midpoint of mandible base)
mandBaseMP = tbox.findFKglobalPosition(ant.antTree, ant.q, ant.position, 'mandible_base_link');
nSubObj = length(env.objectHandles);
closeVertexDist = nan([nSubObj, 2]);
for i = 1:nSubObj
[closeVertexDist(i,1), closeVertexDist(i,2)] = nearestNeighbor(env.FBT{i}, mandBaseMP);
end

[dist, idx] = min(closeVertexDist(:,2));
closestVertex = env.FBT{idx}.Points(closeVertexDist(idx,1),:);

%% Assume the ant moves without rotating head (For Baseline V1) and find the vector
directionVector = closestVertex - mandBaseMP;

%% Find a position trajectory to make the mandible base be at the closest vertex
startPosition = ant.position(1:3);
endPosition = ant.position(1:3) + directionVector;


collisionDistThreshold = 0.09;
increment = sqrt((collisionDistThreshold^2)/3);

nStep = ceil(dist/collisionDistThreshold);


x_traj = [startPosition(1):directionVector(1)/nStep:endPosition(1)];
y_traj = [startPosition(2):directionVector(2)/nStep:endPosition(2)];
z_traj = [startPosition(3):directionVector(3)/nStep:endPosition(3)];

trajectoryOut = [x_traj', y_traj', z_traj'];

%Check that the trajectory is not moving more than the threshold
distancePerStep = trajectoryOut(2:end,:) - trajectoryOut(1:end-1,:);
magDist = vecnorm(distancePerStep, 2, 2);
overRun = find(magDist>collisionDistThreshold);

%% Move the ant closer to the object and check for collisions
nPosition = length(x_traj);

for j = 1:nPosition
    ant.position(1:3) = trajectoryOut(j,:);

    ant.poseController.tactileSenseEval()

    %[limbs{i}, dataStruct] = obj.tactileSenseEval(limbs{i}, ant.q, ant.position, env);
end



