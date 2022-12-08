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


%% Generate a large number of points in space
%Find the range of the vertices that make up the shape to refine the search
%space for point generation
%Currently for single shape [TODO]
vert = env.FBT{1}.Points;
%Find maximum  and minimum vertices
maxV = max(vert);
minV = min(vert);

nRandomPt = 1000;

r = @(a, b, set) (a + (b-a).*set);
randomScale = rand([nRandomPt,3]);

randomSampleFull = r(minV,maxV,randomScale);

%% Find the points on the surface of the object that are guided by these points
% for single triangulation only [TODO] need to remove points that are
% considered to be internal
%{TODO] Can I do this by literally adding all of the triangulations
%together?

%For each point, find the face it is contained by
[pointOnObj, normalVArray, vertIDArray, faceIDArray] = tbox.findPointOnObjNormalID(env.FBT{1}, randomSampleFull);

%% Plot a subset to make sure they look right
% subsetSize = 5000;
% subsetPForPlot = pointOnObj(1:subsetSize,:);
% subsetFForPlot = normalVArray(1:subsetSize,:);
% 
% %Plotting
% hold on
% quiver3(subsetPForPlot(:,1),subsetPForPlot(:,2),subsetPForPlot(:,3), ...
%     subsetFForPlot(:,1),subsetFForPlot(:,2),subsetFForPlot(:,3),'off','color','r');

%% For finding the worst (completely random) - find the quality for every pair then find mean

%[From the grasp synth class]

obj = ant.graspGen;

contactStruct = struct();
contactStruct(:).point = pointOnObj;
contactStruct(:).normal = normalVArray;

goalOut = goalStruct();

%Cartesian contact points
cartPointArray = pointOnObj;
nContactPoint = size(cartPointArray,1);

[distanceMat, ~, ~] = obj.findInterPointDistance(cartPointArray, "none");

mandMaxFlag = obj.mandibleLimit(distanceMat);

[alignMeasure, ~, ~] = obj.findInterPointGraspAlign(contactStruct);

qualityMap = distanceMat.*mandMaxFlag.*alignMeasure;
[bestQuality, goalIndex] = max(qualityMap,[],'all');

%% Find the mean interpoint distance
meanIPD = mean(distanceMat,'all', 'omitnan');
%% Save
save("comparisonQualities.mat", "distanceMat", "mandMaxFlag", "alignMeasure", "contactStruct");

%% Find the mean midpoint offset

%Find midpoint of all pairs
COMOffset = nan([nContactPoint, nContactPoint]);
idx = 1;
objPose = env.objectHandles{idx}.Pose;
objCOM = env.COM{idx};
globalCOM = tbox.local2global(objCOM, objPose);
for n = 1:nContactPoint
    for m = n:nContactPoint
        if n~=m
            mp = mean([contactStruct.point(n,:); contactStruct.point(m,:)]);
            COMOffset(n, m) = vecnorm(mp-globalCOM,2,2);

        end
    end
end
meanCOMOffset = mean(COMOffset,'all', 'omitnan');
%%
save("comparisonQualities.mat", "distanceMat", "mandMaxFlag", "alignMeasure", "contactStruct", "COMOffset");

%% Find the mean wrench quality


%Calculate the volume and epsilon for the given goal grasp
%point locations
obj = ant.graspEval;
vertices = env.objectHandles{idx}.Vertices;
volumeArray = nan([nContactPoint, nContactPoint]);
epsilonArray = nan([nContactPoint, nContactPoint]);


for n = 1:nContactPoint
    for m = n:nContactPoint
        if n~=m
            A = contactStruct.point(n,:);
            B = contactStruct.point(m,:);
            if any([A-B])
                forces = obj.genOpposeForces(A, B);
                [volumeArray(n,m), epsilonArray(n,m)] = obj.findWrenchQuality(forces, [A;B], vertices, globalCOM, objCOM);
            else
                volumeArray(n,m) = 0;
                epsilonArray(n,m) = 0;
            end

        end
    end
end



%% Find the wrench quality
% [a, b] = ind2sub([nContactPoint, nContactPoint], goalIndex);
% [goalOut, ~] = goalOut.setcontact(contactStruct([a,b]));






%% For finding best (perfect placement) - find the maximum quality grasps

%Convert all quality measures to a scalar between lowest and highest (worst
%and best)
volumeScore = rescale(volumeArray);
epsilonScore = rescale(epsilonArray);
offsetScore = rescale(COMOffset);
alignScore = rescale(alignMeasure);

%Sum/multiply all values to find which scores the best on all fronts
%Assumes all features are weighted equally
sumScore = (volumeScore + epsilonScore + offsetScore + alignScore) .* mandMaxFlag;
prodScore = volumeScore .* epsilonScore .* offsetScore .* alignScore .* mandMaxFlag;

[maxScore, idx] = max(sumScore, 'all');
idx2rc  
%Find the grasps the score the best on each single measure and see how the
%other qualities compare to the best all round grasp
